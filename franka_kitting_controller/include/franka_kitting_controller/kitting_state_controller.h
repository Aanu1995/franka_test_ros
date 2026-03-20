// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <realtime_tools/realtime_buffer.h>

#include <actionlib/server/simple_action_server.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <franka_kitting_controller/KittingGripperCommand.h>
#include <franka_kitting_controller/KittingState.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include <sms_cusum/sms_cusum.hpp>

namespace franka_kitting_controller {

  /// Grasp states for data collection and automated force ramp.
  /// Controller starts in START — no baseline collection, no contact detection,
  /// just Cartesian passthrough and state data publishing. The grasp state machine
  /// begins when the user explicitly publishes BASELINE on /kitting_controller/state_cmd.
  ///
  /// User-commanded: START→BASELINE, →CLOSING_COMMAND, →GRASPING
  /// Automatic:      CLOSING_COMMAND→CLOSING (gripper move confirmed)
  ///                 CLOSING→CONTACT_CONFIRMED (contact detected)
  ///                 CONTACT_CONFIRMED→CONTACT (gripper stopped)
  /// Internal (realtime):  GRASPING→UPLIFT→EVALUATE→SUCCESS
  ///                 On slip: EVALUATE→SLIP→DOWNLIFT→SETTLING→GRASPING (retry)
  ///                 On F > f_max: SETTLING→FAILED
  enum class GraspState {
    START,           // Initial state: controller running, awaiting BASELINE command
    BASELINE,        // Collecting reference signals
    CLOSING_COMMAND, // Gripper close queued — awaiting execution confirmation
    CLOSING,            // Gripper confirmed moving toward object
    CONTACT_CONFIRMED,  // Contact detected (SMS-CUSUM) — gripper stopping
    CONTACT,            // Gripper stopped — contact confirmed
    GRASPING,     // Applying grasp force, waiting for completion + stabilization
    UPLIFT,       // Cosine-smoothed micro-lift trajectory
    EVALUATE,     // Hold + slip detection (hold for uplift_hold, then evaluate immediately)
    SLIP,         // Slip detected — preparing DOWNLIFT
    DOWNLIFT,     // Returning to z_initial before force increment
    SETTLING,     // Post-downlift stabilization
    SUCCESS,      // Stable grasp confirmed
    FAILED        // Max force exceeded
  };

  /// Command types for the gripper command thread.
  enum class GripperCommandType { NONE, MOVE, GRASP, HOMING };

  /// Command queued from subscriber thread to the gripper command thread.
  struct GripperCommand {
    GripperCommandType type{GripperCommandType::NONE};
    double width{0.0};
    double speed{0.0};
    double force{0.0};
    double epsilon_inner{0.008};
    double epsilon_outer{0.008};
    std::shared_ptr<std::promise<bool>> result_promise;  // nullptr = fire-and-forget
  };

  /// Data transferred from gripper read thread (non-realtime) to update() (realtime) via RealtimeBuffer.
  struct GripperData {
    double width{0.0};       // from franka::GripperState.width [m]
    double max_width{0.0};   // from franka::GripperState.max_width [m]
    double width_dot{0.0};   // Finite difference velocity [m/s]
    bool is_grasped{false};  // from franka::GripperState.is_grasped
    ros::Time stamp;         // Timestamp of the measurement
  };

  /**
  * Controller that publishes comprehensive robot state, implements contact
  * detection, and executes an automated force ramp with micro-lift evaluation.
  *
  * Claims FrankaModelInterface, FrankaStateInterface, and FrankaPoseCartesianInterface.
  * The Cartesian pose interface is used for UPLIFT/DOWNLIFT trajectories. In all other
  * states, the controller operates in passthrough mode (holds current position).
  *
  * Grasp topics:
  *   /kitting_controller/state_cmd [subscribed]  KittingGripperCommand (BASELINE, CLOSING, GRASPING)
  *                                           with per-object parameters (0 = use YAML default)
  *   /kitting_controller/state     [published]   State labels for all states (BASELINE, CLOSING,
  *                                           CONTACT, GRASPING, UPLIFT, EVALUATE, DOWNLIFT,
  *                                           SETTLING, SUCCESS, FAILED).
  *
  * Recording is controlled separately via /kitting_controller/record_control
  * (handled by the logger node, not the controller).
  */
  class KittingStateController
      : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
                                                              franka_hw::FrankaStateInterface,
                                                              franka_hw::FrankaPoseCartesianInterface> {
  public:
    ~KittingStateController() override;

    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time& time, const ros::Duration& period) override;

  private:
    // --- Hardware handles ---
    franka_hw::FrankaStateInterface* franka_state_interface_{nullptr};
    std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;
    franka_hw::FrankaModelInterface* model_interface_{nullptr};
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_{nullptr};
    std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;

    // --- State data publisher ---
    realtime_tools::RealtimePublisher<franka_kitting_controller::KittingState> kitting_publisher_;
    franka_hw::TriggerRate rate_trigger_{250.0};

    // --- Grasp: State label publisher ---
    realtime_tools::RealtimePublisher<std_msgs::String> state_publisher_;

    // Logger readiness gate (dual check: latched topic + live publisher count)
    bool require_logger_{false};
    ros::Subscriber logger_ready_sub_;
    std::atomic<bool> logger_ready_{false};
    void loggerReadyCallback(const std_msgs::Bool::ConstPtr& msg);

    // --- Grasp: Command subscriber (listens to /kitting_controller/state_cmd) ---
    ros::Subscriber state_cmd_sub_;
    void stateCmdCallback(const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg);

    // --- Direct gripper connection (libfranka) ---
    std::unique_ptr<franka::Gripper> gripper_;
    std::string robot_ip_;

    // Gripper read thread: continuous readOnce() at firmware rate
    std::thread gripper_read_thread_;
    std::atomic<bool> gripper_shutdown_{false};
    std::atomic<bool> stop_requested_{false};  // Realtime → read thread: call stop()
    void gripperReadLoop();

    // Gripper command thread: executes blocking move()/grasp()
    std::thread gripper_cmd_thread_;
    std::mutex cmd_mutex_;
    std::condition_variable cmd_cv_;
    GripperCommand pending_cmd_;       // protected by cmd_mutex_
    bool cmd_ready_{false};            // protected by cmd_mutex_
    std::atomic<bool> cmd_executing_{false};
    void gripperCommandLoop();
    void queueGripperCommand(const GripperCommand& cmd);

    // Gripper data buffer (read thread → realtime update)
    realtime_tools::RealtimeBuffer<GripperData> gripper_data_buf_;

    // --- Baseline preparation: sequential lower → open → collect ---
    // baseline_prep_done_ is the SOLE gate for baseline collection.
    // It starts FALSE when BASELINE is entered and becomes TRUE only after:
    //   1. Downlift completes (if arm was elevated)
    //   2. Gripper open completes (if open_gripper or record:=true)
    // The RT thread sets it when no prep is needed or when all prep finishes.
    std::atomic<bool> baseline_prep_done_{true};         // FALSE = prep in progress, baseline gated
    bool baseline_needs_open_{false};                    // Open requested (subscriber thread → RT thread)
    std::atomic<bool> baseline_open_dispatched_{false};  // Open command queued by read thread
    bool baseline_open_seen_executing_{false};           // cmd_executing_ went true after dispatch (read thread)
    double baseline_open_width_{0.0};                    // Width to open to [m]

    // --- Grasp: Gripper default parameters (overridable per-command) ---
    double closing_width_{0.001};
    double closing_speed_{0.05};

    // State machine — cross-thread synchronization
    // current_state_: read by subscriber thread (precondition checks), written by Realtime thread.
    //   Also written by Realtime thread for internal transitions (GRASPING→UPLIFT→EVALUATE etc.)
    // pending_state_: written by subscriber thread, read by Realtime thread.
    // state_changed_: synchronization flag — release on write, acquire on read.
    //   All stores before the release-store of state_changed_ are visible to the Realtime thread
    //   after its acquire-load of state_changed_ returns true.
    std::atomic<GraspState> current_state_{GraspState::START};
    std::atomic<GraspState> pending_state_{GraspState::START};
    std::atomic<bool> state_changed_{false};
    bool contact_latched_{false};

    // --- SMS-CUSUM contact detection (noise-adaptive, replaces fixed threshold) ---
    sms_cusum::SMSCusum sms_detector_{sms_cusum::SMSCusumConfig{}};
    std::atomic<bool> gripper_stop_sent_{false};  // Written by realtime, read by command thread
    std::atomic<bool> gripper_stopped_{false};    // Read thread → realtime: stop() completed AND post-stop width captured
    std::atomic<bool> width_capture_pending_{false};  // Read thread internal: capture contact_width_ on next readOnce()
    std::atomic<double> contact_width_{0.0};  // [m] Width at contact — written by read thread after stop()

    // RT-local copies of CLOSING params (snapshotted at CLOSING_COMMAND entry)
    double closing_w_cmd_{0.001};    // Resolved target width for active MoveAction
    double closing_v_cmd_{0.05};     // Resolved speed for active MoveAction
    double rt_closing_w_cmd_{0.001}; // Realtime-local copy
    double rt_closing_v_cmd_{0.05};  // Realtime-local copy
    bool closing_cmd_seen_executing_{false};  ///< True once move() seen running during CLOSING
    bool closing_command_entered_{false};     ///< First-tick flag: ensures CLOSING_COMMAND label is published before transition

    // Contact detection runtime state (RT-thread owned, read by timer thread via AUTO polling)
    int    cd_baseline_count_{0};
    double cd_baseline_{0.0};
    std::atomic<bool> cd_baseline_ready_{false};  // Atomic: written by RT, read by AUTO poll timer

    // Slow-rate logger for contact signal monitoring (2 Hz — readable in terminal)
    franka_hw::TriggerRate signal_log_trigger_{2.0};

    // Faster gripper velocity logger (10 Hz — for debugging gripper speed profile)
    franka_hw::TriggerRate gripper_log_trigger_{10.0};

    // --- UPLIFT trajectory state (Realtime-thread owned) ---
    std::atomic<bool> uplift_active_{false};
    double uplift_elapsed_{0.0};
    std::array<double, 16> uplift_start_pose_{};
    double uplift_z_start_{0.0};
    // Realtime-local copies of UPLIFT parameters — set when UPLIFT starts internally.
    double rt_uplift_distance_{0.010};
    double rt_uplift_duration_{1.0};

    // --- DOWNLIFT trajectory state (Realtime-thread owned) ---
    std::atomic<bool> downlift_active_{false};
    double downlift_elapsed_{0.0};
    std::array<double, 16> downlift_start_pose_{};
    double downlift_z_start_{0.0};
    double rt_downlift_distance_{0.010};
    double rt_downlift_duration_{1.0};

    // --- Force ramp: YAML default parameters (overridable per-command via KittingGripperCommand) ---
    double fr_f_min_{3.0};
    double fr_f_step_{3.0};
    double fr_f_max_{70.0};
    double fr_uplift_distance_{0.010};
    double fr_lift_speed_{0.01};
    double fr_uplift_hold_{1.0};
    double fr_grasp_speed_{0.02};
    double fr_epsilon_{0.008};         // Single epsilon (inner == outer)
    double fr_slip_drop_thresh_{0.15};       // DF_TH: max allowed relative support force drop (15% = fail)
    double fr_slip_width_thresh_{0.0005};    // [m] W_TH: max allowed jaw widening P95-P5 (0.5mm = fail)
    double fr_load_transfer_min_{1.5};       // [N] Min floor for load transfer threshold

    // RT-local copies of force ramp params (snapshotted at GRASPING entry)
    double rt_fr_f_min_{3.0};
    double rt_fr_f_step_{3.0};
    double rt_fr_f_max_{70.0};
    double rt_fr_uplift_distance_{0.010};
    double rt_fr_lift_speed_{0.01};
    double rt_fr_uplift_hold_{1.0};
    double rt_fr_grasp_speed_{0.02};
    double rt_fr_epsilon_{0.008};
    double rt_fr_slip_drop_thresh_{0.15};
    double rt_fr_slip_width_thresh_{0.0005};
    double rt_fr_load_transfer_min_{1.5};

    // Staging variables (subscriber → RT via state_changed_ release/acquire)
    double staging_fr_f_min_{3.0};
    double staging_fr_f_step_{3.0};
    double staging_fr_f_max_{70.0};
    double staging_fr_uplift_distance_{0.010};
    double staging_fr_lift_speed_{0.01};
    double staging_fr_uplift_hold_{1.0};
    double staging_fr_grasp_speed_{0.02};
    double staging_fr_epsilon_{0.008};
    double staging_fr_slip_drop_thresh_{0.15};
    double staging_fr_slip_width_thresh_{0.0005};
    double staging_fr_load_transfer_min_{1.5};

    // --- Force ramp: runtime state (Realtime-thread owned) ---
    double fr_f_current_{0.0};           // Current grasp force [N]
    int fr_iteration_{0};                // Current iteration (0 = first attempt)
    ros::Time fr_phase_start_time_;      // Phase timing via timestamps
    bool fr_grasping_phase_initialized_{false};  // First tick of GRASPING has set timer
    bool fr_grasp_cmd_seen_executing_{false};  // Has cmd_executing_ gone true yet?
    ros::Time fr_grasp_stabilize_start_;       // Timer for post-grasp stabilization
    bool fr_grasp_stabilizing_{false};         // In post-grasp stabilization phase

    // Slip detection: support force (Fn) accumulators
    double fr_pre_sum_{0.0};              // W_pre: Σ Fn (support force)
    double fr_pre_sum_sq_{0.0};           // W_pre: Σ Fn² (for σ_pre)
    int    fr_pre_count_{0};              // W_pre: sample count
    double fr_early_sum_{0.0};            // W_hold_early: Σ Fn
    int    fr_early_count_{0};            // W_hold_early: sample count
    double fr_late_sum_{0.0};             // W_hold_late: Σ Fn
    int    fr_late_count_{0};             // W_hold_late: sample count
    double fr_grasp_width_snapshot_{0.0}; // Gripper width snapshot taken right before UPLIFT — used for retry grasp
    std::vector<double> fr_width_samples_;  // Width samples during EVALUATE for P5/P95 (pre-allocated in starting())
    double accumulated_uplift_{0.0};       // Uncorrected uplift from SUCCESS [m]

    // Deferred grasp (RT → read thread → command thread, via acquire/release on flag)
    std::atomic<bool> deferred_grasp_pending_{false};
    double deferred_grasp_width_{0.0};
    double deferred_grasp_speed_{0.0};
    double deferred_grasp_force_{0.0};
    double deferred_grasp_epsilon_{0.0};  // Same for inner/outer

    // --- Closing speed safety limit ---
    static constexpr double kMaxClosingSpeed{0.10};    // [m/s] Hard cap on closing speed
    static constexpr double kMaxUpliftDistance{0.3};    // [m] Safety cap on uplift distance
    static constexpr double kMinUpliftHold{0.5};        // [s] Minimum uplift hold (ensures W_pre ≥ 0.25s)
    static constexpr double kMaxUpliftHold{120.0};        // [s] Maximum uplift hold (kMaxWidthSamples / 250Hz)
    static constexpr double kMinLiftSpeed{0.001};        // [m/s] Minimum lift speed (prevents div-by-zero in duration calc)
    static constexpr int kWidthSamplesPerSec{250};        // Width sample rate [Hz] — for reserve sizing
    static constexpr int kMaxWidthSamples{30000};          // Pre-allocated capacity (120s at 250Hz)
    static constexpr int kActionTimeoutSec{30};              // Action server command timeout [s]

    // --- Force ramp internal timing constants ---
    static constexpr double kClosingCmdTimeout{10.0};   // [s] Fail if move command doesn't start executing
    static constexpr double kClosingTimeout{30.0};     // [s] Fail if CLOSING phase exceeds this without contact
    static constexpr double kGraspSettleDelay{0.1};     // [s] Wait for command thread pickup
    static constexpr double kGraspTimeout{10.0};        // [s] Fail if grasp doesn't complete

    /// Compute cosine-smoothed uplift pose for the current elapsed time.
    std::array<double, 16> computeUpliftPose(double elapsed) const;

    /// Compute cosine-smoothed downlift pose for the current elapsed time.
    std::array<double, 16> computeDownliftPose(double elapsed) const;

    /// Publish a state label string exactly once per transition.
    void publishStateLabel(const std::string& label);

    /// Convert enum to string literal (Realtime-safe, no allocation).
    static const char* stateToString(GraspState state);

    // --- update() decomposition helpers (Realtime-safe, no locks, no allocation) ---

    /// Apply a pending state transition from the subscriber thread.
    void applyPendingStateTransition();

    /// Generate and send the Cartesian pose command for this tick (1kHz).
    void updateCartesianCommand(const ros::Duration& period);

    /// Run closing-phase transitions (CLOSING_COMMAND→CLOSING→CONTACT_CONFIRMED→CONTACT or →FAILED).
    void runClosingTransitions(const ros::Time& time,
                               const GripperData& gripper_snapshot,
                               double tau_ext_norm);

    /// Run internal force ramp transitions (GRASPING→UPLIFT→EVALUATE→...).
    /// Called at 250Hz from update(). Drives all internal state transitions.
    void runInternalTransitions(const ros::Time& time,
                                double tau_ext_norm,
                                double support_force,
                                const GripperData& gripper_snapshot);

    /// Reset force ramp runtime state for a fresh grasp cycle. Realtime-safe.
    void resetForceRampState();

    /// Request a deferred grasp command from the Realtime thread.
    /// Realtime-safe: only atomic stores, no mutex or allocation.
    void requestDeferredGrasp(double width, double speed, double force, double epsilon);

    /// Populate KittingState message fields. Called inside trylock() guard.
    void fillKittingStateMsg(const ros::Time& time,
                            const franka::RobotState& robot_state,
                            const std::array<double, 42>& jacobian,
                            const std::array<double, 7>& gravity,
                            const std::array<double, 7>& coriolis,
                            const std::array<double, 6>& ee_velocity,
                            double tau_ext_norm, double wrench_norm,
                            double support_force, double tangential_force,
                            const GripperData& gripper_snapshot);

    /// Request the read thread to call stop(). Realtime-safe (atomic store only).
    void requestGripperStop(const char* source);

    // --- Static helpers ---

    /// State predicate: CLOSING_COMMAND, CLOSING, or CONTACT_CONFIRMED.
    static bool isClosingPhase(GraspState s) {
      return s == GraspState::CLOSING_COMMAND ||
             s == GraspState::CLOSING ||
             s == GraspState::CONTACT_CONFIRMED;
    }

    /// State predicate: GRASPING, UPLIFT, EVALUATE, DOWNLIFT, or SETTLING.
    static bool isForceRampPhase(GraspState s) {
      return s == GraspState::GRASPING || s == GraspState::UPLIFT ||
             s == GraspState::EVALUATE || s == GraspState::SLIP ||
             s == GraspState::DOWNLIFT || s == GraspState::SETTLING;
    }

    /// Resolve a per-command parameter: use msg_value if positive, else default_value.
    /// Convention: ROS message float64 fields default to 0.0, which means "use YAML default".
    /// Any positive value overrides the YAML default. This is safe because all gripper
    /// parameters (widths, speeds, forces, thresholds) are strictly positive in valid usage.
    static inline double resolveParam(double msg_value, double default_value) {
      return (msg_value > 0.0) ? msg_value : default_value;
    }

    /// Compute Euclidean norm of a fixed-size array. Realtime-safe, no allocation.
    template <std::size_t N>
    static inline double arrayNorm(const std::array<double, N>& arr) {
      double sum = 0.0;
      for (std::size_t i = 0; i < N; ++i) {
        sum += arr[i] * arr[i];
      }
      return std::sqrt(sum);
    }

    /// Log a state transition with separator banners.
    void logStateTransition(const char* label, const char* detail = nullptr);

    // --- stateCmdCallback decomposition ---
    void handleBaselineCmd(const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg);
    void handleClosingCmd(const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg);
    void handleGraspingCmd(const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg);

    // --- Auto mode (single-command full grasp sequence) ---
    std::atomic<bool> auto_mode_{false};  // Accessed from subscriber + timer threads
    ros::NodeHandle auto_nh_;
    ros::Timer auto_delay_timer_;
    ros::Timer auto_baseline_poll_timer_;
    ros::Timer auto_contact_poll_timer_;
    double auto_delay_{5.0};
    franka_kitting_controller::KittingGripperCommand auto_cmd_;

    void handleAutoCmd(const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg);
    void cancelAutoMode();
    void autoBaselinePollCallback(const ros::TimerEvent&);
    void autoClosingCallback(const ros::TimerEvent&);
    void autoContactPollCallback(const ros::TimerEvent&);
    void autoGraspingCallback(const ros::TimerEvent&);

    // --- Gripper action servers (franka_gripper API) ---
    using MoveActionServer = actionlib::SimpleActionServer<franka_gripper::MoveAction>;
    using GraspActionServer = actionlib::SimpleActionServer<franka_gripper::GraspAction>;
    using HomingActionServer = actionlib::SimpleActionServer<franka_gripper::HomingAction>;
    using StopActionServer = actionlib::SimpleActionServer<franka_gripper::StopAction>;

    std::unique_ptr<MoveActionServer> move_action_server_;
    std::unique_ptr<GraspActionServer> grasp_action_server_;
    std::unique_ptr<HomingActionServer> homing_action_server_;
    std::unique_ptr<StopActionServer> stop_action_server_;

    void executeMoveAction(const franka_gripper::MoveGoalConstPtr& goal);
    void executeGraspAction(const franka_gripper::GraspGoalConstPtr& goal);
    void executeHomingAction(const franka_gripper::HomingGoalConstPtr& goal);
    void executeStopAction(const franka_gripper::StopGoalConstPtr& goal);

    /// Returns true if external gripper actions are currently allowed.
    /// Only permits actions when the internal state machine is idle.
    bool isActionAllowed() const;

    /// Common boilerplate for gripper action servers: guard, queue, await result.
    template <typename ActionServer, typename Result>
    void executeGripperActionImpl(ActionServer& server,
                                  GripperCommand cmd,
                                  const char* action_name);

    // --- runClosingTransitions decomposition ---
    void detectContact(const ros::Time& time,
                       const GripperData& gripper_snapshot,
                       double tau_ext_norm);

    // --- runInternalTransitions decomposition ---
    void tickGrasping(const ros::Time& time, double tau_ext_norm,
                      double support_force,
                      const GripperData& gripper_snapshot);
    void tickUplift(const ros::Time& time, double tau_ext_norm,
                    double support_force,
                    const GripperData& gripper_snapshot);
    void tickEvaluate(const ros::Time& time, double tau_ext_norm,
                      double support_force,
                      const GripperData& gripper_snapshot);
    void tickSlip(const ros::Time& time, double tau_ext_norm,
                  double support_force,
                  const GripperData& gripper_snapshot);
    void tickDownlift(const ros::Time& time, double tau_ext_norm,
                      double support_force,
                      const GripperData& gripper_snapshot);
    void tickSettling(const ros::Time& time, double tau_ext_norm,
                      double support_force,
                      const GripperData& gripper_snapshot);

    // Test support
    friend class KittingControllerTestFixture;
  };

}  // namespace franka_kitting_controller

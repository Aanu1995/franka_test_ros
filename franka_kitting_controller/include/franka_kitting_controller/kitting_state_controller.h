// Copyright (c) 2024
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <realtime_tools/realtime_buffer.h>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <franka_kitting_controller/KittingGripperCommand.h>
#include <franka_kitting_controller/KittingState.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

namespace franka_kitting_controller {

/// Grasp states for data collection and automated force ramp.
/// Controller starts in START — no baseline collection, no contact detection,
/// just Cartesian passthrough and state data publishing. The grasp state machine
/// begins when the user explicitly publishes BASELINE on /kitting_controller/state_cmd.
///
/// User-commanded: START→BASELINE, →CLOSING, →GRASPING
/// Automatic:      CLOSING→CONTACT (contact detection)
/// Internal (RT):  GRASPING→UPLIFT→EVALUATE→SUCCESS
///                 On slip: EVALUATE→DOWNLIFT→SETTLING→GRASPING (retry)
///                 On F > f_max: SETTLING→FAILED
enum class GraspState {
  START,        // Initial state: controller running, awaiting BASELINE command
  BASELINE,     // Collecting reference signals
  CLOSING,      // Gripper approaching object
  CONTACT,      // Contact detected
  GRASPING,     // Applying grasp force, waiting for completion + stabilization
  UPLIFT,       // Cosine-smoothed micro-lift trajectory
  EVALUATE,     // Hold + slip detection (hold for uplift_hold, then evaluate immediately)
  DOWNLIFT,     // Returning to z_initial before force increment
  SETTLING,     // Post-downlift stabilization
  SUCCESS,      // Stable grasp confirmed
  FAILED        // Max force exceeded
};

/// Command types for the gripper command thread.
enum class GripperCommandType { NONE, MOVE, GRASP };

/// Command queued from subscriber thread to the gripper command thread.
struct GripperCommand {
  GripperCommandType type{GripperCommandType::NONE};
  double width{0.0};
  double speed{0.0};
  double force{0.0};
  double epsilon_inner{0.005};
  double epsilon_outer{0.005};
};

/// Data transferred from gripper read thread (non-RT) to update() (RT) via RealtimeBuffer.
struct GripperData {
  double width{0.0};       // from franka::GripperState.width [m]
  double max_width{0.0};   // from franka::GripperState.max_width [m]
  double width_dot{0.0};   // Finite difference velocity [m/s]
  bool is_grasped{false};  // from franka::GripperState.is_grasped
  ros::Time stamp;         // Timestamp of the measurement
};

/// Lightweight debounce helper for sustained-threshold detection. RT-safe.
struct DebounceState {
  bool active{false};
  ros::Time start_time;

  /// Check if a condition has been sustained for at least hold_duration.
  /// Returns true exactly once when the debounce threshold is reached.
  bool check(bool condition, const ros::Time& now, double hold_duration) {
    if (condition) {
      if (!active) {
        active = true;
        start_time = now;
        return false;
      }
      return (now - start_time).toSec() >= hold_duration;
    }
    active = false;
    return false;
  }

  void reset() { active = false; }
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

  // --- Grasp: Logger readiness gate ---
  // The controller rejects all Grasp commands until the logger node is running.
  // Detection uses a latched topic (/kitting_controller/logger_ready) plus a live
  // publisher check (getNumPublishers > 0) at the point of use. This dual check
  // prevents stale latched messages from a previous logger run and also detects
  // logger shutdown mid-session.
  bool require_logger_{true};
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
  std::atomic<bool> stop_requested_{false};  // RT → read thread: call stop()
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

  // Gripper data buffer (read thread → RT update)
  realtime_tools::RealtimeBuffer<GripperData> gripper_data_buf_;

  // --- Grasp: Gripper default parameters (overridable per-command) ---
  double closing_width_{0.01};
  double closing_speed_{0.02};

  // State machine — cross-thread synchronization
  // current_state_: read by subscriber thread (precondition checks), written by RT thread.
  //   Also written by RT thread for internal transitions (GRASPING→UPLIFT→EVALUATE etc.)
  // pending_state_: written by subscriber thread, read by RT thread.
  // state_changed_: synchronization flag — release on write, acquire on read.
  //   All stores before the release-store of state_changed_ are visible to the RT thread
  //   after its acquire-load of state_changed_ returns true.
  std::atomic<GraspState> current_state_{GraspState::START};
  std::atomic<GraspState> pending_state_{GraspState::START};
  std::atomic<bool> state_changed_{false};
  bool contact_latched_{false};

  // --- Gripper contact detection parameters ---
  double stall_velocity_threshold_{0.008};  // [m/s] Speed below this = stalled
  double width_gap_threshold_{0.002};       // [m] Min gap: (w - w_cmd) > this

  // Debounce state (RT-thread owned)
  DebounceState gripper_debounce_;
  std::atomic<bool> gripper_stop_sent_{false};  // Written by RT, read by command thread
  const char* contact_source_{""};  // "GRIPPER" — records which detector fired (RT-safe)
  std::atomic<double> contact_width_{0.0};  // Gripper width at CONTACT — used as grasp width in GRASPING

  // RT-local copies of CLOSING command parameters — snapshotted when CLOSING begins.
  // Written by subscriber (stateCmdCallback), synchronized via release/acquire on state_changed_.
  double closing_w_cmd_{0.01};     // Resolved target width for active MoveAction
  double closing_v_cmd_{0.02};     // Resolved speed for active MoveAction
  double rt_closing_w_cmd_{0.01};  // RT-local copy
  double rt_closing_v_cmd_{0.02};  // RT-local copy
  double rt_T_hold_gripper_{0.35}; // Computed from rt_closing_v_cmd_ when CLOSING starts

  // Slow-rate logger for contact signal monitoring (2 Hz — readable in terminal)
  franka_hw::TriggerRate signal_log_trigger_{2.0};

  // Faster gripper velocity logger (10 Hz — for debugging gripper speed profile)
  franka_hw::TriggerRate gripper_log_trigger_{10.0};

  // --- UPLIFT trajectory state (RT-thread owned) ---
  std::atomic<bool> uplift_active_{false};
  double uplift_elapsed_{0.0};
  std::array<double, 16> uplift_start_pose_{};
  double uplift_z_start_{0.0};
  // RT-local copies of UPLIFT parameters — set when UPLIFT starts internally.
  double rt_uplift_distance_{0.003};
  double rt_uplift_duration_{0.3};

  // --- DOWNLIFT trajectory state (RT-thread owned) ---
  std::atomic<bool> downlift_active_{false};
  double downlift_elapsed_{0.0};
  std::array<double, 16> downlift_start_pose_{};
  double downlift_z_start_{0.0};
  double rt_downlift_distance_{0.003};
  double rt_downlift_duration_{0.3};

  // --- Force ramp: YAML default parameters (overridable per-command via KittingGripperCommand) ---
  double fr_f_min_{3.0};
  double fr_f_step_{2.0};
  double fr_f_max_{15.0};
  double fr_uplift_distance_{0.003};
  double fr_lift_speed_{0.01};
  double fr_uplift_hold_{0.6};
  double fr_grasp_speed_{0.02};
  double fr_epsilon_{0.008};        // Single epsilon (inner == outer)
  double fr_stabilization_{0.3};
  double fr_slip_tau_drop_{0.20};
  double fr_slip_width_change_{0.001};

  // --- Force ramp: RT-local copies of resolved per-command parameters ---
  // Snapshotted in applyPendingStateTransition() when entering GRASPING.
  // Prevents concurrent subscriber writes from corrupting mid-ramp parameters.
  double rt_fr_f_min_{3.0};
  double rt_fr_f_step_{2.0};
  double rt_fr_f_max_{15.0};
  double rt_fr_uplift_distance_{0.003};
  double rt_fr_lift_speed_{0.01};
  double rt_fr_uplift_hold_{0.6};
  double rt_fr_grasp_speed_{0.02};
  double rt_fr_epsilon_{0.008};
  double rt_fr_stabilization_{0.3};
  double rt_fr_slip_tau_drop_{0.20};
  double rt_fr_slip_width_change_{0.001};

  // --- Force ramp: staging variables (subscriber → RT via state_changed_) ---
  // Written by stateCmdCallback() with resolved per-command values, before release-store
  // on state_changed_. RT thread snapshots these to rt_fr_* in applyPendingStateTransition().
  double staging_fr_f_min_{3.0};
  double staging_fr_f_step_{2.0};
  double staging_fr_f_max_{15.0};
  double staging_fr_uplift_distance_{0.003};
  double staging_fr_lift_speed_{0.01};
  double staging_fr_uplift_hold_{0.6};
  double staging_fr_grasp_speed_{0.02};
  double staging_fr_epsilon_{0.008};
  double staging_fr_stabilization_{0.3};
  double staging_fr_slip_tau_drop_{0.20};
  double staging_fr_slip_width_change_{0.001};

  // --- Force ramp: runtime state (RT-thread owned) ---
  double fr_f_current_{0.0};           // Current grasp force [N]
  int fr_iteration_{0};                // Current iteration (0 = first attempt)
  double fr_z_initial_{0.0};           // EE z-height when force ramp started
  double fr_grasp_width_{0.0};         // Width for GraspAction (always from contact_width_)
  double rt_fr_grasp_width_{0.0};      // RT-local snapshot of fr_grasp_width_
  ros::Time fr_phase_start_time_;      // Phase timing via timestamps
  bool fr_grasping_phase_initialized_{false};  // First tick of GRASPING has set timer
  bool fr_grasp_cmd_seen_executing_{false};  // Has cmd_executing_ gone true yet?
  ros::Time fr_grasp_stabilize_start_;       // Timer for post-grasp stabilization
  bool fr_grasp_stabilizing_{false};         // In post-grasp stabilization phase

  // Load-transfer slip detection (wrench_norm accumulators)
  double fr_pre_sum_{0.0};              // W_pre: Σ wrench_norm
  double fr_pre_sum_sq_{0.0};           // W_pre: Σ wrench_norm² (for σ_pre)
  int    fr_pre_count_{0};              // W_pre: sample count
  double fr_early_sum_{0.0};            // W_hold_early: Σ wrench_norm
  int    fr_early_count_{0};            // W_hold_early: sample count
  double fr_late_sum_{0.0};             // W_hold_late: Σ wrench_norm
  int    fr_late_count_{0};             // W_hold_late: sample count
  double fr_width_before_uplift_{0.0};  // Gripper width before UPLIFT
  double fr_max_width_during_eval_{0.0}; // Maximum gripper width during EVALUATE hold

  // --- Deferred grasp command (RT → read thread → command thread) ---
  // RT thread requests a grasp command via this mechanism (can't call queueGripperCommand
  // from RT because it uses a mutex):
  //   RT thread: write parameters, then release-store deferred_grasp_pending_ = true
  //   Read thread: acquire-load flag, read params, call queueGripperCommand(), clear flag
  std::atomic<bool> deferred_grasp_pending_{false};
  double deferred_grasp_width_{0.0};
  double deferred_grasp_speed_{0.0};
  double deferred_grasp_force_{0.0};
  double deferred_grasp_epsilon_{0.0};  // Same for inner/outer

  // --- Closing speed safety limit + dynamic gripper hold time ---
  static constexpr double kMaxClosingSpeed{0.10};    // [m/s] Hard cap on closing speed
  static constexpr double kGripperHoldBase{0.35};     // [s] Hold time intercept at zero speed
  static constexpr double kGripperHoldSlope{0.5};     // [s/(m/s)] Linear slope
  static constexpr double kMaxUpliftDistance{0.01};   // [m] Safety cap on uplift distance

  // --- Force ramp internal timing constants ---
  static constexpr double kGraspSettleDelay{0.1};     // [s] Wait for command thread pickup
  static constexpr double kGraspTimeout{10.0};        // [s] Fail if grasp doesn't complete

  /// Compute gripper stall debounce time from closing speed.
  /// Formula: T_hold = 0.35 + 0.5 * clamp(speed, 0, 0.10)
  static double computeGripperHoldTime(double closing_speed);

  /// Compute cosine-smoothed uplift pose for the current elapsed time.
  std::array<double, 16> computeUpliftPose(double elapsed) const;

  /// Compute cosine-smoothed downlift pose for the current elapsed time.
  std::array<double, 16> computeDownliftPose(double elapsed) const;

  /// Publish a state label string exactly once per transition.
  void publishStateLabel(const std::string& label);

  /// Convert enum to string literal (RT-safe, no allocation).
  static const char* stateToString(GraspState state);

  // --- update() decomposition helpers (RT-safe, no locks, no allocation) ---

  /// Apply a pending state transition from the subscriber thread.
  void applyPendingStateTransition();

  /// Generate and send the Cartesian pose command for this tick (1kHz).
  void updateCartesianCommand(const ros::Duration& period);

  /// Run Grasp contact detection (gripper stall detection during CLOSING).
  void runContactDetection(const ros::Time& time,
                           const GripperData& gripper_snapshot);

  /// Run internal force ramp transitions (GRASPING→UPLIFT→EVALUATE→...).
  /// Called at 250Hz from update(). Drives all internal state transitions.
  void runInternalTransitions(const ros::Time& time,
                              double tau_ext_norm, double wrench_norm,
                              const GripperData& gripper_snapshot);

  /// Request a deferred grasp command from the RT thread.
  /// RT-safe: only atomic stores, no mutex or allocation.
  void requestDeferredGrasp(double width, double speed, double force, double epsilon);

  /// Populate KittingState message fields. Called inside trylock() guard.
  void fillKittingStateMsg(const ros::Time& time,
                           const franka::RobotState& robot_state,
                           const std::array<double, 42>& jacobian,
                           const std::array<double, 7>& gravity,
                           const std::array<double, 7>& coriolis,
                           const std::array<double, 6>& ee_velocity,
                           double tau_ext_norm, double wrench_norm,
                           const GripperData& gripper_snapshot);

  /// Request the read thread to call stop(). RT-safe (atomic store only).
  void requestGripperStop(const char* source);

  // --- Static helpers ---

  /// Resolve a per-command parameter: use msg_value if positive, else default_value.
  static inline double resolveParam(double msg_value, double default_value) {
    return (msg_value > 0.0) ? msg_value : default_value;
  }

  /// Compute Euclidean norm of a fixed-size array. RT-safe, no allocation.
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

  // --- runContactDetection decomposition ---
  void detectGripperContact(const ros::Time& time, const GripperData& gripper_snapshot);

  // --- runInternalTransitions decomposition ---
  void tickGrasping(const ros::Time& time, double tau_ext_norm,
                    double wrench_norm, const GripperData& gripper_snapshot);
  void tickUplift(const ros::Time& time, double tau_ext_norm,
                  double wrench_norm, const GripperData& gripper_snapshot);
  void tickEvaluate(const ros::Time& time, double tau_ext_norm,
                    double wrench_norm, const GripperData& gripper_snapshot);
  void tickDownlift(const ros::Time& time, double tau_ext_norm,
                    double wrench_norm, const GripperData& gripper_snapshot);
  void tickSettling(const ros::Time& time, double tau_ext_norm,
                    double wrench_norm, const GripperData& gripper_snapshot);
};

}  // namespace franka_kitting_controller

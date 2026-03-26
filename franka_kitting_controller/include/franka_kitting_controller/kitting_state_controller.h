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
  /// Internal (realtime):  GRASPING (force ramp f_min→f_max on table) → UPLIFT → EVALUATE → SUCCESS/FAILED
  enum class GraspState {
    START,           // Initial state: controller running, awaiting BASELINE command
    UNKNOWN,         // Preparing for baseline (downlift, open, settle) — data not used for baseline
    BASELINE,        // Collecting reference signals
    CLOSING_COMMAND, // Gripper close queued — awaiting execution confirmation
    CLOSING,            // Gripper confirmed moving toward object
    CONTACT_CONFIRMED,  // Contact detected (SMS-CUSUM) — gripper stopping
    CONTACT,            // Gripper stopped — contact confirmed
    GRASPING,     // Multi-step force ramp (f_min→f_max), object on table. Published as GRASP_1..GRASP_N
    UPLIFT,       // Cosine-smoothed micro-lift trajectory (single pass after ramp)
    EVALUATE,     // Hold + 3-gate slip evaluation
    SUCCESS,      // Stable grasp confirmed
    FAILED        // Grasp failed (no contact, timeout, slip, or max force)
  };

  /// Sub-phases within GRASPING ramp (RT-thread owned, not published as states).
  enum class RampPhase {
    COMMAND_SENT,        // Grasp command dispatched, waiting for command thread pickup
    WAITING_EXECUTION,   // cmd_executing_ seen true, waiting for completion
    SETTLING,            // Post-grasp settle (grasp_settle_time)
    HOLDING,             // Hold at force level (grasp_force_hold_time)
    STEP_COMPLETE        // Ready to advance to next step or transition to UPLIFT
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

  struct GripperData {
    double width{0.0};
    double max_width{0.0};
    double width_dot{0.0};
    bool is_grasped{false};
    ros::Time stamp;
  };

  /**
  * Controller that publishes comprehensive robot state, implements contact
  * detection, and executes an automated force ramp with micro-lift evaluation.
  *
  * Claims FrankaModelInterface, FrankaStateInterface, and FrankaPoseCartesianInterface.
  * The Cartesian pose interface is used for UPLIFT trajectories and BASELINE prep lowering.
  * In all other states, the controller operates in passthrough mode (holds current position).
  *
  * Grasp topics:
  *   /kitting_controller/state_cmd [subscribed]  KittingGripperCommand (BASELINE, CLOSING, GRASPING)
  *                                           with per-object parameters (0 = use YAML default)
  *   /kitting_controller/state     [published]   State labels: BASELINE, CLOSING, CONTACT,
  *                                           GRASP_1..GRASP_N, UPLIFT, EVALUATE, SUCCESS, FAILED.
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
    franka_hw::FrankaStateInterface* franka_state_interface_{nullptr};
    std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;
    franka_hw::FrankaModelInterface* model_interface_{nullptr};
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_{nullptr};
    std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;

    realtime_tools::RealtimePublisher<franka_kitting_controller::KittingState> kitting_publisher_;
    franka_hw::TriggerRate rate_trigger_{250.0};

    realtime_tools::RealtimePublisher<std_msgs::String> state_publisher_;

    bool require_logger_{false};
    ros::Subscriber logger_ready_sub_;
    std::atomic<bool> logger_ready_{false};
    void loggerReadyCallback(const std_msgs::Bool::ConstPtr& msg);

    ros::Subscriber state_cmd_sub_;
    void stateCmdCallback(const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg);

    std::unique_ptr<franka::Gripper> gripper_;
    std::string robot_ip_;

    std::thread gripper_read_thread_;
    std::atomic<bool> gripper_shutdown_{false};
    std::atomic<bool> stop_requested_{false};
    void gripperReadLoop();

    std::thread gripper_cmd_thread_;
    std::mutex cmd_mutex_;
    std::condition_variable cmd_cv_;
    GripperCommand pending_cmd_;
    bool cmd_ready_{false};
    std::atomic<bool> cmd_executing_{false};
    std::atomic<bool> cmd_success_{false};
    std::atomic<uint32_t> cmd_gen_{0};
    void gripperCommandLoop();
    void queueGripperCommand(const GripperCommand& cmd);

    realtime_tools::RealtimeBuffer<GripperData> gripper_data_buf_;

    std::atomic<bool> baseline_prep_done_{true};
    std::atomic<bool> baseline_needs_open_{false};
    std::atomic<bool> baseline_open_dispatched_{false};
    bool baseline_open_seen_executing_{false};
    std::atomic<double> baseline_open_width_{0.0};

    bool unknown_settle_started_{false};
    ros::Time unknown_settle_start_;

    double closing_width_{0.001};
    double closing_speed_{0.05};

    std::atomic<GraspState> current_state_{GraspState::START};
    std::atomic<GraspState> pending_state_{GraspState::START};
    std::atomic<bool> state_changed_{false};
    bool contact_latched_{false};

    sms_cusum::SMSCusum sms_detector_{sms_cusum::SMSCusumConfig{}};
    std::atomic<bool> gripper_stop_sent_{false};
    std::atomic<bool> gripper_stopped_{false};
    std::atomic<bool> width_capture_pending_{false};
    std::atomic<double> contact_width_{0.0};

    double closing_w_cmd_{0.001};
    double closing_v_cmd_{0.05};
    double rt_closing_w_cmd_{0.001};
    double rt_closing_v_cmd_{0.05};
    bool closing_cmd_seen_executing_{false};
    bool closing_command_entered_{false};

    int    cd_baseline_count_{0};
    double cd_baseline_{0.0};
    std::atomic<bool> cd_baseline_ready_{false};

    franka_hw::TriggerRate signal_log_trigger_{2.0};
    franka_hw::TriggerRate gripper_log_trigger_{10.0};

    std::atomic<bool> uplift_active_{false};
    double uplift_elapsed_{0.0};
    std::array<double, 16> uplift_start_pose_{};
    double uplift_z_start_{0.0};
    double rt_uplift_distance_{0.010};
    double rt_uplift_duration_{1.0};

    std::atomic<bool> downlift_active_{false};
    double downlift_elapsed_{0.0};
    std::array<double, 16> downlift_start_pose_{};
    double downlift_z_start_{0.0};
    double rt_downlift_distance_{0.010};
    double rt_downlift_duration_{1.0};

    // Force ramp YAML defaults (overridable per-command)
    double fr_f_min_{3.0};
    double fr_f_step_{3.0};
    double fr_f_max_{70.0};
    double fr_uplift_distance_{0.010};
    double fr_lift_speed_{0.01};
    double fr_uplift_hold_{1.0};
    double fr_grasp_speed_{0.02};
    double fr_epsilon_{0.008};
    double fr_slip_drop_thresh_{0.15};
    double fr_slip_width_thresh_{0.0005};
    double fr_load_transfer_min_{1.5};
    double fr_grasp_force_hold_time_{1.0};
    double fr_grasp_settle_time_{0.5};

    // RT-local copies (snapshotted at GRASPING entry)
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
    double rt_fr_grasp_force_hold_time_{1.0};
    double rt_fr_grasp_settle_time_{0.5};

    // Staging variables (subscriber → RT via state_changed_)
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
    double staging_fr_grasp_force_hold_time_{1.0};
    double staging_fr_grasp_settle_time_{0.5};
    uint32_t staging_fr_expected_cmd_gen_{0};

    // Force ramp runtime state (RT-thread owned)
    double fr_f_current_{0.0};
    int fr_iteration_{0};
    ros::Time fr_phase_start_time_;
    bool fr_grasping_phase_initialized_{false};
    bool fr_grasp_cmd_seen_executing_{false};
    uint32_t fr_expected_cmd_gen_{0};
    RampPhase fr_ramp_phase_{RampPhase::COMMAND_SENT};
    ros::Time fr_ramp_step_start_time_;

    // Secure grasp: fraction of HOLDING phase used for late-segment statistics
    double sg_late_fraction_{0.5};

    // HOLDING sample tracking for secure grasp late-segment feeding
    int fr_holding_sample_count_{0};
    int fr_holding_late_start_{0};

    // Slip evaluation accumulators
    double fr_pre_sum_{0.0};
    double fr_pre_sum_sq_{0.0};
    int    fr_pre_count_{0};
    double fr_early_sum_{0.0};
    int    fr_early_count_{0};
    double fr_late_sum_{0.0};
    int    fr_late_count_{0};
    std::vector<double> fr_width_samples_;
    double accumulated_uplift_{0.0};

    // Deferred grasp (RT → read thread → command thread)
    std::atomic<bool> deferred_grasp_pending_{false};
    double deferred_grasp_width_{0.0};
    double deferred_grasp_speed_{0.0};
    double deferred_grasp_force_{0.0};
    double deferred_grasp_epsilon_{0.0};

    // Safety limits and timing constants
    static constexpr double kMaxClosingSpeed{0.10};
    static constexpr double kMaxUpliftDistance{0.3};
    static constexpr double kMinUpliftHold{0.5};
    static constexpr double kMaxUpliftHold{120.0};
    static constexpr double kMinLiftSpeed{0.001};
    static constexpr int kWidthSamplesPerSec{250};
    static constexpr int kMaxWidthSamples{30000};
    static constexpr int kActionTimeoutSec{30};
    static constexpr double kBaselineSettleTime{2.0};
    static constexpr double kClosingCmdTimeout{10.0};
    static constexpr double kClosingTimeout{30.0};
    static constexpr double kGraspSettleDelay{0.1};
    static constexpr double kGraspTimeout{10.0};

    std::array<double, 16> computeUpliftPose(double elapsed) const;
    std::array<double, 16> computeDownliftPose(double elapsed) const;
    void publishStateLabel(const std::string& label);
    static const char* stateToString(GraspState state);

    void applyPendingStateTransition();
    void updateCartesianCommand(const ros::Duration& period);
    void runClosingTransitions(const ros::Time& time,
                               const GripperData& gripper_snapshot,
                               double tau_ext_norm);
    void runInternalTransitions(const ros::Time& time,
                                double tau_ext_norm,
                                double support_force,
                                const GripperData& gripper_snapshot);
    void resetForceRampState();
    void requestDeferredGrasp(double width, double speed, double force, double epsilon);
    void fillKittingStateMsg(const ros::Time& time,
                            const franka::RobotState& robot_state,
                            const std::array<double, 42>& jacobian,
                            const std::array<double, 7>& gravity,
                            const std::array<double, 7>& coriolis,
                            const std::array<double, 6>& ee_velocity,
                            double tau_ext_norm, double wrench_norm,
                            double support_force, double tangential_force,
                            const GripperData& gripper_snapshot);

    void requestGripperStop(const char* source);

    static bool isClosingPhase(GraspState s) {
      return s == GraspState::CLOSING_COMMAND ||
             s == GraspState::CLOSING ||
             s == GraspState::CONTACT_CONFIRMED;
    }

    static bool isForceRampPhase(GraspState s) {
      return s == GraspState::GRASPING || s == GraspState::UPLIFT ||
             s == GraspState::EVALUATE;
    }

    static inline double resolveParam(double msg_value, double default_value) {
      return (msg_value > 0.0) ? msg_value : default_value;
    }

    template <std::size_t N>
    static inline double arrayNorm(const std::array<double, N>& arr) {
      double sum = 0.0;
      for (std::size_t i = 0; i < N; ++i) {
        sum += arr[i] * arr[i];
      }
      return std::sqrt(sum);
    }

    void logStateTransition(const char* label, const char* detail = nullptr);
    void handleBaselineCmd(const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg);
    void handleClosingCmd(const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg);
    void handleGraspingCmd(const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg);

    std::atomic<bool> auto_mode_{false};
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

    bool isActionAllowed() const;

    template <typename ActionServer, typename Result>
    void executeGripperActionImpl(ActionServer& server,
                                  GripperCommand cmd,
                                  const char* action_name);

    void detectContact(const ros::Time& time,
                       const GripperData& gripper_snapshot,
                       double tau_ext_norm);

    void tickGrasping(const ros::Time& time, double tau_ext_norm,
                      double support_force,
                      const GripperData& gripper_snapshot);
    void tickUplift(const ros::Time& time, double tau_ext_norm,
                    double support_force,
                    const GripperData& gripper_snapshot);
    void tickEvaluate(const ros::Time& time, double tau_ext_norm,
                      double support_force,
                      const GripperData& gripper_snapshot);

    // Test support
    friend class KittingControllerTestFixture;
  };

}  // namespace franka_kitting_controller

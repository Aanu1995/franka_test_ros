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

/// Grasp states for Phase 2 data collection.
/// Controller starts in START — no baseline collection, no contact detection,
/// just Cartesian passthrough and state data publishing. The grasp state machine
/// begins when the user explicitly publishes BASELINE on /kitting_phase2/state.
enum class GraspPhase {
  START,          // Initial state: controller running, awaiting BASELINE command
  BASELINE,
  CLOSING,
  CONTACT,
  SECURE_GRASP,
  UPLIFT
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

/**
 * Controller that publishes comprehensive robot state, implements Phase 2
 * contact detection with a 6-state machine, and executes Cartesian micro-lift
 * (UPLIFT) internally.
 *
 * Claims FrankaModelInterface, FrankaStateInterface, and FrankaPoseCartesianInterface.
 * The Cartesian pose interface is used for UPLIFT micro-lift execution. In all other
 * states, the controller operates in passthrough mode (holds current position).
 *
 * Phase 2 topics:
 *   /kitting_phase2/state_cmd [subscribed]  KittingGripperCommand (CLOSING, SECURE_GRASP, UPLIFT)
 *                                           with per-object parameters (0 = use YAML default)
 *   /kitting_phase2/state     [subscribed]  State labels from user (BASELINE)
 *                              [published]  CONTACT (auto), CLOSING, SECURE_GRASP, UPLIFT (from state_cmd)
 *
 * Recording is controlled separately via /kitting_phase2/record_control
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

  // --- Phase 1: State data publisher ---
  realtime_tools::RealtimePublisher<franka_kitting_controller::KittingState> kitting_publisher_;
  franka_hw::TriggerRate rate_trigger_{250.0};

  // --- Phase 2: State label publisher ---
  realtime_tools::RealtimePublisher<std_msgs::String> state_publisher_;

  // --- Phase 2: Logger readiness gate ---
  // The controller rejects all Phase 2 commands until the logger node is running.
  // Detection uses a latched topic (/kitting_phase2/logger_ready) plus a live
  // publisher check (getNumPublishers > 0) at the point of use. This dual check
  // prevents stale latched messages from a previous logger run and also detects
  // logger shutdown mid-session.
  ros::Subscriber logger_ready_sub_;
  std::atomic<bool> logger_ready_{false};
  void loggerReadyCallback(const std_msgs::Bool::ConstPtr& msg);

  // --- Phase 2: State subscriber (listens to /kitting_phase2/state) ---
  ros::Subscriber state_sub_;
  void stateCallback(const std_msgs::String::ConstPtr& msg);

  // --- Phase 2: Command subscriber (listens to /kitting_phase2/state_cmd) ---
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

  // --- Phase 2: Gripper default parameters (overridable per-command) ---
  bool execute_gripper_actions_{true};
  double closing_width_{0.04};
  double closing_speed_{0.04};
  double grasp_width_{0.02};
  double epsilon_inner_{0.005};
  double epsilon_outer_{0.005};
  double grasp_speed_{0.04};
  double grasp_force_{10.0};

  // State machine — cross-thread synchronization
  // current_phase_: read by subscriber thread (precondition checks), written by RT thread.
  // pending_phase_: written by subscriber thread, read by RT thread.
  // phase_changed_: the synchronization flag — release on write, acquire on read.
  //   All stores before the release-store of phase_changed_ are visible to the RT thread
  //   after its acquire-load of phase_changed_ returns true.
  std::atomic<GraspPhase> current_phase_{GraspPhase::START};
  std::atomic<GraspPhase> pending_phase_{GraspPhase::START};
  std::atomic<bool> phase_changed_{false};
  bool contact_latched_{false};

  // --- Phase 2: Arm contact detector parameters ---
  bool enable_contact_detector_{true};
  bool enable_arm_contact_{true};
  bool enable_gripper_contact_{true};
  double T_base_{0.7};
  int N_min_{50};
  double k_sigma_{3.0};
  double T_hold_arm_{0.10};
  bool use_slope_gate_{false};
  double slope_dt_{0.02};
  double slope_min_{5.0};

  // --- Gripper contact detection parameters ---
  double stall_velocity_threshold_{0.005};  // [m/s] Speed below this = stalled
  double width_gap_threshold_{0.002};       // [m] Min gap: (w - w_cmd) > this
  double T_hold_gripper_{0.10};             // [s] Gripper stall debounce time
  bool stop_on_contact_{true};              // Call stop() on contact

  // Baseline statistics (computed during BASELINE phase)
  double baseline_sum_{0.0};
  double baseline_sum_sq_{0.0};
  int baseline_n_{0};
  ros::Time baseline_start_time_;
  bool baseline_collecting_{false};
  bool baseline_armed_{false};  // True once baseline stats are computed

  double baseline_mu_{0.0};
  double baseline_sigma_{0.0};
  double contact_threshold_{0.0};  // theta = mu + k * sigma

  // Arm debounce state
  ros::Time exceed_start_time_;
  bool exceeding_{false};

  // Gripper debounce state (RT-thread owned)
  ros::Time gripper_stall_start_time_;
  bool gripper_stall_active_{false};
  bool gripper_stop_sent_{false};
  std::string contact_source_;  // "ARM" or "GRIPPER" — records which detector fired

  // RT-local copies of CLOSING command parameters — snapshotted when CLOSING begins.
  // Written by subscriber (stateCmdCallback) via release/acquire on phase_changed_.
  double closing_w_cmd_{0.04};     // Resolved target width for active MoveAction
  double closing_v_cmd_{0.04};     // Resolved speed for active MoveAction
  double rt_closing_w_cmd_{0.04};  // RT-local copy
  double rt_closing_v_cmd_{0.04};  // RT-local copy

  // Slope gate state
  double prev_tau_ext_norm_{0.0};
  ros::Time prev_tau_ext_time_;
  bool prev_tau_ext_valid_{false};

  // Slow-rate logger for contact signal monitoring (2 Hz — readable in terminal)
  franka_hw::TriggerRate signal_log_trigger_{2.0};

  // --- UPLIFT trajectory state (RT-thread owned, except uplift_active_) ---
  std::atomic<bool> uplift_active_{false};  // Read by subscriber (duplicate guard), written by RT
  double uplift_elapsed_{0.0};
  std::array<double, 16> uplift_start_pose_{};
  double uplift_z_start_{0.0};

  // RT-local copies of UPLIFT parameters — snapshotted when UPLIFT starts in update().
  // These isolate the RT trajectory from concurrent subscriber writes.
  double rt_uplift_distance_{0.003};
  double rt_uplift_duration_{1.0};

  // --- UPLIFT parameters (loaded from YAML, overridable per-command) ---
  // Written by subscriber thread (stateCmdCallback), read by RT thread only via rt_ copies.
  double uplift_distance_{0.003};
  double uplift_duration_{1.0};
  std::string uplift_reference_frame_{"world"};
  bool require_secure_grasp_{true};
  static constexpr double kMaxUpliftDistance{0.01};

  /// Compute cosine-smoothed uplift pose for the current elapsed time.
  /// Uses rt_uplift_distance_ and rt_uplift_duration_ (RT-local copies).
  std::array<double, 16> computeUpliftPose(double elapsed) const;

  /// Publish a state label string exactly once per transition.
  void publishStateLabel(const std::string& label);

  /// Convert enum to string.
  static std::string phaseToString(GraspPhase phase);
};

}  // namespace franka_kitting_controller

// Copyright (c) 2024
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
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

/// Action client type aliases for gripper control.
using MoveClient = actionlib::SimpleActionClient<franka_gripper::MoveAction>;
using GraspClient = actionlib::SimpleActionClient<franka_gripper::GraspAction>;

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

  // --- Phase 2: Gripper action clients ---
  std::unique_ptr<MoveClient> move_client_;
  std::unique_ptr<GraspClient> grasp_client_;

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

  // =====================================================================
  // Phase 2: Hybrid Contact Detection (Gripper + Arm Fusion)
  //
  // CONTACT = GripperContact OR ArmContact
  //   GripperContact: gripper stalls before reaching commanded width
  //   ArmContact:     tau_ext_norm exceeds statistical threshold + delta_min
  //
  // Each detector has independent debounce. CONTACT is latched once declared.
  // =====================================================================

  // --- Global enables ---
  bool enable_gripper_contact_{true};
  bool enable_arm_contact_{true};

  // --- Baseline parameters (shared by arm detector) ---
  double T_base_{0.7};
  int N_min_{50};

  // --- Baseline statistics (computed during BASELINE phase) ---
  double baseline_sum_{0.0};
  double baseline_sum_sq_{0.0};
  int baseline_n_{0};
  ros::Time baseline_start_time_;
  bool baseline_collecting_{false};
  bool baseline_armed_{false};  // True once baseline stats are computed

  double baseline_mu_{0.0};
  double baseline_sigma_{0.0};
  double contact_threshold_{0.0};  // theta = mu + k * sigma

  // --- Arm-based contact detector ---
  double k_sigma_{3.0};
  double delta_min_{0.3};      // Minimum absolute rise above mu (noise guard)
  double T_arm_hold_{0.10};    // Debounce hold time for arm detector

  // Arm debounce state
  ros::Time arm_exceed_start_time_;
  bool arm_exceeding_{false};

  // --- Gripper-based contact detector ---
  // Subscribes to /franka_gripper/joint_states for measured finger positions.
  // Gripper width w(t) = q_finger1 + q_finger2.
  // GripperContact: |w_dot| < v_stall AND |w - w_cmd| > epsilon_w for T_gripper_hold.
  ros::Subscriber gripper_joint_state_sub_;
  void gripperJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  double v_stall_{0.003};            // Stall velocity threshold [m/s]
  double epsilon_w_{0.002};          // Width tolerance to commanded [m]
  double T_gripper_hold_{0.10};      // Debounce hold time for gripper detector

  // Gripper state (written by subscriber callback, read by RT via atomic)
  std::atomic<double> gripper_width_{0.0};    // Current measured gripper width w(t)
  double prev_gripper_width_{0.0};            // Previous w(t-1) for velocity computation
  bool prev_gripper_width_valid_{false};
  double gripper_cmd_width_{0.04};            // Commanded target width for current closing

  // Gripper debounce state
  ros::Time gripper_exceed_start_time_;
  bool gripper_exceeding_{false};

  // --- Slow-rate logger for contact signal monitoring (2 Hz) ---
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

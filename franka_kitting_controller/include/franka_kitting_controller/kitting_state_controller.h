// Copyright (c) 2024
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/String.h>

#include <franka_kitting_controller/KittingState.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

namespace franka_kitting_controller {

/// The 5 grasp states for Phase 2 data collection.
enum class GraspPhase {
  BASELINE,
  CLOSING,
  CONTACT,
  SECURE_GRASP,
  UPLIFT
};

/**
 * Passive read-only controller that publishes comprehensive robot state
 * and implements Phase 2 contact detection with a 5-state machine.
 *
 * This controller does NOT command torques, modify stiffness, or change impedance.
 *
 * Phase 2 topics:
 *   /kitting_phase2/state  [subscribed]  State labels from user
 *                                        (BASELINE, CLOSING, SECURE_GRASP, UPLIFT)
 *                          [published]   CONTACT (auto-detected by controller)
 *
 * The user publishes state labels on /kitting_phase2/state to transition the
 * controller's phase. Recording is controlled separately via
 * /kitting_phase2/record_control (handled by the logger node, not the controller).
 */
class KittingStateController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  // --- Hardware handles ---
  franka_hw::FrankaStateInterface* franka_state_interface_{nullptr};
  std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;
  franka_hw::FrankaModelInterface* model_interface_{nullptr};
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  // --- Phase 1: State data publisher ---
  realtime_tools::RealtimePublisher<franka_kitting_controller::KittingState> kitting_publisher_;
  franka_hw::TriggerRate rate_trigger_{250.0};

  // --- Phase 2: State label publisher ---
  realtime_tools::RealtimePublisher<std_msgs::String> state_publisher_;

  // --- Phase 2: State subscriber (listens to /kitting_phase2/state) ---
  ros::Subscriber state_sub_;
  void stateCallback(const std_msgs::String::ConstPtr& msg);

  // State machine
  GraspPhase current_phase_{GraspPhase::BASELINE};
  GraspPhase pending_phase_{GraspPhase::BASELINE};
  bool phase_changed_{false};  // Set by stateCallback when logger sends START
  bool contact_latched_{false};

  // --- Phase 2: Contact detector parameters ---
  bool enable_contact_detector_{true};
  double T_base_{0.7};
  int N_min_{50};
  double k_sigma_{5.0};
  double T_hold_{0.12};
  bool use_slope_gate_{false};
  double slope_dt_{0.02};
  double slope_min_{5.0};

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

  // Debounce state
  ros::Time exceed_start_time_;
  bool exceeding_{false};

  // Slope gate state
  double prev_tau_ext_norm_{0.0};
  ros::Time prev_tau_ext_time_;
  bool prev_tau_ext_valid_{false};

  /// Publish a state label string exactly once per transition.
  void publishStateLabel(const std::string& label);

  /// Convert enum to string.
  static std::string phaseToString(GraspPhase phase);
};

}  // namespace franka_kitting_controller

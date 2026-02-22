// Copyright (c) 2024
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <topic_tools/shape_shifter.h>

#include <franka_kitting_controller/KittingState.h>

namespace franka_kitting_controller {

/**
 * C++ rosbag recording manager for Phase 2 data collection.
 *
 * Recording starts automatically when the node launches — no START command
 * is needed. The logger opens a new trial bag immediately on startup and
 * records all configured topics until STOP or ABORT is published, or the
 * node is shut down (which triggers an automatic STOP).
 *
 *   /kitting_phase2/record_control  ->  STOP, ABORT
 *   /kitting_phase2/state           ->  BASELINE, CLOSING, CONTACT, SECURE_GRASP, UPLIFT
 *
 * States are labels for offline analysis segmentation only.
 * They do NOT control recording (except optional auto-stop on CONTACT).
 *
 * One rosbag per trial. All state transitions and signals are recorded
 * into the same bag file.
 *
 * Uses rosbag::Bag C++ API directly and topic_tools::ShapeShifter for
 * generic topic subscription.
 */
class KittingPhase2Logger {
 public:
  KittingPhase2Logger();
  void run();

 private:
  // --- Parameters ---
  std::string base_directory_;
  std::string object_name_;
  bool auto_stop_on_contact_;
  bool export_csv_on_stop_;
  std::vector<std::string> topics_to_record_;

  // Detector params (read from parameter server for metadata)
  double T_base_, k_sigma_, slope_dt_, slope_min_;
  double T_hold_arm_, T_hold_gripper_;
  double stall_velocity_threshold_, width_gap_threshold_;
  int N_min_;
  bool use_slope_gate_, enable_arm_contact_, enable_gripper_contact_;

  // CSV export thread (joinable, not detached)
  std::thread csv_export_thread_;

  // --- Trial state ---
  std::mutex trial_mutex_;  // Protects all trial state below
  std::unique_ptr<rosbag::Bag> bag_;
  std::string trial_dir_;
  std::string bag_path_;
  std::string start_time_str_;
  std::string stop_time_str_;
  int trial_number_{0};
  int total_samples_{0};
  bool is_recording_{false};

  // --- ROS ---
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher logger_ready_pub_;     // Latched /kitting_phase2/logger_ready
  ros::Subscriber record_control_sub_;  // STOP, ABORT
  ros::Subscriber state_sub_;           // State labels (for auto-stop on CONTACT)
  std::vector<ros::Subscriber> topic_subs_;

  // --- Callbacks ---
  void recordControlCallback(const std_msgs::String::ConstPtr& msg);
  void stateCallback(const std_msgs::String::ConstPtr& msg);
  void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg,
                     const std::string& topic);

  // --- Trial management (caller must hold trial_mutex_) ---
  void startTrial();
  void stopTrial();
  void abortTrial();
  int nextTrialNumber(const std::string& obj_dir) const;
  void writeMetadata();
  void onShutdown();

  // --- CSV export (runs in background thread, does not require trial_mutex_) ---
  void exportCsv(const std::string& bag_path, const std::string& csv_path);
};

}  // namespace franka_kitting_controller

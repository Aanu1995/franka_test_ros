// Copyright (c) 2024
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_kitting_controller/kitting_logger.h>

#include <algorithm>
#include <ctime>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace franka_kitting_controller {

/// Recursively create directories (like mkdir -p).
static bool mkdirp(const std::string& path) {
  size_t pos = 0;
  while (pos != std::string::npos) {
    pos = path.find('/', pos + 1);
    std::string sub = path.substr(0, pos);
    if (!sub.empty()) {
      struct stat st;
      if (stat(sub.c_str(), &st) != 0) {
        if (mkdir(sub.c_str(), 0755) != 0 && errno != EEXIST) {
          return false;
        }
      }
    }
  }
  return true;
}

/// Recursively remove a directory (like rm -rf).
static void rmrf(const std::string& path) {
  DIR* dir = opendir(path.c_str());
  if (!dir) return;

  struct dirent* entry;
  while ((entry = readdir(dir)) != nullptr) {
    std::string name(entry->d_name);
    if (name == "." || name == "..") continue;
    std::string full = path + "/" + name;
    struct stat st;
    if (stat(full.c_str(), &st) == 0) {
      if (S_ISDIR(st.st_mode)) {
        rmrf(full);
      } else {
        unlink(full.c_str());
      }
    }
  }
  closedir(dir);
  rmdir(path.c_str());
}

// ============================================================================
// Construction
// ============================================================================

KittingLogger::KittingLogger()
    : private_nh_("~") {
  // --- Load parameters ---
  private_nh_.param<std::string>("base_directory", base_directory_, "~/kitting_bags");
  if (!base_directory_.empty() && base_directory_[0] == '~') {
    const char* home = getenv("HOME");
    if (home) {
      base_directory_ = std::string(home) + base_directory_.substr(1);
    }
  }

  private_nh_.param<std::string>("object_name", object_name_, "default_object");
  private_nh_.param("export_csv_on_stop", export_csv_on_stop_, true);

  if (!private_nh_.getParam("topics_to_record", topics_to_record_)) {
    topics_to_record_ = {
        "/kitting_state_controller/kitting_state_data",
        "/kitting_controller/state"};
  }

  // Detector params for metadata (read from controller namespace)
  std::string ctrl_ns = "/kitting_state_controller/";
  nh_.param(ctrl_ns + "T_base", T_base_, 0.7);
  nh_.param(ctrl_ns + "N_min", N_min_, 50);
  nh_.param(ctrl_ns + "k_sigma", k_sigma_, 3.0);
  nh_.param(ctrl_ns + "T_hold_arm", T_hold_arm_, 0.10);
  nh_.param(ctrl_ns + "use_slope_gate", use_slope_gate_, false);
  nh_.param(ctrl_ns + "slope_min", slope_min_, 5.0);
  nh_.param(ctrl_ns + "stall_velocity_threshold", stall_velocity_threshold_, 0.0075);
  nh_.param(ctrl_ns + "width_gap_threshold", width_gap_threshold_, 0.002);
  nh_.param(ctrl_ns + "enable_arm_contact", enable_arm_contact_, true);
  nh_.param(ctrl_ns + "enable_gripper_contact", enable_gripper_contact_, true);

  // --- Subscribe to record control (STOP, ABORT) ---
  record_control_sub_ = nh_.subscribe(
      "/kitting_controller/record_control", 10,
      &KittingLogger::recordControlCallback, this);

  // --- Subscribe to all data topics using ShapeShifter (always subscribed) ---
  for (const auto& topic : topics_to_record_) {
    auto sub = nh_.subscribe<topic_tools::ShapeShifter>(
        topic, 100,
        boost::bind(&KittingLogger::topicCallback, this, _1, topic));
    topic_subs_.push_back(sub);
  }

  // --- Publish latched logger_ready signal ---
  // The controller gates Grasp commands behind this signal.
  // Latched = new subscribers immediately receive the last published message.
  logger_ready_pub_ = nh_.advertise<std_msgs::Bool>(
      "/kitting_controller/logger_ready", 1, true /* latched */);
  std_msgs::Bool ready_msg;
  ready_msg.data = true;
  logger_ready_pub_.publish(ready_msg);

  // --- Auto-start recording on launch ---
  // Recording begins immediately when the logger node starts.
  // No need to publish START on /kitting_controller/record_control.
  {
    std::lock_guard<std::mutex> lock(trial_mutex_);
    startTrial();
  }

  ROS_INFO("KittingLogger: Ready | base_dir=%s object=%s csv_export=%s",
           base_directory_.c_str(), object_name_.c_str(),
           export_csv_on_stop_ ? "true" : "false");
}

// ============================================================================
// Callbacks
// ============================================================================

void KittingLogger::recordControlCallback(const std_msgs::String::ConstPtr& msg) {
  std::string cmd = msg->data;
  cmd.erase(0, cmd.find_first_not_of(" \t\n\r"));
  cmd.erase(cmd.find_last_not_of(" \t\n\r") + 1);

  std::lock_guard<std::mutex> lock(trial_mutex_);

  if (cmd == "STOP") {
    if (!is_recording_) {
      ROS_DEBUG("KittingLogger: Not recording, ignoring STOP");
      return;
    }
    stopTrial();

  } else if (cmd == "ABORT") {
    if (!is_recording_) {
      ROS_DEBUG("KittingLogger: Not recording, ignoring ABORT");
      return;
    }
    abortTrial();

  } else {
    ROS_WARN("KittingLogger: Unknown command '%s' on record_control "
             "(expected STOP or ABORT)", cmd.c_str());
  }
}

void KittingLogger::topicCallback(
    const topic_tools::ShapeShifter::ConstPtr& msg,
    const std::string& topic) {
  ros::Time now = ros::Time::now();

  std::lock_guard<std::mutex> lock(trial_mutex_);
  if (is_recording_ && bag_) {
    try {
      bag_->write(topic, now, *msg);
      if (topic == "/kitting_state_controller/kitting_state_data") {
        total_samples_++;
      }
    } catch (const rosbag::BagIOException& e) {
      ROS_ERROR_THROTTLE(1.0, "KittingLogger: Failed to write to bag: %s", e.what());
    }

    // Auto-stop on terminal state: the message is already in the bag,
    // so stopTrial() can safely close it.
    if (topic == "/kitting_controller/state") {
      auto str_msg = msg->instantiate<std_msgs::String>();
      if (str_msg &&
          (str_msg->data == "SUCCESS" || str_msg->data == "FAILED")) {
        ROS_INFO("KittingLogger: Terminal state '%s' — auto-stopping recording",
                 str_msg->data.c_str());
        stopTrial();
      }
    }
  }
}

// ============================================================================
// Trial management
// ============================================================================

int KittingLogger::nextTrialNumber(const std::string& obj_dir) const {
  DIR* dir = opendir(obj_dir.c_str());
  if (!dir) return 1;

  int max_n = 0;
  struct dirent* entry;
  while ((entry = readdir(dir)) != nullptr) {
    std::string name(entry->d_name);
    if (name.find("trial_") == 0 && name.size() >= 10) {
      try {
        int n = std::stoi(name.substr(6));
        max_n = std::max(max_n, n);
      } catch (...) {}
    }
  }
  closedir(dir);
  return max_n + 1;
}

void KittingLogger::startTrial() {
  // Caller must hold trial_mutex_

  // Directory: <base>/<object>/trial_NNN/
  std::string obj_dir = base_directory_ + "/" + object_name_;
  trial_number_ = nextTrialNumber(obj_dir);

  std::ostringstream trial_ss;
  trial_ss << obj_dir << "/trial_"
           << std::setw(3) << std::setfill('0') << trial_number_;
  trial_dir_ = trial_ss.str();

  if (!mkdirp(trial_dir_)) {
    ROS_ERROR("KittingLogger: Failed to create directory: %s", trial_dir_.c_str());
    return;
  }

  // Timestamp (localtime_r is thread-safe, unlike std::localtime)
  auto now = std::time(nullptr);
  struct tm tm;
  localtime_r(&now, &tm);
  std::ostringstream ts;
  ts << std::put_time(&tm, "%Y%m%d_%H%M%S");
  start_time_str_ = ts.str();

  total_samples_ = 0;

  // Bag filename
  std::ostringstream bag_ss;
  bag_ss << trial_dir_ << "/" << start_time_str_
         << "_" << object_name_
         << "_trial_" << std::setw(3) << std::setfill('0') << trial_number_
         << ".bag";
  bag_path_ = bag_ss.str();

  // Open bag
  try {
    bag_ = std::make_unique<rosbag::Bag>();
    bag_->open(bag_path_, rosbag::bagmode::Write);
    is_recording_ = true;
    ROS_INFO("KittingLogger: Recording started (trial %d) -> %s",
             trial_number_, bag_path_.c_str());
  } catch (const rosbag::BagException& e) {
    ROS_ERROR("KittingLogger: Failed to open bag: %s", e.what());
    bag_.reset();
    rmrf(trial_dir_);
  }
}

void KittingLogger::stopTrial() {
  // Caller must hold trial_mutex_
  if (!is_recording_) return;

  // Close bag
  if (bag_) {
    try {
      bag_->close();
    } catch (const rosbag::BagException& e) {
      ROS_WARN("KittingLogger: Error closing bag: %s", e.what());
    }
    bag_.reset();
  }

  is_recording_ = false;

  // Record stop timestamp (localtime_r is thread-safe)
  auto now = std::time(nullptr);
  struct tm tm;
  localtime_r(&now, &tm);
  std::ostringstream ts;
  ts << std::put_time(&tm, "%Y%m%d_%H%M%S");
  stop_time_str_ = ts.str();

  // Build CSV path: trial_dir/trial_NNN_signals.csv
  std::ostringstream csv_ss;
  csv_ss << trial_dir_ << "/trial_"
         << std::setw(3) << std::setfill('0') << trial_number_
         << "_signals.csv";
  std::string csv_path = csv_ss.str();

  writeMetadata();

  ROS_INFO("KittingLogger: Recording stopped (trial %d) -> %s",
           trial_number_, trial_dir_.c_str());

  // Launch CSV export in background thread (non-blocking, joined on shutdown)
  if (export_csv_on_stop_) {
    if (csv_export_thread_.joinable()) {
      csv_export_thread_.join();
    }
    std::string bag_path_copy = bag_path_;
    csv_export_thread_ = std::thread(&KittingLogger::exportCsv, this,
                                      bag_path_copy, csv_path);
  }
}

void KittingLogger::abortTrial() {
  // Caller must hold trial_mutex_
  if (!is_recording_) return;

  // Close bag
  if (bag_) {
    try {
      bag_->close();
    } catch (const rosbag::BagException& e) {
      ROS_WARN("KittingLogger: Error closing bag: %s", e.what());
    }
    bag_.reset();
  }

  is_recording_ = false;
  ROS_INFO("KittingLogger: Recording ABORTED (trial %d) - deleting %s",
           trial_number_, trial_dir_.c_str());
  rmrf(trial_dir_);
}

void KittingLogger::writeMetadata() {
  // Caller must hold trial_mutex_
  std::string meta_path = trial_dir_ + "/metadata.yaml";
  std::ofstream f(meta_path);
  if (!f.is_open()) {
    ROS_ERROR("KittingLogger: Failed to write metadata: %s", meta_path.c_str());
    return;
  }

  // Extract just the filenames (not full paths)
  std::string bag_filename = bag_path_.substr(bag_path_.find_last_of('/') + 1);
  std::ostringstream csv_fn_ss;
  csv_fn_ss << "trial_" << std::setw(3) << std::setfill('0') << trial_number_
            << "_signals.csv";
  std::string csv_filename = csv_fn_ss.str();

  f << "object_name: " << object_name_ << "\n";
  f << "trial_number: " << trial_number_ << "\n";
  f << "bag_filename: \"" << bag_filename << "\"\n";
  f << "csv_filename: \"" << csv_filename << "\"\n";
  f << "start_time: \"" << start_time_str_ << "\"\n";
  f << "stop_time: \"" << stop_time_str_ << "\"\n";
  f << "total_samples: " << total_samples_ << "\n";

  f << "topics_recorded:\n";
  for (const auto& t : topics_to_record_) {
    f << "  - " << t << "\n";
  }

  f << "export_csv_on_stop: " << (export_csv_on_stop_ ? "true" : "false") << "\n";

  f << "detector_parameters:\n";
  f << "  T_base: " << T_base_ << "\n";
  f << "  N_min: " << N_min_ << "\n";
  f << "  k_sigma: " << k_sigma_ << "\n";
  f << "  T_hold_arm: " << T_hold_arm_ << "\n";
  f << "  T_hold_gripper: \"computed: 0.35 + 0.5 * closing_speed\"\n";
  f << "  use_slope_gate: " << (use_slope_gate_ ? "true" : "false") << "\n";
  f << "  slope_min: " << slope_min_ << "\n";
  f << "  enable_arm_contact: " << (enable_arm_contact_ ? "true" : "false") << "\n";
  f << "  enable_gripper_contact: " << (enable_gripper_contact_ ? "true" : "false") << "\n";
  f << "  stall_velocity_threshold: " << stall_velocity_threshold_ << "\n";
  f << "  width_gap_threshold: " << width_gap_threshold_ << "\n";

  double mu = 0, sigma = 0, theta = 0;
  bool has_mu = nh_.getParam("/kitting_state_controller/baseline_mu", mu);
  bool has_sigma = nh_.getParam("/kitting_state_controller/baseline_sigma", sigma);
  bool has_theta = nh_.getParam("/kitting_state_controller/contact_threshold", theta);

  f << "baseline_statistics:\n";
  f << "  baseline_mu_tau_ext_norm: " << (has_mu ? std::to_string(mu) : "null") << "\n";
  f << "  baseline_sigma_tau_ext_norm: " << (has_sigma ? std::to_string(sigma) : "null") << "\n";
  f << "  contact_threshold_theta: " << (has_theta ? std::to_string(theta) : "null") << "\n";

  f.close();
  ROS_INFO("KittingLogger: Metadata written -> %s", meta_path.c_str());
}

// ============================================================================
// CSV export (runs in background thread)
// ============================================================================

void KittingLogger::exportCsv(const std::string& bag_path,
                                    const std::string& csv_path) {
  ROS_INFO("KittingLogger: CSV export starting -> %s", csv_path.c_str());

  try {
    // Open bag for reading
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    // Create view over kitting_state_data and state topics
    std::vector<std::string> query_topics = {
        "/kitting_state_controller/kitting_state_data",
        "/kitting_controller/state"};
    rosbag::View view(bag, rosbag::TopicQuery(query_topics));

    // Open CSV file
    std::ofstream csv(csv_path);
    if (!csv.is_open()) {
      ROS_ERROR("KittingLogger: Failed to open CSV: %s", csv_path.c_str());
      bag.close();
      return;
    }

    // Write header
    csv << "timestamp_sec,timestamp_nsec,time_float,state_label";
    for (int i = 1; i <= 7; ++i) csv << ",q_" << i;
    for (int i = 1; i <= 7; ++i) csv << ",dq_" << i;
    for (int i = 1; i <= 7; ++i) csv << ",tau_J_" << i;
    for (int i = 1; i <= 7; ++i) csv << ",tau_ext_" << i;
    csv << ",tau_ext_norm";
    csv << ",wrench_fx,wrench_fy,wrench_fz,wrench_tx,wrench_ty,wrench_tz";
    csv << ",wrench_norm";
    csv << ",ee_vx,ee_vy,ee_vz,ee_wx,ee_wy,ee_wz";
    for (int i = 1; i <= 7; ++i) csv << ",gravity_" << i;
    for (int i = 1; i <= 7; ++i) csv << ",coriolis_" << i;
    csv << "\n";

    // Set high precision for floating point
    csv << std::setprecision(12);

    // Iterate messages chronologically and export
    std::string current_state = "UNKNOWN";
    int sample_count = 0;

    for (const rosbag::MessageInstance& m : view) {
      const std::string& topic = m.getTopic();

      // Update state label
      if (topic == "/kitting_controller/state") {
        auto state_msg = m.instantiate<std_msgs::String>();
        if (state_msg) {
          current_state = state_msg->data;
        }
        continue;
      }

      // Write KittingState row
      if (topic == "/kitting_state_controller/kitting_state_data") {
        auto ks = m.instantiate<franka_kitting_controller::KittingState>();
        if (!ks) continue;

        ros::Time t = ks->header.stamp;
        double time_float = t.sec + t.nsec * 1e-9;

        csv << t.sec << "," << t.nsec << "," << time_float
            << "," << current_state;

        // q[7]
        for (size_t i = 0; i < 7; ++i) csv << "," << ks->q[i];
        // dq[7]
        for (size_t i = 0; i < 7; ++i) csv << "," << ks->dq[i];
        // tau_J[7]
        for (size_t i = 0; i < 7; ++i) csv << "," << ks->tau_J[i];
        // tau_ext[7]
        for (size_t i = 0; i < 7; ++i) csv << "," << ks->tau_ext[i];
        // tau_ext_norm
        csv << "," << ks->tau_ext_norm;
        // wrench_ext[6] (fx, fy, fz, tx, ty, tz)
        for (size_t i = 0; i < 6; ++i) csv << "," << ks->wrench_ext[i];
        // wrench_norm
        csv << "," << ks->wrench_norm;
        // ee_velocity[6] (vx, vy, vz, wx, wy, wz)
        for (size_t i = 0; i < 6; ++i) csv << "," << ks->ee_velocity[i];
        // gravity[7]
        for (size_t i = 0; i < 7; ++i) csv << "," << ks->gravity[i];
        // coriolis[7]
        for (size_t i = 0; i < 7; ++i) csv << "," << ks->coriolis[i];

        csv << "\n";
        sample_count++;
      }
    }

    csv.close();
    bag.close();

    ROS_INFO("KittingLogger: CSV export complete | %d samples -> %s",
             sample_count, csv_path.c_str());

  } catch (const rosbag::BagException& e) {
    ROS_ERROR("KittingLogger: CSV export failed: %s", e.what());
  } catch (const std::exception& e) {
    ROS_ERROR("KittingLogger: CSV export failed: %s", e.what());
  }
}

void KittingLogger::onShutdown() {
  {
    std::lock_guard<std::mutex> lock(trial_mutex_);
    if (is_recording_) {
      ROS_INFO("KittingLogger: Shutdown - saving trial %d", trial_number_);
      stopTrial();
    }
  }
  // Join outside mutex — exportCsv doesn't need trial_mutex_
  if (csv_export_thread_.joinable()) {
    csv_export_thread_.join();
  }
}

void KittingLogger::run() {
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  onShutdown();
}

}  // namespace franka_kitting_controller

int main(int argc, char** argv) {
  ros::init(argc, argv, "kitting_logger");
  franka_kitting_controller::KittingLogger logger;
  logger.run();
  return 0;
}

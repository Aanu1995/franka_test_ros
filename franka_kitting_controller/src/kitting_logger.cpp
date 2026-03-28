// Copyright (c) 2026
// Author: Aanu Olakunle
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

    // --- Subscribe to record control (STOP, ABORT) ---
    record_control_sub_ = nh_.subscribe(
        "/kitting_controller/record_control", 10,
        &KittingLogger::recordControlCallback, this);

    // --- Subscribe to all data topics using ShapeShifter (always subscribed) ---
    // Subscriber callbacks enqueue messages to write_queue_ (no disk I/O).
    // The dedicated write_thread_ drains the queue and writes to the bag
    // under trial_mutex_, decoupling callback throughput from disk latency.
    // Start the writer thread before subscribing so no messages are lost.
    write_thread_shutdown_.store(false, std::memory_order_relaxed);
    write_thread_ = std::thread(&KittingLogger::writeLoop, this);

    for (const auto& topic : topics_to_record_) {
      auto sub = nh_.subscribe<topic_tools::ShapeShifter>(
          topic, 100,
          boost::bind(&KittingLogger::topicCallback, this, _1, topic));
      topic_subs_.push_back(sub);
    }

    // --- Auto-start recording on launch ---
    // Recording begins immediately when the logger node starts.
    // The ready signal is published AFTER recording starts, so the controller
    // never accepts commands without an active recording session.
    {
      std::lock_guard<std::mutex> lock(trial_mutex_);
      startTrial();
    }

    if (!is_recording_) {
      ROS_ERROR("KittingLogger: Failed to start recording — logger_ready NOT published. "
                "Controller will reject all commands until logger is relaunched.");
      return;
    }

    // --- Publish latched logger_ready signal ---
    // The controller gates Grasp commands behind this signal.
    // Latched = new subscribers immediately receive the last published message.
    logger_ready_pub_ = nh_.advertise<std_msgs::Bool>(
        "/kitting_controller/logger_ready", 1, true /* latched */);
    std_msgs::Bool ready_msg;
    ready_msg.data = true;
    logger_ready_pub_.publish(ready_msg);

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

    // Reap completed exports (non-blocking) to keep the task list short.
    reapFinishedExports();

    // Drain the write queue before stopping/aborting so no messages are lost.
    // The writer thread may be mid-batch, so we signal it and wait for the
    // queue to empty.  This is safe because we are on a callback thread,
    // not the RT loop.
    if (cmd == "STOP" || cmd == "ABORT") {
      write_queue_cv_.notify_one();
      // Spin briefly until the writer thread drains the queue.
      for (int i = 0; i < 200; ++i) {
        {
          std::lock_guard<std::mutex> qlock(write_queue_mutex_);
          if (write_queue_.empty()) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }

    std::lock_guard<std::mutex> lock(trial_mutex_);

    if (cmd == "STOP") {
      if (!is_recording_) {
        ROS_DEBUG("KittingLogger: Not recording, ignoring STOP");
        return;
      }
      stopTrial();
      trial_generation_.fetch_add(1, std::memory_order_relaxed);

    } else if (cmd == "ABORT") {
      if (!is_recording_) {
        ROS_DEBUG("KittingLogger: Not recording, ignoring ABORT");
        return;
      }
      abortTrial();
      trial_generation_.fetch_add(1, std::memory_order_relaxed);

    } else {
      ROS_WARN("KittingLogger: Unknown command '%s' on record_control "
              "(expected STOP or ABORT)", cmd.c_str());
    }
  }

  void KittingLogger::topicCallback(
      const topic_tools::ShapeShifter::ConstPtr& msg,
      const std::string& topic) {
    // Enqueue the message for the writer thread.  This is the only work
    // done on the subscriber callback — no disk I/O, no trial_mutex_.
    uint64_t gen = trial_generation_.load(std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(write_queue_mutex_);
      if (write_queue_.size() >= kMaxQueueSize) {
        write_queue_.pop_front();  // Drop oldest to bound memory
        ROS_WARN_THROTTLE(5.0, "KittingLogger: Write queue full (%zu), "
                "dropping oldest message — disk I/O may be too slow",
                kMaxQueueSize);
      }
      write_queue_.push_back({msg, topic, ros::Time::now(), gen});
    }
    write_queue_cv_.notify_one();
  }

  void KittingLogger::writeLoop() {
    std::deque<PendingMsg> batch;

    while (!write_thread_shutdown_.load(std::memory_order_relaxed)) {
      // Wait for messages (with timeout so shutdown flag is checked).
      {
        std::unique_lock<std::mutex> lock(write_queue_mutex_);
        write_queue_cv_.wait_for(lock, std::chrono::milliseconds(100),
            [this]{ return !write_queue_.empty() || write_thread_shutdown_.load(std::memory_order_relaxed); });
        batch.swap(write_queue_);
      }

      if (batch.empty()) continue;

      // Reap finished exports (non-blocking, outside trial_mutex_).
      reapFinishedExports();

      // Write the whole batch under trial_mutex_.
      uint64_t cur_gen = trial_generation_.load(std::memory_order_relaxed);
      std::lock_guard<std::mutex> lock(trial_mutex_);
      for (auto& pm : batch) {
        // Skip messages from a previous trial generation.  This prevents
        // stale traffic (e.g., a BASELINE label still in the queue after
        // STOP) from inadvertently starting a new recording.
        if (pm.generation != cur_gen) continue;

        bool is_state_topic = (pm.topic == "/kitting_controller/state");

        if (is_recording_ && bag_) {
          try {
            bag_->write(pm.topic, pm.stamp, *pm.msg);
            if (pm.topic == "/kitting_state_controller/kitting_state_data") {
              total_samples_++;
            }
          } catch (const rosbag::BagIOException& e) {
            ROS_ERROR_THROTTLE(1.0, "KittingLogger: Failed to write to bag: %s", e.what());
          }

          // Auto-stop on terminal state.
          if (is_state_topic) {
            auto str_msg = pm.msg->instantiate<std_msgs::String>();
            if (str_msg &&
                (str_msg->data == "SUCCESS" || str_msg->data == "FAILED")) {
              ROS_INFO("KittingLogger: Terminal state '%s' — auto-stopping recording",
                      str_msg->data.c_str());
              stopTrial();
              // Advance generation so remaining batch messages are skipped.
              trial_generation_.fetch_add(1, std::memory_order_relaxed);
              cur_gen = trial_generation_.load(std::memory_order_relaxed);
            }
          }
        }

        // Auto-start new trial on BASELINE.
        if (!is_recording_ && is_state_topic) {
          auto str_msg = pm.msg->instantiate<std_msgs::String>();
          if (str_msg && str_msg->data == "BASELINE") {
            ROS_INFO("KittingLogger: BASELINE detected — starting new trial");
            startTrial();
            // New trial gets a fresh generation.
            trial_generation_.fetch_add(1, std::memory_order_relaxed);
            cur_gen = trial_generation_.load(std::memory_order_relaxed);
            if (is_recording_ && bag_) {
              try {
                bag_->write(pm.topic, pm.stamp, *pm.msg);
              } catch (const rosbag::BagIOException& e) {
                ROS_ERROR_THROTTLE(1.0, "KittingLogger: Failed to write BASELINE to new bag: %s", e.what());
              }
            }
          }
        }
      }
      batch.clear();
    }

    // Drain remaining messages on shutdown.
    {
      std::lock_guard<std::mutex> qlock(write_queue_mutex_);
      batch.swap(write_queue_);
    }
    if (!batch.empty()) {
      std::lock_guard<std::mutex> lock(trial_mutex_);
      for (auto& pm : batch) {
        if (is_recording_ && bag_) {
          try {
            bag_->write(pm.topic, pm.stamp, *pm.msg);
          } catch (...) {}
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
      if (name.find("trial_") == 0 && name.size() > 6) {
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

    // Launch CSV export in background — never blocks the callback thread.
    if (export_csv_on_stop_) {
      launchExport(bag_path_, csv_path);
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
    f << "  method: torque_drop\n";

    f.close();
    ROS_INFO("KittingLogger: Metadata written -> %s", meta_path.c_str());
  }

  // ============================================================================
  // CSV export (runs in background thread)
  // ============================================================================

  void KittingLogger::launchExport(const std::string& bag_path,
                                   const std::string& csv_path) {
    std::lock_guard<std::mutex> lock(export_mutex_);
    auto task = std::make_unique<ExportTask>();
    auto* done_ptr = &task->done;
    task->thread = std::thread(&KittingLogger::exportCsv, this,
                               bag_path, csv_path, std::ref(*done_ptr));
    export_tasks_.push_back(std::move(task));
  }

  void KittingLogger::reapFinishedExports() {
    // Non-blocking: join and remove completed export threads.
    std::lock_guard<std::mutex> lock(export_mutex_);
    auto it = export_tasks_.begin();
    while (it != export_tasks_.end()) {
      if ((*it)->done.load(std::memory_order_acquire)) {
        (*it)->thread.join();
        it = export_tasks_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void KittingLogger::joinAllExports() {
    // Blocking: join all threads (shutdown only).
    std::lock_guard<std::mutex> lock(export_mutex_);
    for (auto& task : export_tasks_) {
      if (task->thread.joinable()) {
        task->thread.join();
      }
    }
    export_tasks_.clear();
  }

  void KittingLogger::exportCsv(const std::string& bag_path,
                                const std::string& csv_path,
                                std::atomic<bool>& done) {
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
        done.store(true, std::memory_order_release);
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
      csv << ",support_force,tangential_force";
      csv << ",gripper_width,gripper_width_dot,gripper_width_cmd,gripper_max_width,gripper_is_grasped";
      csv << ",grasp_force,grasp_iteration";
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
          // Derived forces
          csv << "," << ks->support_force
              << "," << ks->tangential_force;
          // Gripper signals
          csv << "," << ks->gripper_width
              << "," << ks->gripper_width_dot
              << "," << ks->gripper_width_cmd
              << "," << ks->gripper_max_width
              << "," << (ks->gripper_is_grasped ? 1 : 0);
          // Force ramp state
          csv << "," << ks->grasp_force
              << "," << ks->grasp_iteration;

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
    } catch (...) {
      ROS_ERROR("KittingLogger: CSV export failed: unknown exception");
    }
    done.store(true, std::memory_order_release);
  }

  void KittingLogger::onShutdown() {
    // Stop the writer thread first so all queued messages are flushed.
    write_thread_shutdown_.store(true, std::memory_order_relaxed);
    write_queue_cv_.notify_one();
    if (write_thread_.joinable()) {
      write_thread_.join();
    }

    {
      std::lock_guard<std::mutex> lock(trial_mutex_);
      if (is_recording_) {
        ROS_INFO("KittingLogger: Shutdown - saving trial %d", trial_number_);
        stopTrial();
      }
    }
    // Join all exports (including the one just launched by stopTrial)
    joinAllExports();
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

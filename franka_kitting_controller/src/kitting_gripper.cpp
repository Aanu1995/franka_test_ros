// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//
// Gripper thread management and action server callbacks.
// Part of KittingStateController — see kitting_state_controller.h for class definition.

#include <franka_kitting_controller/kitting_state_controller.h>

#include <chrono>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <franka/exception.h>

namespace franka_kitting_controller {

  // ============================================================================
  // Gripper threads (non-realtime)
  // ============================================================================

  void KittingStateController::gripperReadLoop() {
    double prev_width = 0.0;
    ros::Time prev_stamp;
    bool prev_valid = false;

    while (!gripper_shutdown_.load(std::memory_order_relaxed)) {
      try {
        franka::GripperState gs = gripper_->readOnce();

        double w_dot = 0.0;
        ros::Time now = ros::Time::now();
        if (prev_valid) {
          double dt = (now - prev_stamp).toSec();
          if (dt > 1e-6) {
            w_dot = (gs.width - prev_width) / dt;
          }
        }
        prev_width = gs.width;
        prev_stamp = now;
        prev_valid = true;

        GripperData data;
        data.width = gs.width;
        data.max_width = gs.max_width;
        data.width_dot = w_dot;
        data.is_grasped = gs.is_grasped;
        data.stamp = now;
        gripper_data_buf_.writeFromNonRT(data);

        // Post-stop signal: once the gripper has stopped, signal the RT thread.
        // contact_width_ is NOT captured here — it will be read fresh from gs.width
        // at the moment the grasp command is dispatched (either in handleGraspingCmd
        // via the GripperData buffer, or in the deferred grasp path below).
        // This ensures the grasp target always matches the gripper's actual position.
        if (width_capture_pending_.load(std::memory_order_relaxed)) {
          contact_width_.store(gs.width, std::memory_order_relaxed);
          gripper_stopped_.store(true, std::memory_order_release);
          width_capture_pending_.store(false, std::memory_order_relaxed);
          ROS_INFO("  [GRIPPER]  Post-stop width: %.4f m  w_dot=%.6f m/s", gs.width, w_dot);
        }

        // Handle stop request from RT thread
        if (stop_requested_.load(std::memory_order_relaxed)) {
          try {
            gripper_->stop();
            ROS_INFO("  [GRIPPER]  stop() executed by read thread");
            // Don't set gripper_stopped_ yet — buffer has pre-stop width.
            // Signal on next readOnce() iteration.
            width_capture_pending_.store(true, std::memory_order_relaxed);
            stop_requested_.store(false, std::memory_order_relaxed);
          } catch (const franka::Exception& ex) {
            // Leave stop_requested_ true so we retry on the next readOnce() iteration.
            // Do NOT set gripper_stopped_ — state machine must not proceed to CONTACT.
            ROS_WARN_STREAM("KittingStateController: stop() failed (will retry): " << ex.what());
          }
        }

        // Deferred grasp (RT → read thread handoff via acquire/release)
        // Uses gs.width (current readOnce() measurement) as the grasp target
        // instead of a pre-stored width — guarantees the target matches the
        // gripper's actual position, preventing the gripper from opening.
        if (deferred_grasp_pending_.load(std::memory_order_acquire)) {
          // Release any current grasp before retrying — libfranka grasp() hangs
          // if the gripper is already in a grasped state at the target width.
          try {
            gripper_->stop();
          } catch (const franka::Exception&) {}

          GripperCommand grasp_cmd;
          grasp_cmd.type = GripperCommandType::GRASP;
          grasp_cmd.width = gs.width;
          grasp_cmd.speed = deferred_grasp_speed_;
          grasp_cmd.force = deferred_grasp_force_;
          grasp_cmd.epsilon_inner = deferred_grasp_epsilon_;
          grasp_cmd.epsilon_outer = deferred_grasp_epsilon_;
          queueGripperCommand(grasp_cmd);

          contact_width_.store(gs.width, std::memory_order_relaxed);

          ROS_INFO("  [GRIPPER]  Deferred grasp queued: width=%.4f (current) speed=%.4f "
                  "force=%.1f eps=%.4f",
                  gs.width, deferred_grasp_speed_, deferred_grasp_force_,
                  deferred_grasp_epsilon_);
          deferred_grasp_pending_.store(false, std::memory_order_relaxed);
        }

        // Baseline prep step 2: queue gripper open after downlift completes.
        // baseline_needs_open_ and baseline_open_width_ are synchronized via
        // baseline_prep_done_ (release on subscriber, acquire here).
        if (baseline_needs_open_.load(std::memory_order_relaxed) &&
            !baseline_prep_done_.load(std::memory_order_acquire) &&
            !baseline_open_dispatched_.load(std::memory_order_relaxed) &&
            !downlift_active_.load(std::memory_order_relaxed) &&
            current_state_.load(std::memory_order_relaxed) == GraspState::BASELINE) {
          GripperCommand open_cmd;
          open_cmd.type = GripperCommandType::MOVE;
          open_cmd.width = baseline_open_width_.load(std::memory_order_relaxed);
          open_cmd.speed = 0.1;
          // Set tracking state BEFORE queueing to avoid race where the command
          // completes before we initialize the tracking flag.
          baseline_open_seen_executing_ = false;
          queueGripperCommand(open_cmd);
          baseline_open_dispatched_.store(true, std::memory_order_relaxed);
          ROS_INFO("  [BASELINE]  Step 2: Gripper open queued (post-downlift): "
                   "move(width=%.4f, speed=0.1)", baseline_open_width_.load(std::memory_order_relaxed));
        }

        // Baseline prep complete: gripper open has finished → mark prep done.
        // Guard: wait for cmd_executing_ to go true (command picked up)
        // then false (completed), to avoid false-triggering on dispatch delay.
        if (baseline_open_dispatched_.load(std::memory_order_relaxed) &&
            !baseline_prep_done_.load(std::memory_order_relaxed)) {
          if (!baseline_open_seen_executing_ &&
              cmd_executing_.load(std::memory_order_relaxed)) {
            baseline_open_seen_executing_ = true;
          }
          if (baseline_open_seen_executing_ &&
              !cmd_executing_.load(std::memory_order_relaxed)) {
            baseline_prep_done_.store(true, std::memory_order_release);
            ROS_INFO("  [BASELINE]  Gripper open complete — baseline collection starting");
          }
        }
      } catch (const franka::Exception& ex) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "KittingStateController: readOnce() failed: " << ex.what());
        prev_valid = false;
        ros::Duration(0.01).sleep();  // Back off before retry
      }
    }
  }

  void KittingStateController::gripperCommandLoop() {
    while (!gripper_shutdown_.load(std::memory_order_relaxed)) {
      GripperCommand cmd;
      {
        std::unique_lock<std::mutex> lock(cmd_mutex_);
        cmd_cv_.wait(lock, [this]() {
          return cmd_ready_ || gripper_shutdown_.load(std::memory_order_relaxed);
        });
        if (gripper_shutdown_.load(std::memory_order_relaxed)) {
          // Fulfill any queued promise so action server callbacks don't hang
          if (cmd_ready_ && pending_cmd_.result_promise) {
            pending_cmd_.result_promise->set_value(false);
          }
          return;
        }
        cmd = pending_cmd_;
        cmd_ready_ = false;
      }

      cmd_executing_.store(true, std::memory_order_relaxed);
      bool success = false;
      try {
        if (cmd.type == GripperCommandType::MOVE) {
          success = gripper_->move(cmd.width, cmd.speed);
          ROS_INFO("KittingStateController: move(%.4f, %.4f) -> %s",
                  cmd.width, cmd.speed, success ? "OK" : "FAIL");
        } else if (cmd.type == GripperCommandType::GRASP) {
          success = gripper_->grasp(cmd.width, cmd.speed, cmd.force,
                                    cmd.epsilon_inner, cmd.epsilon_outer);
          ROS_INFO("KittingStateController: grasp(%.4f, %.4f, %.1f, eps=%.4f/%.4f) -> %s",
                  cmd.width, cmd.speed, cmd.force,
                  cmd.epsilon_inner, cmd.epsilon_outer,
                  success ? "OK" : "FAIL");
        } else if (cmd.type == GripperCommandType::HOMING) {
          success = gripper_->homing();
          ROS_INFO("KittingStateController: homing() -> %s", success ? "OK" : "FAIL");
        }
      } catch (const franka::Exception& ex) {
        // stop() aborts the in-flight move()/grasp(), which throws "command aborted".
        // This is expected after contact detection — log as INFO, not ERROR.
        success = false;
        if (gripper_stop_sent_.load(std::memory_order_relaxed)) {
          ROS_INFO("KittingStateController: Gripper command aborted by contact stop (expected)");
        } else {
          ROS_ERROR_STREAM("KittingStateController: Gripper command failed: " << ex.what());
        }
      }
      if (cmd.result_promise) {
        cmd.result_promise->set_value(success);
      }
      cmd_success_.store(success, std::memory_order_relaxed);
      // Release ordering ensures cmd_success_ is visible to the RT thread
      // before it observes cmd_executing_ == false (loaded with acquire in tickGrasping).
      cmd_executing_.store(false, std::memory_order_release);
    }
  }

  void KittingStateController::queueGripperCommand(const GripperCommand& cmd) {
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      // Check if a command is executing; if so, request stop so it unblocks
      // and the new command can proceed. Note: cmd_executing_ is atomic and
      // may transition false between the check and the store — this is benign
      // because a spurious stop() is safely caught by the read thread.
      if (cmd_executing_.load(std::memory_order_relaxed)) {
        stop_requested_.store(true, std::memory_order_relaxed);
      }
      // Fulfill orphaned promise from overwritten pending command
      if (cmd_ready_ && pending_cmd_.result_promise) {
        pending_cmd_.result_promise->set_value(false);
      }
      pending_cmd_ = cmd;
      cmd_ready_ = true;
    }
    cmd_cv_.notify_one();
  }

  // ============================================================================
  // Gripper action servers (franka_gripper-compatible API)
  // ============================================================================

  bool KittingStateController::isActionAllowed() const {
    auto state = current_state_.load(std::memory_order_relaxed);
    return state == GraspState::START ||
           state == GraspState::SUCCESS ||
           state == GraspState::FAILED;
  }

  template <typename ActionServer, typename Result>
  void KittingStateController::executeGripperActionImpl(
      ActionServer& server, GripperCommand cmd, const char* action_name) {
    Result result;
    if (!isActionAllowed()) {
      result.success = false;
      result.error = "Gripper busy — state machine in " +
                     std::string(stateToString(current_state_.load(std::memory_order_relaxed)));
      server.setAborted(result, result.error);
      return;
    }
    auto promise = std::make_shared<std::promise<bool>>();
    cmd.result_promise = promise;
    queueGripperCommand(cmd);
    auto future = promise->get_future();
    if (future.wait_for(std::chrono::seconds(kActionTimeoutSec)) ==
        std::future_status::ready) {
      result.success = future.get();
      if (result.success) {
        server.setSucceeded(result);
      } else {
        result.error = std::string(action_name) + " failed";
        server.setAborted(result, result.error);
      }
    } else {
      result.success = false;
      result.error = std::string(action_name) + " timed out (" +
                     std::to_string(kActionTimeoutSec) + "s)";
      server.setAborted(result, result.error);
      ROS_ERROR("KittingStateController: %s", result.error.c_str());
    }
  }

  void KittingStateController::executeMoveAction(const franka_gripper::MoveGoalConstPtr& goal) {
    GripperCommand cmd;
    cmd.type = GripperCommandType::MOVE;
    cmd.width = goal->width;
    cmd.speed = goal->speed;
    executeGripperActionImpl<MoveActionServer, franka_gripper::MoveResult>(
        *move_action_server_, cmd, "move()");
  }

  void KittingStateController::executeGraspAction(const franka_gripper::GraspGoalConstPtr& goal) {
    GripperCommand cmd;
    cmd.type = GripperCommandType::GRASP;
    cmd.width = goal->width;
    cmd.speed = goal->speed;
    cmd.force = goal->force;
    cmd.epsilon_inner = goal->epsilon.inner;
    cmd.epsilon_outer = goal->epsilon.outer;
    executeGripperActionImpl<GraspActionServer, franka_gripper::GraspResult>(
        *grasp_action_server_, cmd, "grasp()");
  }

  void KittingStateController::executeHomingAction(const franka_gripper::HomingGoalConstPtr& /*goal*/) {
    GripperCommand cmd;
    cmd.type = GripperCommandType::HOMING;
    executeGripperActionImpl<HomingActionServer, franka_gripper::HomingResult>(
        *homing_action_server_, cmd, "homing()");
  }

  void KittingStateController::executeStopAction(const franka_gripper::StopGoalConstPtr& /*goal*/) {
    // Route stop through the read thread's stop_requested_ flag to avoid
    // concurrent gripper_->stop() calls from multiple threads (the read thread
    // may be calling readOnce() or stop() simultaneously). The read thread
    // serializes all gripper I/O.
    franka_gripper::StopResult result;
    stop_requested_.store(true, std::memory_order_relaxed);
    // Wait briefly for the read thread to execute stop()
    for (int i = 0; i < 50; ++i) {  // 50 × 10ms = 500ms max
      if (!stop_requested_.load(std::memory_order_relaxed)) {
        result.success = true;
        stop_action_server_->setSucceeded(result);
        return;
      }
      ros::Duration(0.01).sleep();
    }
    result.success = false;
    result.error = "stop() timed out (read thread did not execute within 500ms)";
    stop_action_server_->setAborted(result, result.error);
  }

}  // namespace franka_kitting_controller

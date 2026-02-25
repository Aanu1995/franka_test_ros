// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//
// Gripper thread management and action server callbacks.
// Part of KittingStateController — see kitting_state_controller.h for class definition.

#include <franka_kitting_controller/kitting_state_controller.h>

#include <memory>
#include <string>

#include <ros/ros.h>

#include <franka/exception.h>

namespace franka_kitting_controller {

  // ============================================================================
  // Gripper threads (non-realtime)
  // ============================================================================

  void KittingStateController::gripperReadLoop() {
    // Runs at firmware rate (blocking readOnce()). No artificial rate limit.
    // Computes width velocity via finite difference and writes to RealtimeBuffer.
    // Also checks stop_requested_ flag after each read and calls stop() if set.
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

        // Check if Realtime thread requested a stop (contact detected)
        if (stop_requested_.load(std::memory_order_relaxed)) {
          try {
            gripper_->stop();
            ROS_INFO("  [GRIPPER]  stop() executed by read thread");
          } catch (const franka::Exception& ex) {
            ROS_WARN_STREAM("KittingStateController: stop() failed: " << ex.what());
          }
          gripper_stopped_.store(true, std::memory_order_relaxed);
          stop_requested_.store(false, std::memory_order_relaxed);
        }

        // Dispatch deferred grasp command from Realtime thread (force ramp iteration).
        // Acquire-load ensures we see the parameter stores made by the Realtime thread
        // before its release-store of the flag.
        if (deferred_grasp_pending_.load(std::memory_order_acquire)) {
          GripperCommand grasp_cmd;
          grasp_cmd.type = GripperCommandType::GRASP;
          grasp_cmd.width = deferred_grasp_width_;
          grasp_cmd.speed = deferred_grasp_speed_;
          grasp_cmd.force = deferred_grasp_force_;
          grasp_cmd.epsilon_inner = deferred_grasp_epsilon_;
          grasp_cmd.epsilon_outer = deferred_grasp_epsilon_;
          queueGripperCommand(grasp_cmd);
          ROS_INFO("  [GRIPPER]  Deferred grasp queued: width=%.4f speed=%.4f force=%.1f eps=%.4f",
                  deferred_grasp_width_, deferred_grasp_speed_,
                  deferred_grasp_force_, deferred_grasp_epsilon_);
          deferred_grasp_pending_.store(false, std::memory_order_relaxed);
        }
      } catch (const franka::Exception& ex) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "KittingStateController: readOnce() failed: " << ex.what());
        prev_valid = false;
        ros::Duration(0.01).sleep();  // Back off before retry
      }
    }
  }

  void KittingStateController::gripperCommandLoop() {
    // Waits on condition variable for commands, then executes blocking move()/grasp().
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
          ROS_INFO("KittingStateController: grasp(%.4f, %.4f, %.1f) -> %s",
                  cmd.width, cmd.speed, cmd.force, success ? "OK" : "FAIL");
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
      cmd_executing_.store(false, std::memory_order_relaxed);
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

  void KittingStateController::executeMoveAction(const franka_gripper::MoveGoalConstPtr& goal) {
    franka_gripper::MoveResult result;
    if (!isActionAllowed()) {
      result.success = false;
      result.error = "Gripper busy — state machine in " +
                     std::string(stateToString(current_state_.load(std::memory_order_relaxed)));
      move_action_server_->setAborted(result, result.error);
      return;
    }
    auto promise = std::make_shared<std::promise<bool>>();
    auto future = promise->get_future();
    GripperCommand cmd;
    cmd.type = GripperCommandType::MOVE;
    cmd.width = goal->width;
    cmd.speed = goal->speed;
    cmd.result_promise = promise;
    queueGripperCommand(cmd);
    result.success = future.get();
    if (result.success) {
      move_action_server_->setSucceeded(result);
    } else {
      result.error = "move() failed";
      move_action_server_->setAborted(result, result.error);
    }
  }

  void KittingStateController::executeGraspAction(const franka_gripper::GraspGoalConstPtr& goal) {
    franka_gripper::GraspResult result;
    if (!isActionAllowed()) {
      result.success = false;
      result.error = "Gripper busy — state machine in " +
                     std::string(stateToString(current_state_.load(std::memory_order_relaxed)));
      grasp_action_server_->setAborted(result, result.error);
      return;
    }
    auto promise = std::make_shared<std::promise<bool>>();
    auto future = promise->get_future();
    GripperCommand cmd;
    cmd.type = GripperCommandType::GRASP;
    cmd.width = goal->width;
    cmd.speed = goal->speed;
    cmd.force = goal->force;
    cmd.epsilon_inner = goal->epsilon.inner;
    cmd.epsilon_outer = goal->epsilon.outer;
    cmd.result_promise = promise;
    queueGripperCommand(cmd);
    result.success = future.get();
    if (result.success) {
      grasp_action_server_->setSucceeded(result);
    } else {
      result.error = "grasp() failed";
      grasp_action_server_->setAborted(result, result.error);
    }
  }

  void KittingStateController::executeHomingAction(const franka_gripper::HomingGoalConstPtr& /*goal*/) {
    franka_gripper::HomingResult result;
    if (!isActionAllowed()) {
      result.success = false;
      result.error = "Gripper busy — state machine in " +
                     std::string(stateToString(current_state_.load(std::memory_order_relaxed)));
      homing_action_server_->setAborted(result, result.error);
      return;
    }
    auto promise = std::make_shared<std::promise<bool>>();
    auto future = promise->get_future();
    GripperCommand cmd;
    cmd.type = GripperCommandType::HOMING;
    cmd.result_promise = promise;
    queueGripperCommand(cmd);
    result.success = future.get();
    if (result.success) {
      homing_action_server_->setSucceeded(result);
    } else {
      result.error = "homing() failed";
      homing_action_server_->setAborted(result, result.error);
    }
  }

  void KittingStateController::executeStopAction(const franka_gripper::StopGoalConstPtr& /*goal*/) {
    franka_gripper::StopResult result;
    try {
      gripper_->stop();
      result.success = true;
      stop_action_server_->setSucceeded(result);
    } catch (const franka::Exception& ex) {
      result.success = false;
      result.error = ex.what();
      stop_action_server_->setAborted(result, result.error);
    }
  }

}  // namespace franka_kitting_controller

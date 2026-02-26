// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//
// Contact detection during CLOSING and force ramp state machine
// (GRASPING → UPLIFT → EVALUATE → SUCCESS, with slip retry loop).
// Part of KittingStateController — see kitting_state_controller.h for class definition.

#include <franka_kitting_controller/kitting_state_controller.h>

#include <algorithm>
#include <cmath>

#include <ros/ros.h>

namespace franka_kitting_controller {

  // ============================================================================
  // Deferred grasp dispatch (realtime-safe → read thread)
  // ============================================================================

  void KittingStateController::requestDeferredGrasp(double width, double speed,
                                                    double force, double epsilon) {
    // Realtime-safe: only atomic stores, no mutex or allocation.
    // Gripper read thread acquire-loads the flag and dispatches to command thread.
    deferred_grasp_width_ = width;
    deferred_grasp_speed_ = speed;
    deferred_grasp_force_ = force;
    deferred_grasp_epsilon_ = epsilon;
    deferred_grasp_pending_.store(true, std::memory_order_release);
  }

  // ============================================================================
  // Contact detection during CLOSING
  // ============================================================================

  void KittingStateController::runContactDetection(const ros::Time& time,
                                                    const GripperData& gripper_snapshot) {
    if (current_state_.load(std::memory_order_relaxed) == GraspState::CLOSING &&
        !contact_latched_) {
      // Track when the gripper move command starts executing.
      // Stall detection is deferred until the move is confirmed running to prevent
      // false contacts during the pre-movement window (gripper stationary, velocity ≈ 0,
      // width gap large — looks like a stall but the move hasn't started yet).
      if (!closing_cmd_seen_executing_) {
        if (cmd_executing_.load(std::memory_order_relaxed)) {
          closing_cmd_seen_executing_ = true;
        }
      }

      if (closing_cmd_seen_executing_) {
        // Stall detection (only after move is confirmed executing)
        detectGripperContact(time, gripper_snapshot);

        // No-contact: move completed without triggering stall → FAILED
        if (!contact_latched_ && !cmd_executing_.load(std::memory_order_relaxed)) {
          current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
          publishStateLabel("FAILED");
          logStateTransition("FAILED",
              "Gripper closed to target width — no contact detected");
          ROS_WARN("  [CLOSING]  No contact: w=%.4f  w_cmd=%.4f  -> FAILED",
                   gripper_snapshot.width, rt_closing_w_cmd_);
        }
      }
    }

    // Deferred CONTACT transition: wait for gripper to physically stop
    if (contact_latched_ &&
        current_state_.load(std::memory_order_relaxed) == GraspState::CLOSING &&
        gripper_stopped_.load(std::memory_order_relaxed)) {
      current_state_.store(GraspState::CONTACT, std::memory_order_relaxed);
      publishStateLabel("CONTACT");
      logStateTransition("CONTACT", "Gripper stopped — contact confirmed");
      ROS_INFO("    contact_width=%.4f m", contact_width_.load(std::memory_order_relaxed));
    }
  }

  void KittingStateController::detectGripperContact(const ros::Time& time,
                                                      const GripperData& gripper_snapshot) {
    bool gripper_data_valid = (gripper_snapshot.stamp != ros::Time(0)) &&
                              ((time - gripper_snapshot.stamp).toSec() < 0.5);
    if (!gripper_data_valid) {
      return;
    }

    double w = gripper_snapshot.width;
    double w_dot = gripper_snapshot.width_dot;

    bool velocity_stalled = (std::abs(w_dot) < stall_velocity_threshold_);
    bool width_gap_exists = ((w - rt_closing_w_cmd_) > width_gap_threshold_);
    bool stall_detected = velocity_stalled && width_gap_exists;

    if (gripper_debounce_.check(stall_detected, time, rt_T_hold_gripper_)) {
      double hold_elapsed = (time - gripper_debounce_.start_time).toSec();
      contact_latched_ = true;
      requestGripperStop("Stall"); // Request gripper closing to stop

      contact_width_.store(gripper_snapshot.width, std::memory_order_relaxed);
      ROS_INFO("  [CLOSING]  Stall detected, waiting for gripper stop: "
              "w=%.4f  w_cmd=%.4f  gap=%.4f  w_dot=%.6f  hold=%.3fs",
              w, rt_closing_w_cmd_, w - rt_closing_w_cmd_, w_dot, hold_elapsed);
    }
  }

  // ============================================================================
  // Force ramp state machine (250Hz, realtime-safe)
  // Called from update() when state is GRASPING/UPLIFT/EVALUATE/DOWNLIFT/SETTLING.
  // ============================================================================

  void KittingStateController::runInternalTransitions(const ros::Time& time,
                                                      double tau_ext_norm,
                                                      double wrench_norm,
                                                      const GripperData& gripper_snapshot) {
    switch (current_state_.load(std::memory_order_relaxed)) {
      case GraspState::GRASPING:  tickGrasping(time, tau_ext_norm, wrench_norm, gripper_snapshot); break;
      case GraspState::UPLIFT:    tickUplift(time, tau_ext_norm, wrench_norm, gripper_snapshot);   break;
      case GraspState::EVALUATE:  tickEvaluate(time, tau_ext_norm, wrench_norm, gripper_snapshot); break;
      case GraspState::DOWNLIFT:  tickDownlift(time, tau_ext_norm, wrench_norm, gripper_snapshot); break;
      case GraspState::SETTLING:  tickSettling(time, tau_ext_norm, wrench_norm, gripper_snapshot); break;
      default: break;
    }
  }

  void KittingStateController::tickGrasping(const ros::Time& time,
                                            double tau_ext_norm,
                                            double wrench_norm,
                                            const GripperData& gripper_snapshot) {
    // First tick after entering GRASPING: initialize phase timer
    if (!fr_grasping_phase_initialized_) {
      fr_phase_start_time_ = time;
      fr_grasping_phase_initialized_ = true;
    }

    double elapsed = (time - fr_phase_start_time_).toSec();

    // Step 1: Settle delay — wait for command thread to pick up the grasp command
    if (elapsed < kGraspSettleDelay) {
      return;
    }

    // Timeout: if grasp command doesn't complete in time, fail
    if (elapsed > kGraspTimeout) {
      current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
      publishStateLabel("FAILED");
      logStateTransition("FAILED", "GRASPING timeout");
      ROS_INFO("    elapsed=%.1fs  cmd_executing_=%s  seen_executing=%s  stabilizing=%s",
              elapsed,
              cmd_executing_.load(std::memory_order_relaxed) ? "true" : "false",
              fr_grasp_cmd_seen_executing_ ? "true" : "false",
              fr_grasp_stabilizing_ ? "true" : "false");
      return;
    }

    // Step 2: Poll cmd_executing_ — wait for grasp to start then complete
    if (!fr_grasp_stabilizing_) {
      bool executing = cmd_executing_.load(std::memory_order_relaxed);
      if (!fr_grasp_cmd_seen_executing_) {
        if (executing) {
          fr_grasp_cmd_seen_executing_ = true;
        }
        return;
      }
      if (executing) {
        return;
      }
      fr_grasp_stabilizing_ = true;
      fr_grasp_stabilize_start_ = time;
      fr_pre_sum_ = 0.0;
      fr_pre_sum_sq_ = 0.0;
      fr_pre_count_ = 0;
      return;
    }

    // Step 3: Post-grasp stabilization delay + W_pre accumulation
    double stab_elapsed = (time - fr_grasp_stabilize_start_).toSec();
    fr_pre_sum_ += wrench_norm;
    fr_pre_sum_sq_ += wrench_norm * wrench_norm;
    fr_pre_count_++;
    if (stab_elapsed < rt_fr_stabilization_) {
      return;
    }

    ROS_INFO("    Grasp force applied: %.2f N (iteration %d)", fr_f_current_, fr_iteration_);

    // Step 4: Record reference width before UPLIFT
    fr_width_before_uplift_ = gripper_snapshot.width;

    // Step 5: Initialize UPLIFT trajectory
    uplift_start_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    uplift_z_start_ = uplift_start_pose_[14];
    uplift_elapsed_ = 0.0;
    rt_uplift_distance_ = rt_fr_uplift_distance_;
    rt_uplift_duration_ = rt_fr_uplift_distance_ / rt_fr_lift_speed_;
    uplift_active_.store(true, std::memory_order_relaxed);

    // Step 6: Transition to UPLIFT
    current_state_.store(GraspState::UPLIFT, std::memory_order_relaxed);
    publishStateLabel("UPLIFT");
    logStateTransition("UPLIFT", "Micro-lift starting");
    ROS_INFO("    iter=%d  F=%.1f N  width_before=%.4f  "
            "distance=%.4f  duration=%.2f",
            fr_iteration_, fr_f_current_, fr_width_before_uplift_,
            rt_fr_uplift_distance_, rt_uplift_duration_);
  }

  void KittingStateController::tickUplift(const ros::Time& time,
                                          double /* tau_ext_norm */,
                                          double /* wrench_norm */,
                                          const GripperData& gripper_snapshot) {
    if (uplift_active_.load(std::memory_order_relaxed)) {
      return;  // Trajectory still running
    }

    fr_phase_start_time_ = time;
    fr_early_sum_ = 0.0;
    fr_early_count_ = 0;
    fr_late_sum_ = 0.0;
    fr_late_count_ = 0;
    fr_max_width_during_eval_ = gripper_snapshot.width;

    current_state_.store(GraspState::EVALUATE, std::memory_order_relaxed);
    publishStateLabel("EVALUATE");
    logStateTransition("EVALUATE", "Hold + load-transfer slip detection");
    ROS_INFO("    early=%.2fs  late=%.2fs  (total hold=%.2fs)  W_pre samples=%d",
            rt_fr_uplift_hold_ / 2.0, rt_fr_uplift_hold_ / 2.0,
            rt_fr_uplift_hold_, fr_pre_count_);
  }

  void KittingStateController::tickEvaluate(const ros::Time& time,
                                            double /* tau_ext_norm */,
                                            double wrench_norm,
                                            const GripperData& gripper_snapshot) {
    const double kEarlyEnd = rt_fr_uplift_hold_ / 2.0;  // W_hold_early [0, half)
    const double kLateEnd  = rt_fr_uplift_hold_;         // W_hold_late  [half, hold)

    double elapsed = (time - fr_phase_start_time_).toSec();
    double gw = gripper_snapshot.width;

    // Track max gripper width over entire hold
    if (gw > fr_max_width_during_eval_) {
      fr_max_width_during_eval_ = gw;
    }

    // Accumulate early window [0, 0.3)
    if (elapsed < kEarlyEnd) {
      fr_early_sum_ += wrench_norm;
      fr_early_count_++;
      return;
    }

    // Accumulate late window [0.3, 0.6)
    if (elapsed < kLateEnd) {
      fr_late_sum_ += wrench_norm;
      fr_late_count_++;
      return;
    }

    // --- Compute metrics after both windows complete ---
    double mu_pre = (fr_pre_count_ > 0) ? fr_pre_sum_ / fr_pre_count_ : 0.0;
    double sigma_pre = 0.0;
    if (fr_pre_count_ > 1) {
      double var = fr_pre_sum_sq_ / fr_pre_count_ - mu_pre * mu_pre;
      sigma_pre = std::sqrt(std::max(var, 0.0));
    }
    double mu_early = (fr_early_count_ > 0) ? fr_early_sum_ / fr_early_count_ : 0.0;
    double mu_late  = (fr_late_count_ > 0)  ? fr_late_sum_ / fr_late_count_   : 0.0;

    // Gate 1: Load transfer confirmation
    double delta_F = mu_early - mu_pre;
    bool load_transfer = delta_F > std::max(3.0 * sigma_pre, 1.0);

    // Gate 2: Drop ratio
    constexpr double kEpsilon = 1e-6;
    double drop_ratio = (mu_early - mu_late) / std::max(mu_early, kEpsilon);

    // Width change (existing criterion)
    double width_change = fr_max_width_during_eval_ - fr_width_before_uplift_;

    // Final decision: SECURE requires all gates pass
    bool secure = load_transfer &&
                  (drop_ratio <= rt_fr_slip_tau_drop_) &&
                  (width_change <= rt_fr_slip_width_change_);
    bool slip = !secure;

    ROS_INFO("  [EVALUATE]  mu_pre=%.3f  sigma_pre=%.3f  mu_early=%.3f  mu_late=%.3f  "
            "delta_F=%.3f  load_xfer=%s  drop=%.3f (thresh=%.3f)  "
            "width_change=%.4f (thresh=%.4f)  %s",
            mu_pre, sigma_pre, mu_early, mu_late,
            delta_F, load_transfer ? "YES" : "NO", drop_ratio, rt_fr_slip_tau_drop_,
            width_change, rt_fr_slip_width_change_,
            slip ? "SLIP" : "SECURE");

    if (!slip) {
      current_state_.store(GraspState::SUCCESS, std::memory_order_relaxed);
      publishStateLabel("SUCCESS");
      logStateTransition("SUCCESS", "Secure grasp confirmed");
      ROS_INFO("    F=%.1f N  iter=%d  delta_F=%.3f  drop=%.3f  width_change=%.4f",
              fr_f_current_, fr_iteration_, delta_F, drop_ratio, width_change);
    } else {
      // Initialize DOWNLIFT trajectory
      downlift_start_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
      downlift_z_start_ = downlift_start_pose_[14];
      downlift_elapsed_ = 0.0;
      rt_downlift_distance_ = rt_fr_uplift_distance_;
      rt_downlift_duration_ = rt_fr_uplift_distance_ / rt_fr_lift_speed_;
      downlift_active_.store(true, std::memory_order_relaxed);

      current_state_.store(GraspState::DOWNLIFT, std::memory_order_relaxed);
      publishStateLabel("DOWNLIFT");
      logStateTransition("DOWNLIFT", "Returning before force increment");
      ROS_INFO("    %s  distance=%.4f m  duration=%.2f s",
              load_transfer ? "Drop detected" : "No load transfer",
              rt_downlift_distance_, rt_downlift_duration_);
    }
  }

  void KittingStateController::tickDownlift(const ros::Time& time,
                                            double /* tau_ext_norm */,
                                            double /* wrench_norm */,
                                            const GripperData& /* gripper_snapshot */) {
    if (downlift_active_.load(std::memory_order_relaxed)) {
      return;  // Trajectory still running
    }

    fr_phase_start_time_ = time;
    current_state_.store(GraspState::SETTLING, std::memory_order_relaxed);
    publishStateLabel("SETTLING");
    ROS_INFO("  [STATE]  >>  SETTLING  <<  Post-downlift stabilization (%.2fs)",
            rt_fr_stabilization_);
  }

  void KittingStateController::tickSettling(const ros::Time& time,
                                            double /* tau_ext_norm */,
                                            double /* wrench_norm */,
                                            const GripperData& /* gripper_snapshot */) {
    double elapsed = (time - fr_phase_start_time_).toSec();
    if (elapsed < rt_fr_stabilization_) {
      return;  // Still settling
    }

    // Increment force for next iteration
    fr_f_current_ += rt_fr_f_step_;
    fr_iteration_++;

    if (fr_f_current_ > rt_fr_f_max_) {
      current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
      publishStateLabel("FAILED");
      logStateTransition("FAILED", "Max force exceeded");
      ROS_INFO("    f_current=%.1f N > f_max=%.1f N  after %d iterations",
              fr_f_current_, rt_fr_f_max_, fr_iteration_);
      return;
    }

    // Re-enter GRASPING with incremented force
    fr_phase_start_time_ = time;
    fr_grasping_phase_initialized_ = true;
    fr_grasp_cmd_seen_executing_ = false;
    fr_grasp_stabilizing_ = false;
    requestDeferredGrasp(fr_width_before_uplift_, rt_fr_grasp_speed_,
                        fr_f_current_, rt_fr_epsilon_);
    current_state_.store(GraspState::GRASPING, std::memory_order_relaxed);
    publishStateLabel("GRASPING");
    logStateTransition("GRASPING", "Force ramp retry");
    ROS_INFO("    iter=%d  F=%.1f N", fr_iteration_, fr_f_current_);
  }

}  // namespace franka_kitting_controller

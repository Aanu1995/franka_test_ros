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
  // Closing-phase transitions
  // CLOSING_COMMAND → CLOSING → CONTACT_CONFIRMED → CONTACT  or  → FAILED
  // ============================================================================

  void KittingStateController::runClosingTransitions(const ros::Time& time,
                                                      const GripperData& gripper_snapshot) {
    auto state = current_state_.load(std::memory_order_relaxed);

    if ((state == GraspState::CLOSING_COMMAND || state == GraspState::CLOSING) &&
        !contact_latched_) {
      // Track when the gripper move command starts executing.
      // Stall detection is deferred until the move is confirmed running to prevent
      // false contacts during the pre-movement window (gripper stationary, velocity ≈ 0,
      // width gap large — looks like a stall but the move hasn't started yet).
      if (!closing_cmd_seen_executing_) {
        if (cmd_executing_.load(std::memory_order_relaxed)) {
          closing_cmd_seen_executing_ = true;
          // Transition CLOSING_COMMAND → CLOSING now that gripper is confirmed moving
          if (state == GraspState::CLOSING_COMMAND) {
            current_state_.store(GraspState::CLOSING, std::memory_order_relaxed);
            state = GraspState::CLOSING;
            publishStateLabel("CLOSING");
            logStateTransition("CLOSING", "Gripper move confirmed — closing");
          }
        }
      }

      if (closing_cmd_seen_executing_) {
        // Stall detection (only after move is confirmed executing).
        // On stall: detectGripperContact transitions to CONTACT_CONFIRMED internally.
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
        current_state_.load(std::memory_order_relaxed) == GraspState::CONTACT_CONFIRMED &&
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
      contact_width_.store(gripper_snapshot.width, std::memory_order_relaxed);

      // Publish CONTACT_CONFIRMED immediately at the point of detection,
      // before requesting gripper stop — earliest possible label for data analysis.
      current_state_.store(GraspState::CONTACT_CONFIRMED, std::memory_order_relaxed);
      publishStateLabel("CONTACT_CONFIRMED");
      logStateTransition("CONTACT_CONFIRMED", "Stall detected — gripper stopping");

      requestGripperStop("Stall");
      ROS_INFO("  [CONTACT_CONFIRMED]  Stall detected, waiting for gripper stop: "
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
                                                      double support_force,
                                                      double tangential_force,
                                                      const GripperData& gripper_snapshot) {
    switch (current_state_.load(std::memory_order_relaxed)) {
      case GraspState::GRASPING:  tickGrasping(time, tau_ext_norm, support_force, tangential_force, gripper_snapshot); break;
      case GraspState::UPLIFT:    tickUplift(time, tau_ext_norm, support_force, tangential_force, gripper_snapshot);   break;
      case GraspState::EVALUATE:  tickEvaluate(time, tau_ext_norm, support_force, tangential_force, gripper_snapshot); break;
      case GraspState::SLIP:      tickSlip(time, tau_ext_norm, support_force, tangential_force, gripper_snapshot);     break;
      case GraspState::DOWNLIFT:  tickDownlift(time, tau_ext_norm, support_force, tangential_force, gripper_snapshot); break;
      case GraspState::SETTLING:  tickSettling(time, tau_ext_norm, support_force, tangential_force, gripper_snapshot); break;
      default: break;
    }
  }

  void KittingStateController::tickGrasping(const ros::Time& time,
                                            double /* tau_ext_norm */,
                                            double support_force,
                                            double /* tangential_force */,
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

    // Step 3: Post-grasp W_pre accumulation (Fn = support force, duration = uplift_hold/2)
    double stab_elapsed = (time - fr_grasp_stabilize_start_).toSec();
    fr_pre_sum_ += support_force;
    fr_pre_sum_sq_ += support_force * support_force;
    fr_pre_count_++;
    if (stab_elapsed < (rt_fr_uplift_hold_ / 2.0)) {
      return;
    }

    ROS_INFO("    Grasp force applied: %.2f N (iteration %d)", fr_f_current_, fr_iteration_);

    // Snapshot gripper width right before uplift — used for retry grasp
    fr_grasp_width_snapshot_ = gripper_snapshot.width;

    // Step 4: Initialize UPLIFT trajectory
    uplift_start_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    uplift_z_start_ = uplift_start_pose_[14];
    uplift_elapsed_ = 0.0;
    rt_uplift_distance_ = rt_fr_uplift_distance_;
    rt_uplift_duration_ = rt_fr_uplift_distance_ / rt_fr_lift_speed_;
    uplift_active_.store(true, std::memory_order_relaxed);

    // Step 5: Transition to UPLIFT
    current_state_.store(GraspState::UPLIFT, std::memory_order_relaxed);
    publishStateLabel("UPLIFT");
    logStateTransition("UPLIFT", "Micro-lift starting");
    ROS_INFO("    iter=%d  F=%.1f N  width=%.4f  "
            "distance=%.4f  duration=%.2f",
            fr_iteration_, fr_f_current_, gripper_snapshot.width,
            rt_fr_uplift_distance_, rt_uplift_duration_);
  }

  void KittingStateController::tickUplift(const ros::Time& time,
                                          double /* tau_ext_norm */,
                                          double /* support_force */,
                                          double /* tangential_force */,
                                          const GripperData& /* gripper_snapshot */) {
    if (uplift_active_.load(std::memory_order_relaxed)) {
      return;  // Trajectory still running
    }

    fr_phase_start_time_ = time;
    fr_early_sum_ = 0.0;
    fr_early_count_ = 0;
    fr_late_sum_ = 0.0;
    fr_late_count_ = 0;
    fr_width_samples_.clear();
    fr_width_samples_.reserve(static_cast<size_t>(rt_fr_uplift_hold_ * kWidthSamplesPerSec) + 100);
    fr_friction_sum_ = 0.0;
    fr_friction_count_ = 0;
    fr_friction_max_ = 0.0;

    current_state_.store(GraspState::EVALUATE, std::memory_order_relaxed);
    publishStateLabel("EVALUATE");
    logStateTransition("EVALUATE", "Hold + slip score evaluation");
    ROS_INFO("    early=%.2fs  late=%.2fs  (total hold=%.2fs)  W_pre samples=%d",
            rt_fr_uplift_hold_ / 2.0, rt_fr_uplift_hold_ / 2.0,
            rt_fr_uplift_hold_, fr_pre_count_);
  }

  void KittingStateController::tickEvaluate(const ros::Time& time,
                                            double /* tau_ext_norm */,
                                            double support_force,
                                            double tangential_force,
                                            const GripperData& gripper_snapshot) {
    const double kEarlyEnd = rt_fr_uplift_hold_ / 2.0;  // W_hold_early [0, half)
    const double kLateEnd  = rt_fr_uplift_hold_;         // W_hold_late  [half, hold)
    constexpr double kEpsilon = 1e-6;

    double elapsed = (time - fr_phase_start_time_).toSec();
    double gw = gripper_snapshot.width;

    // Collect width samples for P5/P95 percentile computation
    fr_width_samples_.push_back(gw);

    // Track friction utilization over entire hold: u = Ft / max(Fn, eps)
    double u_sample = tangential_force / std::max(support_force, kEpsilon);
    fr_friction_sum_ += u_sample;
    fr_friction_count_++;
    if (u_sample > fr_friction_max_) {
      fr_friction_max_ = u_sample;
    }

    // Accumulate early window [0, half) — support force (Fn)
    if (elapsed < kEarlyEnd) {
      fr_early_sum_ += support_force;
      fr_early_count_++;
      return;
    }

    // Accumulate late window [half, hold) — support force (Fn)
    if (elapsed < kLateEnd) {
      fr_late_sum_ += support_force;
      fr_late_count_++;
      return;
    }

    // ================================================================
    // Hold complete — compute slip score
    // ================================================================

    // --- Window statistics (all based on Fn = support force) ---
    double Fn_pre   = (fr_pre_count_ > 0) ? fr_pre_sum_ / fr_pre_count_ : 0.0;
    double sigma_pre = 0.0;
    if (fr_pre_count_ > 1) {
      double var = fr_pre_sum_sq_ / fr_pre_count_ - Fn_pre * Fn_pre;
      sigma_pre = std::sqrt(std::max(var, 0.0));
    }
    double Fn_early = (fr_early_count_ > 0) ? fr_early_sum_ / fr_early_count_ : 0.0;
    double Fn_late  = (fr_late_count_ > 0)  ? fr_late_sum_ / fr_late_count_   : 0.0;

    // --- Step A: Load Transfer Gate (mandatory) ---
    double deltaF = Fn_early - Fn_pre;
    double load_thresh = std::max(3.0 * sigma_pre, rt_fr_load_transfer_min_);
    bool load_transferred = deltaF > load_thresh;

    // --- Step B: Primary kitting cue — support force decay (s_drop) ---
    double dF = (Fn_early - Fn_late) / std::max(Fn_early, kEpsilon);  // relative decay
    double s_drop = std::max(0.0, std::min(1.0, dF / rt_fr_slip_drop_thresh_));

    // --- Step C: Secondary cue — jaw widening (s_motion) ---
    // dw = P95 - P5 of width over entire hold — robust to transient outliers
    double p5 = 0.0, p95 = 0.0, dw = 0.0;
    int n = static_cast<int>(fr_width_samples_.size());
    if (n >= 20) {
      int i5  = static_cast<int>(0.05 * (n - 1));
      int i95 = static_cast<int>(0.95 * (n - 1));
      std::nth_element(fr_width_samples_.begin(), fr_width_samples_.begin() + i5,
                       fr_width_samples_.end());
      p5 = fr_width_samples_[i5];
      std::nth_element(fr_width_samples_.begin() + i5 + 1, fr_width_samples_.begin() + i95,
                       fr_width_samples_.end());
      p95 = fr_width_samples_[i95];
      dw = p95 - p5;
    }
    double s_motion = std::max(0.0, std::min(1.0, dw / rt_fr_slip_width_thresh_));

    // --- Step D: Optional low-priority cue — friction utilization (s_force) ---
    double u_hold = (fr_friction_count_ > 0) ? fr_friction_sum_ / fr_friction_count_ : 0.0;
    double u_peak = fr_friction_max_;
    double s_force = std::max(0.0, std::min(1.0, u_hold / rt_fr_slip_friction_thresh_));
    s_force = std::max(s_force,
        std::max(0.0, std::min(1.0, u_peak / (rt_fr_slip_friction_thresh_ * 1.1))));
    // Combine friction utilization with support drop
    s_force = std::max(s_force, s_drop);

    // --- Step E: Fused slip score (soft-OR) ---
    double slip_score = 1.0 - (1.0 - s_motion) * (1.0 - std::max(s_drop, s_force));
    bool is_slipping = !load_transferred || (slip_score > rt_fr_slip_score_thresh_);

    // --- Terminal logging: each sub-score on its own line with threshold ---
    ROS_INFO("  [SLIP] Load Transfer:  deltaF=%.3f N  threshold=%.3f N  (Fn_pre=%.3f  sigma=%.3f  Fn_early=%.3f)  %s",
             deltaF, load_thresh, Fn_pre, sigma_pre, Fn_early,
             load_transferred ? "PASSED" : "FAILED");
    ROS_INFO("  [SLIP] s_drop:         %.3f / %.3f  (dF=%.1f%%  DF_TH=%.0f%%)",
             s_drop, rt_fr_slip_drop_thresh_, dF * 100.0, rt_fr_slip_drop_thresh_ * 100.0);
    ROS_INFO("  [SLIP] s_motion:       %.3f / %.4f  (P95-P5=%.5f m  P5=%.5f  P95=%.5f  W_TH=%.4f m)",
             s_motion, rt_fr_slip_width_thresh_, dw, p5, p95, rt_fr_slip_width_thresh_);
    ROS_INFO("  [SLIP] s_force:        %.3f / %.3f  (u_hold=%.3f  u_peak=%.3f  U_TH=%.3f)",
             s_force, rt_fr_slip_friction_thresh_, u_hold, u_peak, rt_fr_slip_friction_thresh_);
    ROS_INFO("  [SLIP] slip_score:     %.3f / %.3f  -> %s",
             slip_score, rt_fr_slip_score_thresh_,
             is_slipping ? "SLIPPING" : "SECURE");

    if (!is_slipping) {
      accumulated_uplift_ += rt_fr_uplift_distance_;  // This UPLIFT wasn't DOWNLIFTed
      current_state_.store(GraspState::SUCCESS, std::memory_order_relaxed);
      publishStateLabel("SUCCESS");
      logStateTransition("SUCCESS", "Secure grasp confirmed");
      ROS_INFO("    F=%.1f N  iter=%d  slip_score=%.3f",
              fr_f_current_, fr_iteration_, slip_score);
    } else {
      current_state_.store(GraspState::SLIP, std::memory_order_relaxed);
      publishStateLabel("SLIP");
      logStateTransition("SLIP", load_transferred ? "Slip detected" : "No load transfer");
      ROS_INFO("    slip_score=%.3f  F=%.1f N  iter=%d",
              slip_score, fr_f_current_, fr_iteration_);
    }
  }

  void KittingStateController::tickSlip(const ros::Time& /* time */,
                                        double /* tau_ext_norm */,
                                        double /* support_force */,
                                        double /* tangential_force */,
                                        const GripperData& /* gripper_snapshot */) {
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
    ROS_INFO("    distance=%.4f m  duration=%.2f s",
            rt_downlift_distance_, rt_downlift_duration_);
  }

  void KittingStateController::tickDownlift(const ros::Time& time,
                                            double /* tau_ext_norm */,
                                            double /* support_force */,
                                            double /* tangential_force */,
                                            const GripperData& /* gripper_snapshot */) {
    if (downlift_active_.load(std::memory_order_relaxed)) {
      return;  // Trajectory still running
    }

    fr_phase_start_time_ = time;
    current_state_.store(GraspState::SETTLING, std::memory_order_relaxed);
    publishStateLabel("SETTLING");
    ROS_INFO("  [STATE]  >>  SETTLING  <<  Post-downlift stabilization (%.2fs)",
            rt_fr_uplift_hold_ / 2.0);
  }

  void KittingStateController::tickSettling(const ros::Time& time,
                                            double /* tau_ext_norm */,
                                            double /* support_force */,
                                            double /* tangential_force */,
                                            const GripperData& /* gripper_snapshot */) {
    double elapsed = (time - fr_phase_start_time_).toSec();
    if (elapsed < (rt_fr_uplift_hold_ / 2.0)) {
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
    requestDeferredGrasp(fr_grasp_width_snapshot_, rt_fr_grasp_speed_,
                        fr_f_current_, rt_fr_epsilon_);
    current_state_.store(GraspState::GRASPING, std::memory_order_relaxed);
    publishStateLabel("GRASPING");
    logStateTransition("GRASPING", "Force ramp retry");
    ROS_INFO("    iter=%d  F=%.1f N", fr_iteration_, fr_f_current_);
  }

}  // namespace franka_kitting_controller

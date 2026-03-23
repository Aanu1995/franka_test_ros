// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//
// Contact detection during CLOSING and contact-only force ramp state machine
// (GRASPING ramp f_min→f_max on table → single UPLIFT → EVALUATE → SUCCESS/FAILED).
// Part of KittingStateController — see kitting_state_controller.h for class definition.

#include <franka_kitting_controller/kitting_state_controller.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <ros/ros.h>

namespace franka_kitting_controller {

  // ============================================================================
  // Deferred grasp dispatch (realtime-safe → read thread)
  // ============================================================================

  void KittingStateController::requestDeferredGrasp(double width, double speed,
                                                    double force, double epsilon) {
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
                                                      const GripperData& gripper_snapshot,
                                                      double tau_ext_norm) {
    auto state = current_state_.load(std::memory_order_relaxed);

    if ((state == GraspState::CLOSING_COMMAND || state == GraspState::CLOSING) &&
        !contact_latched_) {
      // Hold one tick so CLOSING_COMMAND label is published before transitioning
      if (closing_command_entered_) {
        closing_command_entered_ = false;
        return;
      }

      // Defer contact detection until move is confirmed running
      if (!closing_cmd_seen_executing_) {
        if (cmd_executing_.load(std::memory_order_relaxed)) {
          closing_cmd_seen_executing_ = true;
          // Transition CLOSING_COMMAND → CLOSING now that gripper is confirmed moving
          if (state == GraspState::CLOSING_COMMAND) {
            current_state_.store(GraspState::CLOSING, std::memory_order_relaxed);
            state = GraspState::CLOSING;
            publishStateLabel("CLOSING");
            logStateTransition("CLOSING", "Gripper move confirmed — closing");

            // Activate SMS-CUSUM contact detection
            if (sms_detector_.baseline_ready()) {
              sms_detector_.enter_closing();
              ROS_INFO("  [CLOSING]  SMS-CUSUM activated: k_eff=%.4f  baseline=%.3f  sigma=%.4f",
                       sms_detector_.contact_cusum().k_effective(),
                       sms_detector_.baseline().mean(),
                       sms_detector_.baseline().sigma());
            }
          }
        } else if (state == GraspState::CLOSING_COMMAND &&
                   (time - fr_phase_start_time_).toSec() > kClosingCmdTimeout) {
          // Move command never started — command thread may be stuck
          current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
          publishStateLabel("FAILED");
          logStateTransition("FAILED", "CLOSING_COMMAND timeout — move never started");
          return;
        }
      }

      if (closing_cmd_seen_executing_) {
        detectContact(time, gripper_snapshot, tau_ext_norm);

        // No-contact: move completed without triggering contact → FAILED
        if (!contact_latched_ && !cmd_executing_.load(std::memory_order_relaxed)) {
          current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
          publishStateLabel("FAILED");
          logStateTransition("FAILED",
              "Gripper closed to target width — no contact detected");
          ROS_WARN("  [CLOSING]  No contact: w=%.4f  w_cmd=%.4f  -> FAILED",
                   gripper_snapshot.width, rt_closing_w_cmd_);
          return;
        }

        // Timeout: CLOSING phase exceeded maximum duration without contact → FAILED
        if (!contact_latched_ &&
            (time - fr_phase_start_time_).toSec() > kClosingTimeout) {
          current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
          publishStateLabel("FAILED");
          logStateTransition("FAILED", "CLOSING timeout — no contact detected");
          ROS_WARN("  [CLOSING]  Timeout after %.1f s: w=%.4f  w_cmd=%.4f  -> FAILED",
                   kClosingTimeout, gripper_snapshot.width, rt_closing_w_cmd_);
          requestGripperStop("Closing timeout");
          return;
        }
      }
    }

    // Deferred CONTACT transition: wait for gripper to physically stop.
    // contact_width_ is captured by the read thread (gripperReadLoop) on the
    // first readOnce() after stop(). gripper_stopped_ is stored with release
    // AFTER contact_width_, so acquire here guarantees fresh width visibility.
    if (contact_latched_ &&
        current_state_.load(std::memory_order_relaxed) == GraspState::CONTACT_CONFIRMED &&
        gripper_stopped_.load(std::memory_order_acquire)) {
      // contact_width_ already set by read thread — do NOT capture from RT buffer
      current_state_.store(GraspState::CONTACT, std::memory_order_relaxed);
      publishStateLabel("CONTACT");
      logStateTransition("CONTACT", "Gripper stopped — contact confirmed");
      ROS_INFO("    contact_width=%.4f m", contact_width_.load(std::memory_order_relaxed));
    }
  }

  void KittingStateController::detectContact(const ros::Time& time,
                                              const GripperData& gripper_snapshot,
                                              double tau_ext_norm) {
    bool gripper_data_valid = (gripper_snapshot.stamp != ros::Time(0)) &&
                              ((time - gripper_snapshot.stamp).toSec() < 0.5);
    if (!gripper_data_valid) {
      return;
    }

    // Baseline should already be collected during BASELINE state.
    // Fallback: if not ready (e.g., CLOSING sent without prior BASELINE), collect now.
    if (!sms_detector_.baseline_ready()) {
      sms_detector_.update(tau_ext_norm);
      if (sms_detector_.baseline_ready() && !cd_baseline_ready_.load(std::memory_order_relaxed)) {
        cd_baseline_ready_.store(true, std::memory_order_relaxed);
        cd_baseline_ = sms_detector_.baseline().mean();
        ROS_WARN("  [CONTACT]  SMS-CUSUM baseline ready (fallback): mu=%.3f Nm, sigma=%.4f Nm",
                 sms_detector_.baseline().mean(), sms_detector_.baseline().sigma());
        sms_detector_.enter_closing();
      }
      return;
    }

    // --- SMS-CUSUM contact detection (replaces fixed threshold + debounce) ---
    auto result = sms_detector_.update(tau_ext_norm);

    if (result.detected &&
        result.event.new_state == sms_cusum::GraspState::CONTACT) {
      contact_latched_ = true;

      // Publish CONTACT_CONFIRMED immediately at the point of detection.
      // Note: contact_width_ is NOT stored here — the gripper is still decelerating.
      // It will be captured at the deferred CONTACT transition once the gripper has stopped.
      current_state_.store(GraspState::CONTACT_CONFIRMED, std::memory_order_relaxed);
      publishStateLabel("CONTACT_CONFIRMED");
      logStateTransition("CONTACT_CONFIRMED", "SMS-CUSUM contact detected — gripper stopping");

      requestGripperStop("Contact");

      ROS_INFO("  [CONTACT_CONFIRMED]  SMS-CUSUM detection: "
              "S=%.3f  baseline=%.3f  sigma=%.4f  k_eff=%.4f  tau=%.3f  w=%.4f",
              result.event.cusum_statistic,
              result.event.baseline_mean,
              result.event.baseline_sigma,
              result.event.k_effective,
              tau_ext_norm,
              gripper_snapshot.width);
    }
  }

  // ============================================================================
  // Force ramp state machine (250Hz, realtime-safe)
  // Called from update() when state is GRASPING, UPLIFT, or EVALUATE.
  //
  // GRASPING: ramp force from f_min to f_max in f_step increments while object
  //           remains on table. Each step: grasp → settle → hold/log.
  //           Published as GRASP_1, GRASP_2, ..., GRASP_N.
  // After final step: → UPLIFT → EVALUATE → SUCCESS/FAILED.
  // ============================================================================

  void KittingStateController::runInternalTransitions(const ros::Time& time,
                                                      double tau_ext_norm,
                                                      double support_force,
                                                      const GripperData& gripper_snapshot) {
    switch (current_state_.load(std::memory_order_relaxed)) {
      case GraspState::GRASPING:  tickGrasping(time, tau_ext_norm, support_force, gripper_snapshot); break;
      case GraspState::UPLIFT:    tickUplift(time, tau_ext_norm, support_force, gripper_snapshot);   break;
      case GraspState::EVALUATE:  tickEvaluate(time, tau_ext_norm, support_force, gripper_snapshot); break;
      default: break;
    }
  }

  void KittingStateController::tickGrasping(const ros::Time& time,
                                            double /* tau_ext_norm */,
                                            double support_force,
                                            const GripperData& gripper_snapshot) {
    if (!fr_grasping_phase_initialized_) {
      fr_phase_start_time_ = time;
      fr_grasping_phase_initialized_ = true;
    }

    double elapsed = (time - fr_phase_start_time_).toSec();

    // Global timeout for the current ramp step
    if (elapsed > kGraspTimeout) {
      if (cmd_executing_.load(std::memory_order_relaxed)) {
        stop_requested_.store(true, std::memory_order_relaxed);
      }
      current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
      publishStateLabel("FAILED");
      logStateTransition("FAILED", "GRASPING timeout");
      ROS_INFO("    elapsed=%.1fs  iter=%d  F=%.1f N  ramp_phase=%d",
              elapsed, fr_iteration_, fr_f_current_,
              static_cast<int>(fr_ramp_phase_));
      return;
    }

    switch (fr_ramp_phase_) {
      case RampPhase::COMMAND_SENT: {
        // Wait for command thread to pick up the grasp command
        if (elapsed < kGraspSettleDelay) {
          return;
        }
        // Wait for cmd_executing_ to go true
        if (!fr_grasp_cmd_seen_executing_) {
          if (cmd_executing_.load(std::memory_order_relaxed)) {
            fr_grasp_cmd_seen_executing_ = true;
          }
          return;
        }
        fr_ramp_phase_ = RampPhase::WAITING_EXECUTION;
        break;
      }

      case RampPhase::WAITING_EXECUTION: {
        // Wait for grasp command to finish (cmd_executing_ → false)
        if (cmd_executing_.load(std::memory_order_relaxed)) {
          return;
        }
        // Check hardware result: grasp() returns false if the gripper could not
        // grasp at the commanded width/force/epsilon (e.g., object not present,
        // width outside tolerance). cmd_success_ is set by the command thread
        // with release ordering before cmd_executing_ goes false.
        if (!cmd_success_.load(std::memory_order_acquire)) {
          current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
          publishStateLabel("FAILED");
          logStateTransition("FAILED", "Grasp command failed (hardware rejected)");
          ROS_WARN("    GRASP_%d: F=%.1f N  grasp() returned false  width=%.4f m -> FAILED",
                  fr_iteration_ + 1, fr_f_current_, gripper_snapshot.width);
          return;
        }
        fr_ramp_phase_ = RampPhase::SETTLING;
        fr_ramp_step_start_time_ = time;
        ROS_INFO("    GRASP_%d: grasp complete at F=%.1f N, settling for %.2fs",
                fr_iteration_ + 1, fr_f_current_, rt_fr_grasp_settle_time_);
        break;
      }

      case RampPhase::SETTLING: {
        // Wait grasp_settle_time for gripper to stabilize
        if ((time - fr_ramp_step_start_time_).toSec() < rt_fr_grasp_settle_time_) {
          return;
        }
        fr_ramp_phase_ = RampPhase::HOLDING;
        fr_ramp_step_start_time_ = time;

        // On the last ramp step, initialize W_pre accumulators for EVALUATE
        bool is_last_step = (fr_f_current_ + rt_fr_f_step_ > rt_fr_f_max_);
        if (is_last_step) {
          fr_pre_sum_ = 0.0;
          fr_pre_sum_sq_ = 0.0;
          fr_pre_count_ = 0;
          ROS_INFO("    GRASP_%d: last step — accumulating W_pre during hold",
                  fr_iteration_ + 1);
        }
        ROS_INFO("    GRASP_%d: holding at F=%.1f N for %.2fs",
                fr_iteration_ + 1, fr_f_current_, rt_fr_grasp_force_hold_time_);
        break;
      }

      case RampPhase::HOLDING: {
        double hold_elapsed = (time - fr_ramp_step_start_time_).toSec();

        // On the last ramp step, accumulate W_pre (support force) during hold
        bool is_last_step = (fr_f_current_ + rt_fr_f_step_ > rt_fr_f_max_);
        if (is_last_step) {
          fr_pre_sum_ += support_force;
          fr_pre_sum_sq_ += support_force * support_force;
          fr_pre_count_++;
        }

        if (hold_elapsed < rt_fr_grasp_force_hold_time_) {
          return;
        }

        // Hold complete — update contact_width from current gripper measurement
        contact_width_.store(gripper_snapshot.width, std::memory_order_relaxed);

        ROS_INFO("    GRASP_%d: hold complete  F=%.1f N  width=%.4f m (updated)",
                fr_iteration_ + 1, fr_f_current_, gripper_snapshot.width);

        fr_ramp_phase_ = RampPhase::STEP_COMPLETE;
        break;
      }

      case RampPhase::STEP_COMPLETE: {
        bool is_last_step = (fr_f_current_ + rt_fr_f_step_ > rt_fr_f_max_);

        if (is_last_step) {
          // All ramp steps done — transition to UPLIFT
          uplift_start_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
          uplift_z_start_ = uplift_start_pose_[14];
          uplift_elapsed_ = 0.0;
          rt_uplift_distance_ = rt_fr_uplift_distance_;
          rt_uplift_duration_ = rt_fr_uplift_distance_ / rt_fr_lift_speed_;
          uplift_active_.store(true, std::memory_order_relaxed);

          current_state_.store(GraspState::UPLIFT, std::memory_order_relaxed);
          publishStateLabel("UPLIFT");
          logStateTransition("UPLIFT", "Ramp complete — micro-lift starting");
          ROS_INFO("    final_iter=%d  F=%.1f N  width=%.4f m  "
                  "distance=%.4f m  duration=%.2f s",
                  fr_iteration_ + 1, fr_f_current_, gripper_snapshot.width,
                  rt_fr_uplift_distance_, rt_uplift_duration_);
        } else {
          // Advance to next force step
          fr_f_current_ += rt_fr_f_step_;
          fr_iteration_++;

          // Dispatch deferred grasp at new force with updated contact_width
          double w = contact_width_.load(std::memory_order_relaxed);
          requestDeferredGrasp(w, rt_fr_grasp_speed_, fr_f_current_, rt_fr_epsilon_);

          // Reset for next step
          fr_phase_start_time_ = time;
          fr_grasping_phase_initialized_ = true;
          fr_grasp_cmd_seen_executing_ = false;
          fr_ramp_phase_ = RampPhase::COMMAND_SENT;

          // Publish GRASP_N label for the new step
          char label[16];
          std::snprintf(label, sizeof(label), "GRASP_%d", fr_iteration_ + 1);
          publishStateLabel(label);
          logStateTransition(label, "Force ramp step");
          ROS_INFO("    iter=%d  F=%.1f N  width=%.4f m", fr_iteration_ + 1, fr_f_current_, w);
        }
        break;
      }
    }
  }

  void KittingStateController::tickUplift(const ros::Time& time,
                                          double /* tau_ext_norm */,
                                          double /* support_force */,
                                          const GripperData& /* gripper_snapshot */) {
    if (uplift_active_.load(std::memory_order_relaxed)) {
      return;  // Trajectory still running
    }

    fr_phase_start_time_ = time;
    fr_early_sum_ = 0.0;
    fr_early_count_ = 0;
    fr_late_sum_ = 0.0;
    fr_late_count_ = 0;
    fr_width_samples_.clear();  // Capacity preserved from starting() pre-allocation

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
                                            const GripperData& gripper_snapshot) {
    const double kEarlyEnd = rt_fr_uplift_hold_ / 2.0;  // W_hold_early [0, half)
    const double kLateEnd  = rt_fr_uplift_hold_;         // W_hold_late  [half, hold)
    constexpr double kEpsilon = 1e-6;

    double elapsed = (time - fr_phase_start_time_).toSec();
    double gw = gripper_snapshot.width;

    fr_width_samples_.push_back(gw);

    if (elapsed < kEarlyEnd) {
      fr_early_sum_ += support_force;
      fr_early_count_++;
      return;
    }

    if (elapsed < kLateEnd) {
      fr_late_sum_ += support_force;
      fr_late_count_++;
      return;
    }

    // ================================================================
    // Hold complete — rigid AND-gating (3 gates)
    // ================================================================

    double Fn_pre   = (fr_pre_count_ > 0) ? fr_pre_sum_ / fr_pre_count_ : 0.0;
    double sigma_pre = 0.0;
    if (fr_pre_count_ > 1) {
      double var = fr_pre_sum_sq_ / fr_pre_count_ - Fn_pre * Fn_pre;
      sigma_pre = std::sqrt(std::max(var, 0.0));
    }
    double Fn_early = (fr_early_count_ > 0) ? fr_early_sum_ / fr_early_count_ : 0.0;
    double Fn_late  = (fr_late_count_ > 0)  ? fr_late_sum_ / fr_late_count_   : 0.0;

    // --- Gate 1: Load Transfer (mandatory) ---
    double deltaF = Fn_early - Fn_pre;
    double load_thresh = std::max(3.0 * sigma_pre, rt_fr_load_transfer_min_);
    bool load_transferred = deltaF > load_thresh;

    // --- Gate 2: Support drop check ---
    double dF = (Fn_early - Fn_late) / std::max(Fn_early, kEpsilon);  // relative decay
    bool drop_ok = (dF <= rt_fr_slip_drop_thresh_);

    // --- Gate 3: Jaw widening (P95 - P5) ---
    // Note: nth_element is O(n) average — ~2-5μs for 250 samples, within 4ms RT budget.
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
    bool width_ok = (dw <= rt_fr_slip_width_thresh_);

    // --- Verdict: secure = Gate1 AND Gate2 AND Gate3 ---
    bool is_slipping = !(load_transferred && drop_ok && width_ok);

    ROS_INFO("  [EVAL] Gate 1 — Load Transfer:  deltaF=%.3f N  threshold=%.3f N  "
             "(Fn_pre=%.3f  sigma=%.3f  Fn_early=%.3f)  %s",
             deltaF, load_thresh, Fn_pre, sigma_pre, Fn_early,
             load_transferred ? "PASS" : "FAIL");
    ROS_INFO("  [EVAL] Gate 2 — Support Drop:   dF=%.1f%%  threshold=%.0f%%  %s",
             dF * 100.0, rt_fr_slip_drop_thresh_ * 100.0,
             drop_ok ? "PASS" : "FAIL");
    ROS_INFO("  [EVAL] Gate 3 — Jaw Widening:   P95-P5=%.5f m  threshold=%.4f m  "
             "(P5=%.5f  P95=%.5f)  %s",
             dw, rt_fr_slip_width_thresh_, p5, p95,
             width_ok ? "PASS" : "FAIL");
    ROS_INFO("  [EVAL] Verdict: %s", is_slipping ? "SLIPPING" : "SECURE");

    if (!is_slipping) {
      accumulated_uplift_ += rt_fr_uplift_distance_;  // Track for BASELINE prep lowering
      current_state_.store(GraspState::SUCCESS, std::memory_order_relaxed);
      publishStateLabel("SUCCESS");
      logStateTransition("SUCCESS", "Secure grasp confirmed");
      ROS_INFO("    F=%.1f N  iter=%d", fr_f_current_, fr_iteration_ + 1);
    } else {
      current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
      publishStateLabel("FAILED");
      logStateTransition("FAILED", load_transferred ? "Slip detected" : "No load transfer");
      ROS_INFO("    F=%.1f N  iter=%d", fr_f_current_, fr_iteration_ + 1);
    }
  }

}  // namespace franka_kitting_controller

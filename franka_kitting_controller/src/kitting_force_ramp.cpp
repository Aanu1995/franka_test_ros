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

  void KittingStateController::requestDeferredGrasp(double width, double speed,
                                                    double force, double epsilon) {
    deferred_grasp_width_ = width;
    deferred_grasp_speed_ = speed;
    deferred_grasp_force_ = force;
    deferred_grasp_epsilon_ = epsilon;
    deferred_grasp_pending_.store(true, std::memory_order_release);
  }

  void KittingStateController::runClosingTransitions(const ros::Time& time,
                                                      const GripperData& gripper_snapshot,
                                                      double tau_ext_norm) {
    auto state = current_state_.load(std::memory_order_relaxed);

    if ((state == GraspState::CLOSING_COMMAND || state == GraspState::CLOSING) &&
        !contact_latched_) {
      if (!closing_command_entered_) {
        closing_command_entered_ = true;
        fr_phase_start_time_ = time;
        return;
      }

      if (!closing_cmd_seen_executing_) {
        if (cmd_executing_.load(std::memory_order_relaxed)) {
          closing_cmd_seen_executing_ = true;
          if (state == GraspState::CLOSING_COMMAND) {
            current_state_.store(GraspState::CLOSING, std::memory_order_relaxed);
            state = GraspState::CLOSING;
            publishStateLabel("CLOSING");
            logStateTransition("CLOSING", "Gripper move confirmed — closing");

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
          current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
          publishStateLabel("FAILED");
          logStateTransition("FAILED", "CLOSING_COMMAND timeout — move never started");
          return;
        }
      }

      if (closing_cmd_seen_executing_) {
        detectContact(time, gripper_snapshot, tau_ext_norm);

        if (!contact_latched_ && !cmd_executing_.load(std::memory_order_relaxed)) {
          current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
          publishStateLabel("FAILED");
          logStateTransition("FAILED",
              "Gripper closed to target width — no contact detected");
          ROS_WARN("  [CLOSING]  No contact: w=%.4f  w_cmd=%.4f  -> FAILED",
                   gripper_snapshot.width, rt_closing_w_cmd_);
          return;
        }

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

    if (contact_latched_ &&
        current_state_.load(std::memory_order_relaxed) == GraspState::CONTACT_CONFIRMED &&
        gripper_stopped_.load(std::memory_order_acquire)) {
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

    auto result = sms_detector_.update(tau_ext_norm);

    if (result.detected &&
        result.event.new_state == sms_cusum::GraspState::CONTACT) {
      contact_latched_ = true;
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
                                            double tau_ext_norm,
                                            double support_force,
                                            const GripperData& gripper_snapshot) {
    if (!fr_grasping_phase_initialized_) {
      fr_phase_start_time_ = time;
      fr_grasping_phase_initialized_ = true;
      sms_detector_.enter_grasping();
    }

    double elapsed = (time - fr_phase_start_time_).toSec();

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
        if (elapsed < kGraspSettleDelay) {
          return;
        }
        if (!fr_grasp_cmd_seen_executing_) {
          if (cmd_executing_.load(std::memory_order_relaxed)) {
            fr_grasp_cmd_seen_executing_ = true;
          } else if (cmd_gen_.load(std::memory_order_acquire) > fr_expected_cmd_gen_) {
            fr_grasp_cmd_seen_executing_ = true;
          }
          return;
        }
        fr_ramp_phase_ = RampPhase::WAITING_EXECUTION;
        break;
      }

      case RampPhase::WAITING_EXECUTION: {
        if (cmd_executing_.load(std::memory_order_acquire)) {
          return;
        }
        if (!cmd_success_.load(std::memory_order_relaxed)) {
          current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
          publishStateLabel("FAILED");
          logStateTransition("FAILED", "Grasp command failed (hardware rejected)");
          ROS_WARN("    GRASP_%d: F=%.1f N  grasp() returned false  width=%.4f m -> FAILED",
                  fr_iteration_ + 1, fr_f_current_, gripper_snapshot.width);
          return;
        }
        fr_ramp_phase_ = RampPhase::SETTLING;
        fr_ramp_step_start_time_ = time;
        ROS_DEBUG("    GRASP_%d: grasp complete at F=%.1f N, settling for %.2fs",
                fr_iteration_ + 1, fr_f_current_, rt_fr_grasp_settle_time_);
        break;
      }

      case RampPhase::SETTLING: {
        if ((time - fr_ramp_step_start_time_).toSec() < rt_fr_grasp_settle_time_) {
          return;
        }
        fr_ramp_phase_ = RampPhase::HOLDING;
        fr_ramp_step_start_time_ = time;

        fr_holding_elapsed_ = 0.0;

        ROS_DEBUG("    GRASP_%d: holding at F=%.1f N for %.2fs",
                fr_iteration_ + 1, fr_f_current_, rt_fr_grasp_force_hold_time_);
        break;
      }

      case RampPhase::HOLDING: {
        double hold_elapsed = (time - fr_ramp_step_start_time_).toSec();

        if (hold_elapsed >= rt_fr_grasp_force_hold_time_ / 2.0) {
          sms_detector_.update(tau_ext_norm);
        }

        if (hold_elapsed < rt_fr_grasp_force_hold_time_) {
          return;
        }

        contact_width_.store(gripper_snapshot.width, std::memory_order_relaxed);

        ROS_DEBUG("    GRASP_%d: hold complete  F=%.1f N  width=%.4f m (updated)",
                fr_iteration_ + 1, fr_f_current_, gripper_snapshot.width);

        fr_ramp_phase_ = RampPhase::STEP_COMPLETE;
        break;
      }

      case RampPhase::STEP_COMPLETE: {
        auto sg_result = sms_detector_.finalize_grasp_step();
        bool fixed_mode = rt_fr_fixed_grasp_steps_ > 0;
        bool fixed_reached = fixed_mode &&
                             ((fr_iteration_ + 1) >= rt_fr_fixed_grasp_steps_);

        if (sg_result.detected) {
          ROS_INFO("    GRASP_%d: SECURE_GRASP detected: d_mu=%.4f std=%.4f%s",
                  fr_iteration_ + 1,
                  sg_result.event.baseline_mean,
                  sg_result.event.baseline_sigma,
                  fixed_mode ? " (ignored — fixed_grasp_steps active)" : "");
        }
        if (fixed_reached) {
          ROS_INFO("    GRASP_%d: fixed_grasp_steps=%d reached — declaring secure",
                  fr_iteration_ + 1, rt_fr_fixed_grasp_steps_);
        }

        // When fixed_grasp_steps is active, only the step count decides;
        // the algorithm's secure grasp detection is ignored.
        bool secure = fixed_mode ? fixed_reached : sg_result.detected;
        bool reached_f_max = fr_f_current_ >= rt_fr_f_max_;

        if (reached_f_max && !secure) {
          // Maximum force reached without secure grasp — fail
          current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
          publishStateLabel("FAILED");
          logStateTransition("FAILED", "f_max reached without secure grasp");
          ROS_WARN("    GRASP_%d: FAILED — f_max=%.1f N reached without secure grasp",
                  fr_iteration_ + 1, rt_fr_f_max_);
        } else if (secure) {
          // Secure grasp confirmed — proceed to uplift
          uplift_start_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
          uplift_z_start_ = uplift_start_pose_[14];
          uplift_elapsed_ = 0.0;
          rt_uplift_distance_ = rt_fr_uplift_distance_;
          rt_uplift_duration_ = rt_fr_uplift_distance_ / rt_fr_lift_speed_;
          uplift_active_.store(true, std::memory_order_relaxed);

          current_state_.store(GraspState::UPLIFT, std::memory_order_relaxed);
          publishStateLabel("UPLIFT");
          logStateTransition("UPLIFT", "Secure grasp confirmed — micro-lift starting");
          ROS_INFO("    final_iter=%d  F=%.1f N  width=%.4f m  "
                  "distance=%.4f m  duration=%.2f s",
                  fr_iteration_ + 1, fr_f_current_, gripper_snapshot.width,
                  rt_fr_uplift_distance_, rt_uplift_duration_);
        } else {
          fr_f_current_ = std::min(fr_f_current_ + rt_fr_f_step_, rt_fr_f_max_);
          fr_iteration_++;

          double w = contact_width_.load(std::memory_order_relaxed);
          requestDeferredGrasp(w, rt_fr_grasp_speed_, fr_f_current_, rt_fr_epsilon_);

          fr_phase_start_time_ = time;
          fr_grasping_phase_initialized_ = true;
          fr_grasp_cmd_seen_executing_ = false;
          fr_expected_cmd_gen_ = cmd_gen_.load(std::memory_order_relaxed);
          fr_ramp_phase_ = RampPhase::COMMAND_SENT;

          // Begin next secure grasp step
          sms_detector_.begin_grasp_step(fr_iteration_);

          char label[16];
          std::snprintf(label, sizeof(label), "GRASP_%d", fr_iteration_ + 1);
          publishStateLabel(label);
          logStateTransition(label, "Force ramp step");
          ROS_DEBUG("    iter=%d  F=%.1f N  width=%.4f m", fr_iteration_ + 1, fr_f_current_, w);
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
      return;
    }

    fr_phase_start_time_ = time;
    fr_early_sum_ = 0.0;
    fr_early_count_ = 0;
    fr_late_sum_ = 0.0;
    fr_late_count_ = 0;
    current_state_.store(GraspState::EVALUATE, std::memory_order_relaxed);
    publishStateLabel("EVALUATE");
    logStateTransition("EVALUATE", "Hold + slip score evaluation");
    ROS_DEBUG("    early=%.2fs  late=%.2fs  (total hold=%.2fs)",
            rt_fr_uplift_hold_ / 2.0, rt_fr_uplift_hold_ / 2.0,
            rt_fr_uplift_hold_);
  }

  void KittingStateController::tickEvaluate(const ros::Time& time,
                                            double /* tau_ext_norm */,
                                            double support_force,
                                            const GripperData& gripper_snapshot) {
    const double kEarlyEnd = rt_fr_uplift_hold_ / 2.0;  // W_hold_early [0, half)
    const double kLateEnd  = rt_fr_uplift_hold_;         // W_hold_late  [half, hold)
    constexpr double kEpsilon = 1e-6;

    double elapsed = (time - fr_phase_start_time_).toSec();

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

    double Fn_early = (fr_early_count_ > 0) ? fr_early_sum_ / fr_early_count_ : 0.0;
    double Fn_late  = (fr_late_count_ > 0)  ? fr_late_sum_ / fr_late_count_   : 0.0;

    double deltaF = Fn_early - fn_baseline_;
    double load_thresh = rt_fr_load_transfer_min_;
    bool load_transferred = deltaF > load_thresh;

    double dF = (Fn_early - Fn_late) / std::max(Fn_early, kEpsilon);
    bool drop_ok = (dF <= rt_fr_slip_drop_thresh_);

    bool is_slipping = !(load_transferred && drop_ok);

    ROS_DEBUG("  [EVAL] Gate 1: deltaF=%.3f (Fn_early=%.3f - Fn_baseline=%.3f) threshold=%.3f %s",
              deltaF, Fn_early, fn_baseline_, load_thresh, load_transferred ? "PASS" : "FAIL");
    ROS_DEBUG("  [EVAL] Gate 2: dF=%.1f%% threshold=%.0f%% %s",
              dF * 100.0, rt_fr_slip_drop_thresh_ * 100.0, drop_ok ? "PASS" : "FAIL");
    ROS_INFO("  [EVAL] %s  (deltaF=%.3f  dF=%.1f%%)",
             is_slipping ? "SLIPPING" : "SECURE", deltaF, dF * 100.0);

    if (!is_slipping) {
      accumulated_uplift_ += rt_fr_uplift_distance_;
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

# KittingGripperCommand - Runtime command with per-object parameters
#
# Published by the user on /kitting_controller/state_cmd to trigger BASELINE,
# CLOSING, or GRASPING with optional per-object parameters.
#
# Any float64 parameter left at 0.0 (the default) will fall back to the value
# loaded from the controller's YAML config at startup.  This lets you override
# only the parameters that differ for a particular object.
#
# Example: start baseline with gripper open
#   command: "BASELINE"
#   open_gripper: true
#
# Example: start grasping with custom force range
#   command: "GRASPING"
#   f_min: 5.0
#   f_max: 20.0
#   (grasp width auto-uses the width captured at CONTACT)
#
# Example: automatic full sequence with 3s delay
#   command: "AUTO"
#   open_gripper: true
#   auto_delay: 3.0

# Command: "BASELINE", "CLOSING", "GRASPING", or "AUTO"
string command

# --- BASELINE parameters (used when command == "BASELINE") ---
bool open_gripper           # If true, open gripper before baseline collection. Also auto-enabled when record:=true (default: false)
float64 open_width          # [m] Width to open to (0 = max_width from firmware). Open is deferred until downlift completes

# --- CLOSING parameters (used when command == "CLOSING") ---
float64 closing_width    # [m]   Target width for MoveAction  (0 = use default)
float64 closing_speed    # [m/s] Speed for MoveAction         (0 = use default)
float64 contact_torque_thresh    # [Nm] tau_ext_norm drop threshold for contact detection (0 = use YAML default)
float64 contact_debounce_time    # [s] Sustained drop duration before triggering (0 = use YAML default 0.05)

# --- Force ramp parameters (used when command == "GRASPING") ---
# grasp_width is always the contact_width captured at CONTACT (not configurable).
# All parameters are per-command overridable. 0 = use YAML default.
# The controller runs a contact-only force ramp:
#   GRASP_1 → ... → GRASP_N → UPLIFT → EVALUATE → SUCCESS/FAILED (secure grasp required; f_max without it → FAILED)
# Each ramp step: grasp at force → settle → hold/log → update contact_width
float64 f_min                # [N]   Starting grasp force (0 = use default 3.0)
float64 f_step               # [N]   Force increment per step (0 = use default 3.0)
float64 f_max                # [N]   Maximum force for the final step (0 = use default 70.0)
float64 grasp_force_hold_time   # [s] Hold at each force step before advancing (0 = use default 2.0)
float64 grasp_settle_time       # [s] Settle after each grasp completes (0 = use default 0.5)
float64 fr_uplift_distance   # [m]   Micro-uplift distance after final ramp step (0 = use default 0.010)
float64 fr_lift_speed        # [m/s] Lift speed for final UPLIFT (0 = use default 0.01)
float64 fr_uplift_hold       # [s]   Evaluation hold at top; early + late windows (0 = use default 1.0)
float64 fr_grasp_speed       # [m/s] Gripper speed for ramp grasps (0 = use default 0.02)
float64 fr_epsilon           # [m]   Epsilon for ramp grasps, same for inner/outer (0 = use default 0.008)
float64 fr_slip_drop_thresh     # DF_TH: max allowed relative support force drop (0 = use default 0.15)
float64 fr_slip_width_thresh    # [m] W_TH: max allowed jaw widening P95-P5 (0 = use default 0.0005)
float64 fr_load_transfer_min    # [N] Min load transfer threshold floor (0 = use default 1.5)
string  secure_grasp_mode       # Secure grasp detection: "ewma", "slope", or "both" ("" = use YAML default)

# --- AUTO parameters (used when command == "AUTO") ---
# AUTO runs the full sequence: BASELINE → CLOSING → GRASPING automatically.
# All BASELINE, CLOSING, and GRASPING parameters above are forwarded to each stage.
float64 auto_delay           # [s]   Delay between transitions (0 = default 5.0)

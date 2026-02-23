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

# Command: "BASELINE", "CLOSING", or "GRASPING"
string command

# --- BASELINE parameters (used when command == "BASELINE") ---
bool open_gripper           # If true, open gripper before baseline collection (default: false)
float64 open_width          # [m] Width to open to (0 = max_width from firmware). Only used if open_gripper is true

# --- CLOSING parameters (used when command == "CLOSING") ---
float64 closing_width    # [m]   Target width for MoveAction  (0 = use default)
float64 closing_speed    # [m/s] Speed for MoveAction         (0 = use default)

# --- Force ramp parameters (used when command == "GRASPING") ---
# grasp_width is always the contact_width captured at CONTACT (not configurable).
# All parameters are per-command overridable. 0 = use YAML default.
# The controller runs an automated force ramp loop:
#   GRASPING → UPLIFT → EVALUATE → SUCCESS
#   On slip: DOWNLIFT → SETTLING → GRASPING (retry with F += f_step)
float64 f_min                # [N]   Starting grasp force (0 = use default 3.0)
float64 f_step               # [N]   Force increment per iteration (0 = use default 2.0)
float64 f_max                # [N]   Maximum force, FAILED if exceeded (0 = use default 15.0)
float64 fr_uplift_distance   # [m]   Micro-uplift distance (0 = use default 0.003)
float64 fr_lift_speed        # [m/s] Lift speed for UPLIFT and DOWNLIFT (0 = use default 0.01)
float64 fr_uplift_hold       # [s]   Hold at top; evaluation runs immediately after (0 = use default 0.5)
float64 fr_grasp_speed       # [m/s] Gripper speed for ramp grasps (0 = use default 0.02)
float64 fr_epsilon           # [m]   Epsilon for ramp grasps, same for inner/outer (0 = use default 0.008)
float64 fr_stabilization     # [s]   Post-grasp/post-downlift settle time (0 = use default 0.3)
float64 fr_slip_tau_drop     # Fractional tau_ext_norm drop threshold (0 = use default 0.20)
float64 fr_slip_width_change # [m]   Width change threshold for slip (0 = use default 0.001)

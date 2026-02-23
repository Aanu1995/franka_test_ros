# KittingGripperCommand - Runtime command with per-object parameters
#
# Published by the user on /kitting_controller/state_cmd to trigger BASELINE,
# CLOSING, SECURE_GRASP, or UPLIFT with optional per-object parameters.
#
# Any float64 parameter left at 0.0 (the default) will fall back to the value
# loaded from the controller's YAML config at startup.  This lets you override
# only the parameters that differ for a particular object.
#
# Example: start baseline with gripper open
#   command: "BASELINE"
#   open_gripper: true
#
# Example: grip using contact width with unified epsilon
#   command: "SECURE_GRASP"
#   epsilon: 0.008
#   grasp_force: 30.0
#   (grasp_width auto-uses the width captured at CONTACT)
#
# Example: micro-lift 5mm over 2 seconds
#   command: "UPLIFT"
#   uplift_distance: 0.005
#   uplift_duration: 2.0

# Command: "BASELINE", "CLOSING", "SECURE_GRASP", or "UPLIFT"
string command

# --- BASELINE parameters (used when command == "BASELINE") ---
bool open_gripper           # If true, open gripper before baseline collection (default: false)
float64 open_width          # [m] Width to open to (0 = max_width from firmware). Only used if open_gripper is true

# --- CLOSING parameters (used when command == "CLOSING") ---
float64 closing_width    # [m]   Target width for MoveAction  (0 = use default)
float64 closing_speed    # [m/s] Speed for MoveAction         (0 = use default)

# --- SECURE_GRASP parameters (used when command == "SECURE_GRASP") ---
float64 grasp_width      # [m]   Target width for GraspAction (0 = use contact width, or default)
float64 epsilon           # [m]   Epsilon for GraspAction — sets both inner and outer (0 = use individual fields or default)
float64 epsilon_inner    # [m]   Inner epsilon for GraspAction (0 = use default). Overridden by epsilon if set
float64 epsilon_outer    # [m]   Outer epsilon for GraspAction (0 = use default). Overridden by epsilon if set
float64 grasp_speed      # [m/s] Speed for GraspAction        (0 = use default)
float64 grasp_force      # [N]   Force for GraspAction        (0 = use default)

# --- UPLIFT parameters (used when command == "UPLIFT") ---
float64 uplift_distance  # [m]   Upward displacement (0 = use default, max 0.01m)
float64 uplift_duration  # [s]   Duration of cosine-smoothed micro-lift (0 = use default)

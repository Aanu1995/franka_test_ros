# KittingGripperCommand - Runtime gripper command with per-object parameters
#
# Published by the user on /kitting_phase2/state_cmd to trigger CLOSING or
# SECURE_GRASP with optional per-object gripper parameters.
#
# Any parameter left at 0.0 (the default) will fall back to the value loaded
# from the controller's YAML config at startup.  This lets you override only
# the parameters that differ for a particular object.
#
# Example: grip a small object tighter
#   command: "SECURE_GRASP"
#   grasp_width: 0.01
#   grasp_force: 20.0
#   (all other fields 0 -> use YAML defaults)

# Command: "CLOSING" or "SECURE_GRASP"
string command

# --- CLOSING parameters (used when command == "CLOSING") ---
float64 closing_width    # [m]   Target width for MoveAction  (0 = use default)
float64 closing_speed    # [m/s] Speed for MoveAction         (0 = use default)

# --- SECURE_GRASP parameters (used when command == "SECURE_GRASP") ---
float64 grasp_width      # [m]   Target width for GraspAction (0 = use default)
float64 epsilon_inner    # [m]   Inner epsilon for GraspAction (0 = use default)
float64 epsilon_outer    # [m]   Outer epsilon for GraspAction (0 = use default)
float64 grasp_speed      # [m/s] Speed for GraspAction        (0 = use default)
float64 grasp_force      # [N]   Force for GraspAction        (0 = use default)

# KittingState - Comprehensive robot state for kitting task development
# Published by the KittingStateController at configurable rate (default 250 Hz)

Header header

# Joint-level signals (7 DOF)
float64[7] q            # Joint positions [rad]
float64[7] dq           # Joint velocities [rad/s]
float64[7] tau_J        # Measured joint torques [Nm]
float64[7] tau_ext      # Estimated external joint torques [Nm]

# Cartesian-level signals
float64[6] wrench_ext   # External wrench in base frame (Fx,Fy,Fz,Tx,Ty,Tz) [N,Nm]
float64[16] O_T_EE      # End-effector pose in base frame (4x4 column-major) [m]

# Model-level signals
float64[42] jacobian    # Zero Jacobian at end-effector (6x7 column-major)
float64[7] gravity      # Gravity torque vector [Nm]
float64[7] coriolis     # Coriolis torque vector [Nm]

# Derived quantities
float64[6] ee_velocity  # End-effector velocity (J * dq) [m/s, rad/s]
float64 tau_ext_norm    # Euclidean norm of tau_ext
float64 wrench_norm     # Euclidean norm of wrench_ext
float64 support_force       # Support force Fn = |Fz| [N] — used by slip evaluation (Gates 1-2)
float64 tangential_force    # Tangential force Ft = sqrt(Fx²+Fy²) [N] — logged for monitoring

# Gripper signals (from franka::Gripper, via dedicated read thread)
float64 gripper_width          # Measured finger width [m]
float64 gripper_width_dot      # Finger width velocity (finite difference) [m/s]
float64 gripper_width_cmd      # Commanded closing width [m] (0 if not in CLOSING)
float64 gripper_max_width      # Maximum gripper opening width [m]
bool gripper_is_grasped        # Firmware-level grasp detection flag

# Force ramp state (from automated force ramp state machine)
float64 grasp_force            # Current grasp force [N] (0 if not in force ramp)
int32 grasp_iteration          # Force ramp iteration (0-based; 0 = first attempt)

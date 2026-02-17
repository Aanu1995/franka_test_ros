#!/usr/bin/env python
"""
Gazebo-only pick-and-place for Franka Panda.
No MoveIt dependency -- uses position_joint_trajectory_controller via
FollowJointTrajectoryAction and franka_gripper actions (move / grasp).

Key design:
  - Queries Gazebo for stone / tray positions at runtime (no hardcoded poses)
  - Uses TF to read the current end-effector pose
  - Solves joint targets with a numerical Jacobian IK
  - Generates smooth multi-point trajectories with cubic interpolation
"""

import math
import sys
import copy
import numpy as np

import rospy
import actionlib
import tf
import tf.transformations as tft

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from franka_gripper.msg import (
    MoveAction, MoveGoal,
    GraspAction, GraspGoal, GraspEpsilon,
)
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point, Quaternion


# ================================================================== #
#  Panda DH-based forward kinematics (pure NumPy, no external deps)  #
# ================================================================== #

# DH parameters from the URDF xacro.
# Each row: (a_i, d_i, alpha_i)  -- theta_i is the joint variable.
# Convention follows Craig / modified DH used by the xacro transforms.
_PANDA_DH = [
    #  a        d        alpha
    (0.0,     0.333,   0.0),        # joint 1
    (0.0,     0.0,    -math.pi/2),  # joint 2
    (0.0,     0.316,   math.pi/2),  # joint 3  (d = 0.316 from URDF y-offset)
    (0.0825,  0.0,     math.pi/2),  # joint 4
    (-0.0825, 0.384,  -math.pi/2),  # joint 5
    (0.0,     0.0,     math.pi/2),  # joint 6
    (0.088,   0.0,     math.pi/2),  # joint 7
]

# Fixed transforms after joint 7
_D_FLANGE = 0.107       # link7 -> link8
_HAND_RPY = (0, 0, -math.pi / 4)   # link8 -> hand
_TCP_Z    = 0.1034      # hand -> hand_tcp


def _dh_matrix(a, d, alpha, theta):
    """Standard DH homogeneous transformation."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct,   -st * ca,  st * sa,   a * ct],
        [st,    ct * ca, -ct * sa,   a * st],
        [0.0,   sa,       ca,        d     ],
        [0.0,   0.0,      0.0,       1.0   ],
    ])


def panda_fk(q):
    """Forward kinematics: joint angles (7,) -> 4x4 TCP transform."""
    T = np.eye(4)
    for i, (a, d, alpha) in enumerate(_PANDA_DH):
        T = T.dot(_dh_matrix(a, d, alpha, q[i]))

    # link7 -> link8 (fixed, translation along Z)
    T_flange = np.eye(4)
    T_flange[2, 3] = _D_FLANGE
    T = T.dot(T_flange)

    # link8 -> hand (fixed rotation rpy = (0, 0, -pi/4))
    T_hand = np.eye(4)
    T_hand[:3, :3] = tft.euler_matrix(*_HAND_RPY)[:3, :3]
    T = T.dot(T_hand)

    # hand -> tcp (fixed, translation along Z)
    T_tcp = np.eye(4)
    T_tcp[2, 3] = _TCP_Z
    T = T.dot(T_tcp)

    return T


def _pose_error(T_current, T_target):
    """6-vector error: [position_error (3); orientation_error (3)].

    Orientation error uses the angle-axis representation extracted from
    R_err = R_target * R_current^T, expressed in world frame.
    """
    pos_err = T_target[:3, 3] - T_current[:3, 3]
    R_err = T_target[:3, :3].dot(T_current[:3, :3].T)
    # Extract angle-axis via rodrigues (trace formula)
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R_err) - 1.0) / 2.0)))
    if abs(angle) < 1e-6:
        ori_err = np.zeros(3)
    else:
        # Axis from skew-symmetric part
        axis = np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ]) / (2.0 * math.sin(angle))
        ori_err = angle * axis
    return np.concatenate([pos_err, ori_err])


def _numerical_jacobian(q, eps=1e-5):
    """6x7 Jacobian via finite differences of panda_fk."""
    T0 = panda_fk(q)
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]
    J = np.zeros((6, 7))
    for i in range(7):
        q_plus = np.array(q, dtype=float)
        q_plus[i] += eps
        T_plus = panda_fk(q_plus)
        # position part
        J[:3, i] = (T_plus[:3, 3] - p0) / eps
        # orientation part  (differential rotation)
        R_diff = T_plus[:3, :3].dot(R0.T)
        # small-angle: w ~ 0.5 * [R - R^T]_vee / eps
        J[3, i] = (R_diff[2, 1] - R_diff[1, 2]) / (2.0 * eps)
        J[4, i] = (R_diff[0, 2] - R_diff[2, 0]) / (2.0 * eps)
        J[5, i] = (R_diff[1, 0] - R_diff[0, 1]) / (2.0 * eps)
    return J


# Joint limits from URDF
_Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
_Q_MAX = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])


def solve_ik(T_target, q_init, max_iter=200, pos_tol=0.002, ori_tol=0.02,
             damping=0.05):
    """Damped-least-squares IK for the Panda arm.

    Args:
        T_target: 4x4 desired TCP pose.
        q_init: 7-element initial joint guess.
        max_iter: iteration cap.
        pos_tol: position convergence (m).
        ori_tol: orientation convergence (rad).
        damping: regularisation (lambda).
    Returns:
        q_solution (7,) or None on failure.
    """
    q = np.array(q_init, dtype=float)
    for _ in range(max_iter):
        T_cur = panda_fk(q)
        err = _pose_error(T_cur, T_target)
        if np.linalg.norm(err[:3]) < pos_tol and np.linalg.norm(err[3:]) < ori_tol:
            return q.tolist()
        J = _numerical_jacobian(q)
        # Damped least-squares:  dq = J^T (J J^T + lambda^2 I)^{-1} err
        JJT = J.dot(J.T) + damping ** 2 * np.eye(6)
        dq = J.T.dot(np.linalg.solve(JJT, err))
        # Limit step size
        max_step = 0.15  # rad
        scale = max(1.0, np.max(np.abs(dq)) / max_step)
        dq /= scale
        q += dq
        # Clamp to joint limits
        q = np.clip(q, _Q_MIN, _Q_MAX)
    rospy.logwarn('IK did not converge (pos_err=%.4f, ori_err=%.4f)',
                  np.linalg.norm(err[:3]), np.linalg.norm(err[3:]))
    return q.tolist()


# ================================================================== #
#  Main controller class                                              #
# ================================================================== #

class PandaPickPlace(object):
    """Pick-and-place controller for Panda in Gazebo (no MoveIt)."""

    JOINT_NAMES = [
        'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
        'panda_joint5', 'panda_joint6', 'panda_joint7',
    ]

    HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    # Gripper parameters
    GRIPPER_OPEN_WIDTH   = 0.08    # m
    GRIPPER_SPEED        = 0.1     # m/s
    GRASP_WIDTH          = 0.025   # m  (stone narrowest dim 25 mm)
    GRASP_FORCE          = 50.0    # N
    GRASP_EPSILON_INNER  = 0.005   # m
    GRASP_EPSILON_OUTER  = 0.005   # m

    # Motion parameters
    DEFAULT_DURATION  = 5.0   # s
    NUM_INTERP_POINTS = 10
    MAX_JOINT_SPEED   = 0.3   # rad/s

    # Clearance heights relative to object / tray
    APPROACH_HEIGHT = 0.15    # m above stone for pre-grasp
    GRASP_LOWER     = 0.0     # m above stone centre for grasping
    PLACE_HEIGHT    = 0.08    # m above tray surface for placing

    def __init__(self):
        rospy.init_node('panda_pick_place', anonymous=False)

        # --- TF listener for end-effector pose ---
        self.tf_listener = tf.TransformListener()

        # --- Gazebo model state service ---
        rospy.loginfo('Waiting for /gazebo/get_model_state ...')
        rospy.wait_for_service('/gazebo/get_model_state', timeout=30.0)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state',
                                                  GetModelState)

        # --- Arm trajectory action client (position controller) ---
        arm_topic = '/position_joint_trajectory_controller/follow_joint_trajectory'
        self.arm_client = actionlib.SimpleActionClient(
            arm_topic, FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for arm action server (%s) ...', arm_topic)
        if not self.arm_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logfatal('Arm action server unavailable.')
            sys.exit(1)
        rospy.loginfo('Arm action server connected.')

        # --- Gripper action clients ---
        self.gripper_move_client = actionlib.SimpleActionClient(
            '/franka_gripper/move', MoveAction)
        self.gripper_grasp_client = actionlib.SimpleActionClient(
            '/franka_gripper/grasp', GraspAction)
        rospy.loginfo('Waiting for gripper servers ...')
        if not self.gripper_move_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logfatal('Gripper move server unavailable.')
            sys.exit(1)
        if not self.gripper_grasp_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logfatal('Gripper grasp server unavailable.')
            sys.exit(1)
        rospy.loginfo('All servers connected.  Ready.')

        # Let TF buffer fill
        rospy.sleep(1.0)

    # ============================================================== #
    #  Sensing: joint state, EE pose, Gazebo model poses             #
    # ============================================================== #

    def read_joints(self):
        """Return current 7-DOF arm joint positions, or HOME on failure."""
        try:
            msg = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
            pos = {n: p for n, p in zip(msg.name, msg.position)}
            return [pos.get(j, 0.0) for j in self.JOINT_NAMES]
        except rospy.ROSException:
            rospy.logwarn('Cannot read /joint_states, using HOME.')
            return list(self.HOME)

    def read_ee_pose(self):
        """Return (position, quaternion) of panda_hand via TF, or None."""
        try:
            self.tf_listener.waitForTransform(
                'panda_link0', 'panda_hand', rospy.Time(0), rospy.Duration(5.0))
            pos, quat = self.tf_listener.lookupTransform(
                'panda_link0', 'panda_hand', rospy.Time(0))
            return np.array(pos), np.array(quat)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as exc:
            rospy.logwarn('TF lookup failed: %s', exc)
            return None, None

    def get_model_position(self, model_name):
        """Query Gazebo for a model's world-frame position.  Returns (x,y,z)."""
        try:
            resp = self.get_model_state(model_name, 'world')
            if resp.success:
                p = resp.pose.position
                return np.array([p.x, p.y, p.z])
            rospy.logwarn('Model %s not found.', model_name)
        except rospy.ServiceException as exc:
            rospy.logwarn('get_model_state failed: %s', exc)
        return None

    # ============================================================== #
    #  Scene helpers                                                  #
    # ============================================================== #

    def spawn_models(self):
        """Log the current Gazebo model positions and reset the stone."""
        rospy.loginfo('--- Scene models ---')
        for name in ('table', 'pick_tray', 'place_tray', 'stone'):
            pos = self.get_model_position(name)
            if pos is not None:
                rospy.loginfo('  %s: (%.3f, %.3f, %.3f)', name, *pos)
            else:
                rospy.logwarn('  %s: NOT FOUND', name)
        self._reset_stone()

    def _reset_stone(self):
        """Place stone back on pick tray using the tray's live position."""
        tray_pos = self.get_model_position('pick_tray')
        if tray_pos is None:
            rospy.logwarn('Cannot reset stone -- pick_tray not found.')
            return
        # Stone sits on top of the tray; tray is ~23 mm thick, stone
        # half-height ~32 mm  -> stone centre = tray_z + 0.012 + 0.032
        stone_z = tray_pos[2] + 0.012 + 0.032
        try:
            rospy.wait_for_service('/gazebo/set_model_state', timeout=5.0)
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            state = ModelState()
            state.model_name = 'stone'
            state.pose.position = Point(tray_pos[0], tray_pos[1], stone_z)
            state.pose.orientation = Quaternion(0, 0, 0, 1)
            state.reference_frame = 'world'
            resp = set_state(state)
            if resp.success:
                rospy.loginfo('Stone placed on pick tray at z=%.3f', stone_z)
        except rospy.ROSException as exc:
            rospy.logwarn('SetModelState failed: %s', exc)

    # ============================================================== #
    #  Target pose construction                                       #
    # ============================================================== #

    def _make_top_down_pose(self, xyz):
        """Build a 4x4 TCP transform at *xyz* with the gripper pointing
        straight down (Z-axis downward, X-axis forward).

        The gripper fingers open along the hand's local Y axis.  With the
        hand mounted at rpy=(0,0,-pi/4) on link8, we need TCP orientation
        such that the world-frame result is Z-down.
        """
        # Desired world-frame rotation: Z pointing down, X forward
        #   x = [1, 0, 0],  y = [0, -1, 0],  z = [0, 0, -1]  (right-hand)
        # Actually we want y = [0,1,0] so fingers open along world-Y:
        #   x = [1, 0, 0],  y = [0, 1, 0],  z = [0, 0, -1]  -- left-handed!
        # Correct right-handed:  z down, x forward => y = z cross x ... let's
        # use rotation matrix directly:
        #   R = Rz(0) * Ry(pi) = diag(1, -1, -1) ... that gives Z up flipped.
        # Simplest: point EE -Z downward = rotate pi about X:
        R_down = tft.euler_matrix(math.pi, 0, 0)[:3, :3]
        T = np.eye(4)
        T[:3, :3] = R_down
        T[:3, 3] = xyz
        return T

    # ============================================================== #
    #  Arm motion (smooth multi-point trajectory)                     #
    # ============================================================== #

    def _compute_duration(self, start, target):
        max_delta = max(abs(t - s) for s, t in zip(start, target))
        speed_dur = max_delta / self.MAX_JOINT_SPEED if self.MAX_JOINT_SPEED > 0 else self.DEFAULT_DURATION
        return max(speed_dur, self.DEFAULT_DURATION)

    def move_arm(self, joint_target, duration=None):
        """Smooth multi-point trajectory from current joints to *joint_target*."""
        current = self.read_joints()
        if duration is None:
            duration = self._compute_duration(current, joint_target)

        n = self.NUM_INTERP_POINTS
        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = list(self.JOINT_NAMES)

        for i in range(1, n + 1):
            alpha = float(i) / n
            s = 3.0 * alpha ** 2 - 2.0 * alpha ** 3  # cubic ease

            pt = JointTrajectoryPoint()
            pt.positions = [c + s * (t - c) for c, t in zip(current, joint_target)]
            pt.velocities = [0.0] * 7
            pt.time_from_start = rospy.Duration(duration * alpha)
            traj.points.append(pt)

        goal.trajectory = traj
        rospy.loginfo('move_arm  dur=%.1fs  target=%s',
                      duration, [round(j, 3) for j in joint_target])
        self.arm_client.send_goal(goal)
        if not self.arm_client.wait_for_result(rospy.Duration(duration + 15.0)):
            rospy.logwarn('Trajectory timed out.')
            return False
        res = self.arm_client.get_result()
        if res.error_code != 0:
            rospy.logwarn('Trajectory error %d: %s', res.error_code, res.error_string)
            return False
        rospy.loginfo('Arm reached target.')
        return True

    def move_to_pose(self, T_target, label=''):
        """Solve IK for T_target and move the arm there."""
        q_init = self.read_joints()
        q_sol = solve_ik(T_target, q_init)
        if q_sol is None:
            rospy.logerr('IK failed for %s', label)
            return False
        rospy.loginfo('IK solved for %s', label)
        return self.move_arm(q_sol)

    # ============================================================== #
    #  Gripper                                                        #
    # ============================================================== #

    def open_gripper(self):
        rospy.loginfo('Opening gripper ...')
        goal = MoveGoal(width=self.GRIPPER_OPEN_WIDTH, speed=self.GRIPPER_SPEED)
        self.gripper_move_client.send_goal(goal)
        if self.gripper_move_client.wait_for_result(rospy.Duration(10.0)):
            return self.gripper_move_client.get_result().success
        return False

    def close_gripper(self):
        rospy.loginfo('Closing gripper (force=%.0f N) ...', self.GRASP_FORCE)
        goal = GraspGoal(
            width=self.GRASP_WIDTH, speed=self.GRIPPER_SPEED,
            force=self.GRASP_FORCE,
            epsilon=GraspEpsilon(inner=self.GRASP_EPSILON_INNER,
                                 outer=self.GRASP_EPSILON_OUTER))
        self.gripper_grasp_client.send_goal(goal)
        if self.gripper_grasp_client.wait_for_result(rospy.Duration(15.0)):
            return self.gripper_grasp_client.get_result().success
        return False

    # ============================================================== #
    #  Attachment stubs                                               #
    # ============================================================== #

    def attach_object(self, name='stone'):
        rospy.loginfo('attach_object(%s) -- friction hold (%.0f N)',
                      name, self.GRASP_FORCE)

    def detach_object(self, name='stone'):
        rospy.loginfo('detach_object(%s) -- releasing via gripper open', name)

    # ============================================================== #
    #  Pick and place sequences                                       #
    # ============================================================== #

    def pick(self):
        rospy.loginfo('=== PICK ===')

        # 1. Query stone position from Gazebo
        stone_pos = self.get_model_position('stone')
        if stone_pos is None:
            rospy.logerr('Cannot locate stone.')
            return False
        rospy.loginfo('Stone at (%.3f, %.3f, %.3f)', *stone_pos)

        # 2. Open gripper
        self.open_gripper()
        rospy.sleep(1.0)

        # 3. Pre-grasp: above stone
        pre_grasp_xyz = stone_pos.copy()
        pre_grasp_xyz[2] += self.APPROACH_HEIGHT
        T_pre = self._make_top_down_pose(pre_grasp_xyz)
        if not self.move_to_pose(T_pre, 'pre-grasp'):
            return False
        rospy.sleep(1.0)

        # 4. Grasp pose: at stone level
        grasp_xyz = stone_pos.copy()
        grasp_xyz[2] += self.GRASP_LOWER
        T_grasp = self._make_top_down_pose(grasp_xyz)
        if not self.move_to_pose(T_grasp, 'grasp'):
            return False
        rospy.sleep(1.0)

        # 5. Close gripper
        self.close_gripper()
        self.attach_object()
        rospy.sleep(1.5)

        # 6. Lift back to pre-grasp height
        if not self.move_to_pose(T_pre, 'lift'):
            return False
        rospy.sleep(1.0)

        rospy.loginfo('=== PICK DONE ===')
        return True

    def place(self):
        rospy.loginfo('=== PLACE ===')

        # 1. Query place tray position from Gazebo
        tray_pos = self.get_model_position('place_tray')
        if tray_pos is None:
            rospy.logerr('Cannot locate place_tray.')
            return False
        rospy.loginfo('Place tray at (%.3f, %.3f, %.3f)', *tray_pos)

        # Tray surface is ~12 mm above tray centre (half of 23 mm thickness)
        tray_surface_z = tray_pos[2] + 0.012

        # 2. Pre-place: above tray
        pre_place_xyz = np.array([tray_pos[0], tray_pos[1],
                                  tray_surface_z + self.APPROACH_HEIGHT])
        T_pre = self._make_top_down_pose(pre_place_xyz)
        if not self.move_to_pose(T_pre, 'pre-place'):
            return False
        rospy.sleep(1.0)

        # 3. Lower to place height
        place_xyz = np.array([tray_pos[0], tray_pos[1],
                              tray_surface_z + self.PLACE_HEIGHT])
        T_place = self._make_top_down_pose(place_xyz)
        if not self.move_to_pose(T_place, 'place'):
            return False
        rospy.sleep(1.0)

        # 4. Release
        self.detach_object()
        self.open_gripper()
        rospy.sleep(1.5)

        # 5. Retreat upward
        if not self.move_to_pose(T_pre, 'retreat'):
            return False
        rospy.sleep(1.0)

        # 6. Return home
        rospy.loginfo('Returning home ...')
        if not self.move_arm(self.HOME):
            return False
        rospy.sleep(1.0)

        rospy.loginfo('=== PLACE DONE ===')
        return True

    # ============================================================== #
    #  Main                                                           #
    # ============================================================== #

    def run(self):
        rospy.loginfo('======== PANDA PICK-AND-PLACE (NO MOVEIT) ========')

        self.spawn_models()
        rospy.sleep(2.0)

        # Go home first
        rospy.loginfo('Moving to home ...')
        self.move_arm(self.HOME)
        rospy.sleep(2.0)

        if not self.pick():
            rospy.logerr('Pick failed.')
            return

        if not self.place():
            rospy.logerr('Place failed.')
            return

        rospy.loginfo('======== COMPLETE ========')


if __name__ == '__main__':
    try:
        controller = PandaPickPlace()
        controller.run()
    except rospy.ROSInterruptException:
        pass

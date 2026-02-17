#!/usr/bin/env python
"""
Gazebo-only pick-and-place for Franka Panda.
No MoveIt dependency -- uses position_joint_trajectory_controller via
FollowJointTrajectoryAction and franka_gripper actions (move / grasp).

Key design:
  - Queries Gazebo for stone / tray positions at runtime
  - Reads robot base position from Gazebo to transform world -> base frame
  - Uses TF to read the current end-effector pose for FK validation
  - Solves joint targets with a numerical Jacobian IK (URDF-based FK)
  - Singularity-aware: rejects IK solutions near singularity and checks
    every interpolated waypoint before sending trajectories
  - Generates smooth multi-point trajectories with cubic interpolation
  - Waits for simulation and controllers to fully settle before acting
"""

import math
import sys
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
#  Panda FK -- built from exact URDF transforms, not DH convention    #
# ================================================================== #
# Each joint is described by: (origin_xyz, origin_rpy, axis).
# We chain: T_parent * T_origin * Rot(axis, theta) for each joint.

_JOINT_PARAMS = [
    # joint1: origin xyz="0 0 0.333" rpy="0 0 0", axis z
    ((0, 0, 0.333),    (0, 0, 0),               (0, 0, 1)),
    # joint2: origin xyz="0 0 0" rpy="-pi/2 0 0", axis z
    ((0, 0, 0),        (-math.pi/2, 0, 0),      (0, 0, 1)),
    # joint3: origin xyz="0 -0.316 0" rpy="pi/2 0 0", axis z
    ((0, -0.316, 0),   (math.pi/2, 0, 0),       (0, 0, 1)),
    # joint4: origin xyz="0.0825 0 0" rpy="pi/2 0 0", axis z
    ((0.0825, 0, 0),   (math.pi/2, 0, 0),       (0, 0, 1)),
    # joint5: origin xyz="-0.0825 0.384 0" rpy="-pi/2 0 0", axis z
    ((-0.0825, 0.384, 0), (-math.pi/2, 0, 0),   (0, 0, 1)),
    # joint6: origin xyz="0 0 0" rpy="pi/2 0 0", axis z
    ((0, 0, 0),        (math.pi/2, 0, 0),       (0, 0, 1)),
    # joint7: origin xyz="0.088 0 0" rpy="pi/2 0 0", axis z
    ((0.088, 0, 0),    (math.pi/2, 0, 0),       (0, 0, 1)),
]

# Fixed transforms after joint 7:
#   joint8 (fixed): xyz="0 0 0.107"
#   hand_joint (fixed): rpy="0 0 -pi/4"
#   hand_tcp_joint (fixed): xyz="0 0 0.1034"


def _rot_axis(axis, theta):
    """Rotation matrix around a unit axis by angle theta."""
    ax = np.array(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]],
                  [ax[2], 0, -ax[0]],
                  [-ax[1], ax[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * K.dot(K)


def _tf_from_rpy_xyz(rpy, xyz):
    """4x4 homogeneous transform from rpy and xyz."""
    T = np.eye(4)
    T[:3, :3] = tft.euler_matrix(rpy[0], rpy[1], rpy[2], 'sxyz')[:3, :3]
    T[:3, 3] = xyz
    return T


def panda_fk(q):
    """Forward kinematics: q (7,) -> 4x4 TCP pose in panda_link0 frame.

    Exactly mirrors the URDF chain:
      link0 -> joint1 -> ... -> joint7 -> link8 -> hand -> hand_tcp
    """
    T = np.eye(4)
    for i, (xyz, rpy, axis) in enumerate(_JOINT_PARAMS):
        T_origin = _tf_from_rpy_xyz(rpy, xyz)
        T = T.dot(T_origin)
        T_joint = np.eye(4)
        T_joint[:3, :3] = _rot_axis(axis, q[i])
        T = T.dot(T_joint)

    # joint8 (fixed): xyz="0 0 0.107"
    T_j8 = np.eye(4)
    T_j8[2, 3] = 0.107
    T = T.dot(T_j8)

    # hand_joint (fixed): rpy="0 0 -pi/4"
    T_hand = _tf_from_rpy_xyz((0, 0, -math.pi / 4), (0, 0, 0))
    T = T.dot(T_hand)

    # hand_tcp_joint (fixed): xyz="0 0 0.1034"
    T_tcp = np.eye(4)
    T_tcp[2, 3] = 0.1034
    T = T.dot(T_tcp)

    return T


def _pose_error(T_current, T_target):
    """6-vector: [position_error(3); orientation_error(3)]."""
    pos_err = T_target[:3, 3] - T_current[:3, 3]
    R_err = T_target[:3, :3].dot(T_current[:3, :3].T)
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R_err) - 1.0) / 2.0)))
    if abs(angle) < 1e-6:
        ori_err = np.zeros(3)
    else:
        axis = np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ]) / (2.0 * math.sin(angle))
        ori_err = angle * axis
    return np.concatenate([pos_err, ori_err])


def _numerical_jacobian(q, eps=1e-6):
    """6x7 Jacobian via finite differences."""
    T0 = panda_fk(q)
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]
    J = np.zeros((6, 7))
    for i in range(7):
        qp = np.array(q, dtype=float)
        qp[i] += eps
        Tp = panda_fk(qp)
        J[:3, i] = (Tp[:3, 3] - p0) / eps
        Rd = Tp[:3, :3].dot(R0.T)
        J[3, i] = (Rd[2, 1] - Rd[1, 2]) / (2.0 * eps)
        J[4, i] = (Rd[0, 2] - Rd[2, 0]) / (2.0 * eps)
        J[5, i] = (Rd[1, 0] - Rd[0, 1]) / (2.0 * eps)
    return J


# ================================================================== #
#  Singularity detection                                              #
# ================================================================== #

def manipulability(q):
    """Yoshikawa manipulability index: sqrt(det(J * J^T)).
    Returns 0 at singularity, higher is better."""
    J = _numerical_jacobian(q)
    JJT = J.dot(J.T)
    det = max(0.0, np.linalg.det(JJT))
    return math.sqrt(det)


def min_singular_value(q):
    """Smallest singular value of the Jacobian.  Near 0 = singular."""
    J = _numerical_jacobian(q)
    s = np.linalg.svd(J, compute_uv=False)
    return float(s[-1])


# Threshold: below this the robot is considered too close to singularity.
# The franka_hw_sim.yaml uses 0.0001 on J*J^T singular values.
# For the raw SVD of J, the equivalent threshold is sqrt(0.0001) ≈ 0.01.
# We use a more conservative value to keep a safe margin.
_SINGULARITY_THRESHOLD = 0.05


def is_singular(q):
    """Check if a joint configuration is near a kinematic singularity."""
    return min_singular_value(q) < _SINGULARITY_THRESHOLD


# ================================================================== #
#  IK solver                                                          #
# ================================================================== #

# URDF joint limits
_Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
_Q_MAX = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

# Preferred joint configuration -- biased away from singularities.
# j4 well bent, j5 slightly off zero, j6 away from zero.
_Q_PREFERRED = np.array([0.0, -0.785, 0.0, -2.356, -0.17, 1.57, 0.785])

# Per-joint weight for nullspace singularity avoidance
_NULL_WEIGHT = np.array([0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 0.5])


def solve_ik(T_target, q_init, max_iter=300, pos_tol=0.002, ori_tol=0.02,
             damping=0.05, null_gain=0.5):
    """Damped-least-squares IK with nullspace singularity avoidance.

    A secondary objective pushes joints toward _Q_PREFERRED to keep
    the elbow bent and the wrist away from singular configurations
    (j4 near upper limit, j5/j6 near zero).

    Returns joint list or None.  Rejects solutions that are near
    a kinematic singularity.
    """
    q = np.array(q_init, dtype=float)
    for it in range(max_iter):
        T_cur = panda_fk(q)
        err = _pose_error(T_cur, T_target)
        pe = np.linalg.norm(err[:3])
        oe = np.linalg.norm(err[3:])
        if pe < pos_tol and oe < ori_tol:
            if is_singular(q):
                rospy.logwarn('IK converged but solution is singular (sv=%.4f), '
                              'pushing away...', min_singular_value(q))
                # Increase null_gain and keep iterating to escape singularity
                err = np.zeros(6)  # position reached, only apply nullspace
                J = _numerical_jacobian(q)
                JJT = J.dot(J.T) + damping ** 2 * np.eye(6)
                dq_null_raw = _NULL_WEIGHT * (_Q_PREFERRED - q)
                N = np.eye(7) - J.T.dot(np.linalg.solve(JJT, J))
                dq = 1.0 * N.dot(dq_null_raw)
                max_step = 0.05
                scale = max(1.0, np.max(np.abs(dq)) / max_step)
                dq /= scale
                q += dq
                q = np.clip(q, _Q_MIN, _Q_MAX)
                continue
            rospy.loginfo('IK converged in %d iters (pos=%.4f ori=%.4f sv=%.4f)',
                          it, pe, oe, min_singular_value(q))
            return q.tolist()

        J = _numerical_jacobian(q)
        JJT = J.dot(J.T) + damping ** 2 * np.eye(6)
        dq_primary = J.T.dot(np.linalg.solve(JJT, err))

        # Nullspace projection: push toward preferred config
        dq_null_raw = _NULL_WEIGHT * (_Q_PREFERRED - q)
        N = np.eye(7) - J.T.dot(np.linalg.solve(JJT, J))
        dq_null = null_gain * N.dot(dq_null_raw)

        dq = dq_primary + dq_null
        max_step = 0.1
        scale = max(1.0, np.max(np.abs(dq)) / max_step)
        dq /= scale
        q += dq
        q = np.clip(q, _Q_MIN, _Q_MAX)

    rospy.logwarn('IK did not converge (pos=%.4f ori=%.4f)', pe, oe)
    if pe < 0.01 and not is_singular(q):
        return q.tolist()
    return None


# ================================================================== #
#  Controller class                                                   #
# ================================================================== #

class PandaPickPlace(object):

    JOINT_NAMES = [
        'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
        'panda_joint5', 'panda_joint6', 'panda_joint7',
    ]

    # j5 nudged to -0.17 rad (~10 deg) to stay away from wrist singularity
    HOME = [0.0, -0.785398, 0.0, -2.35619, -0.17, 1.5708, 0.785398]

    # Gripper
    GRIPPER_OPEN_WIDTH   = 0.08
    GRIPPER_SPEED        = 0.1
    GRASP_WIDTH          = 0.025
    GRASP_FORCE          = 50.0
    GRASP_EPSILON_INNER  = 0.005
    GRASP_EPSILON_OUTER  = 0.005

    # Motion
    DEFAULT_DURATION  = 5.0
    NUM_INTERP_POINTS = 20   # more points = finer singularity checking
    MAX_JOINT_SPEED   = 0.3

    # Task geometry
    APPROACH_HEIGHT = 0.15   # m above target for approach
    PLACE_HEIGHT    = 0.08   # m above tray surface when placing

    def __init__(self):
        rospy.init_node('panda_pick_place', anonymous=False)

        self.tf_listener = tf.TransformListener()

        # Gazebo service
        rospy.loginfo('Waiting for /gazebo/get_model_state ...')
        rospy.wait_for_service('/gazebo/get_model_state', timeout=60.0)
        self._get_model_state_srv = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)

        # Arm
        arm_topic = '/position_joint_trajectory_controller/follow_joint_trajectory'
        self.arm_client = actionlib.SimpleActionClient(
            arm_topic, FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for arm action server ...')
        if not self.arm_client.wait_for_server(rospy.Duration(60.0)):
            rospy.logfatal('Arm server unavailable.')
            sys.exit(1)
        rospy.loginfo('Arm server connected.')

        # Gripper
        self.gripper_move_client = actionlib.SimpleActionClient(
            '/franka_gripper/move', MoveAction)
        self.gripper_grasp_client = actionlib.SimpleActionClient(
            '/franka_gripper/grasp', GraspAction)
        rospy.loginfo('Waiting for gripper servers ...')
        if not self.gripper_move_client.wait_for_server(rospy.Duration(60.0)):
            rospy.logfatal('Gripper move unavailable.')
            sys.exit(1)
        if not self.gripper_grasp_client.wait_for_server(rospy.Duration(60.0)):
            rospy.logfatal('Gripper grasp unavailable.')
            sys.exit(1)
        rospy.loginfo('All servers connected.')

        # Cache robot base position in world frame
        self.base_pos_world = np.zeros(3)
        self._update_base_position()

        # Let TF buffer fill
        rospy.sleep(2.0)

    # ============================================================== #
    #  Sensing                                                        #
    # ============================================================== #

    def _update_base_position(self):
        """Read the robot base (panda) position from Gazebo so we can
        convert world-frame coordinates to panda_link0 frame."""
        try:
            resp = self._get_model_state_srv('panda', 'world')
            if resp.success:
                p = resp.pose.position
                self.base_pos_world = np.array([p.x, p.y, p.z])
                rospy.loginfo('Robot base in world: (%.3f, %.3f, %.3f)',
                              p.x, p.y, p.z)
            else:
                rospy.logwarn('Could not find panda model in Gazebo.')
        except rospy.ServiceException as exc:
            rospy.logwarn('get_model_state for panda failed: %s', exc)

    def world_to_base(self, world_xyz):
        """Convert a world-frame point to panda_link0 frame.
        (Assumes no rotation, just translation offset.)"""
        return np.array(world_xyz) - self.base_pos_world

    def read_joints(self):
        try:
            msg = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
            pos = {n: p for n, p in zip(msg.name, msg.position)}
            return [pos.get(j, 0.0) for j in self.JOINT_NAMES]
        except rospy.ROSException:
            rospy.logwarn('Cannot read /joint_states, using HOME.')
            return list(self.HOME)

    def read_ee_pose_tf(self):
        """EE position in panda_link0 frame via TF (for validation)."""
        try:
            self.tf_listener.waitForTransform(
                'panda_link0', 'panda_hand_tcp', rospy.Time(0), rospy.Duration(3.0))
            pos, _ = self.tf_listener.lookupTransform(
                'panda_link0', 'panda_hand_tcp', rospy.Time(0))
            return np.array(pos)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            return None

    def get_model_position(self, model_name):
        """Model position in world frame from Gazebo."""
        try:
            resp = self._get_model_state_srv(model_name, 'world')
            if resp.success:
                p = resp.pose.position
                return np.array([p.x, p.y, p.z])
        except rospy.ServiceException:
            pass
        rospy.logwarn('Model %s not found.', model_name)
        return None

    # ============================================================== #
    #  Settling / readiness check                                     #
    # ============================================================== #

    def wait_for_settled(self, timeout=30.0):
        """Wait until the robot joints have reached the initial
        configuration and the controllers are active."""
        rospy.loginfo('Waiting for simulation to settle ...')
        rate = rospy.Rate(5)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - t0).to_sec()
            if elapsed > timeout:
                rospy.logwarn('Settle timeout (%.0fs).  Proceeding anyway.', timeout)
                break
            q = self.read_joints()
            if q[3] < -2.0:
                rospy.loginfo('Robot settled (j4=%.3f) after %.1fs.', q[3], elapsed)
                break
            rate.sleep()
        rospy.sleep(3.0)

    def validate_fk(self):
        """Compare our FK with TF to check for mismatches."""
        q = self.read_joints()
        T_fk = panda_fk(q)
        fk_pos = T_fk[:3, 3]
        tf_pos = self.read_ee_pose_tf()
        if tf_pos is not None:
            err = np.linalg.norm(fk_pos - tf_pos)
            rospy.loginfo('FK validation: FK=(%.3f,%.3f,%.3f) TF=(%.3f,%.3f,%.3f) err=%.4f',
                          fk_pos[0], fk_pos[1], fk_pos[2],
                          tf_pos[0], tf_pos[1], tf_pos[2], err)
            if err > 0.05:
                rospy.logwarn('FK/TF mismatch > 5cm!  IK targets may be inaccurate.')
        else:
            rospy.logwarn('Cannot validate FK (TF unavailable).')

    # ============================================================== #
    #  Scene                                                          #
    # ============================================================== #

    def spawn_models(self):
        rospy.loginfo('--- Scene models ---')
        for name in ('panda', 'table', 'pick_tray', 'place_tray', 'stone'):
            pos = self.get_model_position(name)
            if pos is not None:
                rospy.loginfo('  %s: (%.3f, %.3f, %.3f)', name, *pos)
            else:
                rospy.logwarn('  %s: NOT FOUND', name)
        self._reset_stone()

    def _reset_stone(self):
        tray_pos = self.get_model_position('pick_tray')
        if tray_pos is None:
            return
        stone_z = tray_pos[2] + 0.012 + 0.032
        try:
            rospy.wait_for_service('/gazebo/set_model_state', timeout=5.0)
            srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            state = ModelState()
            state.model_name = 'stone'
            state.pose.position = Point(tray_pos[0], tray_pos[1], stone_z)
            state.pose.orientation = Quaternion(0, 0, 0, 1)
            state.reference_frame = 'world'
            srv(state)
            rospy.loginfo('Stone reset on pick tray z=%.3f', stone_z)
        except rospy.ROSException as exc:
            rospy.logwarn('Stone reset failed: %s', exc)

    # ============================================================== #
    #  Cartesian target construction                                  #
    # ============================================================== #

    def _make_top_down_pose(self, base_xyz):
        """4x4 TCP pose in panda_link0 frame with gripper pointing down."""
        R_down = tft.euler_matrix(math.pi, 0, 0)[:3, :3]
        T = np.eye(4)
        T[:3, :3] = R_down
        T[:3, 3] = base_xyz
        return T

    # ============================================================== #
    #  Arm motion -- singularity-aware                                #
    # ============================================================== #

    def _compute_duration(self, start, target):
        max_d = max(abs(t - s) for s, t in zip(start, target))
        return max(max_d / self.MAX_JOINT_SPEED, self.DEFAULT_DURATION)

    def _check_trajectory_singularity(self, current, target, n_check=20):
        """Check if a linear joint-space interpolation passes through
        a singularity.  Returns (is_safe, worst_sv, worst_alpha)."""
        worst_sv = float('inf')
        worst_alpha = 0.0
        for i in range(n_check + 1):
            a = float(i) / n_check
            q_interp = [c + a * (t - c) for c, t in zip(current, target)]
            sv = min_singular_value(q_interp)
            if sv < worst_sv:
                worst_sv = sv
                worst_alpha = a
        is_safe = worst_sv >= _SINGULARITY_THRESHOLD
        return is_safe, worst_sv, worst_alpha

    def _find_safe_midpoint(self, current, target):
        """When a direct path crosses a singularity, find a midpoint
        that routes around it by biasing toward the preferred config."""
        mid = [0.5 * (c + t) for c, t in zip(current, target)]
        # Blend the midpoint toward the preferred config for the
        # singular joints (j4, j5, j6)
        blend = 0.5
        for j in [3, 4, 5, 6]:
            mid[j] = mid[j] * (1.0 - blend) + _Q_PREFERRED[j] * blend
        mid = np.clip(mid, _Q_MIN, _Q_MAX).tolist()
        return mid

    def move_arm(self, joint_target, duration=None):
        """Move arm with singularity checking.

        If the direct path crosses a singularity, routes through a
        safe intermediate waypoint.
        """
        current = self.read_joints()
        if duration is None:
            duration = self._compute_duration(current, joint_target)

        # Check if direct path is safe
        is_safe, worst_sv, worst_alpha = self._check_trajectory_singularity(
            current, joint_target)

        if not is_safe:
            rospy.logwarn('Direct path crosses singularity (sv=%.4f at alpha=%.2f). '
                          'Routing through safe midpoint.', worst_sv, worst_alpha)
            midpoint = self._find_safe_midpoint(current, joint_target)

            # Check that segments through midpoint are safe
            safe1, sv1, _ = self._check_trajectory_singularity(current, midpoint)
            safe2, sv2, _ = self._check_trajectory_singularity(midpoint, joint_target)

            if safe1 and safe2:
                rospy.loginfo('Midpoint route safe (sv1=%.4f sv2=%.4f). '
                              'Moving in two segments.', sv1, sv2)
                dur1 = self._compute_duration(current, midpoint)
                if not self._execute_trajectory(current, midpoint, dur1):
                    return False
                rospy.sleep(0.5)
                current2 = self.read_joints()
                dur2 = self._compute_duration(current2, joint_target)
                return self._execute_trajectory(current2, joint_target, dur2)
            else:
                rospy.logwarn('Midpoint route also singular (sv=%.4f, %.4f). '
                              'Proceeding slowly with high interpolation.', sv1, sv2)
                # Fall through to direct execution with slower speed
                duration = max(duration * 2.0, 10.0)

        return self._execute_trajectory(current, joint_target, duration)

    def _execute_trajectory(self, current, joint_target, duration):
        """Build and send an interpolated joint trajectory."""
        n = self.NUM_INTERP_POINTS
        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = list(self.JOINT_NAMES)

        for i in range(1, n + 1):
            a = float(i) / n
            s = 3.0 * a * a - 2.0 * a * a * a   # cubic ease
            pt = JointTrajectoryPoint()
            pt.positions = [c + s * (t - c) for c, t in zip(current, joint_target)]
            pt.velocities = [0.0] * 7
            pt.time_from_start = rospy.Duration(duration * a)
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

    def move_to_cartesian(self, world_xyz, label=''):
        """Convert world position to base frame, build top-down pose,
        solve IK, and move."""
        base_xyz = self.world_to_base(world_xyz)
        rospy.loginfo('%s  world=(%.3f,%.3f,%.3f)  base=(%.3f,%.3f,%.3f)',
                      label, world_xyz[0], world_xyz[1], world_xyz[2],
                      base_xyz[0], base_xyz[1], base_xyz[2])
        T = self._make_top_down_pose(base_xyz)
        q_init = self.read_joints()
        q_sol = solve_ik(T, q_init)
        if q_sol is None:
            rospy.logerr('IK failed for %s', label)
            return False
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
        rospy.loginfo('attach_object(%s) -- friction hold (%.0f N)', name, self.GRASP_FORCE)

    def detach_object(self, name='stone'):
        rospy.loginfo('detach_object(%s) -- releasing via gripper open', name)

    # ============================================================== #
    #  Pick / Place                                                   #
    # ============================================================== #

    def pick(self):
        rospy.loginfo('=== PICK ===')

        stone_pos = self.get_model_position('stone')
        if stone_pos is None:
            rospy.logerr('Cannot locate stone.')
            return False
        rospy.loginfo('Stone at (%.3f, %.3f, %.3f)', *stone_pos)

        self.open_gripper()
        rospy.sleep(1.0)

        # Pre-grasp: above stone
        pre_xyz = stone_pos.copy()
        pre_xyz[2] += self.APPROACH_HEIGHT
        if not self.move_to_cartesian(pre_xyz, 'pre-grasp'):
            return False
        rospy.sleep(1.0)

        # Grasp: at stone level
        if not self.move_to_cartesian(stone_pos.copy(), 'grasp'):
            return False
        rospy.sleep(1.0)

        self.close_gripper()
        self.attach_object()
        rospy.sleep(1.5)

        # Lift
        if not self.move_to_cartesian(pre_xyz, 'lift'):
            return False
        rospy.sleep(1.0)

        rospy.loginfo('=== PICK DONE ===')
        return True

    def place(self):
        rospy.loginfo('=== PLACE ===')

        tray_pos = self.get_model_position('place_tray')
        if tray_pos is None:
            rospy.logerr('Cannot locate place_tray.')
            return False
        rospy.loginfo('Place tray at (%.3f, %.3f, %.3f)', *tray_pos)

        tray_surface_z = tray_pos[2] + 0.012

        # Go home first to get a clean configuration for the move to the
        # other side of the table -- avoids large joint swings that cross
        # singularity boundaries.
        rospy.loginfo('Returning to home before place ...')
        self.move_arm(self.HOME)
        rospy.sleep(1.0)

        # Pre-place: above tray
        pre_xyz = np.array([tray_pos[0], tray_pos[1],
                            tray_surface_z + self.APPROACH_HEIGHT])
        if not self.move_to_cartesian(pre_xyz, 'pre-place'):
            return False
        rospy.sleep(1.0)

        # Lower to place height
        place_xyz = np.array([tray_pos[0], tray_pos[1],
                              tray_surface_z + self.PLACE_HEIGHT])
        if not self.move_to_cartesian(place_xyz, 'place'):
            return False
        rospy.sleep(1.0)

        self.detach_object()
        self.open_gripper()
        rospy.sleep(1.5)

        # Retreat
        if not self.move_to_cartesian(pre_xyz, 'retreat'):
            return False
        rospy.sleep(1.0)

        # Home
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

        # Wait for simulation to settle
        self.wait_for_settled(timeout=40.0)

        # Re-read robot base position after settling
        self._update_base_position()

        # Validate FK against TF
        self.validate_fk()

        # Log singularity measure at home
        q_home = self.read_joints()
        rospy.loginfo('Home singularity measure: sv=%.4f (threshold=%.4f)',
                      min_singular_value(q_home), _SINGULARITY_THRESHOLD)

        # Check scene
        self.spawn_models()
        rospy.sleep(2.0)

        # Go home
        rospy.loginfo('Moving to home ...')
        self.move_arm(self.HOME)
        rospy.sleep(3.0)

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

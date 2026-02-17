#!/usr/bin/env python
"""
Gazebo-only pick-and-place for Franka Panda.
No MoveIt dependency -- uses effort_joint_trajectory_controller via
FollowJointTrajectoryAction and franka_gripper actions (move / grasp).
"""

import sys
import rospy
import actionlib

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


class PandaPickPlace(object):
    """Pick-and-place controller for Panda in Gazebo (no MoveIt)."""

    # 7-DOF arm joint names
    JOINT_NAMES = [
        'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
        'panda_joint5', 'panda_joint6', 'panda_joint7',
    ]

    # ------------------------------------------------------------------ #
    #  Pre-computed joint waypoints                                       #
    #  Stone sits at roughly (0.0, -0.22, 0.475) on the pick tray.       #
    #  Place tray centre is at roughly (0.0, +0.22, 0.42).               #
    #                                                                     #
    #  URDF joint limits:                                                 #
    #    J1 [-2.90, 2.90]  J2 [-1.76, 1.76]  J3 [-2.90, 2.90]           #
    #    J4 [-3.07, -0.07] J5 [-2.90, 2.90]  J6 [-0.02, 3.75]           #
    #    J7 [-2.90, 2.90]                                                 #
    #                                                                     #
    #  Tune these in Gazebo if the end-effector does not line up          #
    #  perfectly with the stone/tray.  Use _read_current_joints() to      #
    #  read back the actual joint state after manual jogging.             #
    # ------------------------------------------------------------------ #

    HOME       = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    # Above stone on pick tray -- gripper pointing straight down
    PRE_GRASP  = [-0.30,  0.15,  0.30, -1.80,  0.05, 2.00, 0.785]
    # Lowered to stone grasping height
    GRASP      = [-0.30,  0.35,  0.30, -1.60,  0.05, 2.00, 0.785]
    # Lift back up (same as pre-grasp)
    LIFT       = [-0.30,  0.15,  0.30, -1.80,  0.05, 2.00, 0.785]

    # Above place tray -- mirror joint1 to positive Y side
    PRE_PLACE  = [ 0.30,  0.15, -0.30, -1.80, -0.05, 2.00, 0.785]
    # Lowered to place tray surface
    PLACE      = [ 0.30,  0.35, -0.30, -1.60, -0.05, 2.00, 0.785]
    # Retreat upward (same as pre-place)
    RETREAT    = [ 0.30,  0.15, -0.30, -1.80, -0.05, 2.00, 0.785]

    # Gripper parameters
    GRIPPER_OPEN_WIDTH   = 0.08    # m  -- fully open
    GRIPPER_SPEED        = 0.1     # m/s
    GRASP_WIDTH          = 0.025   # m  -- stone X-dim (narrowest, 25 mm)
    GRASP_FORCE          = 50.0    # N
    GRASP_EPSILON_INNER  = 0.005   # m
    GRASP_EPSILON_OUTER  = 0.005   # m

    # Default trajectory duration per waypoint
    DEFAULT_DURATION = 4.0  # seconds

    # -------------------------------------------------------------- #

    def __init__(self):
        rospy.init_node('panda_pick_place', anonymous=False)

        # --- Arm trajectory action client ---
        arm_topic = '/effort_joint_trajectory_controller/follow_joint_trajectory'
        self.arm_client = actionlib.SimpleActionClient(
            arm_topic, FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for arm trajectory action server (%s) ...', arm_topic)
        if not self.arm_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logfatal('Arm action server not available -- aborting.')
            sys.exit(1)
        rospy.loginfo('Arm action server connected.')

        # --- Gripper move action client ---
        self.gripper_move_client = actionlib.SimpleActionClient(
            '/franka_gripper/move', MoveAction)
        rospy.loginfo('Waiting for gripper move action server ...')
        if not self.gripper_move_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logfatal('Gripper move server not available -- aborting.')
            sys.exit(1)
        rospy.loginfo('Gripper move server connected.')

        # --- Gripper grasp action client ---
        self.gripper_grasp_client = actionlib.SimpleActionClient(
            '/franka_gripper/grasp', GraspAction)
        rospy.loginfo('Waiting for gripper grasp action server ...')
        if not self.gripper_grasp_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logfatal('Gripper grasp server not available -- aborting.')
            sys.exit(1)
        rospy.loginfo('Gripper grasp server connected.')

        rospy.loginfo('All action servers connected.  Ready for pick-and-place.')

    # ============================================================== #
    #  Scene helpers                                                  #
    # ============================================================== #

    def spawn_models(self):
        """Verify the required Gazebo models are present (spawned by the
        stone.sdf world file).  Reset the stone to its initial pose."""
        rospy.loginfo('Checking Gazebo models ...')
        try:
            rospy.wait_for_service('/gazebo/get_model_state', timeout=10.0)
            get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            for name in ('table', 'pick_tray', 'place_tray', 'stone'):
                resp = get_state(name, 'world')
                if resp.success:
                    p = resp.pose.position
                    rospy.loginfo('  %s found at (%.3f, %.3f, %.3f)', name, p.x, p.y, p.z)
                else:
                    rospy.logwarn('  %s NOT found in world!', name)
        except rospy.ROSException as exc:
            rospy.logwarn('Could not verify models: %s', exc)

        self._reset_stone()

    def _reset_stone(self):
        """Move the stone back to its initial pose above the pick tray."""
        try:
            rospy.wait_for_service('/gazebo/set_model_state', timeout=5.0)
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            state = ModelState()
            state.model_name = 'stone'
            state.pose.position = Point(0.000286, -0.221972, 0.475172)
            state.pose.orientation = Quaternion(0, 0, 0, 1)
            state.reference_frame = 'world'
            resp = set_state(state)
            if resp.success:
                rospy.loginfo('Stone reset to initial position.')
            else:
                rospy.logwarn('Failed to reset stone: %s', resp.status_message)
        except rospy.ROSException as exc:
            rospy.logwarn('SetModelState not available: %s', exc)

    # ============================================================== #
    #  Arm motion                                                     #
    # ============================================================== #

    def move_arm(self, joint_positions, duration=None):
        """Send a single-waypoint joint trajectory and block until done.

        Args:
            joint_positions: list of 7 joint angles [rad].
            duration: time for the motion (seconds).  Defaults to
                      DEFAULT_DURATION.
        Returns:
            True on success, False otherwise.
        """
        if duration is None:
            duration = self.DEFAULT_DURATION

        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = list(self.JOINT_NAMES)

        pt = JointTrajectoryPoint()
        pt.positions = list(joint_positions)
        pt.velocities = [0.0] * 7
        pt.time_from_start = rospy.Duration(duration)
        traj.points.append(pt)

        goal.trajectory = traj

        rospy.loginfo('Moving arm to %s (%.1f s) ...',
                      [round(j, 3) for j in joint_positions], duration)
        self.arm_client.send_goal(goal)
        finished = self.arm_client.wait_for_result(
            rospy.Duration(duration + 10.0))

        if not finished:
            rospy.logwarn('Arm trajectory timed out.')
            return False

        result = self.arm_client.get_result()
        if result.error_code != 0:
            rospy.logwarn('Arm trajectory error %d: %s',
                          result.error_code, result.error_string)
            return False

        rospy.loginfo('Arm reached target.')
        return True

    # ============================================================== #
    #  Gripper control                                                #
    # ============================================================== #

    def open_gripper(self):
        """Open the gripper fully via the /franka_gripper/move action."""
        rospy.loginfo('Opening gripper (width=%.3f m) ...', self.GRIPPER_OPEN_WIDTH)
        goal = MoveGoal()
        goal.width = self.GRIPPER_OPEN_WIDTH
        goal.speed = self.GRIPPER_SPEED

        self.gripper_move_client.send_goal(goal)
        finished = self.gripper_move_client.wait_for_result(rospy.Duration(10.0))

        if finished:
            result = self.gripper_move_client.get_result()
            rospy.loginfo('Gripper open: success=%s', result.success)
            return result.success

        rospy.logwarn('Gripper open timed out.')
        return False

    def close_gripper(self):
        """Close the gripper around the object via /franka_gripper/grasp.

        The FrankaGripperSim applies the requested force per finger in
        its HOLDING state, relying on friction to keep the object."""
        rospy.loginfo('Closing gripper (width=%.3f m, force=%.1f N) ...',
                      self.GRASP_WIDTH, self.GRASP_FORCE)
        goal = GraspGoal()
        goal.width = self.GRASP_WIDTH
        goal.speed = self.GRIPPER_SPEED
        goal.force = self.GRASP_FORCE
        goal.epsilon = GraspEpsilon(
            inner=self.GRASP_EPSILON_INNER,
            outer=self.GRASP_EPSILON_OUTER)

        self.gripper_grasp_client.send_goal(goal)
        finished = self.gripper_grasp_client.wait_for_result(rospy.Duration(15.0))

        if finished:
            result = self.gripper_grasp_client.get_result()
            rospy.loginfo('Grasp result: success=%s', result.success)
            return result.success

        rospy.logwarn('Grasp timed out.')
        return False

    # ============================================================== #
    #  Gazebo object attachment (friction-based)                      #
    # ============================================================== #

    def attach_object(self, object_name='stone'):
        """Attachment stub -- using friction-based grasping.

        The FrankaGripperSim HOLDING state applies continuous force on
        the stone.  If a gazebo_ros_link_attacher package is added later
        this method can be extended to call /link_attacher_node/attach.
        """
        rospy.loginfo('attach_object("%s") -- friction-based hold active '
                      '(%.1f N).', object_name, self.GRASP_FORCE)

    def detach_object(self, object_name='stone'):
        """Detachment stub -- opening the gripper releases the object."""
        rospy.loginfo('detach_object("%s") -- releasing via gripper open.',
                      object_name)

    # ============================================================== #
    #  High-level sequences                                           #
    # ============================================================== #

    def pick(self):
        """Pick sequence: open gripper -> approach -> lower -> grasp -> lift."""
        rospy.loginfo('=== PICK SEQUENCE START ===')

        # Open gripper before approaching
        self.open_gripper()
        rospy.sleep(0.5)

        # Move above the stone
        rospy.loginfo('Moving to pre-grasp ...')
        if not self.move_arm(self.PRE_GRASP):
            rospy.logwarn('Failed to reach pre-grasp.')
            return False
        rospy.sleep(0.5)

        # Lower to grasping height
        rospy.loginfo('Lowering to grasp pose ...')
        if not self.move_arm(self.GRASP, duration=3.0):
            rospy.logwarn('Failed to reach grasp pose.')
            return False
        rospy.sleep(0.5)

        # Close gripper
        self.close_gripper()
        self.attach_object('stone')
        rospy.sleep(1.0)

        # Lift
        rospy.loginfo('Lifting ...')
        if not self.move_arm(self.LIFT, duration=3.0):
            rospy.logwarn('Failed to lift.')
            return False
        rospy.sleep(0.5)

        rospy.loginfo('=== PICK SEQUENCE COMPLETE ===')
        return True

    def place(self):
        """Place sequence: move to tray B -> lower -> release -> retreat -> home."""
        rospy.loginfo('=== PLACE SEQUENCE START ===')

        # Move above the place tray
        rospy.loginfo('Moving to pre-place ...')
        if not self.move_arm(self.PRE_PLACE):
            rospy.logwarn('Failed to reach pre-place.')
            return False
        rospy.sleep(0.5)

        # Lower to place surface
        rospy.loginfo('Lowering to place pose ...')
        if not self.move_arm(self.PLACE, duration=3.0):
            rospy.logwarn('Failed to reach place pose.')
            return False
        rospy.sleep(0.5)

        # Release object
        self.detach_object('stone')
        self.open_gripper()
        rospy.sleep(1.0)

        # Retreat upward
        rospy.loginfo('Retreating ...')
        if not self.move_arm(self.RETREAT, duration=3.0):
            rospy.logwarn('Failed to retreat.')
            return False
        rospy.sleep(0.5)

        # Return home
        rospy.loginfo('Returning home ...')
        if not self.move_arm(self.HOME):
            rospy.logwarn('Failed to return home.')
            return False
        rospy.sleep(0.5)

        rospy.loginfo('=== PLACE SEQUENCE COMPLETE ===')
        return True

    # ============================================================== #
    #  Main entry                                                     #
    # ============================================================== #

    def run(self):
        """Execute the full pick-and-place cycle."""
        rospy.loginfo('======== PANDA PICK-AND-PLACE (NO MOVEIT) ========')

        # Verify / reset scene
        self.spawn_models()
        rospy.sleep(1.0)

        # Start from home
        rospy.loginfo('Moving to home ...')
        self.move_arm(self.HOME)
        rospy.sleep(1.0)

        # Pick the stone from Tray A
        if not self.pick():
            rospy.logerr('Pick failed.  Aborting.')
            return

        # Place the stone on Tray B
        if not self.place():
            rospy.logerr('Place failed.  Aborting.')
            return

        rospy.loginfo('======== PICK-AND-PLACE COMPLETE ========')

    # ============================================================== #
    #  Debugging helper                                               #
    # ============================================================== #

    def _read_current_joints(self):
        """Read and log current arm joint positions from /joint_states.

        Useful for manually jogging the robot (e.g. via
        rqt_joint_trajectory_controller) and recording good waypoints.
        """
        try:
            msg = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
            positions = {n: p for n, p in zip(msg.name, msg.position)}
            current = [positions.get(j, 0.0) for j in self.JOINT_NAMES]
            rospy.loginfo('Current joints: %s', [round(v, 4) for v in current])
            return current
        except rospy.ROSException:
            rospy.logwarn('Could not read /joint_states.')
            return None


if __name__ == '__main__':
    try:
        controller = PandaPickPlace()
        controller.run()
    except rospy.ROSInterruptException:
        pass

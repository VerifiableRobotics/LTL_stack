#! /usr/bin/env python
import rospy
import argparse
import logging
import sys
import traceback
import tf
import math
import time
import actionlib

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import trajectory_msgs.msg
import brics_actuator.msg

import node_logging
node_logger = logging.getLogger("node_logger")

GROUP_NAME_ARM = 'arm_1'
GROUP_NAME_GRIPPER = 'arm_1_gripper'

GRIPPER_FRAME = 'gripper_palm_link'

GRIPPER_OPEN = [0.0115, 0.0]
GRIPPER_CLOSED = [0.00, 0.0]
GRIPPER_NEUTRAL = [0.03, -0.03]
GRIPPER_EFFORT = [1.0, 1.0]

GRIPPER_JOINT_NAMES = ['gripper_finger_joint_l', 'gripper_finger_joint_r']

REFERENCE_FRAME='base_footprint'

# for testing
#rostopic pub /target_pose geometry_msgs/Pose '{position:  {x: 0.20, y: 0.0, z: 0.70}, orientation: {w: 1.0, x: 0.0,y: 0.0,z: 0.0}}'

#OR
# rosrun controller_executor check_if_tag_within_reachy test /test
# rosrun controller_executor find_any_tag.py test2 /test
# rostopic pub /test std_msgs/Bool True
# rosrun controller_executor plan_manipulator_path.py  test3 /test


#roslaunch youbot_moveit move_group.launch
#roslaunch youbot_gazebo_robot youbot.launch world:="empty_world"

class PlanPathAction(object):
    def __init__(self, args):
        allow_replanning = True
        planning_timeout = 20.0
        self._plan = None
        self._planning_traj = False

        try:
            node_logger.info("===== Waiting for MoveIt! to start ==========================")
            moveit_client = actionlib.SimpleActionClient("move_group", moveit_msgs.msg.MoveGroupAction)
            moveit_client.wait_for_server()

            node_logger.info("===== Setting up MoveIt! ==========================")
            moveit_commander.roscpp_initialize(sys.argv) #sys.argv
            self._robot = moveit_commander.RobotCommander()
            self._scene = moveit_commander.PlanningSceneInterface()
            node_logger.info("--setting up MoveGroup: {group_name}--".format(group_name=args.group_name))
            self._group = moveit_commander.MoveGroupCommander(args.group_name)
            self._group.set_goal_tolerance(0.10)
            self._group.set_planner_id("RRTConnectkConfigDefault")
            self._group.allow_replanning(allow_replanning)
            self._group.set_planning_time(planning_timeout)
            self._group.set_pose_reference_frame(REFERENCE_FRAME)
        except:
            node_logger.error("Unexpected error:")
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_tb(exc_traceback, limit=1, file=sys.stdout)
            traceback.print_exception(exc_type, exc_value, exc_traceback,
                              limit=2, file=sys.stdout)
            self.shutdown()

        # publisher for action status
        self._pub_find_path = rospy.Publisher(args.node_subscribe_topic+'_find_path', std_msgs.msg.Bool, queue_size=10, latch=True)
        self._pub_find_path.publish(False)

        # publisher for ac status
        self._pub_status = rospy.Publisher(args.node_subscribe_topic+'_status', std_msgs.msg.Bool, queue_size=10, latch=True)
        self._pub_status.publish(False)

        # publish gripper command
        #self._gripper_pub = rospy.Publisher('/arm_1/gripper_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self._gripper_pub = rospy.Publisher('/arm_1/gripper_controller/position_command', brics_actuator.msg.JointPositions, queue_size=10)

        # just for debugging
        self._transformed_target_pose_pub = rospy.Publisher('/transformed_target_pose', geometry_msgs.msg.PoseStamped, queue_size=10, latch=True)
        self._joint_states = None

        # set up subscriber for target pose info
        self._target_pose = None
        rospy.Subscriber(args.target_pose_topic, geometry_msgs.msg.Pose, callback=self.target_pose_callback, callback_args=args.z_offset)

        # also subscribe to controller request
        rospy.Subscriber(args.node_subscribe_topic, std_msgs.msg.Bool, callback=self.executor_callback)

        # move test pickup to here and try the whole thing

    def target_pose_callback(self, data, z_offset):
        world_tag_pose = [data.position.x, data.position.y, data.position.z]
        world_tag_rot = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

        # transform frame
        # for API visit http://www.lfd.uci.edu/~gohlke/code/transformations.py.html
        z_axis_offset_tf = tf.transformations.translation_matrix([0,0,z_offset]) #x,y,z offset
        about_x_rotation_qua = tf.transformations.quaternion_about_axis(math.pi,[1,0,0]) # about x

        world_to_tag_pose_tf = tf.transformations.translation_matrix(list(world_tag_pose)) # convert pose to translation matrix

        world_transformed_tag_pose = world_to_tag_pose_tf.dot(z_axis_offset_tf.dot([0,0,0,1])) # offset pose with z_offset
        world_transformed_tag_rot  = tf.transformations.quaternion_multiply(list(world_tag_rot), about_x_rotation_qua) # transform quaternion

        #world_transformed_tag_rot  = world_tag_rot # TO REMOVE

        node_logger.debug("world_tag_pose: " + str(world_tag_pose))
        node_logger.debug("world_tag_rot: " + str(world_tag_rot))

        node_logger.debug("world_transformed_tag_pose: " + str(world_transformed_tag_pose))
        node_logger.debug("world_transformed_tag_rot: " + str(world_transformed_tag_rot))

        self._target_pose = geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(*world_transformed_tag_pose[0:3]),\
                                                   orientation=geometry_msgs.msg.Quaternion(*world_transformed_tag_rot))

        # for rviz debugging
        a = geometry_msgs.msg.PoseStamped()
        a.header.frame_id = 'base_footprint'
        a.header.stamp = rospy.Time.now()
        a.pose = self._target_pose
        self._transformed_target_pose_pub.publish(a)


    def executor_callback(self, data):
        if data.data:
            # action is true
            node_logger.debug("Action turns True!")
            if not self._planning_traj:
                if self.plan_traj(): # succesfully found a plan
                    self._pub_find_path.publish(True)
                else:
                    self._pub_find_path.publish(False)

                self._pub_status.publish(True)
                node_logger.debug("Updated status to ac sensor.")

        else:
            # action is false
            node_logger.debug("Action turns False!")
            self._planning_traj = False
            self._group.stop()
            self._pub_status.publish(False)

    def shutdown(self):
        # clean up moveit
        node_logger.info("===== shutdown: Shutting down moveit! ==================")
        moveit_commander.roscpp_shutdown()


    def control_gripper(joint_names_list, joint_positions_list):
        # target pose
        gripper_positions = brics_actuator.msg.JointPositions()
        for idx, gripper_joint_name in enumerate(joint_names_list):
            point = brics_actuator.msg.JointValue(timeStamp=rospy.Time.now()+rospy.Duration(2),\
                joint_uri=gripper_joint_name, unit='m', value=joint_positions_list[idx])
            gripper_positions.positions.append(point)

        return gripper_positions

    def plan_traj(self):
        """
        reuse planning
        """
        self._planning_traj = True
        self._plan = None

        # cannot get target pose
        if not self._target_pose:
            node_logger.warning('No target pose!...')
            return False
        else:
            # first open gripper
            node_logger.debug('Opening gripper...')
            # target pose
            #self._gripper_pub.publish(self.control_gripper(GRIPPER_JOINT_NAMES, GRIPPER_OPEN))
            rospy.sleep(3)
            time.sleep(3)

            # now set pose target
            node_logger.info("===== Setting goal position:{pose_target} =========".format(pose_target=self._target_pose))
            self._group.set_pose_target(self._target_pose)
            #current_pose = self._group.get_current_pose()
            #self._group.set_pose_target(current_pose)

            node_logger.info("===== Planning to goal position ======================")
            plan = self._group.plan()
            if plan and plan.joint_trajectory.points:
                node_logger.info("-- Plan found")
            else:
                node_logger.error("-- No plan found.")
                return False

            # scale plan to mkae it slower
            self._plan = scale_trajectory_speed(plan, 0.3)
            node_logger.debug("scaled_plan:" + str(self._plan))

            node_logger.info("===== Executing plan ======================")
            self._group.execute(self._plan)

            # close gripper
            node_logger.debug('Closing gripper...')
            #self._gripper_pub.publish(self.control_gripper(GRIPPER_JOINT_NAMES, GRIPPER_CLOSED))

            # go to a set position (e.g: the back of the base?)

            return True


def scale_trajectory_speed(traj, scale):
    # Create a new trajectory object
    new_traj = moveit_msgs.msg.RobotTrajectory()

    # Initialize the new trajectory to be the same as the planned trajectory
    new_traj.joint_trajectory = traj.joint_trajectory

    # Get the number of joints involved
    n_joints = len(traj.joint_trajectory.joint_names)

    # Get the number of points on the trajectory
    n_points = len(traj.joint_trajectory.points)

    # Store the trajectory points
    points = list(traj.joint_trajectory.points)

    # Cycle through all points and scale the time from start, speed and acceleration
    for i in range(n_points):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        point.positions = traj.joint_trajectory.points[i].positions

        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * scale
            point.accelerations[j] = point.accelerations[j] * scale * scale

        points[i] = point

    # Assign the modified points to the new trajectory
    new_traj.joint_trajectory.points = points

    # Return the new trajecotry
    return new_traj


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plan a path for manipulator with MoveIt!")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_subscribe_topic', type=str, help='Specify controller topic to respond to')
    parser.add_argument('--group_name', type=str, help='Specify MoveIt! group name to use.', nargs='?', const='arm_1', default='arm_1')
    parser.add_argument('--target_pose_topic', type=str, help='Specify target pose topic.', nargs='?', const='/target_pose', default='/target_pose')
    parser.add_argument('--gripper_topic', type=str, help='Specify topic of the gripper to close.', nargs='?', \
                                           const='arm_1/gripper_controller/position_command', default='arm_1/gripper_controller/position_command')
    parser.add_argument('--z_offset', type=float, help='z offset for targer post in meters.', nargs='?', const=0.3, default=0.3)

    args, unknown = parser.parse_known_args()
    node_logger.debug(args)

    rospy.init_node(args.node_name)

    a = PlanPathAction(args)
    rospy.on_shutdown(a.shutdown)

    rospy.spin()

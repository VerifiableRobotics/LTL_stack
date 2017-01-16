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
import ast
import getpass
import copy


import moveit_commander
import moveit_msgs.msg, moveit_msgs.srv
import geometry_msgs.msg
import std_msgs.msg
import trajectory_msgs.msg
import brics_actuator.msg
import apriltags_ros.msg
import actionlib_msgs.msg

import node_logging
node_logger = logging.getLogger("node_logger")

GROUP_NAME_ARM = 'arm_1'
GROUP_NAME_GRIPPER = 'arm_1_gripper'

GRIPPER_FRAME = 'gripper_palm_link'

GRIPPER_OPEN = [0.0115, 0.0115]
GRIPPER_CLOSED = [0.00, 0.0]
GRIPPER_NEUTRAL = [0.03, -0.03]
GRIPPER_EFFORT = [1.0, 1.0]

GRIPPER_JOINT_NAMES = ['gripper_finger_joint_l', 'gripper_finger_joint_r']

REFERENCE_FRAME='base_footprint'
THRESHOLD_DISTANCE = 0.05 #0.1
VELOCITY_SCALAR = 0.01
MOVE_RATE = 30

# for testing
#rostopic pub /target_pose geometry_msgs/Pose '{position:  {x: 0.20, y: 0.0, z: 0.70}, orientation: {w: 1.0, x: 0.0,y: 0.0,z: 0.0}}'

#OR
# rosrun controller_executor check_if_tag_within_reachy test /test
# rosrun controller_executor find_any_tag.py test2 /test
# rostopic pub /test std_msgs/Bool True
# rosrun controller_executor plan_manipulator_path.py  test3 /test


#roslaunch youbot_moveit move_group.launch
#roslaunch youbot_gazebo_robot youbot.launch world:="empty_world"


import plot_reachable_points

class PlanPathAction(object):
    def __init__(self, args):
        allow_replanning = True
        planning_timeout = 10.0
        self._plan = None
        self._planning_traj = False
        self.args = args # save arguments for future use

        try:
            node_logger.info("===== Waiting for MoveIt! to start ==========================")
            moveit_client = actionlib.SimpleActionClient("/move_group", moveit_msgs.msg.MoveGroupAction)
            moveit_client.wait_for_server()

            node_logger.info("===== Setting up MoveIt! ==========================")
            moveit_commander.roscpp_initialize(sys.argv) #sys.argv
            self._robot = moveit_commander.RobotCommander()
            self._scene = moveit_commander.PlanningSceneInterface()
            node_logger.info("--setting up MoveGroup: {group_name}--".format(group_name=args.group_name))
            self._group = moveit_commander.MoveGroupCommander(args.group_name)
            self._group.set_goal_orientation_tolerance(1.57)
            self._group.set_planner_id("RRTConnectkConfigDefault")
            self._group.allow_replanning(allow_replanning)
            self._group.set_planning_time(planning_timeout)
            self._group.set_pose_reference_frame(REFERENCE_FRAME)
            self._reference_frame = REFERENCE_FRAME
            node_logger.info("===== MoveIt! ready to operate for MoveGroup: {group_name} ====".format(group_name=args.group_name))

        except:
            node_logger.exception("Unexpected error:")
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

        # set up velocity pub
        self._velocity_pub = rospy.Publisher(args.velocity_topic, geometry_msgs.msg.Twist, queue_size=10, latch=True)

        # set up subscriber for target pose info
        self._target_pose = None
        self._target_move_pose = None
        self._target_tag_no = args.target_tag_no
        #self.target_pose_list = []
        #if args.target_pose_topic:
        #    rospy.Subscriber(args.target_pose_topic, geometry_msgs.msg.Pose, callback=self.target_pose_callback, callback_args=args.z_offset)
        #    node_logger.debug('Set target pose topic: {0}'.format(args.target_pose_topic))

        #else:
        #    # use given target pose
        self.target_pose_list = ast.literal_eval(args.target_pose)
        self._target_pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(x=self.target_pose_list[0], y=self.target_pose_list[1], z=self.target_pose_list[2]),\
                            geometry_msgs.msg.Quaternion(x=self.target_pose_list[3], y=self.target_pose_list[4], z=self.target_pose_list[5], w=self.target_pose_list[6]))
        node_logger.debug('Set target pose: {0}'.format(self._target_pose))

        time.sleep(1)

        # also subscribe to controller request
        rospy.Subscriber(args.node_subscribe_topic, std_msgs.msg.Bool, callback=self.executor_callback)

        # subsribe to necessary information
        rospy.Subscriber(args.sensor_topic, apriltags_ros.msg.AprilTagDetectionArray, callback=self.apriltag_callback)

        # try using the fk pose
        # trying setting joint constraints instead
        # if doesn't work, then try to do action not with python wrapping

        # test ik
        #self._target_pose = self.get_fk_from_moveit(self._group.get_joints(), [4.512, 0.386, -2.43, 1.41, 4.57]).pose
        #self.get_position_ik_from_moveit(self.get_fk_from_moveit(self._group.get_joints(), [4.512, 0.386, -2.43, 1.41, 4.57]))


        # move base cancel pub
        self._move_base_cancel_pub = rospy.Publisher('/move_base/cancel', actionlib_msgs.msg.GoalID, queue_size=1)


    def get_fk_from_moveit(self, joint_names, joint_positions):
        request = moveit_msgs.srv.GetPositionFKRequest()
        #change joint target
        request.robot_state.joint_state.position = joint_positions
        request.robot_state.joint_state.name = joint_names

        # header
        request.header.frame_id = '/base_footprint'
        request.header.stamp = rospy.Time.now()
        request.robot_state.joint_state.header = request.header

        #fk link
        request.fk_link_names = [self._group.get_end_effector_link()] #self._group.get_joints() + 'gripper_finger_joint_l', 'gripper_finger_joint_r']

        node_logger.debug('FK request: {0}\n-----------------------\n'.format(request))

        fk_service_name = "/compute_fk"
        rospy.wait_for_service(fk_service_name)
        try:
            fk_service = rospy.ServiceProxy(fk_service_name, moveit_msgs.srv.GetPositionFK)
            resp1 = fk_service(request)
            node_logger.debug('FK Result: {0}\n-----------------------\n'.format(resp1))
            node_logger.debug(type(resp1.error_code))
            if resp1.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
                return resp1.pose_stamped[0] # return posestamp
            else:
                node_logger.exception("No solution found, With result: {0}".format(resp1))
        except rospy.ServiceException, e:
            node_logger.exception("Service call failed: %s"%e)

    def get_position_ik_from_moveit(self, pose_stamped):
        #node_logger.debug(type(pose_stamped))
        request = moveit_msgs.msg.PositionIKRequest()
        request.group_name = self.args.group_name
        #request.robot_state = self._robot.get_current_state()
        #request.robot_state.joint_state.position = self._group.get_current_joint_values()
        #request.robot_state.joint_state.name = self._group.get_joints()
        request.pose_stamped = pose_stamped

        # fix y
        request.pose_stamped.pose.position.y = 0
        #request.pose_stamped.pose.position.z = 0.06

        # downward
        request.pose_stamped.pose.orientation.x = 0
        request.pose_stamped.pose.orientation.y = 1
        request.pose_stamped.pose.orientation.z = 0
        request.pose_stamped.pose.orientation.w = 0

        request.ik_link_name = self._group.get_end_effector_link()

        #request.timeout = rospy.Duration(5.0)
        #request.attempts = 10
        node_logger.debug('IK request: {0}\n-----------------------\n'.format(request))

        ik_service_name = "/compute_ik"
        rospy.wait_for_service(ik_service_name)
        try:
            ik_service = rospy.ServiceProxy(ik_service_name, moveit_msgs.srv.GetPositionIK)
            resp1 = ik_service(request)
            node_logger.debug('IK Result: {0}\n-----------------------\n'.format(resp1))
            if resp1.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
                joint_pos_dict = {}
                for joint in self._group.get_joints():
                    joint_pos_dict[joint] = resp1.solution.joint_state.position[resp1.solution.joint_state.name.index(joint)]
                node_logger.debug('joint_pos_dict: {0}'.format(joint_pos_dict))
                return joint_pos_dict # joint dict
            else:
                node_logger.error("No solution found. errror: {0}".format(resp1.error_code.val))#, With result: {0}".format(resp1))
        except rospy.ServiceException, e:
            node_logger.exception("Service call failed: %s"%e)

    """
    def target_pose_callback(self, data, z_offset):
        world_tag_pose = [data.position.x, data.position.y, data.position.z]
        world_tag_rot = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

        # transform frame
        # for API visit http://www.lfd.uci.edu/~gohlke/code/transformations.py.html
        z_axis_offset_tf = tf.transformations.translation_matrix([0,0,z_offset]) #x,y,z offset
        about_x_rotation_qua = tf.transformations.quaternion_about_axis(math.pi,[0,1,0]) # about y

        world_to_tag_pose_tf = tf.transformations.translation_matrix(list(world_tag_pose)) # convert pose to translation matrix

        world_transformed_tag_pose = world_to_tag_pose_tf.dot(z_axis_offset_tf.dot([0,0,0,1])) # offset pose with z_offset
        world_transformed_tag_rot  = tf.transformations.quaternion_multiply(list(world_tag_rot), about_x_rotation_qua) # transform quaternion

        #world_transformed_tag_rot  = world_tag_rot # TO REMOVE

        node_logger.log(2, "world_tag_pose: " + str(world_tag_pose))
        node_logger.log(2, "world_tag_rot: " + str(world_tag_rot))

        node_logger.log(2, "world_transformed_tag_pose: " + str(world_transformed_tag_pose))
        node_logger.log(2, "world_transformed_tag_rot: " + str(world_transformed_tag_rot))

        self._target_pose = geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(*world_transformed_tag_pose[0:3]),\
                                                   orientation=geometry_msgs.msg.Quaternion(*world_transformed_tag_rot))
        #TODO:maybe change orientation here



        self.publish_transformed_pose()
    """

    def publish_transformed_pose(self):
        # for rviz debugging
        a = geometry_msgs.msg.PoseStamped()
        a.header.frame_id = '/base_footprint'
        a.header.stamp = rospy.Time.now()
        a.pose = self._target_pose
        self._transformed_target_pose_pub.publish(a)

        return a

    def apriltag_callback(self, data):
        # save latest info
        for tag_info in data.detections:
            if tag_info.id == self._target_tag_no:
                node_logger.debug('Got tag at z:{0} x:{1}'.format(\
                    tag_info.pose.pose.position.z, tag_info.pose.pose.position.x))

                self._target_move_pose = tag_info.pose.pose
                break
        #else:
        #    self._target_move_pose = None

    def executor_callback(self, data):
        if data.data:
            # publish transformed pose
            self.publish_transformed_pose()

            if not self._planning_traj:
                # action is true
                node_logger.debug("Action turns True!")

                if self.plan_traj(): # succesfully found a plan
                    self._pub_find_path.publish(True)
                else:
                    self._pub_find_path.publish(False)

                self._pub_status.publish(True)
                node_logger.debug("Updated status to ac sensor.")

        else:
            # action is false
            #node_logger.debug("Action turns False!")
            if self._planning_traj:
                self._group.stop()
            self._planning_traj = False
            self._target_move_pose  = None
            self._pub_status.publish(False)

    def shutdown(self):
        # clean up moveit
        node_logger.info("===== shutdown: Shutting down moveit! ==================")
        moveit_commander.roscpp_shutdown()


    def control_gripper(self, joint_names_list, joint_positions_list):
        # target pose
        gripper_positions = brics_actuator.msg.JointPositions()
        for idx, gripper_joint_name in enumerate(joint_names_list):
            point = brics_actuator.msg.JointValue(timeStamp=rospy.Time.now()+rospy.Duration(2),\
                joint_uri=gripper_joint_name, unit='m', value=joint_positions_list[idx])
            gripper_positions.positions.append(point)

        return gripper_positions

    def call_plan_and_execute(self, position_type_str, pose_obj):
        """
        pose_list: need x,y, and z
        position_type_str: goal or default
        """
        # now set pose target
        node_logger.info("===== Setting {position_type_str} position:{pose_target} =========".format(\
                    position_type_str=position_type_str, \
                    pose_target=[pose_obj.position.x, pose_obj.position.y, pose_obj.position.z]))

        self._group.set_start_state_to_current_state()
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = '/base_footprint'
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose_obj
        self._transformed_target_pose_pub.publish(pose_stamped)

        # try IK then place
        joint_dict = self.get_position_ik_from_moveit(pose_stamped)
        if joint_dict:
            node_logger.info("=====  Joint Planning Yay! =====")
            self._group.set_joint_value_target(joint_dict)
            node_logger.log(8, joint_dict)
        else: # can't find ik
            self._group.set_position_target([pose_obj.position.x, pose_obj.position.y, pose_obj.position.z])

        node_logger.info("===== Planning to {position_type_str} position ======================".format(position_type_str=position_type_str))
        plan = self._group.plan()
        if plan and plan.joint_trajectory.points:
            node_logger.info("-- Plan found")
            node_logger.warning("Yes Plan Found")
        else:
            node_logger.error("-- No plan found.")
            return False

        # scale plan to mkae it slower
        self._plan = scale_trajectory_speed(plan, 1.0)

        node_logger.info("===== Executing plan ======================")
        self._group.execute(self._plan, wait=True)

        return True

    def stop_robot(self):
        self._move_base_cancel_pub.publish(actionlib_msgs.msg.GoalID())
        # stop before reach
        vel_obj = geometry_msgs.msg.Twist()
        self._velocity_pub.publish(vel_obj)

    def move_to_target(self):
        rate = rospy.Rate(MOVE_RATE)
        vel_obj = geometry_msgs.msg.Twist()
        dist = math.sqrt(self._target_move_pose.position.z**2 + self._target_move_pose.position.x**2)
        while not (dist < THRESHOLD_DISTANCE or rospy.is_shutdown()):
            vel_obj.linear.x = self._target_move_pose.position.z/dist*VELOCITY_SCALAR
            vel_obj.linear.y = -self._target_move_pose.position.x/dist*VELOCITY_SCALAR
            self._velocity_pub.publish(vel_obj)
            node_logger.debug('vx: {0}, vy:{1}'.format(vel_obj.linear.x, vel_obj.linear.y))

            # update distance before sleep
            dist -= math.sqrt(vel_obj.linear.x**2 + vel_obj.linear.y**2)*1/MOVE_RATE
            node_logger.debug('dist: {0}'.format(dist))
            rate.sleep()
            #dist = math.sqrt(self._target_move_pose.position.z**2 + self._target_move_pose.position.x**2)


        # stop after threshold
        vel_obj = geometry_msgs.msg.Twist()
        self._velocity_pub.publish(vel_obj)

        return True


    def plan_traj(self):
        """
        reuse planning
        """
        self._planning_traj = True
        self._plan = None

        self.stop_robot()
        # move to location
        while not self._target_move_pose:
            node_logger.warning('Waiting for tag!...')
            rospy.sleep(0.2)
            if rospy.is_shutdown():
                break

        if self._target_move_pose:
            self.move_to_target()

        # cannot get target pose
        if not self._target_pose:
            node_logger.warning('No target pose!...')
            return False
        else:
            # first open gripper
            #node_logger.debug('Opening gripper...')
            # target pose
            #for x in range(0,2):
            #    self._gripper_pub.publish(self.control_gripper(GRIPPER_JOINT_NAMES, GRIPPER_OPEN))
            #rospy.sleep(3)


            #pre-grasp (somewhere on top)
            pre_target_pose = copy.deepcopy(self._target_pose)
            pre_target_pose.position.z = 0.15
            pre_target_pose_execution = self.call_plan_and_execute('goal', pre_target_pose)

            # go to pose
            target_pose_execution = self.call_plan_and_execute('goal', self._target_pose)

            # close gripper
            #node_logger.debug('Closing gripper...')
            #for x in range(0,2):
            #    self._gripper_pub.publish(self.control_gripper(GRIPPER_JOINT_NAMES, GRIPPER_CLOSED))
            #rospy.sleep(3)


        # return to candle
        node_logger.info("=== Returning to Default pose ===")

        self._group.set_named_target('folded')
        plan = self._group.plan()
        self._plan = scale_trajectory_speed(plan, 1.0) # scale plan to make it slower
        self._group.execute(self._plan, wait=True)

        # go to a set position (e.g: back to default pose)
        node_logger.info("=== Returned to Default pose ===")

        return target_pose_execution


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
    parser.add_argument('--sensor_topic', type=str, help='Specify tag topic from camera', nargs='?', const='/tag_detections', default='/tag_detections')
    parser.add_argument('--target_tag_no', type=int, help='Specify target apriltag number.', nargs='?', const=0, default=0)
    #parser.add_argument('--gripper_topic', type=str, help='Specify topic of the gripper to close.', nargs='?', \
    #                                       const='arm_1/gripper_controller/position_command', default='arm_1/gripper_controller/position_command')
    parser.add_argument('--velocity_topic', type=str, help='Specify velocity topic of robot.', nargs='?', const='/cmd_vel', default='/cmd_vel')
    parser.add_argument('--target_pose', type=str, help='Specify target pose in str format [x, y, z, x, y, z, w]. Priorty over target_pose_topic', nargs='?', \
                                const='[0.45, 0.0, 0.10, 0,1, 0, 0]', default='[0.45, 0.0, 0.06, 0,1, 0, 0]')

    args, unknown = parser.parse_known_args()
    node_logger.debug(args)

    rospy.init_node(args.node_name)

    a = PlanPathAction(args)
    rospy.on_shutdown(a.shutdown)
    #a.test_points()
    #a.plan_traj()
    #time.sleep(10)

    #import time
    #while not rospy.is_shutdown():
    #    if a._target_move_pose:
    #        a.move_to_target()
    #        a.plan_traj()
    #        break
    #        time.sleep(10)

    rospy.spin()

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

REFERENCE_FRAME='/base_footprint'

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
        self.plot_obj = None

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

        # set up subscriber for target pose info
        self._target_pose = None
        self.target_pose_list = []
        if args.target_pose_topic:
            rospy.Subscriber(args.target_pose_topic, geometry_msgs.msg.Pose, callback=self.target_pose_callback, callback_args=args.z_offset)
            node_logger.debug('Set target pose topic: {0}'.format(args.target_pose_topic))

        else:
            # use given target pose
            self.target_pose_list = ast.literal_eval(args.target_pose)
            self._target_pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(x=self.target_pose_list[0], y=self.target_pose_list[1], z=self.target_pose_list[2]),\
                                 geometry_msgs.msg.Quaternion(x=self.target_pose_list[3], y=self.target_pose_list[4], z=self.target_pose_list[5], w=self.target_pose_list[6]))
            node_logger.debug('Set target pose: {0}'.format(self._target_pose))

        #self.target_pose_list = [0.05, 0.0, 0.30]
        time.sleep(1)

        # also subscribe to controller request
        rospy.Subscriber(args.node_subscribe_topic, std_msgs.msg.Bool, callback=self.executor_callback)

        # setup default pose
        #self._default_pose = None
        #self.default_pose_list = ast.literal_eval(args.default_pose)
        #if self.default_pose_list:
        #    self._default_pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(x=self.default_pose_list[0], y=self.default_pose_list[1], z=self.default_pose_list[2]),\
        #                    geometry_msgs.msg.Quaternion(x=self.default_pose_list[3], y=self.default_pose_list[4], z=self.default_pose_list[5], w=self.default_pose_list[6]))
        #    node_logger.debug('Set default pose: {0}'.format(self._default_pose))


        # try using the fk pose
        # trying setting joint constraints instead
        # if doesn't work, then try to do action not with python wrapping

        # test ik
        #self._target_pose = self.get_fk_from_moveit(self._group.get_joints(), [4.512, 0.386, -2.43, 1.41, 4.57]).pose
        #self.get_position_ik_from_moveit(self.get_fk_from_moveit(self._group.get_joints(), [4.512, 0.386, -2.43, 1.41, 4.57]))

    def test_points(self):
        self.plot_obj = plot_reachable_points.PlotPoints()
        self.file_handle = open('/home/{0}/points.txt'.format(getpass.getuser()),'w+')
        self.file_handle.write('[{0}, {1}, {2}]\n'.format(-10*0.02 + self.target_pose_list[0],-10*0.02 + self.target_pose_list[1],-10*0.02 + self.target_pose_list[2]))
        while not rospy.is_shutdown():
            increment = 0.02
            for x in range(-5,10):
                for y in range(-1,2):
                    for z in range(-10,5):
                        self._target_pose.position.x = x*0.02 + self.target_pose_list[0]
                        self._target_pose.position.y = y*0.02 + self.target_pose_list[1]
                        self._target_pose.position.z = z*0.02 + self.target_pose_list[2]
                        self._target_pose.orientation.x = 1
                        self._target_pose.orientation.y = 0
                        self._target_pose.orientation.z = 0
                        self._target_pose.orientation.w = 0

                        node_logger.warning('x:{0}, y:{1}, z:{2}'.format(self._target_pose.position.x,self._target_pose.position.y,self._target_pose.position.z))
                        self.publish_transformed_pose()
                        #self.get_position_ik_from_moveit(self.publish_transformed_pose())
                        if self.get_position_ik_from_moveit(self.publish_transformed_pose()):
                            node_logger.warning('IK found')
                            #if self.plan_traj():
                            self.plot_obj.plot_point(self._target_pose.position.x,self._target_pose.position.y,self._target_pose.position.z)
                            #self.file_handle.write('[{0}, {1}, {2}]\n'.format(self._target_pose.position.x,self._target_pose.position.y,self._target_pose.position.z))
                            self.file_handle.write('1\n')
                            time.sleep(0.1)
                        else:
                            node_logger.error('No IK found')
                            self.file_handle.write('0\n')

                        #time.sleep(2)


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
        #request.pose_stamped.pose.position.y = 0
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


    def target_pose_callback(self, data, z_offset):
        world_tag_pose = [data.position.x, data.position.y, data.position.z]
        world_tag_rot = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

        # set target list
        self.target_pose_list=[data.position.x, data.position.y, data.position.z, data.orientation.x, \
                              data.orientation.y, data.orientation.z, data.orientation.w]

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

    def publish_transformed_pose(self):
        # for rviz debugging
        a = geometry_msgs.msg.PoseStamped()
        a.header.frame_id = '/base_footprint'
        a.header.stamp = rospy.Time.now()
        a.pose = self._target_pose
        self._transformed_target_pose_pub.publish(a)

        return a


    def executor_callback(self, data):
        if data.data:
            # action is true
            #node_logger.debug("Action turns True!")

            # publish transformed pose
            self.publish_transformed_pose()

            if not self._planning_traj:
                if self.plan_traj(): # succesfully found a plan
                    self._pub_find_path.publish(True)
                else:
                    self._pub_find_path.publish(False)

                self._pub_status.publish(True)
                node_logger.debug("Updated status to ac sensor.")

        else:
            # action is false
            #node_logger.debug("Action turns False!")
            if not self._planning_traj:
                self._group.stop()
            self._planning_traj = False
            self._pub_status.publish(False)

    def shutdown(self):
        if self.plot_obj:
            # save plot
            node_logger.error('Shutting down.....')
            self.plot_obj.save_plot()
            self.file_handle.close()
            node_logger.error('Done Shutting down.....')

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

    def call_plan_and_execute(self, position_type_str, pose_list, pose_obj):
        """
        pose_list: need x,y, and z
        position_type_str: goal or default
        """
        # now set pose target
        node_logger.info("===== Setting {position_type_str} position:{pose_target} =========".format(position_type_str=position_type_str, pose_target=pose_list))

        self._group.set_start_state_to_current_state()
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = '/base_footprint'
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose_obj
        self._transformed_target_pose_pub.publish(pose_stamped)

        joint_dict = self.get_position_ik_from_moveit(pose_stamped)
        #    count +=1
        if joint_dict:
            node_logger.info("=====  Joint Planning Yay! =====")
            self._group.set_joint_value_target(joint_dict)
            node_logger.log(8, joint_dict)
        else:
            self._group.set_position_target(self.target_pose_list[:3])
        #self._group.set_position_target(self.target_pose_list[:3],\
        #                 self._group.get_end_effector_link())

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
            #pre-grasp (somewhere on top)
            #pre_target_pose_list = copy.deepcopy(self.target_pose_list[:3])
            #pre_target_pose_list[2] = 0.15
            #pre_target_pose = copy.deepcopy(self._target_pose)
            #pre_target_pose.position.z = 0.15
            #pre_target_pose_execution = self.call_plan_and_execute('goal', \
            #            pre_target_pose_list, pre_target_pose)

            # go to pose
            target_pose_execution = self.call_plan_and_execute('goal', self.target_pose_list[:3], self._target_pose)


            # first open gripper
            node_logger.debug('Opening gripper...')
            # target pose
            for x in range(0,5):
                self._gripper_pub.publish(self.control_gripper(GRIPPER_JOINT_NAMES, GRIPPER_OPEN))
            rospy.sleep(3)

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
        #default_pose_execution = True
        #if self.default_pose_list:
        #    default_pose_execution = self.call_plan_and_execute('default', self.default_pose_list[:3], self._default_pose)
        #    #3.2487032170468826, 2.582055259778615, -3.1800517584293253, 2.969312741099614, 4.928763953218504
        #node_logger.info("===Target Pose Execution: {0}, Default Pose Execution: {1}===".format(target_pose_execution, default_pose_execution))
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
    parser.add_argument('--target_pose_topic', type=str, help='Specify target pose topic.', nargs='?', const='/target_pose', default='/target_pose')
    parser.add_argument('--target_pose', type=str, help='Specify target pose in str format [x, y, z, x, y, z, w]. Priorty over target_pose_topic', nargs='?', \
                                const='[0, 0, 0, 0, 0, 0, 0]', default='[0, 0, 0, 0, 0, 0, 0]')
    #parser.add_argument('--gripper_topic', type=str, help='Specify topic of the gripper to close.', nargs='?', \
    #                                       const='arm_1/gripper_controller/position_command', default='arm_1/gripper_controller/position_command')
    parser.add_argument('--z_offset', type=float, help='z offset for target pose in meters.', nargs='?', const=0.3, default=0.3)
    #parser.add_argument('--default_pose', type=str, help='Specify default pose after target pose in str format [x, y, z, x, y, z, w].', nargs='?', \
    #                            const='[]', default='[]')

    args, unknown = parser.parse_known_args()
    node_logger.debug(args)

    rospy.init_node(args.node_name)

    a = PlanPathAction(args)
    rospy.on_shutdown(a.shutdown)
    #a.test_points()
    #import time
    #while not rospy.is_shutdown():
    #    a.plan_traj()
    #    time.sleep(10)

    rospy.spin()

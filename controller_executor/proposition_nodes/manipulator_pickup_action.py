#! /usr/bin/env python
import rospy
import argparse
import logging
import sys
import traceback
import time

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import trajectory_msgs.msg

import node_logging
node_logger = logging.getLogger("node_logger")

# for testing
#rostopic pub /target_pose geometry_msgs/Pose '{position:  {x: 0.20, y: 0.0, z: 0.70}, orientation: {w: 1.0, x: 0.0,y: 0.0,z: 0.0}}'
#roslaunch youbot_moveit move_group.launch
#roslaunch youbot_gazebo_robot youbot.launch world:="empty_world"
#roslaunch youbot_moveit moveit_rviz.launch

class PlanPathAction(object):
    def __init__(self, args):
        allow_replanning = True
        planning_timeout = 20.0
        self._picking_up = False

        # set up subscriber for target pose info
        self._target_pose = None
        rospy.Subscriber(args.target_pose_topic, geometry_msgs.msg.Pose, callback=self.target_pose_callback)

        try:
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
        except:
            node_logger.error("Unexpected error:")
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_tb(exc_traceback, limit=1, file=sys.stdout)
            traceback.print_exception(exc_type, exc_value, exc_traceback,
                              limit=2, file=sys.stdout)
            self.shutdown()

        # also subscribe to controller request
        rospy.Subscriber(args.node_subscribe_topic, std_msgs.msg.Bool, callback=self.executor_callback)

        # publisher for action status
        #self._pub_find_path = rospy.Publisher(args.node_subscribe_topic+'_find_path', std_msgs.msg.Bool, queue_size=10, latch=True)

        # publisher for ac status
        self._pub_status = rospy.Publisher(args.node_subscribe_topic+'_status', std_msgs.msg.Bool, queue_size=10, latch=True)

    def shutdown(self):
        # clean up moveit
        node_logger.info("===== shutdown: Shutting down moveit! ==================")
        moveit_commander.roscpp_shutdown()

    def target_pose_callback(self, data):
        self._target_pose = data

    def executor_callback(self, data):
        if data.data:
            node_logger.debug("Action turns True!")
            if not self._picking_up:
                self._picking_up = True

                # publish a demo scene
                cube_default = 0.02
                given_name = 'tag_0'
                #planning_frame = '/base_link'
                #given_name = 'pillar'

                self._group.pick('pillar') #, grasp = [])


                cube_increment = 0.01
                loc_increment = 0.01

                for cube in range(10):
                    cube_size = cube_default + cube*cube_increment

                    for x in range(100):
                        for y in range(100):
                            for z in range(100):

                                #self._scene.remove_world_object(given_name)
                                p = geometry_msgs.msg.PoseStamped()
                                p.header.frame_id = self._robot.get_planning_frame()
                                p.pose.position.x = -0.1 + x*loc_increment
                                p.pose.position.y = 0 + y*loc_increment
                                p.pose.position.z = 0 + z*loc_increment
                                p.pose.orientation.w = 1.0
                                #p.pose.position = self._target_pose.position
                                #p.pose.orientation = self._target_pose.orientation
                                node_logger.debug(p)

                                #self._scene.attach_box(self._robot.get_planning_frame(), given_name, pose = p, \
                                #    size = (cube_size, cube_size, cube_size))#, touch_links = [])

                                #self._scene.remove_world_object(given_name)
                                self._scene.add_box(given_name, p, (cube_size, cube_size, cube_size))
                                node_logger.debug("Added scene object of size {0}!".format(cube_size))
                                #node_logger.debug(self._robot.get_planning_frame())
                                #node_logger.debug(dir(moveit_commander.PlanningSceneInterface))

                                #node_logger.debug(self._scene.get_attached_objects())
                                #node_logger.debug(self._scene.get_objects())

                                # give it some time to show the scene object
                                time.sleep(1)

                                # now pick up object
                                if self._group.pick(given_name): #, grasp = [])
                                    node_logger.debug("Done.")
                                else:
                                    node_logger.debug("Failed.")

                                time.sleep(1)

                self._pub_status.publish(True)

        else:
            node_logger.debug("Action turns False!")
            self._picking_up = False
            self._group.stop()

            self._pub_status.publish(False)





if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pickup action for manipulator with MoveIt!")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_subscribe_topic', type=str, help='Specify controller topic to respond to')
    parser.add_argument('--group_name', type=str, help='Specify MoveIt! group name to use.', nargs='?', const='arm_1', default='arm_1')
    parser.add_argument('--target_pose_topic', type=str, help='Specify target pose topic.', nargs='?', const='/target_pose', default='/target_pose')
    #parser.add_argument('--gripper_topic', type=str, help='Specify topic of the gripper to close.', nargs='?', \
    #                                       const='arm_1/gripper_controller/position_command', default='arm_1/gripper_controller/position_command')


    args, unknown = parser.parse_known_args()
    node_logger.debug(args)

    rospy.init_node(args.node_name)

    a = PlanPathAction(args)
    rospy.on_shutdown(a.shutdown)

    rospy.spin()

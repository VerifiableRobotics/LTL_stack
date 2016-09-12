#! /usr/bin/env python
import rospy
import argparse
import logging

import std_msgs.msg
import sensor_msgs.msg

import node_logging
node_logger = logging.getLogger('node_logger')


class CheckGripperEffort(object):
    def __init__(self, args):

        # then check (joint_states)
        rospy.Subscriber(args.joint_state_topic, sensor_msgs.msg.JointState, callback=self.joint_state_callback)

        self._pub = rospy.Publisher(args.node_publish_topic, std_msgs.msg.Bool, queue_size=10)

        self._l_finger = args.gripper_finger_l
        self._r_finger = args.gripper_finger_r

    def joint_state_callback(self, data):
        idx_l_finger = data.name.index(self._l_finger)
        idx_r_finger = data.name.index(self._r_finger)

        self._pub.publish(True)
        """
        if data.position[idx_l_finger] < 0.05 or data.position[idx_r_finger] > 0.05: # holding object
            self._pub.publish(True)
        else:
            self._pub.publish(False)
        """

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This node subscribes pose info from gazebo and determine which region the robot is in.")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_publish_topic', type=str, help='Specify name of publishing topic to the controller')
    parser.add_argument('--joint_state_topic', type=str, help='Name of the joint states topic', nargs='?', const='/joint_states', default='/joint_states')
    parser.add_argument('--gripper_finger_l', type=str, help='Name of the gripper finger left', nargs='?', const='gripper_finger_joint_l', default='gripper_finger_joint_l')
    parser.add_argument('--gripper_finger_r', type=str, help='Name of the gripper finger right', nargs='?', const='gripper_finger_joint_r', default='gripper_finger_joint_r')

    args, unknown = parser.parse_known_args()
    node_logger.debug(args)

    rospy.init_node(args.node_name)

    a = CheckGripperEffort(args)

    rospy.spin()
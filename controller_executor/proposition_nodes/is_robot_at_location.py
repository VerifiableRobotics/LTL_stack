#! /usr/bin/env python
import rospy
import argparse
import logging

import std_msgs.msg

import node_logging
node_logger = logging.getLogger('node_logger')


class RobotLocationPublisher(object):
    def __init__(self, node_publish_topic, robot_location_topic, region_in_consideration):
        self._region_in_consideration = region_in_consideration

        # setup publisher
        self.pub = rospy.Publisher(node_publish_topic, std_msgs.msg.Bool, queue_size=10)

        # subscribe to current pose info
        rospy.Subscriber(robot_location_topic, std_msgs.msg.String, callback=self.update_region)

    def update_region(self, data):
        if data.data == self._region_in_consideration:
            node_logger.log(4, 'We are in region {0}'.format(data.data))
            self.pub.publish(True)
        else:
            self.pub.publish(False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This node subscribes pose info from gazebo and determine which region the robot is in.")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_publish_topic', type=str, help='Specify name of publishing topic to the controller')
    parser.add_argument('--robot_location_topic', type=str, help='Name of the robot location topic to subscribe to')
    parser.add_argument('--region_in_consideration', type=str, help='region to check occupancy')

    args, unknown = parser.parse_known_args()
    node_logger.debug(args)

    rospy.init_node(args.node_name)

    a = RobotLocationPublisher(args.node_publish_topic, args.robot_location_topic, args.region_in_consideration)

    rospy.spin()



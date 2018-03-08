#! /usr/bin/env python
import rospy
import argparse
import logging

import turtlesim.msg
import std_msgs.msg
import geometry_msgs.msg
import actionlib_msgs.msg

import node_logging
node_logger = logging.getLogger("node_logger")

class SetColorActuator(object):
    def __init__(self):
        self.controller_request_bool = False
        # move base cancel pub

    def callback(self, data):
        # save latest info
        self.controller_request_bool = data.data
        #print data.data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Set color of robot.")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_subscribe_topic', type=str, help='Specify controller topic to respond to')
    parser.add_argument('--publish_topic', type=str, help='Specify name of topic to publish color info', nargs='?', \
                            const='/set_color', default='/set_color')
    parser.add_argument('--r', type=float, help="linear x color", nargs='?', const=255, default=255)
    parser.add_argument('--g', type=float, help="linear y color", nargs='?', const=255, default=255)
    parser.add_argument('--b', type=float, help="linear z color", nargs='?', const=255, default=255)
    parser.add_argument('--a', type=float, help="angular x color", nargs='?', const=0, default=0)

    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()

    rospy.init_node(args.node_name)
    a = SetColorActuator()

    # subsribe to request
    rospy.Subscriber(args.node_subscribe_topic, std_msgs.msg.Bool, callback=a.callback)

    # publish to topics
    pub = rospy.Publisher(args.publish_topic, std_msgs.msg.ColorRGBA, queue_size=10)
    rate = rospy.Rate(10) # set publish rate

    color_msg = std_msgs.msg.ColorRGBA()
    color_msg.r = args.r
    color_msg.g = args.g
    color_msg.b = args.b
    color_msg.a = args.a

    while not rospy.is_shutdown():
        if a.controller_request_bool:
            pub.publish(color_msg)
            #node_logger.debug(color_msg)
        rate.sleep()

#python set_color.py turn_white /turn_white

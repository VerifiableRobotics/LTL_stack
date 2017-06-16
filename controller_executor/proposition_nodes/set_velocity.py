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

class SetVelocityActuator(object):
    def __init__(self):
        self.controller_request_bool = False
        # move base cancel pub
        self._move_base_cancel_pub = rospy.Publisher('/move_base/cancel', actionlib_msgs.msg.GoalID, queue_size=1)

    def callback(self, data):
        # save latest info
        self.controller_request_bool = data.data
        #print data.data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Set velocity of robot.")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_subscribe_topic', type=str, help='Specify controller topic to respond to')
    parser.add_argument('--publish_topic', type=str, help='Specify name of topic to publish velocity info', nargs='?', \
                            const='/turtle1/cmd_vel', default='/turtle1/cmd_vel')
    parser.add_argument('--vx', type=float, help="linear x velocity", nargs='?', const=0, default=0)
    parser.add_argument('--vy', type=float, help="linear y velocity", nargs='?', const=0, default=0)
    parser.add_argument('--vz', type=float, help="linear z velocity", nargs='?', const=0, default=0)
    parser.add_argument('--ax', type=float, help="angular x velocity", nargs='?', const=0, default=0)
    parser.add_argument('--ay', type=float, help="angular y velocity", nargs='?', const=0, default=0)
    parser.add_argument('--az', type=float, help="angular z velocity", nargs='?', const=0, default=0)

    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()

    rospy.init_node(args.node_name)
    a = SetVelocityActuator()

    # subsribe to request
    rospy.Subscriber(args.node_subscribe_topic, std_msgs.msg.Bool, callback=a.callback)

    # publish to topics
    pub = rospy.Publisher(args.publish_topic, geometry_msgs.msg.Twist, queue_size=10)
    rate = rospy.Rate(10) # set publish rate

    vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(args.vx, args.vy, args.vz), \
                                      geometry_msgs.msg.Vector3(args.ax, args.ay, args.az))

    vel_msg_zero = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), \
                                      geometry_msgs.msg.Vector3(0,0,0))
    while not rospy.is_shutdown():
        if a.controller_request_bool:
            a._move_base_cancel_pub.publish(actionlib_msgs.msg.GoalID())

            pub.publish(vel_msg)
            node_logger.log(4, vel_msg)
        #else: # stop robot
        #    pub.publish(vel_msg_zero)
        rate.sleep()

#python set_velocity.py pick_up /picup  --az 1

#! /usr/bin/env python
import rospy
import argparse

import std_msgs.msg

def callback(data):
    rospy.loginfo(data)
    pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dummy actuator. Only subscribing to a topic but not doing anything else.")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_subscribe_topic', type=str, help='Specify controller topic to respond to')

    args, unknown = parser.parse_known_args()

    rospy.init_node(args.node_name)

    # subsribe to request
    rospy.Subscriber(args.node_subscribe_topic, std_msgs.msg.Bool, callback=callback)

    rospy.spin()

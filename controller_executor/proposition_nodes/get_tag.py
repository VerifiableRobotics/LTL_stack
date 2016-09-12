#! /usr/bin/env python
import rospy
import argparse
import logging

import std_msgs.msg
import apriltags_ros.msg

import node_logging
node_logger = logging.getLogger("node_logger")

class TagSensor(object):
    def __init__(self):
        self.tag_info_list = apriltags_ros.msg.AprilTagDetectionArray()

    def callback(self, data):
        # save latest info
        self.tag_info_list = data
        print data

    # publish information
    def convert_to_bool(self, tag_no):
        #node_logger.debug('Tag detected: {0}'.format(self.tag_info_list.detections))
        for tag_info in self.tag_info_list.detections:
            if tag_info.id == tag_no:
                node_logger.debug('Tag {0} is True!'.format(tag_no))
                return True
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Get AprilTag")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_publish_topic', type=str, help='Specify name of publishing topic to the controller')
    parser.add_argument('--sensor_topic', type=str, help='Specify tag topic', nargs='?', const='/tag_detections', default='/tag_detections')
    parser.add_argument('--tag_no', type=int, help="tag number", nargs='?', const=0, default=0)

    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()
    print args

    rospy.init_node(args.node_name)
    a = TagSensor()

    # subsribe to necessary information
    rospy.Subscriber(args.sensor_topic, apriltags_ros.msg.AprilTagDetectionArray, callback=a.callback)

    # publish to topics
    pub = rospy.Publisher(args.node_publish_topic, std_msgs.msg.Bool, queue_size=10)
    rate = rospy.Rate(10) # set publish rate
    while not rospy.is_shutdown():
        pub.publish(a.convert_to_bool(args.tag_no))
        rate.sleep()

#python get_tag.py 'get_object' '/get_object/get_object' --sensor_topic '/tag_detections' --tag_no 0

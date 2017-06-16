#! /usr/bin/env python
import rospy
import argparse
import logging
import math
import time

import std_msgs.msg
import apriltags_ros.msg

import node_logging
node_logger = logging.getLogger("node_logger")

class TagSensor(object):
    def __init__(self, args):
        self.tag_info_list = apriltags_ros.msg.AprilTagDetectionArray()
        self.distance_thres = args.distance_thres
        self.duration_thres = args.duration_thres
        self.last_seen = 0

    def callback(self, data):
        # save latest info
        self.tag_info_list = data
        print data

    # publish information
    def convert_to_bool(self, tag_no):
        #node_logger.debug('Tag detected: {0}'.format(self.tag_info_list.detections))
        for tag_info in self.tag_info_list.detections:
            if tag_info.id == tag_no:
                node_logger.log(4, 'Tag {0} is True. Check within distance!'.format(tag_no))
                dist = math.sqrt(tag_info.pose.pose.position.z**2 + \
                                 tag_info.pose.pose.position.x**2)
                if self.distance_thres > dist:
                    node_logger.log(4, 'Tag {0} is within distance!'.format(tag_no))
                    self.last_seen = time.time()
                    return True
                else:
                    node_logger.debug('Dist: {0}'.format(dist))

        # now the tag sensor has a duration to avoid sensor jumping
        if (time.time() - self.last_seen) < self.duration_thres:
            return True

        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Get AprilTag")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_publish_topic', type=str, help='Specify name of publishing topic to the controller')
    parser.add_argument('--sensor_topic', type=str, help='Specify tag topic', nargs='?', const='/tag_detections', default='/tag_detections')
    parser.add_argument('--tag_no', type=int, help="tag number", nargs='?', const=0, default=0)
    parser.add_argument('--distance_thres', type=int, help="distance to count as true", nargs='?', const=0.60, default=0.60)
    parser.add_argument('--duration_thres', type=int, help="Sensor duration since last sensed.", nargs='?', const=0.5, default=0.5)

    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()
    print args

    rospy.init_node(args.node_name)
    a = TagSensor(args)

    # subsribe to necessary information
    rospy.Subscriber(args.sensor_topic, apriltags_ros.msg.AprilTagDetectionArray, callback=a.callback)

    # publish to topics
    pub = rospy.Publisher(args.node_publish_topic, std_msgs.msg.Bool, queue_size=10)
    rate = rospy.Rate(10) # set publish rate
    while not rospy.is_shutdown():
        pub.publish(a.convert_to_bool(args.tag_no))
        rate.sleep()

#python get_tag.py 'get_object' '/get_object/get_object' --sensor_topic '/tag_detections' --tag_no 0

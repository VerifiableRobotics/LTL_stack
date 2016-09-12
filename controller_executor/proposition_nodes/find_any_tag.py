#! /usr/bin/env python
import rospy
import argparse
import logging

import std_msgs.msg
import apriltags_ros.msg
import controller_executor.msg

import node_logging
node_logger = logging.getLogger("node_logger")

class TagSensor(object):
    def __init__(self, args):
        self.tag_info_list = apriltags_ros.msg.AprilTagDetectionArray()
        #self.tag_saved = apriltags_ros.msg.AprilTagDetectionArray()
        self.tag_saved = []

        self.pub_status = rospy.Publisher(args.node_subscribe_topic+"_status", std_msgs.msg.Bool, queue_size=10,latch=True)
        self.pub_status.publish(False)

        self.pub_tag_found = rospy.Publisher(args.tag_found_topic, std_msgs.msg.Bool, queue_size=10,latch=True)
        self.pub_tag_found.publish(False)

        #self.pub_tag = rospy.Publisher(args.tag_topic, apriltags_ros.msg.AprilTagDetectionArray, queue_size=10,latch=True)
        self.pub_tag = rospy.Publisher(args.tag_list_topic, controller_executor.msg.StringArray, queue_size=10, latch=True)
        self.pub_tag.publish(controller_executor.msg.StringArray())

    def tag_callback(self, data):
        # save latest info
        self.tag_info_list = data
        #node_logger.debug(data)

    def controller_request_callback(self, data):
        #node_logger.debug('Get a callback...')
        if data.data: # if proposition is true
            if self.tag_info_list.detections:
                # change this to list of target_frames
                #self.tag_saved = self.tag_info_list #save a copy
                self.tag_saved = ["tag_"+str(x.id) for x in self.tag_info_list.detections]
                node_logger.debug("Find tags: {0}".format(self.tag_saved))

                # publish tag info
                self.pub_tag.publish(self.tag_saved)
                self.pub_tag_found.publish(True)
                node_logger.debug('Detect tags...')
            else:
                # apriltag array
                #self.pub_tag.publish(apriltags_ros.msg.AprilTagDetectionArray())
                self.pub_tag.publish([])
                self.pub_tag_found.publish(False)
                node_logger.debug('Did not detect any tags...')

            # publish status of action finished
            self.pub_status.publish(True)

        else:
            # publish status of action is off
            self.pub_status.publish(False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This nodes gets any April Tags and forward status that they are found.")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_subscribe_topic', type=str, help='Specify controller topic to respond to')
    parser.add_argument('--tag_found_topic', type=str, help='Specify boolean status of whether tags are found', nargs='?', \
                            const='/tag_found', default='/tag_found')
    parser.add_argument('--sensor_topic', type=str, help='Specify tag topic from camera', nargs='?', const='/tag_detections', default='/tag_detections')
    parser.add_argument('--tag_list_topic', type=str, help='Specify tag topic that used to forward tag info', nargs='?', const='/tag_list', default='/tag_list')

    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()
    print args

    rospy.init_node(args.node_name)
    a = TagSensor(args)

    # subsribe to necessary information
    rospy.Subscriber(args.sensor_topic, apriltags_ros.msg.AprilTagDetectionArray, callback=a.tag_callback)

    # subscribe to controller info
    rospy.Subscriber(args.node_subscribe_topic, std_msgs.msg.Bool, callback=a.controller_request_callback)

    rospy.spin()


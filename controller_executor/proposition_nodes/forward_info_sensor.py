#! /usr/bin/env python
import rospy
import argparse
import importlib
import logging

import std_msgs.msg

import node_logging
node_logger = logging.getLogger("node_logger")

class ForwardTopicSensor(object):
    def __init__(self, topic_type):
        self.empty_topic_info = eval(topic_type)() #for broadcasting empty array
        self.topic_info = eval(topic_type)()

    def topic_info_callback(self, data):
        # save latest info
        self.topic_info = data
        rospy.logdebug("Latest data: {0}".format(data))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sensor Example.")
    parser.register('type', 'bool', (lambda x: x.lower() in ("yes", "true", "t", "1")))
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_publish_topic', type=str, help='Specify controller topic to respond to')
    parser.add_argument('--sensor_topic', type=str, help='Specify name of topic to subscribe to original info. Bool topic', nargs='?', \
                            const='/tag_detections', default='/tag_detections')
    parser.add_argument('--topic_type', type=str, help='Specify msg type of subscribe and publish topic', nargs='?', \
                            const='apriltags_ros.msg.AprilTagDetectionArray', default='apriltags_ros.msg.AprilTagDetectionArray')

    args, unknown = parser.parse_known_args()

    # import module
    #module = __import__(args.topic_type.partition(".msg.")[0] + '.msg')
    module = importlib.import_module(args.topic_type.partition(".msg.")[0]) # import module
    globals()[args.topic_type.partition(".msg.")[0]] = module
    module_msg = importlib.import_module(args.topic_type.partition(".msg.")[0]+'.msg') # import msg
    globals()[args.topic_type.partition(".msg.")[0]+'.msg'] = module_msg

    rospy.init_node(args.node_name)
    a = ForwardTopicSensor(args.topic_type)

    # subscribe to original info
    rospy.Subscriber(args.sensor_topic, eval(args.topic_type), callback=a.topic_info_callback)

    # publish to topics
    pub = rospy.Publisher(args.node_publish_topic, eval(args.topic_type), queue_size=10)
    rate = rospy.Rate(10) # set publish rate

    while not rospy.is_shutdown():
        # forward
        pub.publish(a.topic_info)
        rospy.loginfo("Publishing: {0}".format(a.topic_info))
        node_logger.log(2, "Publishing: {0}".format(a.topic_info))

        rate.sleep()


#! /usr/bin/env python
import rospy
import argparse
import importlib
import logging

import std_msgs.msg
#import apriltags_ros.msg

import node_logging
node_logger = logging.getLogger("node_logger")

class ForwardTopicActuator(object):
    def __init__(self, topic_type):
        self.controller_request_bool = False
        self.empty_topic_info = eval(topic_type)() #for broadcasting empty array
        self.topic_info = eval(topic_type)()

    def callback(self, data):
        # save latest info
        self.controller_request_bool = data.data
        rospy.logdebug("Lastest controller request: {0}".format(data.data))

    def topic_info_callback(self, data):
        # save latest info
        self.topic_info = data
        rospy.logdebug("Latest data: {0}".format(data))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sensor Example.")
    parser.register('type', 'bool', (lambda x: x.lower() in ("yes", "true", "t", "1")))
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_subscribe_topic', type=str, help='Specify controller topic to respond to')
    parser.add_argument('--publish_topic', type=str, help='Specify name of topic to publish topic info', nargs='?', \
                            const='/tag_detections_forward', default='/tag_detections_forward')
    parser.add_argument('--subscribe_info_topic', type=str, help='Specify name of topic to subscribe to original info', nargs='?', \
                            const='/tag_detections', default='/tag_detections')
    parser.add_argument('--topic_type', type=str, help='Specify msg type of subscribe and publish topic', nargs='?', \
                            const='apriltags_ros.msg.AprilTagDetectionArray', default='apriltags_ros.msg.AprilTagDetectionArray')
    parser.add_argument('--toggle_boolean', type='bool', help='True: forward tags when controller topic outputs false', nargs='?', const=False, default=False)

    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()

    # import module
    #module = __import__(args.topic_type.partition(".msg.")[0] + '.msg')
    module = importlib.import_module(args.topic_type.partition(".msg.")[0]) # import module
    globals()[args.topic_type.partition(".msg.")[0]] = module
    module_msg = importlib.import_module(args.topic_type.partition(".msg.")[0]+'.msg') # import msg
    globals()[args.topic_type.partition(".msg.")[0]+'.msg'] = module_msg

    rospy.init_node(args.node_name)
    a = ForwardTopicActuator(args.topic_type)

    # subsribe to request
    rospy.Subscriber(args.node_subscribe_topic, std_msgs.msg.Bool, callback=a.callback)

    # subscribe to original info
    rospy.Subscriber(args.subscribe_info_topic, eval(args.topic_type), callback=a.topic_info_callback)

    # publish to topics
    pub = rospy.Publisher(args.publish_topic, eval(args.topic_type), queue_size=10)
    rate = rospy.Rate(10) # set publish rate

    while not rospy.is_shutdown():
        if (a.controller_request_bool and not args.toggle_boolean)\
            or (not a.controller_request_bool and args.toggle_boolean):
            # True then forward OR False then forward
            pub.publish(a.topic_info)
            rospy.loginfo("Publishing: {0}".format(a.topic_info))
            node_logger.log(2, "Publishing: {0}".format(a.topic_info))
        else:
            node_logger.log(2, "Not forwarding. Publishing empty info.")
            pub.publish(a.empty_topic_info)

        rate.sleep()

#rosrun controller_executor forward_info.py stop_camera /stop_camera/stop_camera

#! /usr/bin/env python
import roslaunch
import rospy
import argparse
import logging

import std_msgs.msg

import node_logging
node_logger = logging.getLogger("node_logger")


class StartNodeActuator(object):
    def __init__(self, start_node_package, start_node_type, start_node_name):
        self.controller_request_bool = False

        # # create node
        # self.node = roslaunch.core.Node(start_node_package, start_node_type, name=start_node_name, output="screen")
        # node_logger.debug("Created node object")

        # # create launch
        # self.launch = roslaunch.scriptapi.ROSLaunch()
        # self.launch.start()
        # node_logger.debug("Created launch object")

        # #self.process = None

    def callback(self,data):
        # save latest info
        self.controller_request_bool = data.data
        # if data.data:
        #     if not self.process:
        #         self.process = self.launch.launch(self.node)
        #         node_logger.debug("Starting node ....")

        # else:
        #     if self.process:
        #         self.process.stop()
        #         self.process = None
        #         node_logger.debug("Stoping node ....")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Start a node in a package. e.g.: <node name="uvc_camera" pkg="uvc_camera" type="uvc_camera_node"/>')
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_subscribe_topic', type=str, help='Specify controller topic to respond to')
    parser.add_argument('--start_node_name', type=str, help='name of the node to start', \
                            nargs='?', const='uvc_camera', default='uvc_camera')
    parser.add_argument('--start_node_package', type=str, help='name of the package to find the available nodes', \
                            nargs='?', const='uvc_camera', default='uvc_camera')
    parser.add_argument('--start_node_type', type=str, help='name of node in the package to spawn', \
                            nargs='?', const='uvc_camera_node', default='uvc_camera_node')
    #e.g.: <node name="uvc_camera" pkg="uvc_camera" type="uvc_camera_node"/>
    parser.register('type', 'bool', (lambda x: x.lower() in ("yes", "true", "t", "1")))
    parser.add_argument('--toggle_effect', type='bool', help='When node_subscribe_topic is false, the node is launched', \
                            nargs='?', const=False, default=False)

    args, unknown = parser.parse_known_args()

    rospy.init_node(args.node_name, disable_signals=True)
    a = StartNodeActuator(args.start_node_package, args.start_node_type, args.start_node_name)

    # subsribe to request
    rospy.Subscriber(args.node_subscribe_topic, std_msgs.msg.Bool, callback=a.callback)

    # create node
    node = roslaunch.core.Node(args.start_node_package, args.start_node_type, name=args.start_node_name, output="screen")
    node_logger.debug("Created node object")

    # create launch
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    node_logger.debug("Created launch object")

    # ** only works in the main thread
    process = None
    update_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if (not args.toggle_effect and a.controller_request_bool and not process) or \
           (args.toggle_effect and not a.controller_request_bool and not process):
            process = launch.launch(node)
            node_logger.debug("Starting node ....")

        elif (not args.toggle_effect and not a.controller_request_bool and process) or \
             (args.toggle_effect and a.controller_request_bool and process):
            process.stop()
            #launch.unregister(args.start_node_name)
            process = None
            #launch.stop()
            #launch = roslaunch.scriptapi.ROSLaunch()
            #launch.start()
            node_logger.debug("Stoping node ....")

    update_rate.sleep()


    #rospy.spin()


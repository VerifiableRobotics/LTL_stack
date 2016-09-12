#! /usr/bin/env python
import argparse
import rospy
import yaml
import logging
import time
import std_msgs.msg

import controller_executor.msg

import file_operations
import controller_executor_logging
input_manager_logger = logging.getLogger("input_manager_logger")

##### EXAMPLES ##
# roscore
# rosrun turtlesim turtlesim_node
#  rosrun turtlesim turtlesim_node __name:=turtle2 /turtle1:=/turtle2

# rosnode cleanup

#rosservice call /turtle1/set_pen 200 0 0 2 0

# python src/input_manager.py examples/simple.yaml
#######################

input_readings = {}

class InputManager(object):
    _input_readings = {}

    def __init__(self, yaml_file, node_name):
        # load yaml file
        input_prop_to_ros_info, output_prop_to_ros_info = file_operations.loadYAMLFile(yaml_file)

        # subscribe to topics
        self.subscribe_to_topics(input_prop_to_ros_info)

        # setup publisher for inputs dict
        self.input_dict_pub = rospy.Publisher(node_name+'/incoming_inputs', controller_executor.msg.stringKeyBoolValueDict, queue_size=10)
        input_manager_logger.info("Finished Input Manager Initialization...")

    def update_value(self, data, prop):
        if not self._input_readings[prop] == data.data:
            input_manager_logger.debug('Prop-{0}:{1}'.format(prop, data.data))

        self._input_readings[prop] = data.data
        rospy.loginfo(self._input_readings)

    def subscribe_to_topics(self, prop_to_ros_info):
        for prop, prop_info in prop_to_ros_info.iteritems():
            self._input_readings[prop] = False
            rospy.Subscriber(prop_info['node_publish_topic'], std_msgs.msg.Bool, callback=self.update_value, callback_args=prop)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Input Manager")
    parser.add_argument('yaml_file', type=str, help='Specify .yaml file directory')
    #parser.add_argument('--input_manager_name', type=str, help='Specify name of the input manager.',\
    #                         nargs='?', const='input_manager', default='input_manager')

    args, unknown = parser.parse_known_args()

    # initialize node
    rospy.init_node('input_manager') #args.input_manager_name)

    # initialize manager
    manager = InputManager(args.yaml_file, 'input_manager') #args.input_manager_name)

    # publish all collected sensors
    input_manager_logger.info("Start publishing inputs ...")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # form message
        inputs = controller_executor.msg.stringKeyBoolValueDict(manager._input_readings.keys(), manager._input_readings.values())

        # publish
        manager.input_dict_pub.publish(inputs)

        # wait
        rate.sleep()



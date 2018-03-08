#! /usr/bin/env python
import argparse
import rospy
import yaml
import time
import logging
import std_msgs.msg

import controller_executor.msg

import file_operations
import controller_executor_logging
output_manager_logger = logging.getLogger("output_manager_logger")

class OutputManager(object):
    _output_publishers = {}
    _output_dict = {}

    def __init__(self, yaml_file):

        # load yaml file
        input_prop_to_ros_info, output_prop_to_ros_info = file_operations.loadYAMLFile(args.yaml_file)

        # set up ros publishers
        self.setup_publish_topics(output_prop_to_ros_info)

        # subscribe to controller (need to first create the manager)
        rospy.Subscriber('executor/incoming_outputs', controller_executor.msg.stringKeyBoolValueDict, callback=self.update_outputs)

    def update_outputs(self, data):
        # print new info
        if dict(zip(data.keys, data.values)) != self._output_dict:
            output_manager_logger.debug('Publishing outputs:{0}'.format(dict(zip(data.keys, data.values))))
            self._output_dict =dict(zip(data.keys, data.values))

        for idx, prop in enumerate(data.keys):
            for pub in self._output_publishers[prop]:
                pub.publish(data.values[idx])
            #output_manager_logger.debug('OUTPUT prop {0}:{1}'.format(prop, data.values[idx]))

    def setup_publish_topics(self, prop_to_ros_info):
        for prop, prop_info in prop_to_ros_info.iteritems():
            if not prop in self._output_publishers.keys():
                self._output_publishers[prop] = []
            if isinstance(prop_info, list):
                for prop_info_element in prop_info:
                    self._output_publishers[prop].append(rospy.Publisher(prop_info[0]['node_subscribe_topic'], std_msgs.msg.Bool, queue_size=10))
            else:
                self._output_publishers[prop].append(rospy.Publisher(prop_info['node_subscribe_topic'], std_msgs.msg.Bool, queue_size=10))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Launch output manager")
    parser.add_argument('yaml_file', type=str, help='Path to yaml file')

    args, unknown = parser.parse_known_args()

    rospy.init_node('output_manager')

    output_manager = OutputManager(args.yaml_file)

    rospy.spin()







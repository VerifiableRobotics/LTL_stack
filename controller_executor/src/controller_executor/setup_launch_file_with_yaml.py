#! /usr/bin/env python
import argparse
import yaml
import logging

import controller_executor_logging
setup_execution_logger = logging.getLogger("setup_execution_logger")

from controller_executor import setup_launch_file

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This node subscribes pose info from gazebo and determine which region the robot is in.")
    parser.add_argument('yaml_file', type=str, help='Directory to yaml file.', nargs='?', default='/home/catherine/LTLROS_ws/src/controller_executor/examples/firefighting_test/firefighting_robot_location.yaml')

    args, unknown = parser.parse_known_args()
    setup_execution_logger.debug(args)

    # load yaml and form args string
    with open(args.yaml_file, 'r') as stream:
        try:
             setup_args = yaml.load(stream)
        except yaml.YAMLError as exc:
            setup_execution_logger.error("ERROR: {0}".format(exc))

    # form arguments for setup_launch file
    args_to_pass = parser.parse_args()
    arg_to_pass_dict = args_to_pass.__dict__
    for key, value in setup_args.items():
        arg_to_pass_dict[key] = value

    # call setup_launch_file
    setup_execution_logger.debug("To pass args: {0}".format(args_to_pass))
    setup_launch_file.main(args_to_pass)

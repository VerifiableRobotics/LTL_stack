#! /usr/bin/env python
import rospy
import argparse
import logging
import copy

import std_msgs.msg
import controller_executor.msg
import slugs_ros.msg
import geometry_msgs.msg
import actionlib_msgs.msg

from slugs_ros import slugs_requests
import controller_executor_logging
controller_logger = logging.getLogger("controller_logger")

class ControllerExecutor(object):
    _current_inputs = {}
    _incoming_inputs = {}
    _current_outputs = {}
    _incoming_outputs = {}
    _output_list = []
    _input_list = []
    _publisher_rate = 10 # setup publisher rate

    def __init__(self, ltl_filename, synthesis_options, run_executor, init_state_dict={}):
        #self.namespace = namespace
        self.server_name = 'server'
        self._ros_publisher_rate_obj = rospy.Rate(self._publisher_rate) # set publish rate
        self._run_executor = run_executor

        ##### STOP ROBOT #####
        # move base cancel pub
        self._move_base_cancel_pub = rospy.Publisher('/move_base/cancel', actionlib_msgs.msg.GoalID, queue_size=1)

        # pub vel
        self.vel_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        ######################

        # subscribe to input information
        rospy.Subscriber('input_manager/incoming_inputs', controller_executor.msg.stringKeyBoolValueDict, callback=self.update_inputs)

        # subscribe to run executor status
        rospy.Subscriber('executor/run_executor', std_msgs.msg.Bool, callback=self.update_run_executor_status)

        # setup publisher for outputs
        self.pub = rospy.Publisher('executor/incoming_outputs', controller_executor.msg.stringKeyBoolValueDict, queue_size=10)

        # wait for controller to start.
        #controller_logger.info("Ready to Start. Please run executor.")
        #while not self._run_executor:
        #    rospy.Rate(1).sleep()

        # initialize slugs
        slugs_requests.start_slugs_action_client(self.server_name, ltl_filename, synthesis_options)
        init_state_key_list, init_state_value_list = \
            slugs_requests.slugs_init_execution_array_client(self.server_name, init_state_dict.keys(),\
                                                                        init_state_dict.values())
        # publish init outputs
        init_state_dict = dict(zip(init_state_key_list, init_state_value_list))

        # wait for input and output list service
        rospy.wait_for_service(self.server_name+'/slugs_get_outputs_service')
        self._output_list = slugs_requests.slugs_get_outputs_client(self.server_name)
        rospy.wait_for_service(self.server_name+'/slugs_get_inputs_service')
        self._input_list = slugs_requests.slugs_get_inputs_client(self.server_name)

        # extract outputs only
        self._current_outputs = {k: init_state_dict[k] for k in self._output_list if k in init_state_dict}
        self._current_inputs = {k: init_state_dict[k] for k in self._input_list if k in init_state_dict}
        controller_logger.info("Initial inputs: {0}".format(self._current_inputs))
        controller_logger.info("Initial outputs: {0}".format(self._current_outputs))

        outputs = controller_executor.msg.stringKeyBoolValueDict(self._current_outputs.keys(), self._current_outputs.values())
        self.pub.publish(outputs)

    def update_inputs(self, data):
        self._current_inputs = self._incoming_inputs # shouldn't change with current_inputs
        self._incoming_inputs = dict(zip(data.keys, data.values))
        # print only when there's state change
        if self._current_inputs != self._incoming_inputs:
            controller_logger.info("--- Run Executor:{0} ---".format(self._run_executor))
            controller_logger.info("Current Inputs: {0}".format(self._current_inputs))
            controller_logger.info("Current Outputs: {0}".format(self._current_outputs))
            controller_logger.info("Incoming Inputs: {0}".format(self._incoming_inputs))

        # check if we are running
        if self._run_executor:
            # make transition here.
            return_key_list, return_value_list = slugs_requests.slugs_trans_execution_array_client(\
                        self.server_name, self._incoming_inputs.keys(), self._incoming_inputs.values())

            # update outputs
            self._current_outputs = self._incoming_outputs

            return_dict = dict(zip(return_key_list, return_value_list))
            self._incoming_outputs = {k: return_dict[k] for k in self._output_list if k in return_dict}
            # print only when there's state change
            if self._current_outputs != self._incoming_outputs:
                controller_logger.info("Incoming Outputs: {0}".format(self._incoming_outputs))

            # publish information
            outputs = controller_executor.msg.stringKeyBoolValueDict(\
                    self._incoming_outputs.keys(), self._incoming_outputs.values())
            #controller_logger.debug("outputs: {0}".format(outputs))
            self.pub.publish(outputs)
            self._ros_publisher_rate_obj.sleep()

        else:
            ## Pause everything ##
            # Stop robot
            zero_vel_obj = geometry_msgs.msg.Twist()
            controller_logger.debug('Stopping robot...')
            self.vel_pub.publish(zero_vel_obj)

            # cancel actions
            self._move_base_cancel_pub.publish(actionlib_msgs.msg.GoalID())

    def update_run_executor_status(self, data):
        self._run_executor = data.data


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Controller Executor.")
    parser.register('type', 'bool', (lambda x: x.lower() in ("yes", "true", "t", "1"))) # fix bool problem
    parser.add_argument('ltl_filename', type=str, help='Specify .slugsin file')
    #parser.add_argument('slugs_namespace', type=str, help='Specify namespace of slugs.')
    parser.add_argument('--synthesis_options', action='append', help='Synthesis options. Without --') # nargs='*'
    parser.add_argument('--run_executor', type='bool', help='If execution starts immediately.',\
                           nargs='?', const=True, default=True)
    parser.add_argument('--init_props', action='append', help='Specify initial true propositions')

    args, unknown = parser.parse_known_args()
    #controller_logger.debug(args)

    rospy.init_node('executor')#args.slugs_client_name)

    # initialize controller. Note that server = namespace+'/server'
    controller = ControllerExecutor(args.ltl_filename, \
        ['--'+x for x in args.synthesis_options], args.run_executor, {x:True for x in args.init_props} if args.init_props else {})

    # keep the controller running
    rospy.spin()


#! /usr/bin/env python
import time
import subprocess
import io
import os
import sys
import logging
import threading
import re
import argparse
try:
    from Queue import Queue, Empty
except ImportError:
    from queue import Queue, Empty  # python 3.x

import roslib; roslib.load_manifest("slugs_ros")
import rospy
import actionlib
import slugs_ros.msg, slugs_ros.srv

import slugs_logging
slugs_logger = logging.getLogger("slugs_logger")

from slugs_startup_server_base import SlugsSynthesisBase, SlugsExecutorBase

class SlugsSynthesisAction(SlugsSynthesisBase):
    """
    This class prepares and synthesizes controller.
    """

    ### ABSTRACT CLASSES ###
    def init_feedback_and_result_obj(self):
        # create messages that are used to publish feedback/result
        self._feedback = slugs_ros.msg.SlugsSynthesisFeedback()
        self._result   = slugs_ros.msg.SlugsSynthesisResult()


    def init_implementation(self, args):
        # abstract class. initialize all required variables/calls for the implementation.
        self._as = actionlib.SimpleActionServer(self._action_name+"/slugs_synthesis_action", slugs_ros.msg.SlugsSynthesisAction, \
                                                execute_cb=self.synthesis_cb, auto_start = False)
        slugs_logger.info("Starting SlugsSynthesisAction server with name: {name}".format(name=self._action_name+"/slugs_synthesis_action"))

        self._as.start()

        # register shutdown
        rospy.on_shutdown(self.shutdown)


    def get_execution_status(self):
        # abstract class. Determine and return result of whether exectuion should be aborted
        return not rospy.is_shutdown()

    def handle_no_interactive_flag(self):
        # abstract class. Actions to perform when interactiveStrategy option is not supplemented
        slugs_logger.info("Not using interactiveStrategy")
        slugs_cmd.append(inputs.output_filename)

    def handle_intermediate_synthesis(self, syn_start_time):
        # abstract class. Actions to perform when the synthesis process is running (providing feedback for example)
        self._feedback.elapsed_time = time.time() - syn_start_time
        self._as.publish_feedback(self._feedback)
        r = rospy.Rate(1)
        r.sleep()

        # check if action is terminated
        if self._as.is_preempt_requested():
            slugs_logger.info("%s: Preempted" % self._action_name)
            self._as.set_preempted()

    def handle_completed_synthesis(self):
        # abstract class. Actions to perform when the synthesis process is done
        self._as.set_succeeded(self._result)

    ############################


class SlugsExecutor(SlugsExecutorBase):
    """
    This class handler all services in execution
    """

    ### ABSTRACT CLASSES ###
    def init_implementation(self, args):
        # abstact class. Init objects
        pass

    def create_init_response_obj(self):
        # abstract class. Create a response object
        return slugs_ros.srv.SlugsInitExecutionArrayResponse()

    def create_trans_response_obj(self):
        # abstract class. Create a response object
        return slugs_ros.srv.SlugsTransExecutionArrayResponse()

    def get_execution_status(self):
        # abstract class. Determine and return result of whether exectuion should be aborted
        return not rospy.is_shutdown()
    ########################

    def slugs_set_goal_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsSetGoal, self.handle_set_goal)
        slugs_logger.info("Starting SlugsSetGoal service with name: {name}".format(name=name))

    def slugs_init_execution_string_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsInitExecutionString, self.handle_init_inputs_outputs_string)
        slugs_logger.info("Starting SlugsInitExecution(String) service with name: {name}".format(name=name))

    def slugs_init_execution_array_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsInitExecutionArray, self.handle_init_inputs_outputs_array)
        slugs_logger.info("Starting SlugsInitExecution(Array) service with name: {name}".format(name=name))

    def slugs_get_transition_string_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsTransExecutionString, self.handle_trans_inputs_string)
        slugs_logger.info("Starting SlugsTransExecution(String) service with name: {name}".format(name=name))

    def slugs_get_transition_array_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsTransExecutionArray, self.handle_trans_inputs_array)
        slugs_logger.info("Starting SlugsTransExecution(Array) service with name: {name}".format(name=name))

    #####################################
    ##### Inputs and Outputs Service ####
    #####################################
    def slugs_get_inputs_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsGetInputs, self.handle_get_inputs)
        slugs_logger.info("Starting SlugsGetInputs service with name: {name}".format(name=name))

    def slugs_get_outputs_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsGetOutputs, self.handle_get_outputs)
        slugs_logger.info("Starting SlugsGetOutputs service with name: {name}".format(name=name))

    def handle_get_inputs(self, req):
        response = slugs_ros.srv.SlugsGetInputsResponse()
        response.inputs_array = self._inputs
        return response

    def handle_get_outputs(self, req):
        response = slugs_ros.srv.SlugsGetOutputsResponse()
        response.outputs_array = self._outputs
        return response

    #####################################
    ### Publisher inputs and outputs ####
    #####################################
    def publish_AP_list(self, topic_name, string_list):
        pub = rospy.Publisher(topic_name, slugs_ros.msg.StringArray, queue_size=10)
        rate = rospy.Rate(self._publisher_rate) # set publish rate
        while self.get_execution_status():
            #slugs_logger.info(string_list)
            pub.publish(string_list)
            rate.sleep()

    def start_topic_thread(self, target, args):
        t = threading.Thread(target=target, args=args)
        t.daemon = True # thread dies with the program
        t.start()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Slugs Action Server.")

    args, unknown = parser.parse_known_args()

    rospy.init_node('slugs_server')#args.namespace
    interactive_setup = False
    synthesis_action = SlugsSynthesisAction(rospy.get_name())

    slugs_logger.info('Started slugs action server. Waiting for request...')

    while not rospy.is_shutdown():
        # only start service if interactiveStrategy is called
        if "--interactiveStrategy" in synthesis_action._options and \
            synthesis_action._synthesis_done and synthesis_action._realizable and not interactive_setup:
            # initialize executor
            slugs_executor = SlugsExecutor(synthesis_action)

            slugs_logger.info('Running interactive strategy now ...')

            slugs_logger.info(rospy.is_shutdown())
            # start topics
            slugs_executor.set_inputs()
            slugs_executor.set_outputs()
            slugs_executor.start_topic_thread(slugs_executor.publish_AP_list, (rospy.get_name()+"/input_list", slugs_executor._inputs))
            slugs_executor.start_topic_thread(slugs_executor.publish_AP_list, (rospy.get_name()+"/output_list", slugs_executor._outputs))

            # start inputs/outputs service
            slugs_executor.slugs_get_inputs_service(rospy.get_name()+"/slugs_get_inputs_service")
            slugs_executor.slugs_get_outputs_service(rospy.get_name()+"/slugs_get_outputs_service")

            # start services
            slugs_executor.slugs_init_execution_string_service(rospy.get_name()+"/slugs_init_execution_string_service")
            slugs_executor.slugs_init_execution_array_service(rospy.get_name()+"/slugs_init_execution_array_service")
            slugs_executor.slugs_get_transition_string_service(rospy.get_name()+"/slugs_trans_execution_string_service")
            slugs_executor.slugs_get_transition_array_service(rospy.get_name()+"/slugs_trans_execution_array_service")
            slugs_executor.slugs_set_goal_service(rospy.get_name()+"/slugs_set_goal_service")

            interactive_setup = True




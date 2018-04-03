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
import copy

try:
    from Queue import Queue, Empty
except ImportError:
    from queue import Queue, Empty  # python 3.x

import abc
import signal

import slugs_logging
slugs_logger = logging.getLogger("slugs_logger")

from slugs_startup_server_base import SlugsSynthesisBase, SlugsExecutorBase

class ResultObj(object):
    def __init__(self):
        self.realizable = False
        self.output_message = ""
        self.synthesis_time = 0.0

class ResponseObj(object):
    def __init__(self, key=[], value=[]):
        self.current_inputs_outputs_key_array = key
        self.current_inputs_outputs_value_array = value

    def __iter__(self):
        return iter([self.current_inputs_outputs_key_array, self.current_inputs_outputs_value_array])

class RequestInitStrObj(object):
    def __init__(self, string=""):
        self.init_inputs_outputs = string

class RequestInitArrayObj(object):
    def __init__(self, key=[], value=[]):
        self.init_inputs_outputs_key_array = key
        self.init_inputs_outputs_value_array = value

class RequestTransStrObj(object):
    def __init__(self, string=""):
        self.trans_inputs = string

class RequestTransArrayObj(object):
    def __init__(self, key=[], value=[]):
        self.trans_inputs_key_array = key
        self.trans_inputs_value_array = value

class RequestGoal(object):
    def __init__(self,goal_id=0):
        self.goal_id = goal_id


class SlugsSynthesisStandalone(SlugsSynthesisBase):
    """
    This class prepares and synthesizes controller.
    """

    ### ABSTRACT CLASS ###
    def init_feedback_and_result_obj(self):
        self._result = ResultObj()

    def init_implementation(self, args):
        signal.signal(signal.SIGINT, self.quit_handler)

        result =  self.synthesis_cb(args)
        if result.realizable:
            slugs_logger.info("The spec is realizable. Ready to execute.")
        else:
            slugs_logger.warning("The spec is unrealizable. Aborting.")
            slugs_logger.warning(result.output_message)
            self.quit_handler(None, None)

    def get_execution_status(self):
        # abstract class. Determine and return result of whether exectuion should be aborted
        return not self._quit


    def handle_no_interactive_flag(self):
        # abstract class. Actions to perform when interactiveStrategy option is not supplemented
        slugs_logger.error('interactiveStrategy only! We are exitting.')
        self.quit_handler(None, None)

    def handle_intermediate_synthesis(self, syn_start_time):
        # abstract class. Actions to perform when the synthesis process is running (providing feedback for example)
        pass

    def handle_completed_synthesis(self):
        # abstract class. Actions to perform when the synthesis process is done
        pass


    ####################


class SlugsExecutor(SlugsExecutorBase):
    """
    This class handler all services in execution
    """


    ### ABSTRACT CLASSES ###
    def init_implementation(self, args):
        # abstact class. Init objects
        signal.signal(signal.SIGINT, self.quit_handler)

    def create_init_response_obj(self):
        # abstract class. Create a response object
        return copy.deepcopy(ResponseObj())

    def create_trans_response_obj(self):
        # abstract class. Create a response object
        return copy.deepcopy(ResponseObj())

    def get_execution_status(self):
        # abstract class. Determine and return result of whether exectuion should be aborted
        return not self._quit

    def init_inputs_outputs_wrapper(self, *arg):
        if len(arg) == 1: # string
            req = RequestInitStrObj()
            req.init_inputs_outputs = arg[0]
            return self.handle_init_inputs_outputs_string(req)
        elif len(arg) == 2: # key array and value array
            slugs_logger.debug(arg)
            req = RequestInitArrayObj()
            req.init_inputs_outputs_key_array = arg[0]
            req.init_inputs_outputs_value_array = arg[1]
            return self.handle_init_inputs_outputs_array(req)
        else:
            slugs_logger.error("Initial inputs and outputs are in the wrong format: {0}".format(arg))
            slugs_logger.error("It should be either a string or two lists.")

    def trans_inputs_wrapper(self, *arg):
        if len(arg) == 1: # string
            req = RequestTransStrObj()
            req.trans_inputs = arg[0]
            return self.handle_trans_inputs_string(req)
        elif len(arg) == 2: # key array and value array
            req = RequestTransArrayObj()
            req.trans_inputs_key_array = arg[0]
            req.trans_inputs_value_array = arg[1]
            return self.handle_trans_inputs_array(req)
        else:
            slugs_logger.error("Transition inputs are in the wrong format: {0}".format(arg))
            slugs_logger.error("It should be either a string or two lists.")

    def set_goal_wrapper(self, goal_id):
        return self.handle_set_goal(RequestGoal(goal_id))
    #######################




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Slugs Action Server.")
    parser.add_argument('ltl_filename', type=str, help='Specify .slugsin for now')
    parser.add_argument('--options', action='append', help='Synthesis options in SLUGS. Without --') # nargs='*'

    args, unknown = parser.parse_known_args()
    args.options = ['--'+x for x in args.options] if args.options is not None else []

    interactive_setup = False
    synthesis_action = SlugsSynthesisStandalone('SLUGS', args)

    slugs_logger.info('Started slugs server. Waiting for request...')

    #while synthesis_action.get_execution_status():
    # only start if interactiveStrategy is called
    if "--interactiveStrategy" in synthesis_action._options and \
        synthesis_action._synthesis_done and synthesis_action._realizable and not interactive_setup:
        # initialize executor
        slugs_executor = SlugsExecutor(synthesis_action)

        slugs_logger.info('Running interactive strategy now ...')

        # start topics
        slugs_executor.set_inputs()
        slugs_executor.set_outputs()

        ### test service #####
        print  "Init_state_string:" + slugs_executor.init_inputs_outputs_wrapper("")

        init_key = ['person']
        init_value = [True]
        key, value = slugs_executor.init_inputs_outputs_wrapper(init_key, init_value)
        print  "Init_state_key:" + str(key)
        print  "Init_state_value:" + str(value)

        print  "Init_state_string:" + slugs_executor.init_inputs_outputs_wrapper("000000...")

        print  "Trans_state_string:" + str(slugs_executor.trans_inputs_wrapper("00"))
        print  "Trans_state_string:" + str(slugs_executor.trans_inputs_wrapper("10"))

        trans_key = ['person', 'hazardous_item']
        trans_value = [True, False]
        key, value = slugs_executor.trans_inputs_wrapper(trans_key, trans_value)
        print  "Trans_state_key:" + str(key)
        print  "Trans_state_value:" + str(value)

        trans_value = [False, True]
        key, value = slugs_executor.trans_inputs_wrapper(trans_key, trans_value)
        print  "Trans_state_key:" + str(key)
        print  "Trans_state_value:" + str(value)

        print  "Set current goal to:" + str(slugs_executor.set_goal_wrapper(0))

        print "Input list: {0}".format(slugs_executor.get_inputs())
        print "Output list: {0}".format(slugs_executor.get_outputs())



        interactive_setup = True

# mac
# python slugs_startup_server_standalone.py --ltl_filename  /Users/wongkaiweng/Dropbox/LTLMoP-mac/src/examples/firefighting/firefighting.slugsin --option interactiveStrategy

# ubuntu
# python slugs_startup_server_standalone.py --ltl_filename  /home/catherine/LTLMoP/src/examples/firefighting/firefighting.slugsin --option interactiveStrategy







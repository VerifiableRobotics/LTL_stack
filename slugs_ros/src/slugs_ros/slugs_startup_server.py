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

class SlugsSynthesisAction(object):
    """
    This class prepares and synthesizes controller.
    """
    # create messages that are used to publish feedback/result
    _feedback = slugs_ros.msg.SlugsSynthesisFeedback()
    _result   = slugs_ros.msg.SlugsSynthesisResult()

    def __init__(self, name):
        self._action_name = name
        self._options = []
        self._slugs_process = None
        self._q_stdout = None
        self._q_stderr = None
        self._synthesis_done = False
        self._realizable = False
        self._lock = threading.Lock() # lock when using slugs process

        self._as = actionlib.SimpleActionServer(self._action_name+"/slugs_synthesis_action", slugs_ros.msg.SlugsSynthesisAction, \
                                                execute_cb=self.synthesis_cb, auto_start = False)
        rospy.loginfo("Starting SlugsSynthesisAction server with name: {name}".format(name=self._action_name+"/slugs_synthesis_action"))

        self._as.start()
        rospy.on_shutdown(self.shutdown)

    def start_pipe_thread(self, out):
        """
        This function starts queuing pipe outputs.
        @param out: pipe to read and queue
        @type  out: subprocess.PIPE
        @return: queue object that can retreive pipe outputs
        @rtype: Queue.Queue()
        """
        def enqueue_output(out, queue):
            for line in iter(out.readline, b''):
                #slugs_logger.log(6,line)
                queue.put(line)
            out.close()

        q_out = Queue()
        t_out = threading.Thread(target=enqueue_output, args=(out, q_out))
        t_out.daemon = True # thread dies with the program
        t_out.start()
        return q_out

    def retrieve_output(self, queue):
        """
        This function retrieve outputs from a queue (without blocking)
        @param queue: queue object that can retreive pipe outputs
        @type: Queue.Queue()
        @return: output
        @rtype: string
        """
        line = ""
        while not rospy.is_shutdown():
            try:  line += queue.get_nowait() # or q.get(timeout=.1)
            except Empty:
                #slugs_logger.debug('No more output')
                return line

    def shutdown(self):
        with self._lock:
            if self._slugs_process:
                rospy.loginfo("Terminating slugs process ...")
                self._slugs_process.terminate()
                self._slugs_process.wait()

    def synthesis_cb(self, inputs):
        # helper variables
        r = rospy.Rate(1)
        self._synthesis_done = False

        # prepare for slugs command to synthesize
        rospy.loginfo("Preparing for slugs command...")
        slugs_cmd = ["slugs", inputs.ltl_filename]
        slugs_cmd[1:1] = inputs.options # append options to the command

        if "--interactiveStrategy" not in inputs.options:
            rospy.loginfo("Not using interactiveStrategy")
            slugs_cmd.append(inputs.output_filename)

        rospy.loginfo(" ".join(slugs_cmd))
        self._options = inputs.options

        # Open Slugs. non-blocking call for synthesis
        rospy.loginfo("Synthesizing finite state machine for {slugsin}...".format(slugsin=inputs.ltl_filename))
        with self._lock:
            if self._slugs_process:
                rospy.loginfo("First killing the previous slugs process ...")
                self._slugs_process.terminate()
                self._slugs_process.wait()

            start_time = time.time()
            self._slugs_process = subprocess.Popen(slugs_cmd, bufsize=1048000, stderr=subprocess.PIPE, \
                                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, preexec_fn=os.setsid, \
                                    close_fds='posix' in sys.builtin_module_names)

            # start queue for output msgs
            self._q_stdout = self.start_pipe_thread(self._slugs_process.stdout)
            self._q_stderr = self.start_pipe_thread(self._slugs_process.stderr)

            # wait for synthesis to be done
            stderr, stdout = "",""
            while not rospy.is_shutdown() and not self._synthesis_done: #or self._slugs_process.poll() is None): # 0 when finished?
                self._feedback.elapsed_time = time.time() - start_time
                self._as.publish_feedback(self._feedback)
                r.sleep()

                # check if action is terminated
                if self._as.is_preempt_requested():
                    rospy.loginfo("%s: Preempted" % self._action_name)
                    self._as.set_preempted()

                # retrieve outputs
                stdout += self.retrieve_output(self._q_stdout)
                stderr += self.retrieve_output(self._q_stderr)
                #(stdout, stderr) = self._slugs_process.communicate() # stdout, stderr, can input stdin
                #slugs_logger.log(2,stderr)

                # exit if synthesis is done and ready for execution
                if "error" in stderr.lower():
                    self._synthesis_done = True
                elif "realizable" in stderr:
                    if "--interactiveStrategy" in inputs.options:
                        if "Starting Interactive Strategy Execution" in stderr:
                            self._synthesis_done = True
                    else:
                        self._synthesis_done = True
                    self._realizable = False if "unrealizable" in stderr else True

        self._result.synthesis_time = time.time()- start_time
        self._result.output_message = stderr+stdout
        rospy.loginfo(self._result.output_message)
        if not "unrealizable" in stderr and not "error" in stderr.lower():
            self._result.realizable = True
        self._as.set_succeeded(self._result)
        rospy.loginfo("Synthesis is done.")


class SlugsExecutor(object):
    """
    This class handler all services in execution
    """
    def __init__(self, synthesis_action):
        # obtain slugs instance from SlugsSynthesisAction
        self._synthesis_action = synthesis_action
        self._publisher_rate = 10
        self._inputs = []
        self._outputs = []
        self._current_state = ""
        self._current_goal = "0"

    ###########################################
    ### Initalize Slugs Execution Service  ####
    ###########################################
    def get_init_position(self, init_inputs_outputs):
        with self._synthesis_action._lock:
            if not init_inputs_outputs:
                # No input provided. Return the initial state specified in the slugsin file.
                slugs_logger.debug("No input provided.")
                self._synthesis_action._slugs_process.stdin.write("XGETINIT\n")
                self._synthesis_action._slugs_process.stdin.flush()

                stdout, stderr = "", ""
                while not rospy.is_shutdown() and not "," in stdout:
                    stdout += self._synthesis_action.retrieve_output(self._synthesis_action._q_stdout)

            else:
                slugs_logger.debug("Input provided. Initial inputs and outputs: {init_inputs_outputs}".format(init_inputs_outputs=init_inputs_outputs))
                self._synthesis_action._slugs_process.stdin.write("XCOMPLETEINIT\n" + init_inputs_outputs)
                self._synthesis_action._slugs_process.stdin.flush()

                # iterate until we actually get our state
                stdout, stderr = "", ""
                while not rospy.is_shutdown() and not (re.search('[aAgGsS01]',stdout) or 'FORCEDNONWINNING' in stdout):
                    stdout += self._synthesis_action.retrieve_output(self._synthesis_action._q_stdout)

                if 'FORCEDNONWINNING' in stdout:
                    return ""

            #slugs_logger.log(2,stdout)
            init_state = stdout.replace(">","").strip()
            slugs_logger.debug("Init state: {init_state}".format(init_state=init_state))

            # save and return current status
            #slugs_logger.log(4,init_state.partition(","))
            self._current_state = init_state.partition(",")[0]
            if init_state.partition(",")[2]:
                self._current_goal = init_state.partition(",")[2]
                #slugs_logger.debug("Setting current goal too. ")
            return init_state.partition(",")[0]

    def handle_init_inputs_outputs_string(self, req):
        return self.get_init_position(req.init_inputs_outputs)

    def handle_init_inputs_outputs_array(self, req):
        init_APs_dict = dict(zip(req.init_inputs_outputs_key_array, req.init_inputs_outputs_value_array))

        # convert arrays into a string
        init_inputs_outputs = ""
        for prop in self._inputs+self._outputs:
            if prop in req.init_inputs_outputs_key_array:
                init_inputs_outputs += "1" if (init_APs_dict[prop] is True) else "0"
            else:
                init_inputs_outputs += "."

        # find init state
        init_state = self.get_init_position(init_inputs_outputs)

        # create list output with the current state prop assignments
        # in the form of AaGgSs
        # A: given true value,    a:given false value
        # G: possible true value, g:possible false value
        response = slugs_ros.srv.SlugsInitExecutionArrayResponse()
        response.current_inputs_outputs_key_array = self._inputs + self._outputs
        true_AP_idx_list = []
        for idx,element in enumerate(init_state):
            value = True if element == 'A' or element == 'G' or element == '1' or element == 'S' else False
            response.current_inputs_outputs_value_array.append(value)
            if value:
                true_AP_idx_list.append(idx)
        slugs_logger.debug("Init state list: {init_state}".format(init_state=map(response.current_inputs_outputs_key_array.__getitem__, true_AP_idx_list)))

        with self._synthesis_action._lock:
            # set position in slugs
            self._synthesis_action._slugs_process.stdin.write("SETPOS\n" + init_state.replace("1","1\n").replace("0","0\n")\
                                                                 .replace("A","1\n").replace("a","0\n")\
                                                                 .replace("G","1\n").replace("g","0\n"))
            self._synthesis_action._slugs_process.stdin.flush()

            # remove all outputs
            stdout = ""
            while not rospy.is_shutdown() and not all(i in stdout for i in response.current_inputs_outputs_key_array):
                stdout += self._synthesis_action.retrieve_output(self._synthesis_action._q_stdout)
                #slugs_logger.log(4, all(i in stdout for i in response.current_inputs_outputs_key_array))
                #slugs_logger.log(2,stdout)

        return response


    ##################################
    ### Slugs Transition Service  ####
    ##################################
    def get_trans_position(self, trans_inputs):
        """
        Return a list of states that can be reached based on transition inputs.
        """
        with self._synthesis_action._lock:
            # Make the transition
            self._synthesis_action._slugs_process.stdin.write("XMAKETRANS\n"+trans_inputs)
            self._synthesis_action._slugs_process.stdin.flush()

            stdout = ""
            while not rospy.is_shutdown() and not ("," in stdout or "error" in stdout.lower()):
                stdout += self._synthesis_action.retrieve_output(self._synthesis_action._q_stdout)
                #slugs_logger.log(2, stdout)

            if "error" in stdout.lower():
                rospy.logerr("No next state! Incoming inputs: {inputs}\n stdout: {stdout}"\
                    .format(inputs=trans_inputs, stdout=stdout))
                return ""
            #slugs_logger.log(2, stdout)
            slugs_logger.log(2, "Transition to: {current_state}".format(current_state=stdout.replace(">","").strip().partition(",")[0]))

            self._current_state = stdout.replace(">","").strip().partition(",")[0]
            self._current_goal = stdout.replace(">","").strip().partition(",")[2]
            #slugs_logger.log(2, self._current_state + "," + self._current_goal)
            return stdout.replace(">","").strip().partition(",")[0]

    def handle_trans_inputs_string(self, req):
        return self.get_trans_position(req.trans_inputs)

    def handle_trans_inputs_array(self, req):
        trans_APs_dict = dict(zip(req.trans_inputs_key_array, req.trans_inputs_value_array))

        # convert arrays into a string
        trans_inputs = ""
        for prop in self._inputs:
            trans_inputs += "1" if (trans_APs_dict[prop] is True) else "0"

        # find curent state
        current_state = self.get_trans_position(trans_inputs)

        # create list output with the current state prop assignments
        response = slugs_ros.srv.SlugsTransExecutionArrayResponse()
        response.current_inputs_outputs_key_array = self._inputs + self._outputs
        true_AP_idx_list = []
        for idx, element in enumerate(current_state):
            value = True if element == '1' else False
            response.current_inputs_outputs_value_array.append(value)
            if value:
                true_AP_idx_list.append(idx)
        slugs_logger.log(2, "Trans state list: {init_state}".format(init_state=map(response.current_inputs_outputs_key_array.__getitem__, true_AP_idx_list)))

        return response

    ##################################
    ### Slugs Goal Service  ####
    ##################################
    def handle_set_goal(self, req):
        with self._synthesis_action._lock:
            # set goal
            slugs_logger.debug('Rewriting current goal to: {goal_id}'.format(goal_id=req.goal_id))
            self._synthesis_action._slugs_process.stdin.write("XMAKEGOAL\n" + str(req.goal_id) + "\n")
            self._synthesis_action._slugs_process.stdin.flush()
            stdout = ""
            while not rospy.is_shutdown() and not re.search(">",stdout):
                stdout += self._synthesis_action.retrieve_output(self._synthesis_action._q_stdout)
                #slugs_logger.log(2,repr(stdout))

            # get current goal
            self._synthesis_action._slugs_process.stdin.write("XGETCURRENTGOAL\n")
            self._synthesis_action._slugs_process.stdin.flush()

            stdout = ""
            while not rospy.is_shutdown() and not re.search("\d\n",stdout):
                stdout += self._synthesis_action.retrieve_output(self._synthesis_action._q_stdout)
                #slugs_logger.log(2,repr(stdout))
            current_goal = stdout.partition(">")[2]
            slugs_logger.debug("current_goal:" + str(current_goal))

            return int(current_goal)


    def slugs_set_goal_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsSetGoal, self.handle_set_goal)
        rospy.loginfo("Starting SlugsSetGoal service with name: {name}".format(name=name))

    def slugs_init_execution_string_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsInitExecutionString, self.handle_init_inputs_outputs_string)
        rospy.loginfo("Starting SlugsInitExecution(String) service with name: {name}".format(name=name))

    def slugs_init_execution_array_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsInitExecutionArray, self.handle_init_inputs_outputs_array)
        rospy.loginfo("Starting SlugsInitExecution(Array) service with name: {name}".format(name=name))

    def slugs_get_transition_string_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsTransExecutionString, self.handle_trans_inputs_string)
        rospy.loginfo("Starting SlugsTransExecution(String) service with name: {name}".format(name=name))

    def slugs_get_transition_array_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsTransExecutionArray, self.handle_trans_inputs_array)
        rospy.loginfo("Starting SlugsTransExecution(Array) service with name: {name}".format(name=name))

    #####################################
    ##### Inputs and Outputs Service ####
    #####################################
    def slugs_get_inputs_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsGetInputs, self.handle_get_inputs)
        rospy.loginfo("Starting SlugsGetInputs service with name: {name}".format(name=name))

    def slugs_get_outputs_service(self, name):
        s = rospy.Service(name, slugs_ros.srv.SlugsGetOutputs, self.handle_get_outputs)
        rospy.loginfo("Starting SlugsGetOutputs service with name: {name}".format(name=name))

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
        while not rospy.is_shutdown():
            #rospy.loginfo(string_list)
            pub.publish(string_list)
            rate.sleep()

    def get_inputs(self): return self.get_AP_list("XPRINTINPUTS\n")
    def get_outputs(self): return self.get_AP_list("XPRINTOUTPUTS\n")
    def set_inputs(self): self._inputs = self.get_inputs()
    def set_outputs(self): self._outputs = self.get_outputs()

    def get_AP_list(self, command):
        """
        This function retrieve list of atomic proposition based on the command
        @param command: command to send to slugs
        @type: string
        @return: list of APs
        @rtype: string[]
        """
        self._synthesis_action._slugs_process.stdin.write(command)
        self._synthesis_action._slugs_process.stdin.flush()

        # get all data
        stdout = ""
        while not rospy.is_shutdown() and not (re.search('\w+',stdout) and stdout.endswith("\n\n")):
            stdout += self._synthesis_action.retrieve_output(self._synthesis_action._q_stdout)
            #slugs_logger.log(4,repr(stdout))
        #slugs_logger.log(4,stdout.replace(">","").split())
        return stdout.replace(">","").split()

    def start_topic_thread(self, target, args):
        t = threading.Thread(target=target, args=args)
        t.daemon = True # thread dies with the program
        t.start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Slugs Action Server.")
    #parser.add_argument('namespace', type=str, help='Namespace of the node.')

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

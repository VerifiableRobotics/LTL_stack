#! /usr/bin/env python
import abc
import time
import subprocess
import io
import os
import sys
import logging
import threading
import re
import argparse
import signal
import termios
try:
    from Queue import Queue, Empty
except ImportError:
    from queue import Queue, Empty  # python 3.x

import slugs_logging
slugs_logger = logging.getLogger("slugs_logger")


class SlugsSynthesisBase(object):
    __metaclass__ = abc.ABCMeta

    """
    This class prepares and synthesizes controller.
    """
    def __init__(self, name, args=[]):
        self._action_name = name
        self._options = []
        self._slugs_process = None
        self._q_stdout = None
        self._q_stderr = None
        self._synthesis_done = False
        self._realizable = False
        self._lock = threading.Lock() # lock when using slugs process
        self._quit = False

        self.init_feedback_and_result_obj() # implement by plugin class
        self.init_implementation(args) # implement by plugin class


    @abc.abstractmethod
    def init_feedback_and_result_obj(self):
        # abstract class. initialize feedback and result objects
        return

    @abc.abstractmethod
    def init_implementation(self):
        # abstract class. initialize all required variables/calls for the implementation.
        return

    @abc.abstractmethod
    def get_execution_status(self):
        # abstract class. Determine and return result of whether exectuion should be aborted
        return

    @abc.abstractmethod
    def handle_no_interactive_flag(self):
        # abstract class. Actions to perform when interactiveStrategy option is not supplemented
        return

    @abc.abstractmethod
    def handle_intermediate_synthesis(self, syn_start_time):
        # abstract class. Actions to perform when the synthesis process is running (providing feedback for example)
        return

    @abc.abstractmethod
    def handle_completed_synthesis(self):
        # abstract class. Actions to perform when the synthesis process is done
        return

    def quit_handler(self, signal, frame):
        slugs_logger.info("Synthesis Program exiting gracefully")
        self._quit = True
        #self.shutdown() # shutdown is done by SlugsExecutor
        sys.exit(0)

    ############################

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
        while self.get_execution_status():
            # make sure the full output is obtained
            try:  line += queue.get(timeout=.05) #queue.get_nowait() # or
            except Empty:
                #slugs_logger.debug('No more output')
                return line

    def shutdown(self):
        with self._lock:
            if self._slugs_process:
                slugs_logger.info("Terminating slugs process ...")
                self._slugs_process.terminate()
                self._slugs_process.wait()

    def synthesis_cb(self, inputs):
        # helper variables
        self._synthesis_done = False

        # prepare for slugs command to synthesize
        slugs_logger.info("Preparing for slugs command...")
        slugs_cmd = ["slugs", inputs.ltl_filename]
        slugs_cmd[1:1] = inputs.options # append options to the command

        if "--interactiveStrategy" not in inputs.options:
            self.handle_no_interactive_flag()

        slugs_logger.info(" ".join(slugs_cmd))
        self._options = inputs.options

        # Open Slugs. non-blocking call for synthesis
        slugs_logger.info("Synthesizing finite state machine for {slugsin}...".format(slugsin=inputs.ltl_filename))
        with self._lock:
            if self._slugs_process:
                slugs_logger.info("First killing the previous slugs process ...")
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
            while self.get_execution_status() and not self._synthesis_done: #or self._slugs_process.poll() is None): # 0 when finished?
                self.handle_intermediate_synthesis(start_time)

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
        #slugs_logger.info(self._result.output_message.split(" "))
        if not "unrealizable" in stderr and not "error" in stderr.lower():
            self._result.realizable = True

        self.handle_completed_synthesis()

        slugs_logger.info("Synthesis is done.")

        return self._result


class SlugsExecutorBase(object):
    """
    This class handler all services in execution
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, synthesis_action):
        # obtain slugs instance from SlugsSynthesisAction
        self._synthesis_action = synthesis_action
        self._publisher_rate = 10
        self._inputs = []
        self._outputs = []
        self._current_state = ""
        self._current_goal = "0"
        self._quit = False

    @abc.abstractmethod
    def init_implementation(self):
        # abstract class. Init objects
        return

    @abc.abstractmethod
    def create_init_response_obj(self):
        # abstract class. Create a initial response object
        return

    @abc.abstractmethod
    def create_trans_response_obj(self):
        # abstract class. Create a transition response object
        return

    @abc.abstractmethod
    def get_execution_status(self):
        # abstract class. Determine and return result of whether exectuion should be aborted
        return

    def quit_handler(self, signal, frame):
        self._quit = True
        slugs_logger.info("Executor exiting gracefully")
        #self._synthesis_action.shutdown()
        #sys.exit(0)

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
                while self.get_execution_status() and not "," in stdout:
                    stdout += self._synthesis_action.retrieve_output(self._synthesis_action._q_stdout)

            else:
                slugs_logger.debug("Input provided. Initial inputs and outputs: {init_inputs_outputs}".format(init_inputs_outputs=init_inputs_outputs))
                self._synthesis_action._slugs_process.stdin.write("XCOMPLETEINIT\n" + init_inputs_outputs)
                self._synthesis_action._slugs_process.stdin.flush()

                # iterate until we actually get our state
                stdout, stderr = "", ""
                while self.get_execution_status() and not (re.search('[aAgGsS01]',stdout) or 'FORCEDNONWINNING' in stdout):
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
        response = self.create_init_response_obj()
        response.current_inputs_outputs_key_array = self._inputs + self._outputs
        true_AP_idx_list = []
        for idx,element in enumerate(init_state):
            value = True if element == 'A' or element == 'G' or element == '1' or element == 'S' else False
            response.current_inputs_outputs_value_array.append(value)
            if value:
                true_AP_idx_list.append(idx)
        slugs_logger.debug("Init state list: {init_state}".format(init_state=map(response.current_inputs_outputs_key_array.__getitem__, true_AP_idx_list)))

        stdout = ""
        with self._synthesis_action._lock:
            # remove all outputs
            while self.get_execution_status() and not all(i in stdout for i in response.current_inputs_outputs_key_array):

                if not stdout or "Error" in stdout:
                    stdout = ""
                    # set position in slugs
                    self._synthesis_action._slugs_process.stdin.write("SETPOS\n" + init_state.replace("1","1\n").replace("0","0\n")\
                                                                         .replace("A","1\n").replace("a","0\n")\
                                                                         .replace("G","1\n").replace("g","0\n"))
                    self._synthesis_action._slugs_process.stdin.flush()


                slugs_logger.log(4, "Still in SETPOS here")
                stdout += self._synthesis_action.retrieve_output(self._synthesis_action._q_stdout)
                #slugs_logger.log(4, all(i in stdout for i in response.current_inputs_outputs_key_array))
                slugs_logger.log(2,stdout)


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
            while self.get_execution_status() and not ("," in stdout or "error" in stdout.lower()):
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
        response = self.create_trans_response_obj()
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
            while self.get_execution_status() and not re.search(">",stdout):
                stdout += self._synthesis_action.retrieve_output(self._synthesis_action._q_stdout)
                #slugs_logger.log(2,repr(stdout))

            # get current goal
            self._synthesis_action._slugs_process.stdin.write("XGETCURRENTGOAL\n")
            self._synthesis_action._slugs_process.stdin.flush()

            stdout = ""
            while self.get_execution_status() and not re.search("\d\n",stdout):
                stdout += self._synthesis_action.retrieve_output(self._synthesis_action._q_stdout)
                #slugs_logger.log(2,repr(stdout))
            current_goal = stdout.partition(">")[2]
            slugs_logger.debug("current_goal:" + str(current_goal))

            return int(current_goal)


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
        while self.get_execution_status() and not (re.search('\w+',stdout) and stdout.endswith("\n\n")):
            stdout += self._synthesis_action.retrieve_output(self._synthesis_action._q_stdout)
            #slugs_logger.log(4,repr(stdout))
        #slugs_logger.log(4,stdout.replace(">","").split())

        return stdout.replace(">","").split()






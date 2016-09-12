#! /usr/bin/env python
import rospy
import argparse
import logging
try:
    # Python2
    import Tkinter as tk
except ImportError:
    # Python3
    import tkinter as tk

import controller_executor.msg
import controller_executor_logging
prop_monitor_logger = logging.getLogger("prop_monitor_logger")

class PropositionMonitor(object):
    _input_labels = {}
    _output_labels = {}
    def __init__(self, namespace):
        self.master = tk.Tk()

        # separate into two columns, inputs and outputs
        self.input_frame = tk.LabelFrame(self.master, text="Inputs", padx=5, pady=5)
        self.input_frame.pack(padx=10, pady=10, side=tk.LEFT)
        self.output_frame = tk.LabelFrame(self.master, text="Outputs", padx=5, pady=5)
        self.output_frame.pack(padx=10, pady=10, side=tk.RIGHT)

        # set up subscriber
        inputs_topic = namespace+'/input_manager/incoming_inputs' if namespace else 'input_manager/incoming_inputs'
        outputs_topic = namespace+'/executor/incoming_outputs' if namespace else 'executor/incoming_outputs'

        rospy.Subscriber(inputs_topic, controller_executor.msg.stringKeyBoolValueDict, \
                    callback=self.update_incoming_props, callback_args=(self._input_labels, self.input_frame))
        rospy.Subscriber(outputs_topic, controller_executor.msg.stringKeyBoolValueDict, \
                    callback=self.update_incoming_props, callback_args=(self._output_labels, self.output_frame))

    def start_monitor(self):
        self.master.mainloop()

    def update_incoming_props(self, data, args):
        labels_dict = args[0]
        label_frame = args[1]

        for idx, prop in enumerate(data.keys):
            prop_monitor_logger.log(2, prop)
            # create label if it's not there
            if not prop in labels_dict:
                labels_dict[prop] = tk.Label(label_frame, \
                        text=prop+'-False', bg="red", width=30, height=0, padx=5, pady=5)
                labels_dict[prop].pack(padx=10, pady=5)

            # update data
            if data.values[idx]:
                labels_dict[prop].config(text=prop+'-True', activebackground="green", bg = "green")
            else:
                labels_dict[prop].config(text=prop+'-False', activebackground="red", bg = "red")

    def quit(self):
        self.master.destroy()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Get AprilTag")
    parser.add_argument('--namespace', type=str, help='Namespace of current controller', \
                            nargs='?', const="", default="")

    args, unknown = parser.parse_known_args()
    print args

    rospy.init_node("monitor")

    a = PropositionMonitor(args.namespace)

    # register shutdown hook
    rospy.on_shutdown(a.quit)

    a.start_monitor()

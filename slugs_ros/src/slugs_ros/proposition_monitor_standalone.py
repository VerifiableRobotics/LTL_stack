#! /usr/bin/env python
import argparse
import logging
import signal
import logging
import socket
import threading
import ast
try:
    # Python2
    import Tkinter as tk
except ImportError:
    # Python3
    import tkinter as tk

import slugs_logging
slugs_logger = logging.getLogger("slugs_logger")

class UDP_Listener(object):
    def __init__(self, UDP_IP="127.0.0.1", UDP_PORT=5000):
        self._listen_sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        self._listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self._listen_sock.bind((UDP_IP, UDP_PORT))

    def retrieve_msg(self):
        data, addr = self._listen_sock.recvfrom(1024) # buffer size is 1024 bytes
        return data

class PropositionMonitor(object):
    _input_labels = {}
    _output_labels = {}
    def __init__(self, namespace):
        self.master = tk.Tk()

        # separate into two columns, inputs and outputs
        self.input_frame = tk.LabelFrame(self.master, text="Env.", padx=5, pady=5)
        self.input_frame.pack(padx=10, pady=10, side=tk.LEFT)
        self.output_frame = tk.LabelFrame(self.master, text="Sys.", padx=5, pady=5)
        self.output_frame.pack(padx=10, pady=10, side=tk.RIGHT)

        self.start_listeners()

    def start_listeners(self):
        t_out = threading.Thread(target = self.check_props)
        t_out.start()

    def check_props(self):
        listener = UDP_Listener()
        while True:
            received = listener.retrieve_msg()#number of bytes recived
            received_dict = ast.literal_eval(received)

            input_dict = {key.replace("i:",""): received_dict[key] for key in received_dict.keys() if "i:" in key}
            output_dict = {key.replace("o:",""): received_dict[key] for key in received_dict.keys() if "o:" in key}

            self.update_incoming_props(output_dict, \
                [self._output_labels, self.output_frame])
            self.update_incoming_props(input_dict, \
                [self._input_labels, self.input_frame])


    def start_monitor(self):
        self.master.mainloop()

    def update_incoming_props(self, data, args): # args = input_frame or outputframe
        labels_dict = args[0]
        label_frame = args[1]

        print data

        for idx, prop in enumerate(data.keys()):
            slugs_logger.log(2, prop)
            # create label if it's not there
            if not prop in labels_dict:
                labels_dict[prop] = tk.Label(label_frame, \
                        text=prop+'-False', bg="blue", width=30, height=0, padx=5, pady=5)
                labels_dict[prop].pack(padx=10, pady=5)

            # update data
            if data.values()[idx]:
                labels_dict[prop].config(text=prop+'-True', activebackground="yellow", bg = "yellow")
            else:
                labels_dict[prop].config(text=prop+'-False', activebackground="blue", bg = "blue")

    def quit(self):
        self.master.destroy()

    def quit_handler(self, signal, frame):
        self.quit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Get AprilTag")
    parser.add_argument('--namespace', type=str, help='Namespace of current controller', \
                            nargs='?', const="", default="")

    args, unknown = parser.parse_known_args()
    print args

    a = PropositionMonitor(args.namespace)

    # register shutdown hook
    signal.signal(signal.SIGINT, a.quit_handler)

    a.start_monitor()

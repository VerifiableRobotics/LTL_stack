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

import std_msgs.msg

import node_logging
node_logger = logging.getLogger("node_logger")

class TkSimpleTextbox(object):
    def __init__(self, args):
        self.root = None
        self.T = None
        self.destroy = True

        # publisher for ac status
        self._pub_status = rospy.Publisher(args.node_subscribe_topic+'_status', std_msgs.msg.Bool, queue_size=10, latch=True)
        self._pub_status.publish(False)

    def callback(self, data, args):
        if data.data:
            if self.destroy: #not self.root:
                self.destroy = False
                node_logger.debug('Proposition {0} is True!'.format(args.node_name))
                self.root = tk.Tk()
                self.root.wm_title(args.node_name)
                self.T = tk.Text(self.root, height=2, width=30)
                self.T.pack()
                self.T.insert(tk.END, args.text)
                self.T.configure(state="disabled")
                self._pub_status.publish(True)
                self.root.mainloop()
        else:
            if self.root and not self.destroy:
                node_logger.debug('Proposition {0} is False!'.format(args.node_name))
                self.destroy = True
                self._pub_status.publish(False)
                self.T.destroy()
                node_logger.debug(self.root)
                self.root.destroy()
                node_logger.debug(self.root) # kind of blocking here?
                self.root = None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="tkinter textbox")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_subscribe_topic', type=str, help='Specify controller topic to respond to')
    parser.add_argument('--text', type=str, help='Specify the message to display.',\
                           nargs='?', const='Warning Message', default='Warning Message')

    args, unknown = parser.parse_known_args()
    node_logger.debug(args)

    rospy.init_node(args.node_name)

    a = TkSimpleTextbox(args)
    # subsribe to request
    rospy.Subscriber(args.node_subscribe_topic, std_msgs.msg.Bool, callback=a.callback, callback_args=args)
    node_logger.debug('Everything has started.')

    rospy.spin()

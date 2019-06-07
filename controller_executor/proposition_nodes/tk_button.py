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

class TkSimpleButton(object):
    def __init__(self, node_name, node_publish_topic, init_value=False):
        self.root = tk.Tk()
        self.root.wm_title(node_name)
        self.node_name = node_name

        # create button
        node_logger.log(6, "{0} Init_value: {1}".format(self.node_name, init_value))
        background_color = "yellow" if init_value else "blue"
        button_name = node_name+"-True" if init_value else node_name+"-false"
        self.button = tk.Button(text=button_name, width=20, command=self.toggle, \
                                activebackground=background_color, bg=background_color)
        self.button.pack()

        # set up publisher
        self.pub = rospy.Publisher(node_publish_topic, std_msgs.msg.Bool, queue_size=10)
        self.pub_rate = rospy.Rate(10) # set publish rate

    def start_button(self):
        self.root.after(100, self.publish_info)
        node_logger.info("Starting sensor {0} Tk button mainloop".format(self.node_name))
        self.root.mainloop()

    def toggle(self):
        '''
        use
        self.button.config('text')[-1]
        to get the present state of the toggle button
        '''
        if self.button.config('activebackground')[-1] == "yellow":
            self.button.config(text=self.node_name+'-False', activebackground="blue", bg = "blue")
            node_logger.debug("Sensor {0} turned False.".format(self.node_name))
        else:
            self.button.config(text=self.node_name+'-True', activebackground="yellow", bg = "yellow")
            node_logger.debug("Sensor {0} turned True.".format(self.node_name))

    def button_state(self):
        return True if self.button.config('activebackground')[-1] == "yellow" else False

    def publish_info(self):
        # publish to topics
        self.pub.publish(a.button_state())
        self.pub_rate.sleep()

        # trigger again
        self.root.after(10, self.publish_info)

    def quit(self):
        self.root.destroy()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="tkinter Sensor button")
    parser.register('type', 'bool', (lambda x: x.lower() in ("yes", "true", "t", "1"))) # fix bool problem
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_publish_topic', type=str, help='Specify name of publishing topic to the controller')
    parser.add_argument('--init_value', type='bool', help='Specify initial value of the input proposition.',\
                           nargs='?', const=False, default=False)

    args, unknown = parser.parse_known_args()
    node_logger.debug(args)

    rospy.init_node(args.node_name)

    # start button
    a = TkSimpleButton(args.node_name, args.node_publish_topic, args.init_value)

    # register shutdown hook
    rospy.on_shutdown(a.quit)

    # start button
    a.start_button()



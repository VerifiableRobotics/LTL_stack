#! /usr/bin/env python
import rospy
import argparse

import turtlesim.msg
import std_msgs.msg


class ColorSensor(object):
    def __init__(self):
        self.color_info = None

    def callback(self, data):
        # save latest info
        self.color_info = data
        print data

    # publish information
    def convert_to_bool(self, color, color_thres):
        if self.color_info:
            if color == 'r':
                return self.color_info.r < color_thres
            elif color == 'g':
                return self.color_info.g < color_thres
            else:
                return self.color_info.b < color_thres
        else:
            return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sensor Example.")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_publish_topic', type=str, help='Specify name of publishing topic to the controller')
    parser.add_argument('--sensor_topic', type=str, help='Specify sensor topics', nargs='?', const='/turtle1/color_sensor', \
                                                                                           default='/turtle1/color_sensor')
    parser.add_argument('--color', type=str, help="'r','g' or 'b'", nargs='?', const='r', default='r')
    parser.add_argument('--color_thres', type=int, help="between 0 to 256", nargs='?', const=100, default=100)

    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()
    print args

    rospy.init_node(args.node_name)
    a = ColorSensor()

    # subsribe to necessary information
    rospy.Subscriber(args.sensor_topic, turtlesim.msg.Color, callback=a.callback)

    # publish to topics
    pub = rospy.Publisher(args.node_publish_topic, std_msgs.msg.Bool, queue_size=10)
    rate = rospy.Rate(10) # set publish rate
    while not rospy.is_shutdown():
        pub.publish(a.convert_to_bool(args.color, args.color_thres))
        rate.sleep()

#python color_sensor.py 'see_hazardous_item' '/see_item' --sensor_topic '/turtle1/color_sensor'
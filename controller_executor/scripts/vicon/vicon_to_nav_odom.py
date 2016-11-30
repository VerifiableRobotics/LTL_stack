#! /usr/bin/env python
import argparse

import rospy
import nav_msgs.msg
import tf
import geometry_msgs.msg

class Vicon2Odom(object):
    def update_odom(self, odom_trans):
        #odom_trans in the form of geometry_msgs.msg.TransformStamped()

        delay = rospy.Duration(0.1)
        current_time = rospy.Time.now()
        header_frame_id = "world"#self._vicon_obj_topic #"odom"
        child_frame_id = "base_link"

        #next, we'll publish the odometry message over ROS
        self._odom = nav_msgs.msg.Odometry()
        self._odom.header.stamp = current_time + delay # need to future time odom or global plan in move_base complains
        self._odom.header.frame_id = header_frame_id

        #set the position of msg
        self._odom.pose.pose.position = odom_trans.transform.translation
        self._odom.pose.pose.orientation = odom_trans.transform.rotation

        #set the child frame of msg
        self._odom.child_frame_id = child_frame_id

    def save_vel(self, data):
        if self._odom:
            # set the velocity of msg
            self._odom.twist.twist = data

    def __init__(self, vicon_obj_topic):
        self._odom = None
        self._vicon_obj_topic = vicon_obj_topic
        # replace odom with vicon
        self._odom_pub = rospy.Publisher("odom", nav_msgs.msg.Odometry, queue_size=10, latch=True)

        # subscribe to vicon
        rospy.Subscriber(vicon_obj_topic, geometry_msgs.msg.TransformStamped, callback=self.update_odom)

        # subscrbied to cmd_vel
        rospy.Subscriber('cmd_vel', geometry_msgs.msg.Twist, callback = self.save_vel)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Broadcast vicon pose to odom")
    parser.add_argument('vicon_obj_topic', type=str, help='name for vicon object topic')
    args, unknown = parser.parse_known_args()

    rospy.init_node("odometry_publisher")
    a = Vicon2Odom(args.vicon_obj_topic)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if a._odom:
            a._odom_pub.publish(a._odom)
        rate.sleep()



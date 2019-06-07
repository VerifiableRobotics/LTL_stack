#!/usr/bin/env python
'''
subscribe robot vicon position
judge if the robot is in a certain region
send bool to /adhoc/inputs/PNext
'''

import rospy
import tf
import threading
import time
from numpy import *
import sys
from std_msgs.msg import Bool

class ViconTracker(object):

	robot_Xx = 0
	robot_Yy = 0
	robot_Zz = 0

	def __init__(self, SpheroNumber):
		self.target_robot = 'vicon/jackal3/jackal3'
		self.robot_x = 0
		self.robot_y = 0
		self.robot_z = 0
		self.t = tf.TransformListener()
		self.thread_robot = threading.Thread(target=self.updatePose_robot)
		self.thread_robot.daemon = True
		self.thread_robot.start()


	def updatePose_robot(self):
		self.t.waitForTransform('world',self.target_robot, rospy.Time(0), rospy.Duration(4.0))
		a = self.t.lookupTransform('world',self.target_robot, rospy.Time(0))
		self.robot_x = a[0][0]
		self.robot_y = a[0][1]
		euler = tf.transformations.euler_from_quaternion(a[1])
		self.robot_z = euler[2]
		robot_Xx = self.robot_x
		robot_Yy = self.robot_y
		robot_Zz = self.robot_z

	def getPose(self, cached=False):
		self.updatePose_robot()
		return array([self.robot_x, self.robot_y, self.robot_z])

	def calc(self):
		result=0
		data = self.getPose()
		if (data[1]>0):
			result=1
		return result

if __name__ == "__main__":
	rospy.init_node('RNext')
	pub = rospy.Publisher('/adhoc2/inputs/RNext', Bool, queue_size=10)
	rate = rospy.Rate(10)
	a = ViconTracker(7)
	time.sleep(1)
	while not rospy.is_shutdown():
		pub.publish(a.calc()==1)

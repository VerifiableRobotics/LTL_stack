#!/usr/bin/env python

'''
subscribe Person vicon position
judge if the person is in a certain region
send bool to /adhoc/inputs/PNext
'''

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

	def __init__(self):
		self.target_person = 'vicon/Helmet/Helmet'
		self.person_x = 0
		self.person_y = 0
		self.person_z = 0
		self.t = tf.TransformListener()
		self.thread_person = threading.Thread(target=self.updatePose_person)
		self.thread_person.daemon = True
		self.thread_person.start()


	def updatePose_person(self):
		self.t.waitForTransform('world',self.target_person, rospy.Time(0), rospy.Duration(4.0))
		a = self.t.lookupTransform('world',self.target_person, rospy.Time(0))
		self.person_x = a[0][0]
		self.person_y = a[0][1]
		euler = tf.transformations.euler_from_quaternion(a[1])
		self.person_z = euler[2]
		person_Xx = self.person_x
		person_Yy = self.person_y
		person_Zz = self.person_z

	def getPose(self, cached=False):
		self.updatePose_person()
		return array([self.person_x, self.person_y, self.person_z])

	def calc(self):
		result=0
		data = self.getPose()
		if (data[1]>0):
			result=1
		return result

if __name__ == "__main__":
	rospy.init_node('PNext')
	pub = rospy.Publisher('/adhoc2/inputs/PNext', Bool, queue_size=10)
	rate = rospy.Rate(10)
	a = ViconTracker()
	time.sleep(1)
	while not rospy.is_shutdown():
		pub.publish(a.calc()==1)

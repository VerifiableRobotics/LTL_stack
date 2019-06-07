#!/usr/bin/env python
'''
subscribe robot and person vicon position
judge if person and robot 's relative position is within a certain range
send bool to /adhoc/inputs/seeP
'''
import rospy
import tf
import threading
import time
from numpy import *
import sys
from std_msgs.msg import Bool
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib


class ViconTracker(object):

	person_Xx = 0
	person_Yy = 0
	person_Zz = 0
	robot_Xx = 0
	robot_Yy = 0
	robot_Zz = 0

	def __init__(self, SpheroNumber):
		self.target_person = 'vicon/Helmet/Helmet'
		self.person_x = 0
		self.person_y = 0
		self.person_z = 0
		self.t = tf.TransformListener()
		self.thread_person = threading.Thread(target=self.updatePose_person)
		self.thread_person.daemon = True
		self.thread_person.start()
		self.target_robot = 'vicon/jackal3/jackal3'
		self.robot_x = 0
		self.robot_y = 0
		self.robot_z = 0
		self.thread_robot = threading.Thread(target=self.updatePose_robot)
		self.thread_robot.daemon = True
		self.thread_robot.start()
		self.poseP = np.zeros((4,1))
		self.poseR = np.zeros((3,1))

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
		self.updatePose_person()
		self.updatePose_robot()
		return array([self.person_x, self.person_y, self.person_z,self.robot_x, self.robot_y, self.robot_z])
	'''
	def calc(self):
		result = 0
		data = self.getPose()
		dist = math.sqrt((data[0] - data[3])**2 + (data[1] - data[4])**2)
		if (data[2] <= 0):
			desired_angle =  data[2] + math.pi
		else:
			desired_angle =  data[2] - math.pi
		if (dist < 0.8 and abs(desired_angle - data[5]) < 0.3):
			result = 1
		return result
	'''
	def calc(self):
		result = 0
		data = self.getPose()
		self.poseR = data[3:6]
		self.poseR[2] = self.poseR[2] - 2.3
		robotAngle = (self.poseR[2] + np.pi) % (2 * np.pi ) - np.pi
		self.poseP[0] = data[0]
		self.poseP[1] = data[1]
		self.poseP[2] = self.poseP[0] + 0.75*np.cos(data[2]) # ??? Goal destination at beginning
		self.poseP[3] = self.poseP[1] + 0.75*np.sin(data[2])
		xerror = self.poseP[2] - self.poseR[0]
		yerror = self.poseP[3] - self.poseR[1]
		dist2goal = np.sqrt([xerror ** 2 + yerror ** 2])
		if dist2goal < .2:
			#align with person
			# desired orientation of aligning
			theta_d = np.arctan2((self.poseP[1] - self.poseR[1]),(self.poseP[0]-self.poseR[0]))
			#robotAngle = (poseR[2] + np.pi) % (2 * np.pi ) - np.pi
			theta_e = theta_d - robotAngle
			if theta_e < .1:
				result = 1
		return result										
		
if __name__ == "__main__":
	#rospy.init_node('Hexbug_listener')
	rospy.init_node('seeP')
	pub = rospy.Publisher('/adhoc2/inputs/seeP',Bool,queue_size=10)
	rate = rospy.Rate(10)
	a = ViconTracker(7)
	#print(a.getPose())
	time.sleep(1)
	while not rospy.is_shutdown():
		#time.sleep(0.5)
		# b = a.calc()
		#print(a.calc() == 1)
		pub.publish(a.calc() == 1)
		#print a.calc()

#! /usr/bin/env python
'''
subscribe to robot vicon postion and person postion
calculation of dynamics. jijie
pub to cmd_vel

jackal2 specs: https://www.clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/
'''

import rospy
import tf
import threading
import time
from numpy import *
import sys
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import numpy as np
import cvxopt


class robotObj(object):
	def __init__(self, pose, alphaR, l):
		self.pose = pose
		self.alphaR = alphaR
		self.l = l

	def updatepos(self, x, y, theta, v, omega, uR, T):
		self.x = x
		self.y = y
		self.theta = theta
		self.v = v
		self.omega = omega
		self.T = T

		poseR = np.zeros((3, 1))
		poseR[0] = x + T*np.cos(theta)*v
		poseR[1] = y + T*np.sin(theta)*v
		poseR[2] = theta + T*omega

# for the sphero
#        poseR[0] = x + T*uR[0]
#        poseR[1] = y + T*uR[1]
		return poseR

	def distwall(self,p1,p2,pt):
		dx = p2[0] - p1[0]
		dy = p2[1] - p1[1]
		t = ((pt[0]-p1[0])*dx + (pt[1]-p1[1])*dy)/(dx*dx + dy*dy)

		if dx == 0 and dy ==0:
			closestP = p1
			dx = pt[0] - p1[0]
			dy = pt[1] - p1[1]
			dist = np.sqrt(dx*dx + dy*dy)
		elif t< 0:
			dx = pt[0] - p1[0]
			dy = pt[1] - p1[1]
			closestP = p1
		elif t>1:
			dx = pt[0] - p2[0]
			dy = pt[1] - p2[1]
			closestP = p2
		else:
			closestP = np.array([p1[0] + t*dx, p1[1]+t*dy])
			dx = pt[0] - closestP[0]
			dy = pt[1] - closestP[1]

		dx = np.asscalar(dx)
		dy = np.asscalar(dy)
		dist2wall = np.array([dx,dy])
		dist2wall = dist2wall.reshape([1,2])

		return dist2wall

	def getCommandR(self,x, y, xg, yg, xp, yp, alphaR, alphaP, gamma,h,ur):
		walls = np.array([[-2, -2, 2.4, -2.5],[2.4,-2.5,2.23,2.54],[2.23, 2.54, -2.3, 2.6],[-2.3, 2.6, -2,-2]])
		xerror = xg - x
		yerror = yg - y
		dist2goal = np.sqrt([xerror ** 2 + yerror ** 2])

		if dist2goal > .1:
			unomR = alphaR * np.array([xerror, yerror]) / dist2goal
			unomR = unomR.reshape([1, 2])
			unomR = np.nan_to_num(unomR)
		else:
			unomR = np.array([0., 0.])
			unomR = unomR.reshape([1, 2])

		deltapij = np.array([x - xp, y - yp])
		deltapij = deltapij.reshape([1, 2])
		dist2p = np.sqrt([(x-xp)**2 + (y-yp)**2])
		deltawall = np.array([])
		dist2wall = []
		bwall = np.array([])

		for i in range(0,walls.shape[0]):
			if deltawall.size == 0:
				delt = self.distwall(walls[i,0:2],walls[i,2:4],np.array([x,y]))
				deltawall = delt
				dist2wall = np.sqrt([delt[0][0]**2 +delt[0][1]**2])
				bwall = dist2wall
			else:
				delt = self.distwall(walls[i,0:2],walls[i,2:4],np.array([x,y]))
				deltawall = np.concatenate((deltawall,delt))
				dist2wall = np.sqrt([delt[0][0]**2 + delt[0][1]**2])
				bwall = np.concatenate((bwall,gamma*dist2wall**4))

		bwall = bwall.reshape([4,1])
		Awall = -deltawall

		bij = (alphaR/(alphaR+alphaP))*gamma*np.power(h, 3)*dist2p
		bij = np.asscalar(bij)
		aij = -deltapij

		row1 = deltawall[:,0]
		row1 = row1.reshape([4,1])
		row1 = row1.astype(np.double)
		row2 = deltawall[:,1]
		row2 = row2.reshape([4,1])
		row2 = row2.astype(np.double)

		from cvxopt import matrix
		P = matrix([[2.0, 0.0], [0.0, 2.0]])
		q = matrix([-2*unomR[0,0], -2*unomR[0,1]])
		G = matrix([[aij[0,0],matrix(row1), ur[0]], [aij[0,1],matrix(row2), ur[1]]])
		h = matrix([bij, matrix(bwall), alphaR**2])

		from cvxopt import solvers
		solvers.options['show_progress'] = False
		sol = solvers.qp(P, q, G, h)
		uR = sol['x']
		uR = np.array([uR[0],uR[1]])
		return uR


	def barrier(self, poseR, poseP, r):
		self.poseR = poseR
		self.poseP = poseP
		self.r = r

		h = np.sqrt([(poseR[0]-poseP[0])**2 + (poseR[1]-poseP[1])**2]) - r
		return h

	def feedbackLin(self, vx, vy, theta, epsilon):
		R = np.array([[np.asscalar(np.cos(theta)), np.asscalar(np.sin(theta))], \
                      [np.asscalar(-np.sin(theta)), np.asscalar(np.cos(theta))]])
		dirVelocities = np.array([[vx], [vy]])
		rot = np.array([[1.0, 0.0], [0.0, 1/epsilon]])

		fVelAngVel = np.dot(np.dot(rot, R),dirVelocities)

		return fVelAngVel

class personObj(object):
	def __init__(self,poseP,alphaP,r,gaze):
		self.poseP = poseP
		self.alphaP = alphaP
		self.r = r
		self.gaze = gaze

	def updatepos(self, x, y, u, T, gaze):
		self.x = x
		self.y = y
		self.u = u
		self.T = T

		poseP = np.zeros((4, 1))
		poseP[0] = x + T*u[0]
		poseP[1] = y + T*u[1]
		poseP[2] = poseP[0] + 0.75*np.cos(gaze) # ??? Goal destination at beginning
		poseP[3] = poseP[1] + 0.75*np.sin(gaze)
		return poseP

	def getCommandP(self, x, y, xg, yg, alphaP):
		xerror = xg - x
		yerror = yg - y
		dist2goal = np.sqrt([xerror**2 + yerror**2])
		if dist2goal > 0:
			uP = alphaP*np.array([xerror, yerror])/dist2goal
			uP = uP.reshape([1,2])
			uP = np.nan_to_num(uP)
		else:
			uP = np.array([0, 0])
			uP = uP.reshape([1, 2])

		return uP


class ViconTracker(object):

	person_Xx = 0
	person_Yy = 0
	person_Zz = 0
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
		self.target_robot = 'vicon/jackal3/jackal3'
		self.robot_x = 0
		self.robot_y = 0
		self.robot_z = 0
		self.thread_robot = threading.Thread(target=self.updatePose_robot)
		self.thread_robot.daemon = True
		self.thread_robot.start()

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

		poseP = np.zeros((4, 1))
		poseP[0] = person_Xx
		poseP[1] = person_Yy
		poseP[2] = poseP[0] + 0.75*np.cos(person_Zz) # ??? Goal destination at beginning
		poseP[3] = poseP[1] + 0.75*np.sin(person_Zz)
		return poseP


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

		poseR = np.zeros((3, 1))
		poseR[0] = robot_Xx
		poseR[1] = robot_Yy
		poseR[2] = robot_Zz

		return poseR

	def getPose(self, cached=False):
		self.updatePose_person()
		self.updatePose_robot()
		return array([self.person_x, self.person_y, self.person_z,self.robot_x, self.robot_y, self.robot_z])
        # return array([person_Xx, person_Yy, person_Zz, robot_Xx, robot_Yy, robot_Zz])

class ltlStack(object):
	def __init__(self):
		self.request_data=False
	def callback(self,data):
		self.request_data=data.data


if __name__ == '__main__':
	rospy.init_node('moveToNext')
	temp=ltlStack()
	sub = rospy.Subscriber('/adhoc2/outputs/moveToNext', Bool,temp.callback)
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	rate = rospy.Rate(10)
	viconInformation = ViconTracker()
	poseInformation = viconInformation.getPose()
	# The initial position of the ROBOT
	# initPose = np.array([[-2.0], [-.1],[np.pi]])

	poseInformation[5]=poseInformation[5]-2.3
	if (poseInformation[5]<-np.pi):
		poseInformation[5]= abs(poseInformation[5] + np.pi)
	initPose = np.array([[poseInformation[3]],[poseInformation[4]],[poseInformation[5]]])
	#print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

	# The initial position of the PERSON
	# initPoseP = np.array([[1.], [0.]])
	initPoseP = np.array([[poseInformation[0]], [poseInformation[1]]])
	# The goal position of the PERSON (To enable human movement)
	# posePG = np.array([[1.], [0.]])
	posePG = initPoseP
	# Max speed of person
	alphaP = 2
	# Radius of bubble surrounding person
	r = 0.3
	# gaze of person
	gaze = poseInformation[2] # np.pi

	# Max speed of robot
	alphaR = 0.3
	# alphaR = 0.1
	# Distance between wheels
	# l = 0.2
	# according to jackal's spec sheet
	l = 0.323 + (0.43 - 0.323)/2
	# tuning parameter for barrier function
	gamma = 2
	# Rotation Gain
	k_omega = .3
	# used for rate of rotation of robot
	# epsilon = .2
	epsilon = 0.2

	#intialize person and object
	robot = robotObj(initPose,alphaR,l)
	person = personObj(initPoseP,alphaP,r,gaze)

	#Used to preallocate matrices
	k_max = 700
	hertz = 20
	T = 1.0/hertz
	# Initialize Matrices
	poseRx = np.zeros((k_max, 1))
	poseRy = np.zeros((k_max, 1))
	posePx = np.zeros((k_max, 1))
	posePy = np.zeros((k_max, 1))
	thetaR = np.zeros((k_max, 1))
	goalx = np.zeros((k_max, 1))
	goaly = np.zeros((k_max, 1))
	h = np.zeros((k_max, 1))
	uP = np.zeros((k_max,2))
	uR = np.zeros((k_max,2))
	v = np.zeros((k_max, 1))
	theta_d = np.zeros((k_max,1))
	theta_e = np.zeros((k_max,1))
	omega = np.zeros((k_max, 1))

	poseRx[0] = np.array([initPose[0]]) #X Initial position of robot
	poseRy[0] = np.array([initPose[1]]) #Y Initial position of robot
	thetaR[0] = np.array([initPose[2]]) #Initial orientation of robot
	posePx[0] = np.array([initPoseP[0]])
	posePy[0] = np.array([initPoseP[1]])

	vel_msg = Twist()
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
	a = ViconTracker()
	#result=	True
	while not rospy.is_shutdown():
		while (temp.request_data):
			for i in range(0,k_max-1):
                # Update states of robot
				poseR = a.updatePose_robot()
				poseR[2] = poseR[2] - 2.3
				robotAngle = (poseR[2] + np.pi) % (2 * np.pi ) - np.pi
				#if robotAngle > 0:
				#	robotAngle = robotAngle - 2.3
				#else:
				#	robotAngle = robotAngle + 2.3

				poseR[2] = robotAngle
				poseRx[i+1] = poseR[0]
				poseRy[i+1] = poseR[1]
				thetaR[i+1] = poseR[2]

				#print(robotAngle)


		        # Update state of person
				poseP = a.updatePose_person()
				posePx[i+1] = poseP[0]
				posePy[i+1] = poseP[1]

				poseP[2] = 0
				poseP[3] = 1.2

				#PoseP[0] and poseP[1] represents the location of the person viewed as an 					obstacle 
				#State of gaze
				goalx[i+1] = poseP[2]
				goaly[i+1] = poseP[3]
				#PoseP[2] and poseP[3] represents the GOAL location. change this to change where 					the robot wants to go

				# Calculate Barrier Function
				h[i] = robot.barrier(poseR, poseP, r)

				# Get commands for the person
				uP[i+1, :] = person.getCommandP(poseP[0], poseP[1], posePG[0], posePG[1], alphaP)

				# Get commands for the robot. will be in the form vx and vy
				uR[i+1, :] = robot.getCommandR(poseR[0], poseR[1], poseP[2], poseP[3], poseP[0], poseP[1], alphaR, alphaP, gamma, h[i], uR[i,:])
				# change commands into velocity and omega
				sendVx = float(uR[i+1, 0])
				sendVy = float(uR[i+1, 1])
				# poseR[2] = poseR[2] - 0.81997077 - np.pi/2
				#print(poseR[2])
				commands = robot.feedbackLin(uR[i+1,0], uR[i+1,1], poseR[2], epsilon)
				v[i+1] = commands[0]
				omega[i+1] = commands[1]

				
				# print(commands)
				print(poseR)
				linear_velocity = float(commands[0])
				angular_velocity = float(commands[1])
				# print(poseR[0],poseR[1])
				# vel_msg.linear.x = sendVx
				# vel_msg.linear.y = sendVy

				xerror = poseP[2] - poseR[0]
				yerror = poseP[3] - poseR[1]
				dist2goal = np.sqrt([xerror ** 2 + yerror ** 2])

				#print dist2goal
				if dist2goal < .2:
					#align with person
					# desired orientation of aligning
					theta_d = np.arctan2((poseP[1] - poseR[1]),(poseP[0]-poseR[0]))
					#robotAngle = (poseR[2] + np.pi) % (2 * np.pi ) - np.pi
					theta_e = theta_d - robotAngle										
					linear_velocity = float(0)
					angular_velocity = float(k_omega*theta_e)
					if theta_e < .1:
						print('alignment complete')
					else: 
						print('trying to align')
						

				vel_msg.linear.x = linear_velocity
				vel_msg.angular.z = angular_velocity
				#print(angular_velocity)
				#t0 = rospy.Time.now().to_sec()
				#t1 = t0
				pub.publish(vel_msg)
				#rate.sleep()
		
				#while t1 - t0 < T:
				#	t1 = rospy.Time.now().to_sec()
				#	pub.publish(vel_msg)
				#After the loop, stops the robot
			#vel_msg.linear.x = 0
			#vel_msg.angular.z = 0
			#Force the robot to stop
			#pub.publish(vel_msg)
#        print('COMMANDS')
#        print(commands)
#print([poseRx,poseRy])

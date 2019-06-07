#! /usr/bin/env python
import rospy
import numpy as np
import logging
import argparse
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped

class testing(object):
	def __init__(self):
		self.pos_x=0
		self.pos_y=0
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback)
		self.pub1 = rospy.Publisher('/test_723/inputs/r550_rc', Bool, queue_size=10)	
		self.pub2 = rospy.Publisher('/test_723/inputs/r547_rc', Bool, queue_size=10)	
		self.pub3 = rospy.Publisher('/test_723/inputs/r551_rc', Bool, queue_size=10)
		self.pub4 = rospy.Publisher('/test_723/inputs/hallway_rc', Bool, queue_size=10)
		self.rate = rospy.Rate(10) # 10hz
	def callback(self,PoseWithCovarianceStamped):
		self.pos_x=PoseWithCovarianceStamped.pose.pose.position.x
		self.pos_y=PoseWithCovarianceStamped.pose.pose.position.y	
	def move(self):
		result=np.zeros(4)
		if self.pos_x>2:
			result[0]=1
		elif self.pos_x<0:
			if self.pos_y<-34:
				result[1]=1
			elif self.pos_y>-29:
				result[2]=1
			else:
				result[3]=1
		else:
			result[3]=1
		print(self.pos_x)
		print(self.pos_y)
		return (result)
	def talker(self):		
		self.pub1.publish(self.move()[0]==1)
		self.pub2.publish(self.move()[1]==1)
		self.pub3.publish(self.move()[2]==1)
		self.pub4.publish(self.move()[3]==1)
		print(self.move())
		#ros.loginfo("msg")
		self.rate.sleep()		
if __name__ == "__main__":    
	rospy.init_node("name", anonymous=True)
 	testing1=testing()
	while not rospy.is_shutdown():
		testing1.talker()

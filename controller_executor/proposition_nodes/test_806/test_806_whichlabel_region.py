#! /usr/bin/env python
import rospy
import numpy as np
import logging
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from apriltags_ros.msg import AprilTagDetectionArray

class testing(object):
	def __init__(self):
		self.pos_x=0
		self.pos_y=0
		self.pos_x_ini=0
		self.pos_y_ini=0
		self.angle_ini=0
		self.pos_x_label=0
		self.pos_y_label=0
		self.name = ''  
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback1)
		rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback2)
		self.pub1 = rospy.Publisher('/test_806/inputs/r550_label_rc', Bool, queue_size=10)	
		self.pub2 = rospy.Publisher('/test_806/inputs/r547_label_rc', Bool, queue_size=10)	
		self.pub3 = rospy.Publisher('/test_806/inputs/r551_label_rc', Bool, queue_size=10)
		self.rate = rospy.Rate(10) # 10hz
	def callback1(self,PoseWithCovarianceStamped):
		self.pos_x_ini=PoseWithCovarianceStamped.pose.pose.position.x
		self.pos_y_ini=PoseWithCovarianceStamped.pose.pose.position.y
		self.angle_ini=PoseWithCovarianceStamped.pose.pose.orientation.z
	def callback2(self,data):
		self.pos_x_label=data.detections[0].pose.pose.position.x
		self.pos_y_label=data.detections[0].pose.pose.position.z
	def move(self):
		self.pos_x=self.pos_x_ini+self.pos_y_label*cos(self.angle_ini)-self.pos_x_label*sin(self.angle_ini)
		self.pos_y=self.pos_y_ini+self.pos_y_label*sin(self.angle_ini)+self.pos_x_label*cos(self.angle_ini)
		result=np.zeros(3)
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
		#print(self.pos_x)
		#print(self.pos_y)
		return (result)
	def talker(self):
		if self.name=='':
			self.pub1.publish(False)
			self.pub2.publish(False)
			self.pub3.publish(False)
		else:
			self.pub1.publish(self.move()[0]==1)
			self.pub2.publish(self.move()[1]==1)
			self.pub3.publish(self.move()[2]==1)
			print(self.move())
			#ros.loginfo("msg")
		self.rate.sleep()		
if __name__ == "__main__":    
	rospy.init_node("position", anonymous=True)
 	testing1=testing()
	while not rospy.is_shutdown():
		testing1.talker()

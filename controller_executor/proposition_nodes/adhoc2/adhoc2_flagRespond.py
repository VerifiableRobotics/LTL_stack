#! /usr/bin/env python

'''
subscribe to /adhoc/inputs/respondR
pub to /adhoc/inputs/flagrespond
'''


import rospy
import time
from std_msgs.msg import Bool


class testing(object):
	def __init__(self):
		self.flag=False
		self.data=False
		rospy.Subscriber('/adhoc2/inputs/respondR', Bool, self.callback)
		self.pub = rospy.Publisher('/adhoc2/inputs/flagRespond', Bool, queue_size=10)
		self.rate = rospy.Rate(10) # 10hz
	def callback(self,data):
		self.data=self.data or data.data
if __name__ == "__main__":
	rospy.init_node("flagRespond", anonymous=True)
	testing1=testing()
	while not rospy.is_shutdown():
		if testing1.flag == True:
			testing1.pub.publish(True)
		if testing1.data==True and testing1.flag==False:
			time.sleep(0.2)
			testing1.pub.publish(testing1.data)
			testing1.flag=True
		else:
			testing1.pub.publish(testing1.data)

'''
class testing(object):
	def __init__(self):
		self.data=False
		self.pub = rospy.Publisher('/adhoc/inputs/flagRespond', Bool, queue_size=10)
		self.rate = rospy.Rate(10) # 10hz
	def callback(self,data):
		self.data=self.data or data.data
if __name__ == "__main__":
	rospy.init_node("flagRespond", anonymous=True)
 	testing1=testing()
	while not rospy.is_shutdown():
		testing1.pub.publish(testing1.data)
'''

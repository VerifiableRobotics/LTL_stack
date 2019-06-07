#! /usr/bin/env python

'''
subscribe to robot vicon postion and person postion
rosloginfo(bb)
'''
import rospy
from std_msgs.msg import Bool

'''
class testing(object):
    def __init__(self):
        self.data=True
        self.respondR_data=False
        self.PNext_data=False
        sub1=rospy.Subscriber('/adhoc/inputs/respondR', Bool, self.callback_respondR)
        sub2=rospy.Subscriber('/adhoc/inputs/PNext', Bool, self.callback_PNext)
        self.pub = rospy.Publisher('/adhoc/outputs/drawAttention', Bool, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
    def callback_respondR(self,data):
        self.respondR_data=data.data
        #print(self.respondR_data)
    def callback_PNext(self,data):
        self.PNext_data=data.data
        #print(self.PNext_data)
    def calc(self):
        self.data = not(self.respondR_data or self.PNext_data)
        return self.data
if __name__ == "__main__":
    rospy.init_node("drawAttention", anonymous=True)
    testing1=testing()
    while not rospy.is_shutdown():
        testing1.pub.publish(testing1.data)
        if (testing1.calc()==True):
            rospy.loginfo("Drawing Attention")
        else:
            rospy.loginfo("-------")
'''

class act():
	def __init__(self):
		self.act_bool=False
	def callback(self, data):
		# save latest info
		self.act_bool = data.data
	def print_act(self):
		rospy.loginfo("Drawing Attention")

if __name__ == "__main__":
    rospy.init_node('drawAttention')
    rate = rospy.Rate(10) # set publish rate

    a = act()

    rospy.Subscriber('/adhoc2/outputs/drawAttention', Bool, callback=a.callback)

    while not rospy.is_shutdown():
        if a.act_bool:
			a.print_act()
        rate.sleep()


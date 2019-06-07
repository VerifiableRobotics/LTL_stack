#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from apriltags_ros.msg import AprilTagDetectionArray


#pub = rospy.Publisher('tag_detections', std_msgs.msg.String, queue_size = 10)



def callback(data):
    #rospy.loginfo(data.detections) #need to find the format
    #xyz =data.pos6584198
    if data.detections:
	name = data.detections[0].id     
	xyzpos = data.detections[0].pose.pose.position
	print(name)
        print(xyzpos)


def talker():
    rospy.init_node('xyz_coordinates', anonymous=True) #names the node
    #rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("tag_detections", AprilTagDetectionArray, callback) #where the information is coming from, type, information
    rospy.spin() #simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


#pub = rospy.Publisher('andystalker', AprilTagDetectionArray, queue_size=10)
#pub.Publisher(xyZ)

#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from apriltags2_ros.msg import AprilTagDetectionArray


#pub = rospy.Publisher('tag_detections', std_msgs.msg.String, queue_size = 10)


class sensor1(object):
    def __init__(self):
        self.id_num=0
        self.name=0
        self.xyzpos=0
        rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.callback)
        self.pub = rospy.Publisher('/test_806/inputs/sensor1', Bool, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
    def callback(self,AprilTagDetectionArray):
        if AprilTagDetectionArray.detections:
	    self.name = AprilTagDetectionArray.detections[0].id[0]
            #rospy.loginfo(self.name[0]==7) 
	    #self.xyzpos = AprilTagDetectionArray.detections[0].pose.pose.position.x
    def label(self):
        if AprilTagDetectionArray.detections:
            result=0
            if self.name == 3:
                result = 1
            elif self.name == 7:
                result = 0
            else:
                result = 0
            self.id_num=result
            return result
        else:
            return self.id_num
    def talker(self):
        self.pub.publish(self.label()==1)
        #rospy.init_node('xyz_coordinates', anonymous=True) #names the node
    #rate = rospy.Rate(10) # 10hz
         #where the information is coming from, type, information
        self.rate.sleep()#simply keeps python from exiting until this node is stopped

if __name__ == "__main__":    
    rospy.init_node("sensor1", anonymous=True)
    sensor_806=sensor1()
    while not rospy.is_shutdown():
        sensor_806.talker()

#! /usr/bin/env python

'''
Subscribe to Yuhan's training result: ad_hoc
append path
pub to /adhoc/inputs/respondR
'''

import rospy
import time
from std_msgs.msg import Bool
import sys
#sys.path.insert(0,'/ad_hoc')
from ad_hoc.ad_hoc_robot_controller import *


if __name__ == "__main__":
	print("hi")
	rospy.init_node("respondR", anonymous=True)
	data = False
	pub = rospy.Publisher('/adhoc/inputs/respondR', Bool, queue_size=10)
	rate = rospy.Rate(10) # 10hz
	print("start video...")
	stream = WebcamVideoStream(src='http://yuhan:yuhan@199.168.1.149/media/?action=stream').start()  # default camera



	def get_video_frame():
		colorframe = stream.read()
		grayframe = color.rgb2gray(colorframe)
		#crop the video and leave only the middle area
		#cropped = grayframe[0:300, 150:500]
		cropped = grayframe[300:800, 600:1200]
		#cropped = grayframe
		#print(cropped.shape)
		resized = resize(cropped, (100, 100), mode='constant')
		flattened = resized.flatten()
		return grayframe, flattened[np.newaxis, :]
	# Start robot mind
	robot = Robotmind('touch.joblib')

	#playsound('ask.mp3')
	while not rospy.is_shutdown():
		#print("I'm here")
		frame, buffer = get_video_frame()
		interaction,pred = robot.update_state(buffer)
		if pred == 1:
			data = True
		elif pred == 0:
			data = False
		pub.publish(data)
		print(data)
		print("=========")
		cv2.putText(frame, interaction, (50, 50), cv2.FONT_ITALIC, 0.8, 255)
		cv2.imshow('frame', frame)
		cv2.waitKey(10)

		#time.sleep()


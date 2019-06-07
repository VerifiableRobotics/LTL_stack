import numpy as np
import cv2
from skimage.transform import resize
from joblib import load
from imutils.video import WebcamVideoStream
from skimage import color
from robot_mind import Robotmind
from playsound import playsound
from sklearn.externals import joblib

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
	return cropped, flattened[np.newaxis, :]


def label_frame(frame, classification):
	cv2.putText(frame, classification, (50, 50), cv2.FONT_ITALIC, 0.8, 255)
	cv2.imshow('frame', frame)
	cv2.waitKey(10)

	# Main Loop


if __name__ == "__main__":
	# Start web camera stream
	print("start video...")
	#stream = WebcamVideoStream(src='rtsp://admin@192.168.0.100:554/12').start()  # default camera
	#stream = WebcamVideoStream(src='http://yuhan:yuhan@192.168.10.1/media/?action=stream').start()
	stream = WebcamVideoStream(src='http://yuhan:yuhan@199.168.1.149/media/?action=stream').start()
	print('connected')
	# Start robot mind
	#joblib_model = joblib.load("touch.joblib")
	robot = Robotmind('touch.joblib')
	#robot = Robotmind('touch.joblib')

	#playsound('ask.mp3')





	while True:

		# Read and pre-process frame - resize and flatten
		frame, buffer = get_video_frame()

		# Update robot state with flattened frame buffer
		interaction = robot.update_state(buffer)

		# Speak robot's response
		#robot.speak_help()
		#robot.speak_and_display()

		# Show classification on video display
		label_frame(frame, interaction)

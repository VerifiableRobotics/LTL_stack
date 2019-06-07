#! /usr/bin/env python
import argparse
import rospy
import std_msgs.msg
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Point
from actionlib_msgs.msg import *
import nav_msgs.msg
import Image
import geometry_msgs.msg

class GoForwardAvoid():
	_map_occpy_grid = nav_msgs.msg.OccupancyGrid()
	def __init__(self):
		#rospy.init_node('nav_test', anonymous=False)
		self.controller_request_bool = False

		
	
		#tell the action client that we want to spin a thread by default
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("wait for the action server to come up")
		#allow up to 5 seconds for the action server to come up
		self.move_base.wait_for_server(rospy.Duration(5))	

		#we'll send a goal to the robot to move 3 meters forward
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()

		goal.target_pose.pose.position.x= 0 #3 meters
		goal.target_pose.pose.position.y= -31.5
		goal.target_pose.pose.position.z= 0

		goal.target_pose.pose.orientation.x = 0.0
		goal.target_pose.pose.orientation.y = 0.0
		goal.target_pose.pose.orientation.z = 0.0
		goal.target_pose.pose.orientation.w = 0.1 #go forward
		#start moving
		self.move_base.send_goal(goal)

		#allow TurtleBot up to 60 seconds to complete task
		success = self.move_base.wait_for_result(rospy.Duration(300))

	



		if not success:
			self.move_base.cancel_goal()
			rospy.loginfo("The base failed to move forward 3 meters for some reason")
		else:
			# We made it!
			state = self.move_base.get_state()
			if state == GoalStatus.SUCCEEDED:
		 	   rospy.loginfo("you reached position 0")
	def callback(self, data):
		# save latest info
		self.controller_request_bool = data.data

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Set velocity of robot.")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_subscribe_topic', type=str, help='Specify controller topic to respond to')

    # get arguments
    args, unknown = parser.parse_known_args()

    rospy.init_node(args.node_name)
    rate = rospy.Rate(10) # set publish rate

    a = GoForwardAvoid()

    # subsribe to an output proposition topic
    # e.g: arg.node_subscribe_topic = "example_name/outputs/output_name"
    rospy.Subscriber(args.node_subscribe_topic, std_msgs.msg.Bool, callback=a.callback)

    while not rospy.is_shutdown():
        if a.controller_request_bool:
			GoForwardAvoid()
        rate.sleep()

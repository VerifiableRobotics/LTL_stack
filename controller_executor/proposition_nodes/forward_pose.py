#! /usr/bin/env python
import rospy
import argparse
import sys
import logging

import node_logging
node_logger = logging.getLogger("node_logger")

import gazebo_msgs.srv
import geometry_msgs.msg

def gms_client(model_name,relative_entity_name='world'):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', gazebo_msgs.srv.GetModelState)
        resp1 = gms(model_name,relative_entity_name)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Get pose from /gazebo/model_states and forward pose of name in consideration")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_publish_topic', type=str, help='Specify name of publishing topic to the controller')
    parser.add_argument('model_name', type=str, help='Object name to forward pose')

    args, unknown = parser.parse_known_args()
    node_logger.debug(args)

    rospy.init_node(args.node_name)

    pub = rospy.Publisher(args.node_publish_topic, geometry_msgs.msg.Pose, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # get model
        res = gms_client(args.model_name)

        # forward only the pose
        pub.publish(res.pose)

        # put to sleep
        rate.sleep()
        #node_logger.debug("returnd x position {0}".format(res.pose.position.x))

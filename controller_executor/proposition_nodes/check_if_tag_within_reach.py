#! /usr/bin/env python
import rospy
import argparse
import logging
import time

import tf
import std_msgs.msg
import controller_executor.msg
import geometry_msgs.msg

import node_logging
node_logger = logging.getLogger("node_logger")

class CheckWithinBound(object):
    def __init__(self, args):
        # check if both frame_list_topic and target_frame are specified.
        if args.target_frame and not args.target_frame_list_topic:
            # only use target frame if the list topic is not specified
            self._target_frame = args.target_frame
        else:
            self._target_frame = ''
            rospy.Subscriber(args.target_frame_list_topic, controller_executor.msg.StringArray, callback=self.tag_list_callback)

        self._reference_frame = args.reference_frame
        self._xmin = args.xmin
        self._xmax = args.xmax
        self._ymin = args.ymin
        self._ymax = args.ymax
        self._zmin = args.zmin
        self._zmax = args.zmax
        self._action = False

        # publisher for action status
        self._pub_in_bound = rospy.Publisher(args.node_subscribe_topic+'_target_in_bound', std_msgs.msg.Bool, queue_size=10, latch=True)
        self._pub_in_bound.publish(False)

        # publisher for ac status
        self._pub_status = rospy.Publisher(args.node_subscribe_topic+'_status', std_msgs.msg.Bool, queue_size=10, latch=True)
        self._pub_status.publish(False)

        # publish target pose
        self._pub_target_pose = rospy.Publisher(args.target_pose_topic, geometry_msgs.msg.Pose, queue_size=10, latch=True)

        # transfrom tag to body frame?
        node_logger.info("=== Setting up tf ===")
        self.tf_listener = tf.TransformListener()

    def tag_list_callback(self, data):
        if data.data:
            # only take the first element here temporarily.
            self._target_frame = data.data[0]
            node_logger.debug('Target frame is now {0}'.format(data.data[0]))
        else:
            node_logger.debug('No tag is detected!: {0}'.format(data.data))

    def controller_request(self, data):
        if data.data:
            if not self._action:
                self._action = True

                # find target in reference frame
                reference_target_rot = reference_target_pose = None
                startTime = time.time()
                while not (reference_target_pose and reference_target_rot) and time.time()-startTime < 10.0:
                    try:
                        node_logger.debug( "Does " + self._target_frame + " exist?" + str(self.tf_listener.frameExists(self._target_frame)))
                        (reference_target_pose,reference_target_rot) = self.tf_listener.lookupTransform( \
                                                                    self._reference_frame, self._target_frame, rospy.Time(0))
                    except:
                        rospy.sleep(0.5)
                        node_logger.debug('Reference Frame: {0}, Target frame: {1}'.format(self._reference_frame, self._target_frame))
                        node_logger.debug("Stilling waiting for tag pose and orientation in the world frame")

                # check if tag is within bound given by the user
                if reference_target_pose and (self._xmin <= reference_target_pose[0] <= self._xmax) and \
                                             (self._ymin <= reference_target_pose[1] <= self._ymax) and \
                                             (self._zmin <= reference_target_pose[2] <= self._zmax):
                    self._pub_in_bound.publish(True)
                    node_logger.debug("Target frame is found and it's in bound with position {0} and orientation {1}!".format(reference_target_pose, reference_target_rot))

                    # Here we also publish target pose
                    target_pose = geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(*reference_target_pose),\
                                                        orientation=geometry_msgs.msg.Quaternion(*reference_target_rot))
                    self._pub_target_pose.publish(target_pose)

                else:
                    self._pub_in_bound.publish(False)
                    node_logger.debug("Target frame is not found or the frame is out of bound!")


                # publish we have finished status
                self._pub_status.publish(True)

        else:
            self._action = False

            # publish we have stopped status
            self._pub_status.publish(False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="tkinter textbox")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_subscribe_topic', type=str, help='Specify controller topic to respond to')
    parser.add_argument('--target_frame_list_topic', type=str, help='Specify topic to retrieve list of tags detected.', nargs='?', const='/tag_list', default='/tag_list')
    parser.add_argument('--target_pose_topic', type=str, help='Specify topic to publish target pose. geometry_msgs.msg.Pose()', nargs='?', const='/target_pose', default='/target_pose')
    parser.add_argument('--reference_frame', type=str, help='Specify reference frame.', nargs='?', const='base_footprint', default='base_footprint')
    parser.add_argument('--target_frame', type=str, help='Specify target frame.', nargs='?', const='', default='')
    parser.add_argument('--xmin', type=float, help='Specify x minimum bound.', nargs='?', const=-1.0, default=-1.0)
    parser.add_argument('--xmax', type=float, help='Specify x maximum bound.', nargs='?', const=1.0, default=1.0)
    parser.add_argument('--ymin', type=float, help='Specify y minimum bound.', nargs='?', const=-1.0, default=-1.0)
    parser.add_argument('--ymax', type=float, help='Specify y maximum bound.', nargs='?', const=1.0, default=1.0)
    parser.add_argument('--zmin', type=float, help='Specify z minimum bound.', nargs='?', const=-1.0, default=-1.0)
    parser.add_argument('--zmax', type=float, help='Specify z maximum bound.', nargs='?', const=1.0, default=1.0)

    args, unknown = parser.parse_known_args()
    node_logger.debug(args)

    rospy.init_node(args.node_name)

    a = CheckWithinBound(args)

    rospy.Subscriber(args.node_subscribe_topic, std_msgs.msg.Bool, callback=a.controller_request)

    rospy.spin()

#! /usr/bin/env python
import rospy
import argparse
import json
import matplotlib.path as mplPath
import numpy as np
import logging
import importlib
import Image

import std_msgs.msg
import gazebo_msgs.srv

import node_logging
node_logger = logging.getLogger('node_logger')

class RobotLocationChecker(object):
    _region_obj_dict = {}
    _current_region = ""

    def __init__(self, model_name, json_file):
        self._model_name = model_name
        self._json_file = json_file
        self.check_rate = rospy.Rate(10)

        self.pub = rospy.Publisher('robot_current_region', std_msgs.msg.String, queue_size=10)

    def get_region_obj(self, region_info, rot, scale, x_trans, y_trans):

        # translate and rotate point + scale too
        transform_matrix = np.array([[np.cos(rot), -np.sin(rot), x_trans],\
                                    [np.sin(rot), np.cos(rot),  y_trans],\
                                    [0 , 0, 1]])
        scale_matrix = np.array([[scale, 0, 0],\
                               [0, scale, 0],\
                               [0 , 0, 1]])
        #node_logger.debug('Transformation matrix: {0}'.format(transform_matrix))

        # actually need to flip the height here. So load the image to do that.
        im = Image.open(self._json_file.replace('.json','.png')) # open file
        im_width, im_height = im.size
        node_logger.log(2,'im_height: {0}  im_width:{1}'.format(im_height, im_width))

        transformed_region = []
        for point_org in region_info['points']:
            # first translate point based on region relative to full map
            point = [sum(x) for x in zip(point_org, region_info['position'])]
            point[1] = im_height-point[1]

            # now do map rotation
            new_point = np.dot(scale_matrix,np.dot(transform_matrix,np.array([[point[0]],[point[1]],[1]])))
            #node_logger.debug('Old point:{0}, New point:{1}'.format(point, new_point))

            # convert to format for checking occupancy
            transformed_region.append(np.transpose(new_point[0:2]).tolist()[0])

        transformed_region.append(transformed_region[0])
        node_logger.debug('Transformed region: {0}'.format(transformed_region))
        region_path = mplPath.Path(transformed_region)

        return region_path

    def check_current_region(self):
        # first get robot pose
        robot_pose = self.get_model_state_from_gazebo(self._model_name)
        node_logger.log(2, 'robot_pose: {0}.'.format(robot_pose))

        if robot_pose: # we got a valid pose
            robot_in_region = False
            current_region = ""
            for region, region_obj in self._region_obj_dict.iteritems():
                if region != 'boundary':
                    in_region = region_obj.contains_point((robot_pose.pose.position.x, \
                                                           robot_pose.pose.position.y), radius=0.20) #center will be 10cm from bounrdary
                    node_logger.log(2, '{0} region_obj: {1}.'.format(region, region_obj))

                    if in_region:
                        robot_in_region = True
                        current_region = region
                        node_logger.log(6, 'Robot is in region {0}.'.format(region))
                        break

            # if the robot is in between regions
            if not robot_in_region:
                node_logger.warning("We are using the previous region {0} for now!".format(self._current_region))
                current_region = self._current_region

            # now publish region info
            self._current_region = current_region
            self.pub.publish(current_region)

            self.check_rate.sleep()

        # TODO: make it not as sensitive...

    def get_model_state_from_gazebo(self, model_name, relative_entity_name='world'):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', gazebo_msgs.srv.GetModelState)
            resp1 = gms(model_name,relative_entity_name)
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This node subscribes pose info from gazebo and determine which region the robot is in.")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('--model_name', type=str, help='Name of gazebo object')
    parser.add_argument('--json_file', type=str, help='Path to region json file')
    parser.add_argument('--rot', type=float, help='region rotation', nargs='?', const=0, default=0)
    parser.add_argument('--scale', type=float, help='region x scale', nargs='?', const=1, default=1)
    parser.add_argument('--x_trans', type=float, help='region x translation', nargs='?', const=0, default=0)
    parser.add_argument('--y_trans', type=float, help='region y translation', nargs='?', const=0, default=0)

    args, unknown = parser.parse_known_args()
    node_logger.debug(args)

    rospy.init_node(args.node_name)

    # load region json file
    with open(args.json_file,'r') as json_file_obj:
        json_data = json.loads(json_file_obj.read())
    json_file_obj.closed

    a = RobotLocationChecker(args.model_name, args.json_file)
    # get only current region info
    for region_info in json_data:
        a._region_obj_dict[region_info['name']] =  a.get_region_obj(region_info, \
                args.rot, args.scale, args.x_trans, args.y_trans)

        #node_logger.debug('Region_info of {1}: {0}'.format(region_info['name'], region_info))

    while not rospy.is_shutdown():
        a.check_current_region()
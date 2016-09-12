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

import node_logging
node_logger = logging.getLogger('node_logger')

from controller_executor import region_operations

class RegionOccupancyCheck(object):
    _current_pose = None
    _region_path_obj = None

    def __init__(self, node_publish_topic, region_in_consideration, pose_x_string, pose_y_string):
        self._pose_x_string = pose_x_string
        self._pose_y_string = pose_y_string
        self._region_in_consideration = region_in_consideration

        self.pub = rospy.Publisher(node_publish_topic, std_msgs.msg.Bool, queue_size=10)

    # def get_region_obj(self, json_file, region_in_consideration, rot, scale, x_trans, y_trans):
    #    # load region json file
    #     with open(json_file,'r') as json_file_obj:
    #         json_data = json.loads(json_file_obj.read())
    #     json_file_obj.closed

    #     # get only current region info
    #     region_info = next(x for x in json_data if x['name'] == region_in_consideration)
    #     node_logger.debug('Region_info of {1}: {0}'.format(region_in_consideration, region_info))

    #     # translate and rotate point + scale too
    #     transform_matrix = np.array([[np.cos(rot), -np.sin(rot), x_trans],\
    #                                 [np.sin(rot), np.cos(rot),  y_trans],\
    #                                 [0 , 0, 1]])
    #     scale_matrix = np.array([[scale, 0, 0],\
    #                            [0, scale, 0],\
    #                            [0 , 0, 1]])
    #     #node_logger.debug('Transformation matrix: {0}'.format(transform_matrix))

    #     # actually need to flip the height here. So load the image to do that.
    #     im = Image.open(json_file.replace('.json','.png')) # open file
    #     im_width, im_height = im.size
    #     node_logger.debug(im_height)
    #     node_logger.debug(im_width)

    #     transformed_region = []
    #     for point_org in region_info['points']:
    #         # first translate point based on region relative to full map
    #         point = [sum(x) for x in zip(point_org, region_info['position'])]
    #         point[1] = im_height-point[1]

    #         # now do map rotation
    #         new_point = np.dot(scale_matrix,np.dot(transform_matrix,np.array([[point[0]],[point[1]],[1]])))
    #         node_logger.debug('Old point:{0}, New point:{1}'.format(point, new_point))

    #         # convert to format for checking occupancy
    #         transformed_region.append(np.transpose(new_point[0:2]).tolist()[0])



    #     transformed_region.append(transformed_region[0])
    #     node_logger.debug('Transformed region: {0}'.format(transformed_region))
    #     region_path = mplPath.Path(transformed_region)

    #     """
    #     import matplotlib.pyplot as plt
    #     import matplotlib.patches as patches

    #     fig = plt.figure()
    #     ax = fig.add_subplot(111)
    #     patch = patches.PathPatch(region_path, facecolor='orange')
    #     ax.add_patch(patch)
    #     ax.set_xlim(0,10)
    #     ax.set_ylim(0,10)
    #     plt.title(region_in_consideration)
    #     plt.show()
    #     """

    #     return region_path

    def update_current_pose(self, data):
        self._current_pose = data
        in_region = self._region_path_obj.contains_point((eval('self._current_pose.'+self._pose_x_string), \
                                                eval('self._current_pose.'+self._pose_y_string)))
        if in_region:
            pass
            #node_logger.debug("x: {0}".format(eval('self._current_pose.'+self._pose_x_string)))
            #node_logger.debug("y: {0}".format(eval('self._current_pose.'+self._pose_y_string)))
            #node_logger.debug('Robot is in region {0}.'.format(self._region_in_consideration))

        # publish info
        self.pub.publish(in_region)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Check if robot is in region. Currently assumes no rotation and translation of region.")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_publish_topic', type=str, help='Specify name of publishing topic to the controller')
    parser.add_argument('--json_file', type=str, help='Path to region json file')
    parser.add_argument('--region_in_consideration', type=str, help='region to check occupancy')
    parser.add_argument('--rot', type=float, help='region rotation', nargs='?', const=0, default=0)
    parser.add_argument('--scale', type=float, help='region scale in both x and y direction', nargs='?', const=1, default=1)
    parser.add_argument('--x_trans', type=float, help='region x translation', nargs='?', const=0, default=0)
    parser.add_argument('--y_trans', type=float, help='region y translation', nargs='?', const=0, default=0)
    parser.add_argument('--pose_topic', type=str, help='Specify pose topic to subscribe to', \
                nargs='?', const='/mobile_base/pose', default='/mobile_base/pose')
    parser.add_argument('--pose_topic_type', type=str, help='Specify the type of pose topic', \
                nargs='?', const='geometry_msgs.msg.Pose', default='geometry_msgs.msg.Pose')
    parser.add_argument('--pose_x_string', type=str, help='Specify how to retrieve x-coordinate from pose object.',
                nargs='?', const='position.x', default='position.x')
    parser.add_argument('--pose_y_string', type=str, help='Specify how to retrieve y-coordinate from pose object.',
                nargs='?', const='position.y', default='position.y')

    args, unknown = parser.parse_known_args()
    node_logger.debug(args)

    rospy.init_node(args.node_name)

    a = RegionOccupancyCheck(args.node_publish_topic, args.region_in_consideration, args.pose_x_string, args.pose_y_string)

    # get region object to check against pose
    a._region_path_obj = region_operations.get_region_obj(args.json_file, args.region_in_consideration, args.rot, \
                        args.scale, args.x_trans, args.y_trans)

    # subscribe to pose
    # import module for pose
    module = importlib.import_module(args.pose_topic_type.partition(".msg.")[0]) # import module
    globals()[args.pose_topic_type.partition(".msg.")[0]] = module
    module_msg = importlib.import_module(args.pose_topic_type.partition(".msg.")[0]+'.msg') # import msg
    globals()[args.pose_topic_type.partition(".msg.")[0]+'.msg'] = module_msg

    # subscribe to current pose info
    rospy.Subscriber(args.pose_topic, eval(args.pose_topic_type), callback=a.update_current_pose)

    rospy.spin()
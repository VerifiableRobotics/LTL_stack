#! /usr/bin/env python
import json
import argparse
import rospy
import yaml
import Image
import os.path
import logging
import os
import re
import numpy as np
import matplotlib.path as mplPath
import actionlib
import time

import nav_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import move_base_msgs.msg

import node_logging
node_logger = logging.getLogger("node_logger")

from controller_executor import region_operations

# TODO: subscribe to run_executor and cancel goal when it is paused?
PUB_MAP_THRES = 5

class MapPublisher(object):
    _current_region = ""
    _prev_current_region = ""
    _published_current_region = ""
    _x_goal = 0.0
    _y_goal = 0.0
    _nav_goal = move_base_msgs.msg.MoveBaseGoal()
    _map_occpy_grid = nav_msgs.msg.OccupancyGrid()

    def __init__(self, node_subscribe_topic, next_region, image_folder, image_basename, occupied_thresh, free_thresh,\
                    rot, scale, x_trans, y_trans):
        self._next_region = next_region
        self._image_folder = image_folder
        self._image_basename = image_basename
        self._occupied_thresh = occupied_thresh
        self._free_thresh = free_thresh
        self._image_dir = ""
        self._rate = rospy.Rate(10)
        self._map_occpy_grid_dict = {}
        self._map_occpy_grid_width = 0
        self._map_occpy_grid_height = 0
        self._last_pub_time = time.time()#rospy.Time.now() # ros time is much slower than real time if running a lot of things

        # save transformations
        self._map_rot = rot
        self._map_scale = scale
        self._map_x_trans = x_trans
        self._map_y_trans = y_trans

        self.pub_map_count = 0 # only publish when we consistently geting signals

        # preload all possible maps for this next region
        # first load the base case
        im = Image.open(self._image_folder+self._image_basename+'.png') # open file
        self._map_occpy_grid_width,  self._map_occpy_grid_height = im.size
        self._map_occpy_grid_dict['base'] = self.form_occupancy_list(im)

        # load the others
        for file in [f for f in os.listdir(self._image_folder) if \
                    os.path.isfile(os.path.join(self._image_folder, f))]:
            if next_region in file and file.endswith('.png'):
                im = Image.open(self._image_folder+file) # open file

                # find that current region name
                match = re.search(self._image_basename+r'_(?P<first_reg>\w+)_(?P<second_reg>\S+).png', file)
                if match.group('first_reg') == next_region:
                    # form occupancy grid in list form
                    self._map_occpy_grid_dict[match.group('second_reg')] = self.form_occupancy_list(im)

                elif match.group('second_reg') == next_region:
                    self._map_occpy_grid_dict[match.group('first_reg')] = self.form_occupancy_list(im)

                else:
                    node_logger.error('Which one is the current region? {0} with next region {1}'.format(file, next_region))

        node_logger.info('Preloaded all occupancy grids for next region {0}'.format(self._next_region))
        node_logger.debug('Next region {0} with possible transition to {1}'.format(self._next_region, self._map_occpy_grid_dict.keys()))

    def setup_message_passing(self, node_subscribe_topic):
        # set up move base client for navigation goal
        self._move_base_client = actionlib.SimpleActionClient("/move_base", move_base_msgs.msg.MoveBaseAction)

        # subcribe to controller request
        rospy.Subscriber(node_subscribe_topic, std_msgs.msg.Bool, callback=self.publish_map)

        # set up map publisher
        self._map_pub = rospy.Publisher('/map', nav_msgs.msg.OccupancyGrid, queue_size=10, latch=True)

        # maybe publsih init map here...
        # set info
        self._map_occpy_grid.data = self._map_occpy_grid_dict['base']
        init_pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(self._map_x_trans, self._map_y_trans, 0), \
                                           geometry_msgs.msg.Quaternion(0, 0, np.sin(self._map_rot/2), np.cos(self._map_rot/2))) # x y z w
        self._map_occpy_grid.info = nav_msgs.msg.MapMetaData(rospy.Time(), self._map_scale, \
                                            self._map_occpy_grid_width, self._map_occpy_grid_height, init_pose)
        # set header
        self._map_occpy_grid.header.stamp = rospy.Time()
        self._map_occpy_grid.header.frame_id = 'map'

        # update map info
        self._map_pub.publish(self._map_occpy_grid)
        self._last_pub_time = rospy.Time.now()

        # check if server is up
        node_logger.info('Finding for /move_base server ...')
        self._move_base_client.wait_for_server()
        node_logger.info('Found /move_base server ...')

    def find_goal_point(self, json_file, next_region, rot, scale, x_trans, y_trans):
        # find a point inside the next_region to go to
        self._x_goal, self._y_goal = region_operations.find_point_in_region(json_file, next_region, rot, scale, x_trans, y_trans)
        # region_obj = self.get_region_obj(json_file, next_region, rot, scale, x_trans, y_trans)
        # #node_logger.debug(region_obj.get_extents().get_points())
        # [[xmin, ymin], [xmax, ymax]] = region_obj.get_extents().get_points()

        # # find goal point
        # if region_obj.contains_point(((xmin+xmax)/2, (ymax+ymin)/2)):
        #     self._x_goal = (xmin+xmax)/2
        #     self._y_goal = (ymax+ymin)/2
        # else:
        #     # I guess we need to sample point until we find one
        #     x_goal, y_goal = 0,0
        #     while not rospy.is_shutdown() or region_obj.contains_point((x_goal, y_goal)):
        #         x_goal = random.randrange(xmin, xmax)
        #         y_goal = random.randrange(ymin, ymax)

        #     self._x_goal, self._y_goal = x_goal, y_goal

        # format goal
        self._nav_goal.target_pose.header.frame_id = "map"
        self._nav_goal.target_pose.header.stamp = rospy.Time.now()
        self._nav_goal.target_pose.pose.position.x = self._x_goal
        self._nav_goal.target_pose.pose.position.y = self._y_goal
        self._nav_goal.target_pose.pose.orientation.w = 1.0


    def publish_map(self, data):
        if data.data:

            self.pub_map_count += 1
            #node_logger.debug("Map publish request: {0}.".format(rospy.get_name()))

            # update map topic with new occupancy grid (if region changes)
            if self.pub_map_count > PUB_MAP_THRES and \
                (not self._map_occpy_grid or not self._published_current_region or \
                (time.time()-self._last_pub_time) > 5):
                #(rospy.Time.now()-self._last_pub_time) > 10:#rospy.Duration(10):

                self._last_pub_time = time.time()#rospy.Time.now()  # 30s
                # _published_current_region reset if moved to new region

                # form data to send
                # set data
                if not self._current_region or self._current_region == self._next_region:
                    self._map_occpy_grid.data = self._map_occpy_grid_dict['base']
                    self._published_current_region = 'base'
                else:
                    self._map_occpy_grid.data = self._map_occpy_grid_dict[self._current_region]
                    self._published_current_region = self._current_region

                # set info
                init_pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(self._map_x_trans, self._map_y_trans, 0), \
                                                   geometry_msgs.msg.Quaternion(0, 0, np.sin(self._map_rot/2), np.cos(self._map_rot/2))) # x y z w
                self._map_occpy_grid.info = nav_msgs.msg.MapMetaData(rospy.Time(), self._map_scale, \
                                            self._map_occpy_grid_width, self._map_occpy_grid_height, init_pose)
                # set header
                self._map_occpy_grid.header.stamp = rospy.Time()
                #self._map_occpy_grid.header.frame_id = 'map'

                # update map info
                self._map_pub.publish(self._map_occpy_grid)
                node_logger.debug("Published occupancy grid from node {0}.".format(rospy.get_name()))
                #self._rate.sleep()

                # find a point in the region to go to

                # Update the goal pose
                node_logger.info('Waiting for /move_base server ...')
                self._move_base_client.wait_for_server()

                # format goal
                node_logger.info('Goal sending for {0}: {1}'.format(rospy.get_name(), self._nav_goal))
                self._nav_goal.target_pose.header.stamp = rospy.Time.now()
                self._move_base_client.send_goal(self._nav_goal)

                # Goal may not be finished and we don't care here.
                #self._move_base_client.waitForResult();
                #if self._move_base_client.getState() == actionlib.SimpleClientGoalState.SUCCEEDED:
                #    node_logger.info("Finished Goal")
                #else:
                #    node_logger.info("Goal failed or aborted.")
            #else:
            #    node_logger.info('Navigation time lapse: {0}'.format((rospy.Time.now()-self._last_pub_time)))
        else:
            self.pub_map_count = 0 # reset count

    def form_occupancy_list(self, image_old_obj):
        image_obj = image_old_obj.transpose(Image.FLIP_TOP_BOTTOM) # test

        # if there's alpha
        if image_obj.mode in ('RGBA', 'LA') or (image_obj.mode == 'P' and 'transparency' in image_obj.info):
            alpha = image_obj.convert('RGBA').split()[-1]
            pixel_list = [int((255-sum(x[0:3])/3)/255.0*100.0) if alpha.getdata()[idx]> 0 else -1 \
                            for idx, x in enumerate(image_obj.getdata())]
        else:
            pixel_list = [int((255-sum(x)/len(x))/255.0*100.0) for x in image_obj.getdata()]

        # reverse to have the map displaye correctly
        #pixel_list.reverse()

        for idx, value in enumerate(pixel_list):
            if value > self._occupied_thresh:
                pixel_list[idx] = 100
            elif value < self._free_thresh:
                pixel_list[idx] = 0

        return pixel_list

    def subscribe_to_current_region_info(self, region_list, input_namespace):
        # get current region info
        for region_name in region_list:
            rospy.Subscriber(input_namespace+'/inputs/'+region_name+'_rc', std_msgs.msg.Bool, \
                callback=self.update_current_region, callback_args=region_name)

    def update_current_region(self, data, region_name):
        if data.data and self._current_region != region_name:
            self._published_current_region = ""
            self._current_region = region_name
            #node_logger.debug("Current region is {0}".format(region_name))

    # def get_region_obj(self, json_file, next_region, rot, scale, x_trans, y_trans):
    #    # load region json file
    #     with open(json_file,'r') as json_file_obj:
    #         json_data = json.loads(json_file_obj.read())
    #     json_file_obj.closed

    #     # get only current region info
    #     region_info = next(x for x in json_data if x['name'] == next_region)
    #     node_logger.debug('Region_info of {1}: {0}'.format(next_region, region_info))

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

    #     # close region
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Naviation of robot with the navigation stack")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_subscribe_topic', type=str, help='Specify controller topic to respond to')
    parser.add_argument('--next_region', type=str, help='Specify name of next region', nargs='?', \
                            const='bedroom', default='bedroom')
    parser.add_argument('--input_namespace', type=str, help='Specify namespace of inputs', nargs='?', \
                            const='/firefighting', default='/firefighting')
    parser.add_argument('--image_folder', type=str, help='Specify path to folder storing png maps', nargs='?', \
                            const='/home/catherine/LTLMoP/src/examples/firefighting/', \
                            default='/home/catherine/LTLMoP/src/examples/firefighting/')
    parser.add_argument('--image_basename', type=str, help='Specify basename of all images', nargs='?', \
                            const='floorplan', default='floorplan')
    #parser.add_argument('--region_list', action='append', help='Specify basename of all images')
    parser.add_argument('--region_list', nargs = '+', help='Specify basename of all images')

    parser.add_argument('--occupied_thresh', type=int, help='Occcupied Threshold', nargs='?', \
                            const=70, default=70, choices=range(0,101), metavar="[0-100]")
    parser.add_argument('--free_thresh', type=int, help='Free Threshold', nargs='?', \
                            const=20, default=20, choices=range(0,101), metavar="[0-100]")

    # for getting region
    parser.add_argument('--json_file', type=str, help='Path to region json file')
    parser.add_argument('--rot', type=float, help='region rotation', nargs='?', const=0, default=0)
    parser.add_argument('--scale', type=float, help='region scale in both x and y direction. meter to pixel', nargs='?', const=1, default=1)
    parser.add_argument('--x_trans', type=float, help='region x translation', nargs='?', const=0, default=0)
    parser.add_argument('--y_trans', type=float, help='region y translation', nargs='?', const=0, default=0)

    args, unknown = parser.parse_known_args()
    #node_logger.debug("Running robot navigation.")
    #node_logger.debug(args)

    rospy.init_node(args.node_name)

    a = MapPublisher(args.node_subscribe_topic, args.next_region, args.image_folder, args.image_basename, \
                        args.occupied_thresh, args.free_thresh, args.rot, \
                                      args.scale, args.x_trans, args.y_trans)

    # find goal point
    a.find_goal_point(args.json_file, args.next_region, args.rot, \
                                      args.scale, args.x_trans, args.y_trans)

    # setup subscribers, publishers and actions
    a.setup_message_passing(args.node_subscribe_topic)

    # subscribe to current region
    a.subscribe_to_current_region_info(args.region_list, args.input_namespace)

    node_logger.debug('Now all ready for request!')
    rospy.spin()


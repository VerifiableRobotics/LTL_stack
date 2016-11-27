#! /usr/bin/env python
import json
import Image
import numpy as np
import matplotlib.path as mplPath
import logging

import controller_executor_logging
region_operations_logger = logging.getLogger("region_operations_logger")

def get_region_obj(json_file, next_region, rot, scale, x_trans, y_trans):
    """
    This function loads the region json file and retrieve region as a matplotlib path object
    """
    # load region json file
    with open(json_file,'r') as json_file_obj:
        json_data = json.loads(json_file_obj.read())
    json_file_obj.closed

    # get only current region info
    region_info = next(x for x in json_data if x['name'] == next_region)
    region_operations_logger.debug('Region_info of {1}: {0}'.format(next_region, region_info))

    # translate and rotate point + scale too
    transform_matrix = np.array([[np.cos(rot), -np.sin(rot), x_trans],\
                                [np.sin(rot), np.cos(rot),  y_trans],\
                                [0 , 0, 1]])
    scale_matrix = np.array([[scale, 0, 0],\
                           [0, scale, 0],\
                           [0 , 0, 1]])
    #region_operations_logger.debug('Transformation matrix: {0}'.format(transform_matrix))

    # actually need to flip the height here. So load the image to do that.
    im = Image.open(json_file.replace('.json','.png')) # open file
    im_width, im_height = im.size
    region_operations_logger.debug(im_height)
    region_operations_logger.debug(im_width)

    transformed_region = []
    for point_org in region_info['points']:
        # first translate point based on region relative to full map
        point = [sum(x) for x in zip(point_org, region_info['position'])]
        point[1] = point[1] #im_height-point[1]

        # now do map rotation
        new_point = np.dot(transform_matrix,np.dot(scale_matrix,np.array([[point[0]],[point[1]],[1]])))
        region_operations_logger.debug('Old point:{0}, New point:{1}'.format(point, new_point))

        # convert to format for checking occupancy
        transformed_region.append(np.transpose(new_point[0:2]).tolist()[0])

    # close region
    transformed_region.append(transformed_region[0])
    region_operations_logger.debug('Transformed region: {0}'.format(transformed_region))
    region_path = mplPath.Path(transformed_region)

    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    fig = plt.figure()
    ax = fig.add_subplot(111)
    patch = patches.PathPatch(region_path, facecolor='orange')
    ax.add_patch(patch)
    # recompute the ax.dataLim
    ax.relim()
    # update ax.viewLim using the new dataLim
    ax.autoscale_view()
    plt.title(next_region)
    plt.show()
    """
    return region_path


def find_point_in_region(json_file, next_region, rot, scale, x_trans, y_trans):
    # find a point inside the next_region to go to
    region_obj = get_region_obj(json_file, next_region, rot, scale, x_trans, y_trans)
    #node_logger.debug(region_obj.get_extents().get_points())
    [[xmin, ymin], [xmax, ymax]] = region_obj.get_extents().get_points()

    # find goal point
    if region_obj.contains_point(((xmin+xmax)/2, (ymax+ymin)/2)):
        x_goal = (xmin+xmax)/2
        y_goal = (ymax+ymin)/2
    else:
        # I guess we need to sample point until we find one
        x_goal, y_goal = 0,0
        while not rospy.is_shutdown() or region_obj.contains_point((x_goal, y_goal)):
            x_goal = random.randrange(xmin, xmax)
            y_goal = random.randrange(ymin, ymax)

    return x_goal, y_goal

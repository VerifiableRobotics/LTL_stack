#! /usr/bin/env python
import rospy
import logging


import ?????? as regions
import ?????? as vectorControllerHelper
import node_logging
node_logger = logging.getLogger("node_logger")


# Assumptions: It is already regions, not in bits


class VectorFieldController(object):
    _current_region_idx = None # int
    _next_region_idx = None # int
    _current_pose = None

    def __init__(self, region_file, pose_topic, pose_topic_type, next_region, pose_x_string, \
                    pose_y_string, pose_w_string, vel_topic, vel_topic_type):
        # save string to retrieve pose info
        self._pose_x_string = pose_x_string
        self._pose_y_string = pose_y_string
        self._pose_w_string = pose_w_string

        # load region file
        self.rfi = regions.RegionFileInterface()
        self.rfi.readFile(region_file)

        # subscribe to current region info
        for region_idx, region in enumerate(self.rfi.regions):
            self.subscribe_to_current_region_info(region.name, region_idx)

        # subscribe to current pose info
        rospy.Subscriber(pose_topic, eval(pose_topic_type), callback=self.update_current_pose)

        # subcribe to controller request
        rospy.Subscriber('outputs/'+next_region, std_msgs.msg.Bool, callback=self.get_velocity)

        # check next region idx
        for region_idx, region in enumerate(self.rfi.regions):
            if region == next_region:
                self._next_region_idx = region_idx
                break

        # setup velocity publisher to command the robot
        self.vel_pub = rospy.Publisher(vel_topic, eval(vel_topic_type), queue_size=10)
        self._vel_topic_type = vel_topic_type

    def subscribe_to_current_region_info(self, region_name, region_idx):
        # get current region info
        for region in region_list:
            rospy.Subscriber('inputs/'+region_name, std_msgs.msg.Bool, \
                callback=self.update_current_region, callable_args=region_idx)

    def update_current_region(self, data, region_idx):
        if data.data:
            self._current_region_idx = region_idx
            node_logging.debug("Current region is {0}".format(self.rfi.regions[idx].name))

    def update_current_pose(self, data):
        self._current_pose = data

    def get_velocity(self, data):
        if data.data:
            if self.get_transition_face() is None:
                # stay in place if there's no transition face
                V = [0,0]

            else:
                # Run algorithm to find a velocity vector (global frame) to take the robot to the next region
                V = vectorControllerHelper.getController([eval('self._current_pose.'+self._pose_x_string), \
                        eval('self._current_pose.'+self._pose_y_string)], \
                        self.current_region_vertices(), self.get_transition_face())


            # can be done with another node?
            # first convert to local velocity of the robot


            # publish velocity
            #vel_obj = eval(self._vel_topic_type)()
            #self.vel_pub.publish(????)

            # publish vx, vy, theta

    def get current_region_vertices(self):
        # NOTE: Information about region geometry can be found in self.rfi.regions:
        pointArray = [x for x in self.rfi.regions[self._current_region_idx].getPoints()]
        pointArray = map(self.coordmap_map2lab, pointArray)
        vertices = mat(pointArray).T
        return vertices

    def get_transition_face(self):
        # For now, let's just choose the largest face available, because we are probably using a big clunky robot
        # TODO: Why don't we just store this as the index?
        transFaceIdx = None
        max_magsq = 0
        for i, face in enumerate(self.rfi.regions[self._current_region_idx].getFaces()):
            if face not in self.rfi.transitions[self._current_region_idx][self._next_region_idx]:
                continue

            tf_pta, tf_ptb = face
            tf_vector = tf_ptb - tf_pta
            magsq = (tf_vector.x)**2 + (tf_vector.y)**2
            if magsq > max_magsq:
                transFaceIdx = i
                max_magsq = magsq

        if transFaceIdx is None:
            node_logger.warning("ERROR: Unable to find transition face between regions %s and %s.  Please check the decomposition (try viewing projectname_decomposed.regions in RegionEditor or a text editor)."\
             % (self.rfi.regions[current_reg].name, self.rfi.regions[next_reg].name))

        return transFaceIdx

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Vector Control Motion Planner")
    parser.add_argument('node_name', type=str, help='Specify name of ros node')
    parser.add_argument('node_subscribe_topic', type=str, help='Specify controller topic to respond to')
    parser.add_argument('--vel_topic', type=str, help='Specify name of topic to publish velocity info', nargs='?', \
                            const='/turtle1/cmd_vel', default='/turtle1/cmd_vel')
    parser.add_argument('--vel_topic_type', type=str, help='Specify topic type to publish velocity info', nargs='?', \
                            const='geometry_msgs.msg.Twist', default='geometry_msgs.msg.Twist')
    parser.add_argument('region_file', type=str, help='Specify region file directory')
    parser.add_argument('--pose_topic', type=str, help='Specify pose topic to subscribe to', \
                nargs='?', const='/turtle1/pose', default='/turtle1/pose')
    parser.add_argument('--pose_topic_type', type=str, help='Specify the type of pose topic', \
                nargs='?', const='turtlesim_msgs.msg.Pose', default='turtlesim_msgs.msg.Pose')
    parser.add_argument('--pose_x_string', type=str, help='Specify how to retrieve x-coordinate from pose object.',
                nargs='?', const='x', default='x')
    parser.add_argument('--pose_y_string', type=str, help='Specify how to retrieve y-coordinate from pose object.',
                nargs='?', const='y', default='y')
    parser.add_argument('--pose_w_string', type=str, help='Specify how to retrieve orientation from pose object.',
                nargs='?', const='theta', default='theta')

    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()
    node_logger.debug(args)

    rospy.init_node(args.node_name)

    # import module for pose
    module = importlib.import_module(args.pose_topic_type.partition(".msg.")[0]) # import module
    globals()[args.pose_topic_type.partition(".msg.")[0]] = module
    module_msg = importlib.import_module(args.pose_topic_type.partition(".msg.")[0]+'.msg') # import msg
    globals()[args.pose_topic_type.partition(".msg.")[0]+'.msg'] = module_msg

    # import module for velocity
    module = importlib.import_module(args.vel_topic_type.partition(".msg.")[0]) # import module
    globals()[args.vel_topic_type.partition(".msg.")[0]] = module
    module_msg = importlib.import_module(args.vel_topic_type.partition(".msg.")[0]+'.msg') # import msg
    globals()[args.vel_topic_type.partition(".msg.")[0]+'.msg'] = module_msg

    a = VectorFieldController(args.region_file, args.pose_topic, args.pose_topic_type, arg.node_name,\
            args.pose_x_string, args.pose_y_string, args.pose_w_string, args.vel_topic, arg.vel_topic_type)

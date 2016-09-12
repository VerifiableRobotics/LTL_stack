import pydot
import logging
import ast
#import rqt_graph.ros_graph

# increase recursion limit
# todo - maybe do a tail call instead see:
# https://github.com/lihaoyi/macropy#tail-call-optimization
import resource, sys
resource.setrlimit(resource.RLIMIT_STACK, (2**29,-1))
sys.setrecursionlimit(10**6)

import controller_executor_logging
check_resources_logger = logging.getLogger("check_resources_logger")
import file_operations
from controller_executor import check_resource_usage


# --- firefighting ---- #
#load dot file
#dot_file =  pydot.graph_from_dot_file('/home/catherine/Dropbox/ASL/ASL_Summer_2016/exclusions/firefighting_stay_in_place_all.dot')

#load inputs and outputs
#input_prop_to_ros_info, output_prop_to_ros_info = file_operations.loadYAMLFile(\
#    '/home/catherine/LTLROS_ws/src/controller_executor/examples/firefighting/firefighting.yaml')
#example_name = "firefighting"

# ---- simple ---- #
dot_file =  pydot.graph_from_dot_file('/home/catherine/Dropbox/ASL/ASL_Summer_2016/exclusions/simple_all.dot')
input_prop_to_ros_info, output_prop_to_ros_info = file_operations.loadYAMLFile(\
    '/home/catherine/LTLROS_ws/src/controller_executor/examples/simple/simple.yaml')
example_name = "simple"

##################
##### NODES ######
##################
#first grab all the nodes (both n__ and t__)
nodes_dict = {}
for subgraph_name, subgraph in dot_file.obj_dict['subgraphs'].iteritems():
    for node_id, node in subgraph[0]['nodes'].iteritems():
        #if len(node) != 1:
        #    check_resources_logger.warning("Length of nodes is longer than 1!")
        nodes_dict[node_id] = node # to get info should do node[0]
        #else:
        #    nodes_dict[node_id] = node[0]

# also join in obj_dict['nodes']
nodes_dict.update(dot_file.obj_dict["nodes"])
check_resources_logger.log(2, "Nodes dict:\n {0}".format(str(nodes_dict)))

##################
##### Edges ######
##################
# get edges list
edges_dict = dot_file.obj_dict['edges']
check_resources_logger.log(2, "Edges dict:\n {0}".format(str(edges_dict)))

# separate into subscribe topics and publish topics
input_subscribed_topics = {key: [] for key in input_prop_to_ros_info.keys()}
input_published_topics = {key: [] for key in input_prop_to_ros_info.keys()}
output_subscribed_topics = {key: [] for key in output_prop_to_ros_info.keys()}
output_published_topics = {key: [] for key in output_prop_to_ros_info.keys()}

# for storing dotnames
input_subscribed_dotnames = {key: [] for key in input_prop_to_ros_info.keys()}
input_published_dotnames = {key: [] for key in input_prop_to_ros_info.keys()}
output_subscribed_dotnames = {key: [] for key in output_prop_to_ros_info.keys()}
output_published_dotnames = {key: [] for key in output_prop_to_ros_info.keys()}


def get_subscribed_topics(prop_dot_format, prop_dotname_list, prop_list, edges_dict, nodes_dict, topic_filtered_list=[]):
    # recursive subscribed topics
    for src_dest_pair in edges_dict.keys():

        # check if we are the destination while
        # not going back in the case for nodelet and actions
        # and also not keep going with topics in the filtered list
        if src_dest_pair[1] == prop_dot_format and \
        not src_dest_pair[0] in prop_dotname_list and \
        not ast.literal_eval(nodes_dict[src_dest_pair[0]][0]['attributes']['label']) in topic_filtered_list:
        #not ast.literal_eval(nodes_dict[src_dest_pair[0]][0]['attributes']['label']) in prop_list:

            # retrieve our source
            check_resources_logger.log(2, "Source {0} in node_dict: {1}".format(src_dest_pair[0], str(src_dest_pair[0] in nodes_dict.keys())))

            # append dotname no matter what
            prop_dotname_list.append(src_dest_pair[0])

            # check if it is a node or topic. Only add in if it's a topic
            if src_dest_pair[0].startswith('t__'):
                prop_list.append(ast.literal_eval(nodes_dict[src_dest_pair[0]][0]['attributes']['label']))

            # then iterate until we reach the end
            get_subscribed_topics(src_dest_pair[0], prop_dotname_list, prop_list, edges_dict, nodes_dict, topic_filtered_list)


def get_published_topics(prop_dot_format, prop_dotname_list, prop_list, edges_dict, nodes_dict, topic_filtered_list=[]):
    # recursive published topics
    for src_dest_pair in edges_dict.keys():

        # check if we are the source while
        # not going back in the case for nodelet and actions
        # and also not keep going with topics in the filtered list
        if src_dest_pair[0] == prop_dot_format and \
        not src_dest_pair[1] in prop_dotname_list and \
        not ast.literal_eval(nodes_dict[src_dest_pair[1]][0]['attributes']['label']) in topic_filtered_list:
        #not ast.literal_eval(nodes_dict[src_dest_pair[1][0]['attributes']['label']) in prop_list:

            #check_resources_logger.warning("{0}:{1}".format(ast.literal_eval(nodes_dict[src_dest_pair[1]][0]['attributes']['label']), \
            #ast.literal_eval(nodes_dict[src_dest_pair[1]][0]['attributes']['label']) in topic_filtered_list))

            # retrieve our destination
            check_resources_logger.log(2, "Dest {0} in node_dict: {1}".format(src_dest_pair[1], str(src_dest_pair[1] in nodes_dict.keys())))

            # append dotname no matter what
            prop_dotname_list.append(src_dest_pair[1])

            # check if it is a node or topic. Only add in if it's a topic
            if src_dest_pair[1].startswith('t__'):
                prop_list.append(ast.literal_eval(nodes_dict[src_dest_pair[1]][0]['attributes']['label']))

            # then iterate until we reach the end
            get_published_topics(src_dest_pair[1], prop_dotname_list, prop_list, edges_dict, nodes_dict, topic_filtered_list)


####################################
# filter some of the common topics #
####################################
topic_filtered_list = ['/clock','/statistics', '/rosout']

###############
### inputs ####
###############
for input_prop in input_prop_to_ros_info.keys():

    # rename prop to the dot file format
    input_prop_dot_format = "n__"+example_name+'_inputs_'+input_prop.replace("/","_")
    check_resources_logger.debug("input_prop: {0} to {1}".format(input_prop, input_prop_dot_format))

    # recursive subscribed topics
    get_subscribed_topics(input_prop_dot_format, input_subscribed_dotnames[input_prop], \
                          input_subscribed_topics[input_prop], edges_dict, nodes_dict, topic_filtered_list)

    # recursive published topics
    get_published_topics(input_prop_dot_format, input_published_dotnames[input_prop], \
                         input_published_topics[input_prop], edges_dict, nodes_dict, topic_filtered_list)


check_resources_logger.debug("input_subscribed_topics: {0}".format(str(input_subscribed_topics)))
check_resources_logger.debug("input_published_topics: {0}".format(str(input_published_topics)))

#check_resources_logger.info("Published topics - bedroom_rc: {0}".format(input_published_topics['bedroom_rc']))

###############
### outputs ###
###############
for output_prop in output_prop_to_ros_info.keys():

    # rename prop to the dot file format
    output_prop_dot_format = "n__"+example_name+'_outputs_'+output_prop.replace("/","_")
    check_resources_logger.debug("input_prop: {0} to {1}".format(output_prop, output_prop_dot_format))

    # recursive subscribed topics
    get_subscribed_topics(output_prop_dot_format, output_subscribed_dotnames[output_prop], \
                          output_subscribed_topics[output_prop], edges_dict, nodes_dict, topic_filtered_list)

    # recursive published topics
    get_published_topics(output_prop_dot_format, output_published_dotnames[output_prop], \
                         output_published_topics[output_prop], edges_dict, nodes_dict, topic_filtered_list)

#check_resources_logger.info("Published topics - bedroom: {0}".format(output_published_topics['bedroom']))

####################################
# filter some of the common topics #
####################################
#topic_filtered_list = ['/clock','/statistics', '/rosout']

# inputs
#for input_prop in input_prop_to_ros_info.keys():
#    input_subscribed_topics[input_prop] = [x for x in input_subscribed_topics[input_prop] if not x in topic_filtered_list]
#    input_published_topics[input_prop] = [x for x in input_published_topics[input_prop] if not x in topic_filtered_list]

# outputs
#for output_prop in output_prop_to_ros_info.keys():
#    output_subscribed_topics[output_prop] = [x for x in output_subscribed_topics[output_prop] if not x in topic_filtered_list]
#    output_published_topics[output_prop] = [x for x in output_published_topics[output_prop] if not x in topic_filtered_list]


# now call for comparison
check_resource_usage.check_possible_concurrent_topic_access_dot_file(output_published_topics)

check_resource_usage.check_possible_action_affected_sensors_dot_file(input_subscribed_topics, output_published_topics)

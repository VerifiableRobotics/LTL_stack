import pydot
import logging
import ast
import Queue
import itertools
import getpass
import copy

#import rqt_graph.ros_graph

# increase recursion limit
# todo - maybe do a tail call instead see:
# https://github.com/lihaoyi/macropy#tail-call-optimization
import resource, sys
resource.setrlimit(resource.RLIMIT_STACK, (2**29,-1))
sys.setrecursionlimit(10**6)

import rqt_grounding_and_analysis_logging
check_resources_logger = logging.getLogger("check_resources_logger")
from controller_executor import file_operations


def get_subscribed_topics(prop_dot_format, visited_prop_dotname_list, prop_dotname_list, prop_list, edges_dict, nodes_dict, \
                          chain_topic_node_list, chain_topic_node_dict, prop, topic_filtered_list=[], partial_topic_filtered_list=[],\
                          chain_prop_dot_to_topic_dot_subscribed_dict={}):
    # recursive subscribed topics
    for src_dest_pair in edges_dict.keys():

        # check if we are the destination while
        # not going back in the case for nodelet and actions
        # and also not keep going with topics in the filtered list
        if src_dest_pair[1] == prop_dot_format and \
        not src_dest_pair[0] in visited_prop_dotname_list and \
        not ast.literal_eval(nodes_dict[src_dest_pair[0]][0]['attributes']['label']) in topic_filtered_list and \
        not any([x in ast.literal_eval(nodes_dict[src_dest_pair[0]][0]['attributes']['label']) for x in partial_topic_filtered_list]):
        #not ast.literal_eval(nodes_dict[src_dest_pair[0]][0]['attributes']['label']) in prop_list:

            # retrieve our source
            check_resources_logger.log(2, "Source {0} in node_dict: {1}".format(src_dest_pair[0], str(src_dest_pair[0] in nodes_dict.keys())))

            # append dotname no matter what
            if src_dest_pair[0] not in prop_dotname_list:
                prop_dotname_list.append(src_dest_pair[0])
            visited_prop_dotname_list.append(src_dest_pair[0])

            # append to the chain_topic_node_list
            chain_topic_node_list_new = chain_topic_node_list+ [src_dest_pair[0]]

            # save the shorter version
            if not src_dest_pair[0] in chain_topic_node_dict[prop].keys() or \
                    len(chain_topic_node_list_new) < len(chain_topic_node_dict[prop][src_dest_pair[0]]):
                chain_topic_node_dict[prop][src_dest_pair[0]] = chain_topic_node_list_new

            # save node name to topic names
            if prop_dot_format in chain_prop_dot_to_topic_dot_subscribed_dict.keys() and \
            (not src_dest_pair[0] in chain_prop_dot_to_topic_dot_subscribed_dict[prop_dot_format].keys() or \
                    len(chain_topic_node_list_new) < len(chain_prop_dot_to_topic_dot_subscribed_dict[prop_dot_format][src_dest_pair[0]])):
                chain_prop_dot_to_topic_dot_subscribed_dict[prop_dot_format][src_dest_pair[0]] = chain_topic_node_list_new

            # check if it is a node or topic. Only add in if it's a topic
            if src_dest_pair[0].startswith('t__') or "action_topics" in src_dest_pair[0] and\
            ast.literal_eval(nodes_dict[src_dest_pair[0]][0]['attributes']['label']) not in prop_list:
                prop_list.append(ast.literal_eval(nodes_dict[src_dest_pair[0]][0]['attributes']['label']))

            # then iterate until we reach the end
            get_subscribed_topics(src_dest_pair[0], copy.deepcopy(visited_prop_dotname_list), prop_dotname_list, prop_list, edges_dict, nodes_dict, \
                                  copy.deepcopy(chain_topic_node_list_new), chain_topic_node_dict, prop, topic_filtered_list, partial_topic_filtered_list)


def get_published_topics(prop_dot_format, visited_prop_dotname_list, prop_dotname_list, prop_list, edges_dict, nodes_dict, \
                         chain_topic_node_list, chain_topic_node_dict, prop, prop_node, topic_filtered_list=[], partial_topic_filtered_list=[],\
                         chain_prop_dot_to_topic_dot_published_dict={}):
    # recursive published topics
    for src_dest_pair in edges_dict.keys():
        #if "action_topics" in src_dest_pair[0] and "move_base" in src_dest_pair[1]:
        #    check_resources_logger.warning("Transition from action topics to move base!")

        # check if we are the source while
        # not going back in the case for nodelet and actions
        # and also not keep going with topics in the filtered list
        # and not going if we find partial of these topics
        if src_dest_pair[0] == prop_dot_format and \
        not src_dest_pair[1] in visited_prop_dotname_list and \
        not ast.literal_eval(nodes_dict[src_dest_pair[1]][0]['attributes']['label']) in topic_filtered_list and\
        not any([x in ast.literal_eval(nodes_dict[src_dest_pair[1]][0]['attributes']['label']) for x in partial_topic_filtered_list]):
        #not ast.literal_eval(nodes_dict[src_dest_pair[1][0]['attributes']['label']) in prop_list:

            #check_resources_logger.warning("{0}:{1}".format(ast.literal_eval(nodes_dict[src_dest_pair[1]][0]['attributes']['label']), \
            #ast.literal_eval(nodes_dict[src_dest_pair[1]][0]['attributes']['label']) in topic_filtered_list))

            # retrieve our destination
            check_resources_logger.log(2, "Dest {0} in node_dict: {1}, in visited_prop_dotname_list: {2}, in chain_topic_node_list: {3}".format(\
                src_dest_pair[1], str(src_dest_pair[1] in nodes_dict.keys()), \
                str(src_dest_pair[1] in visited_prop_dotname_list), \
                str(src_dest_pair[1] in chain_topic_node_list)))

            # append dotname no matter what
            if src_dest_pair[1] not in prop_dotname_list:
                prop_dotname_list.append(src_dest_pair[1])
                # check if it is a node or topic. Only add in if it's a topic
                if src_dest_pair[1].startswith('t__') or "action_topics" in src_dest_pair[1]:
                    prop_list.append(ast.literal_eval(nodes_dict[src_dest_pair[1]][0]['attributes']['label']))
            visited_prop_dotname_list.append(src_dest_pair[1])

            # append to the chain_topic_node_list
            chain_topic_node_list_new = chain_topic_node_list+ [src_dest_pair[1]]

            #if 'action_topics' in src_dest_pair[1] or any("action_topics" in s for s in chain_topic_node_list_new):
            #    check_resources_logger.log(4, "prop_dot_format: {0}, chain list: {1}".format(prop_dot_format, chain_topic_node_list_new))

            # save the shorter version
            if not src_dest_pair[1] in chain_topic_node_dict[prop].keys() or \
                len(chain_topic_node_list_new) < len(chain_topic_node_dict[prop][src_dest_pair[1]]):
                chain_topic_node_dict[prop][src_dest_pair[1]] = chain_topic_node_list_new

            # keep track of chain with nodes too
            if not src_dest_pair[1] in chain_topic_node_dict[prop_node].keys() or \
                len(chain_topic_node_list_new) < len(chain_topic_node_dict[prop_node][src_dest_pair[1]]):
                chain_topic_node_dict[prop_node][src_dest_pair[1]] = chain_topic_node_list_new

            # save node name to topic names
            if prop_dot_format in chain_prop_dot_to_topic_dot_published_dict.keys() and \
            (not src_dest_pair[1] in chain_prop_dot_to_topic_dot_published_dict[prop_dot_format].keys() or \
                len(chain_topic_node_list_new) < len(chain_prop_dot_to_topic_dot_published_dict[prop_dot_format][src_dest_pair[1]])):
                chain_prop_dot_to_topic_dot_published_dict[prop_dot_format][src_dest_pair[1]] = chain_topic_node_list_new

            # then iterate until we reach the end
            get_published_topics(src_dest_pair[1], copy.deepcopy(visited_prop_dotname_list), prop_dotname_list, prop_list, edges_dict, nodes_dict, \
                                    copy.deepcopy(chain_topic_node_list_new), chain_topic_node_dict, prop, prop_node, topic_filtered_list, partial_topic_filtered_list)

##################################################################################
# This search is modified from:
#http://stackoverflow.com/questions/713508/find-the-paths-between-two-given-nodes
#################################################################################
def BFS(graph,start,end,q):
    check_resources_logger.log(4, "start:{0}, end:{1}".format(start, end))

    temp_path = [start]

    q.put(temp_path)

    while q.empty() == False:
        tmp_path = q.get()
        last_node = tmp_path[len(tmp_path)-1]
        #print tmp_path
        if last_node == end:
            check_resources_logger.log(8, "VALID_PATH : {0}".format(tmp_path))
            return tmp_path
        if last_node in graph.keys(): # check that it is a key first
            for link_node in graph[last_node]:
                if link_node not in tmp_path:
                    #new_path = []
                    new_path = tmp_path + [link_node]
                    q.put(new_path)

    return []

def chain_output(graph, start, end, prop_dot_to_real_name):
    # get chain_list
    chain_list = BFS(graph, start, end, Queue.Queue())
    check_resources_logger.log(4, "chain_list:{0}".format(chain_list))

    # convert to a string (Node to circle, topic to square)
    chain_str_list = []
    for x in chain_list:
        chain_str_list.append(prop_dot_to_real_name[x])
        #if x.startswith('n__'):
        #    chain_str_list.append('(/'+x.replace('n__','').replace('/','_')+')')
        #else:
        #    chain_str_list.append('[/'+x.replace('t__','').replace('/','_')+']')

    return " -> ".join(chain_str_list)

def get_prop_dot_name_real_name_mapping(nodes_dict):
    prop_dot_to_real_name = {}
    prop_real_to_dot_name = {'t':{}, 'n':{}}

    check_resources_logger.log(4, "nodes_dict: {0}".format(nodes_dict))

    for dot_name, dot_name_properties in nodes_dict.iteritems():
        if dot_name_properties[0]['name'] in ['graph']:
            continue

        check_resources_logger.log(4, "prop name: {0}, dot_name_properties[0]['attributes']: {1}".format(dot_name, dot_name_properties[0]['attributes']))
        #check_resources_logger.log(2,dot_name_properties)
        prop_dot_to_real_name[dot_name] = ast.literal_eval(dot_name_properties[0]['attributes']['label'])

        # separate into two sub-dicts
        #if dot_name.startswith('t'):
        prop_real_to_dot_name[dot_name[0]][ast.literal_eval(dot_name_properties[0]['attributes']['label'])] = dot_name
        #elif dot_name.startswith('n'):
        #    prop_real_to_dot_name['n'][ast.literal_eval(dot_name_properties[0]['attributes']['label'])] = dot_name
        #else:
        #    check_resources_logger.error('dot_name does not start with "t" or "n": {0}'.format(dot_name))

    return prop_dot_to_real_name, prop_real_to_dot_name

def get_published_graph(prop_dot_format, prop_dotname_list, graph, edges_dict, nodes_dict, topic_filtered_list=[]):
    # recursive published topics
    for src_dest_pair in edges_dict.keys():

        # check if we are the source while
        # not going back in the case for nodelet and actions
        # and also not keep going with topics in the filtered list
        if src_dest_pair[0] == prop_dot_format and \
        not src_dest_pair[1] in prop_dotname_list and \
        not ast.literal_eval(nodes_dict[src_dest_pair[1]][0]['attributes']['label']) in topic_filtered_list:

            # retrieve our destination
            check_resources_logger.log(2, "Dest {0} in node_dict: {1}".format(src_dest_pair[1], str(src_dest_pair[1] in nodes_dict.keys())))

            # append dotname no matter what
            prop_dotname_list.append(src_dest_pair[1])

            # create chain
            if not prop_dot_format in graph.keys():
                graph[prop_dot_format] = []
            graph[prop_dot_format].append(src_dest_pair[1])

            get_published_graph(src_dest_pair[1], prop_dotname_list, \
                graph, edges_dict, nodes_dict, topic_filtered_list)


def check_possible_concurrent_topic_access(published_topics):
    """
    This function checks if any of the propositions are publishing to the same topics
    @param published_topics: proposition to publishing information
    @type  published_topics: dict
    @return: possible_concurrent_topic_access in the form of topic: list of prositions
    @rtype: dict
    """

    # output publish - output publish
    # input subscribe - output publish same component on off? # (see kinect sensor for example)

    possible_concurrent_topic_access = {}

    # output publish - output publish (pair wise comparison each time)
    for seed_prop, compare_prop in itertools.combinations(published_topics.items(), 2):
        check_resources_logger.log(4, 'seed_prop: {0}'.format((seed_prop[0], seed_prop[1])))
        check_resources_logger.log(4, 'compare_prop: {0}'.format((compare_prop[0], compare_prop[1])))

        # find command elements excluding /rosout
        common_topics = list(set(seed_prop[1]) & set(compare_prop[1]) - set(['/rosout']))
        if common_topics:
            check_resources_logger.log(6, 'common publishing topic(s): {0}'.format(common_topics))

            for topic in common_topics:
                if not topic in possible_concurrent_topic_access.keys():
                    possible_concurrent_topic_access[topic] = []

                # only add new element if it's not already there. (from combinations)
                possible_concurrent_topic_access[topic].append([seed_prop[0], compare_prop[0]])

    check_resources_logger.info('possible_concurrent_topic_access: {0}'.format(possible_concurrent_topic_access))
    return possible_concurrent_topic_access

def check_possible_action_affected_sensors(input_prop_to_ros_info, output_prop_list, prop_real_to_dot_name, output_published_dotnames):
    """
    Given a sensor prop, check if it if subscribing to topics spawned by an action
    @param subscribed_topics: proposition to subscribing information
    @type  subscribed_topics: dict
    @param output_list: list of output propositions
    @type  output_list: list
    @return: possible_action_affected_sensors in the form of output prop: list of input prop
    @rtype: dict
    ['include_props']
    """

    possible_action_affected_sensors = {}

    for input_prop in input_prop_to_ros_info.keys():
        node_dotname = prop_real_to_dot_name['n'][input_prop_to_ros_info[input_prop]['node']]

        for output_prop in output_prop_list:
            check_resources_logger.log(2, 'output_published_dotnames[output_prop]: {0}, output_prop: {1}, node_dotname: {2}'.format(\
                        output_published_dotnames[output_prop], output_prop, node_dotname))

            if node_dotname in output_published_dotnames[output_prop]: # check if input comes from output prop
                check_resources_logger.debug('output prop:{0} to input prop {1}'.format(output_prop, input_prop))

                if not output_prop in possible_action_affected_sensors.keys():
                    possible_action_affected_sensors[output_prop] = []

                # now append which action affects what sensors
                if not input_prop in possible_action_affected_sensors[output_prop]:
                    possible_action_affected_sensors[output_prop].append(input_prop)

    check_resources_logger.info('possible_action_affected_sensors: {0}'.format(possible_action_affected_sensors))
    return possible_action_affected_sensors

def check_left_behind_robot_per_prop(output_published_topics, robot_list, topic_filtered_list = []):
    """
    Given a dict of output_published_topics and a robot list
    Return props + robots that are excluded
    @param output_published_topics: proposition to publishing information
    @type  output_published_topics: dict
    @param robot_list: list of robots
    @type  robot_list: list
    @param topic_filtered_list: list of topics to filter
    @type  robot_list: list
    @return: props_to_left_behind_robots: props with robots that are left behind
    @rtype: dict
    @return: topic_to_left_behind_robots: topics of robots that are not mapped to any propositions, but other robots' are
    @rtype: dict
    """

    props_to_left_behind_robots = {}
    props_to_robots = {}

    robot_topic_usage = {}
    topic_to_left_behind_robots = {}

    # first format data to relate prop with robots
    for output_prop, published_topics_list in output_published_topics.iteritems():
        #check_resources_logger.debug('output_prop: {0}, published_topics_list: {1}'.format(output_prop, published_topics_list))

        props_to_robots[output_prop] = {}
        # find suffix that are the same
        for seed_topic, compare_topic in itertools.combinations(published_topics_list, 2):
            seed_topic_split_list = seed_topic.strip('/').split('/')
            compare_topic_split_list = compare_topic.strip('/').split('/')

            # same suffix
            # suffixes not in filtered list
            # prefixes contain different robot names
            if seed_topic_split_list[-1] == compare_topic_split_list[-1] and \
            "/"+seed_topic_split_list[-1] not in topic_filtered_list and \
            "/"+compare_topic_split_list[-1] not in topic_filtered_list and \
            len((set(seed_topic_split_list) & set(robot_list)) | (set(compare_topic_split_list) & set(robot_list))) > 1:
                root_topic = seed_topic_split_list[-1]

                # track robots per topic per prop
                if not root_topic in props_to_robots[output_prop].keys(): # if it's a new topic
                    props_to_robots[output_prop][root_topic] = \
                                        {'robots': ((set(seed_topic_split_list) & set(robot_list)) | (set(compare_topic_split_list) & set(robot_list))), \
                                         'topics': set([seed_topic,compare_topic])}
                else:
                    props_to_robots[output_prop][root_topic]['robots'] |= \
                                        set(seed_topic_split_list) & set(robot_list) | set(compare_topic_split_list) & set(robot_list)
                    props_to_robots[output_prop][root_topic]['topics'] |=  set([seed_topic,compare_topic])

                # track topic usage overall
                if not root_topic in robot_topic_usage.keys():
                    robot_topic_usage[root_topic] = \
                                        {'robots': ((set(seed_topic_split_list) & set(robot_list)) | (set(compare_topic_split_list) & set(robot_list))), \
                                         'topics': set([seed_topic,compare_topic])}
                else:
                    robot_topic_usage[root_topic]['robots'] |= \
                                        set(seed_topic_split_list) & set(robot_list) | set(compare_topic_split_list) & set(robot_list)
                    robot_topic_usage[root_topic]['topics'] |=  set([seed_topic,compare_topic])

    # now check if each topic covers all robots
    for output_prop, topic_to_robot_dict in props_to_robots.iteritems():
        for topic, robot_info in topic_to_robot_dict.iteritems():
            if (set(robot_list) - robot_info['robots']) != set():
                props_to_left_behind_robots[output_prop] = {topic: robot_info}
                props_to_left_behind_robots[output_prop][topic].update({'missing': set(robot_list)-robot_info['robots']})

    # now check if a topic is used for one robot, it's also used for all robots
    for topic, topic_robot_dict in robot_topic_usage.iteritems():
        if (set(robot_list) - topic_robot_dict['robots']) != set():
            topic_to_left_behind_robots[topic] = topic_robot_dict
            topic_to_left_behind_robots[topic].update({'missing': set(robot_list) - topic_robot_dict['robots']})


    #import yaml
    #with open('props_to_robots.yml', 'w') as outfile:
    #    yaml.dump(props_to_robots, outfile, default_flow_style=False)
    #with open('props_to_left_behind_robots.yml', 'w') as outfile:
    #    yaml.dump(props_to_left_behind_robots, outfile, default_flow_style=False)
    check_resources_logger.info('props_to_left_behind_robots: {0}'.format(props_to_left_behind_robots))
    check_resources_logger.info('topic_to_left_behind_robots: {0}'.format(topic_to_left_behind_robots))

    return props_to_left_behind_robots, topic_to_left_behind_robots

def check_prop_to_same_topic_from_differnt_nodes(output_published_topics, topic_filtered_list=[]):

    prop_to_same_topic_from_differnt_nodes = {}
    check_resources_logger.log(8,"SAME_TOPIC_DIFFERENT_NODES- output_published_topics:{0}".format(\
        output_published_topics))

    for output_prop, published_topics_list in output_published_topics.iteritems():
        duplicate_list = set([x for x in published_topics_list if published_topics_list.count(x) > 1])

        check_resources_logger.log(8,'SAME_TOPIC_DIFFERENT_NODES- output {0}: duplicate_list: {1}'.format(\
            output_prop, duplicate_list))

        # now filter topics
        if len(duplicate_list - set(topic_filtered_list)):
            # now remove partial topics:
            temp_list = duplicate_list - set(topic_filtered_list)
            toDelete = []
            for topic in temp_list:
                if '/'+topic.split('/')[-1] in topic_filtered_list:
                    toDelete.append(topic)

            # check again and see if there're any elements
            if len(temp_list - set(toDelete)):
                prop_to_same_topic_from_differnt_nodes[output_prop] = duplicate_list - set(topic_filtered_list) - set(toDelete)

    check_resources_logger.info('prop_to_same_topic_from_differnt_nodes: {0}'.format(prop_to_same_topic_from_differnt_nodes))
    return prop_to_same_topic_from_differnt_nodes

def check_possible_action_affected_sensors_old(subscribed_topics, output_published_topics):
    """
    Given a sensor prop, check if it if subscribing to topics spawned by an action
    @param subscribed_topics: proposition to subscribing information
    @type  subscribed_topics: dict
    @param output_list: list of output propositions
    @type  output_list: list
    @return: possible_action_affected_sensors in the form of output prop: list of input prop
    @rtype: dict
    """

    possible_action_affected_sensors = {}

    for prop, prop_info_list in subscribed_topics.iteritems():
        for topic_info in prop_info_list: # enumerate all topic subscribing to
            #check_resources_logger.warning('topic_info: {0}'.format(topic_info))

            for output_prop, published_topics_list in output_published_topics.iteritems():
                #check_resources_logger.debug('output_prop: {0}'.format(output_prop))

                if topic_info in published_topics_list: # check if topic_info comes from output prop
                    check_resources_logger.debug('Input prop:{0} contains topic in published topics list: {1} by output prop:{2}'.format(prop, topic_info, output_prop))

                    if not output_prop in possible_action_affected_sensors.keys():
                        possible_action_affected_sensors[output_prop] = []

                    # now append which action affects what sensors
                    if not prop in possible_action_affected_sensors[output_prop]:
                        possible_action_affected_sensors[output_prop].append(prop)

    check_resources_logger.info('possible_action_affected_sensors: {0}'.format(possible_action_affected_sensors))
    return possible_action_affected_sensors


if __name__ == "__main__":
    # --- firefighting ---- #
    #load dot file
    #dot_file =  pydot.graph_from_dot_file('/home/{0}/Dropbox/ASL/ASL_Summer_2016/exclusions/firefighting/firefighting_stay_in_place_all.dot'.format(getpass.getuser()))

    #load inputs and outputs
    #input_prop_to_ros_info, output_prop_to_ros_info = file_operations.loadYAMLFile(\
    #    '/home/{0}/LTLROS_ws/src/controller_executor/examples/firefighting/firefighting.yaml'.format(getpass.getuser()))
    #example_name = "firefighting"

    # ---- simple ---- #
    #dot_file =  pydot.graph_from_dot_file('/home/{0}/Dropbox/ASL/ASL_Summer_2016/exclusions/simple/simple_all.dot'.format(getpass.getuser()))
    #input_prop_to_ros_info, output_prop_to_ros_info = file_operations.loadYAMLFile(\
    #    '/home/{0}/LTLROS_ws/src/controller_executor/examples/simple/simple.yaml'.format(getpass.getuser()))
    #example_name = "simple"

    # --- move_group_and_move_base ---- #
    #load dot file
    dot_file =  pydot.graph_from_dot_file('/home/{0}/Dropbox/ASL/ASL_Summer_2016/exclusions/move_group_and_move_base/youbot_lab.dot'.format(getpass.getuser()))

    #load inputs and outputs
    #input_prop_to_ros_info, output_prop_to_ros_info = file_operations.loadYAMLFile(\
    #    '/home/{0}/LTLROS_ws/src/controller_executor/examples/move_group_and_move_base/move_group_and_move_base.yaml'.format(getpass.getuser()))
    input_prop_to_ros_info, output_prop_to_ros_info = file_operations.loadYAMLFile(\
        '/home/{0}/LTLROS_ws/src/controller_executor/examples/move_group_and_move_base/move_group_and_move_base_same_topic_problem_small.yaml'.format(getpass.getuser()))
    example_name = "move_group_and_move_base"

    # --- spheros ---- #
    #load dot file
    #dot_file =  pydot.graph_from_dot_file('/home/{0}/Dropbox/ASL/ASL_Summer_2016/exclusions/spheros/spheros_two.dot'.format(getpass.getuser()))

    #load inputs and outputs
    #input_prop_to_ros_info, output_prop_to_ros_info = file_operations.loadYAMLFile(\
    #    '/home/{0}/LTLROS_ws/src/controller_executor/examples/spheros/spheros.yaml'.format(getpass.getuser()))
    #example_name = "spheros"

    ##################  #################
    ##### NODES ######  ###### EDGES ####
    ##################  #################
    #first grab all the nodes (both n__ and t__)
    nodes_dict, edges_dict = {}, {}
    for subgraph_name, subgraph in dot_file.obj_dict['subgraphs'].iteritems():
        check_resources_logger.debug("subgraph_name: {0}, length of subgraph: {1}".format(subgraph_name, len(subgraph)))
        check_resources_logger.log(8, "subgraph[0] keys: {0}".format(subgraph[0].keys()))
        check_resources_logger.log(8, subgraph[0]['edges'].keys())
        # add in nodes
        for node_id, node in subgraph[0]['nodes'].iteritems():
            #if len(node) != 1:
            #    check_resources_logger.warning("Length of nodes is longer than 1!")
            nodes_dict[node_id] = node # to get info should do node[0]
            #else:
            #    nodes_dict[node_id] = node[0]

        # add in edges:
        for edge_id, edge in subgraph[0]['edges'].iteritems():
            #if "move_base" in edge_id and "action_topics" in edge_id:
                #check_resources_logger.log(8, edge_id)
            edges_dict[edge_id] = edge
            check_resources_logger.log(6, "edge_id: {0}, edge:{1}".format(edge_id, edge))

    #############################
    #### MORE NODES AND EDGES ###
    #############################
    # also join in obj_dict['nodes']
    nodes_dict.update(dot_file.obj_dict["nodes"])
    check_resources_logger.log(2, "Nodes dict:\n {0}".format(str(nodes_dict)))

    #  also join in obj_dict['edges']
    edges_dict.update(dot_file.obj_dict['edges'])
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

    chain_topic_node_dotnames_subscribed_dict = {key: {} for key in output_prop_to_ros_info.keys()+\
                                                                  input_prop_to_ros_info.keys()}
    chain_topic_node_dotnames_published_dict = {key: {} for key in output_prop_to_ros_info.keys()+\
                                                                  input_prop_to_ros_info.keys()}

    prop_dot_to_real_name, prop_real_to_dot_name = get_prop_dot_name_real_name_mapping(nodes_dict)

    ####################################
    # filter some of the common topics #
    ####################################
    topic_filtered_list = ['/clock','/statistics', '/rosout', '/rviz', '/map', '/tf', '/tf_static', '/diagnostics'] + \
                        [prop_info['node'] for prop_info_list in output_prop_to_ros_info.values() for prop_info in prop_info_list] + \
                        [prop_info['node'] for prop_info in input_prop_to_ros_info.values()]
    partial_topic_filtered_list = ['/rviz']

    ###############
    ### inputs ####
    ###############
    for input_prop in input_prop_to_ros_info.keys():

        # rename prop to the dot file format
        #input_prop_dot_format = "n__"+example_name+'_inputs_'+input_prop.replace("/","_")
        input_prop_dot_format = "n__"+"_".join([x for x in input_prop_to_ros_info[input_prop]['node'].split("/") if x])

        check_resources_logger.debug("input_prop: {0} to {1}".format(input_prop, input_prop_dot_format))

        if input_prop_dot_format not in chain_topic_node_dotnames_published_dict.keys():
            chain_topic_node_dotnames_published_dict.update({input_prop_dot_format:{}})

        # recursive subscribed topics
        get_subscribed_topics(input_prop_dot_format, [], input_subscribed_dotnames[input_prop], \
                              input_subscribed_topics[input_prop], edges_dict, nodes_dict, \
                              [input_prop_dot_format], chain_topic_node_dotnames_subscribed_dict, input_prop, \
                              topic_filtered_list, partial_topic_filtered_list)

        # recursive published topics
        get_published_topics(input_prop_dot_format, [], input_published_dotnames[input_prop], \
                             input_published_topics[input_prop], edges_dict, nodes_dict, \
                             [input_prop_dot_format], chain_topic_node_dotnames_published_dict, input_prop, \
                             input_prop_dot_format, topic_filtered_list, partial_topic_filtered_list)


    check_resources_logger.debug("input_subscribed_topics: {0}".format(str(input_subscribed_topics)))
    check_resources_logger.debug("input_published_topics: {0}".format(str(input_published_topics)))

    #check_resources_logger.info("Published topics - bedroom_rc: {0}".format(input_published_topics['bedroom_rc']))

    ###############
    ### outputs ###
    ###############
    chain_prop_dot_to_topic_dot_subscribed_dict = {}
    chain_prop_dot_to_topic_dot_published_dict = {}

    for output_prop in output_prop_to_ros_info.keys():

        # first check if it's a list
        output_prop_dot_format_list = []
        if isinstance(output_prop_to_ros_info[output_prop], list):
            for prop_info in output_prop_to_ros_info[output_prop]:
                # rename prop to the dot file format
                #output_prop_dot_format = "n__"+example_name+'_outputs_'+output_prop.replace("/","_")
                output_prop_dot_format_list.append("n__"+"_".join([x for x in prop_info['node'].split("/") if x]))
        else:
            # rename prop to the dot file format
            #output_prop_dot_format = "n__"+example_name+'_outputs_'+output_prop.replace("/","_")
            output_prop_dot_format_list.append("n__"+"_".join([x for x in output_prop_to_ros_info[output_prop]['node'].split("/") if x]))

        chain_prop_dot_to_topic_dot_published_dict.update(dict(((output_prop_dot_format, {}) for output_prop_dot_format in output_prop_dot_format_list)))
        chain_prop_dot_to_topic_dot_subscribed_dict.update(dict(((output_prop_dot_format, {}) for output_prop_dot_format in output_prop_dot_format_list)))

        check_resources_logger.debug("output_prop: {0} to {1}".format(output_prop, output_prop_dot_format_list))

        for output_prop_dot_format in output_prop_dot_format_list:
            # expand dict with prop_dot_format as well
            if output_prop_dot_format not in chain_topic_node_dotnames_published_dict.keys():
                chain_topic_node_dotnames_published_dict.update({output_prop_dot_format:{}})

            check_resources_logger.debug(chain_topic_node_dotnames_published_dict)
            temp_output_subscribed_dotnames = []
            temp_output_published_dotnames = []
            temp_output_subscribed_topics = []
            temp_output_published_topics = []

            # recursive subscribed topics
            get_subscribed_topics(output_prop_dot_format, [], temp_output_subscribed_dotnames, \
                                  temp_output_subscribed_topics, edges_dict, nodes_dict, \
                                  [output_prop_dot_format], chain_topic_node_dotnames_subscribed_dict, \
                                  output_prop, topic_filtered_list, partial_topic_filtered_list, chain_prop_dot_to_topic_dot_subscribed_dict)

            # recursive published topics
            get_published_topics(output_prop_dot_format, [], temp_output_published_dotnames, \
                                 temp_output_published_topics, edges_dict, nodes_dict, \
                                 [output_prop_dot_format], chain_topic_node_dotnames_published_dict, \
                                 output_prop, output_prop_dot_format, topic_filtered_list, partial_topic_filtered_list, chain_prop_dot_to_topic_dot_published_dict)

            output_subscribed_dotnames[output_prop] +=copy.deepcopy(temp_output_subscribed_dotnames)
            output_published_dotnames[output_prop] +=copy.deepcopy(temp_output_published_dotnames)
            output_published_topics[output_prop] +=copy.deepcopy(temp_output_published_topics)
            output_subscribed_topics[output_prop] +=copy.deepcopy(temp_output_subscribed_topics)

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
    check_possible_concurrent_topic_access(output_published_topics)

    #check_possible_action_affected_sensors(input_subscribed_topics, output_published_topics)
    check_possible_action_affected_sensors(input_prop_to_ros_info, output_prop_to_ros_info.keys(), \
                                            prop_real_to_dot_name, output_published_dotnames)

    # for mutliple robots
    # sphero_wpw, sphero_ggw, sphero_rgw, sphero_wpp, sphero_wrb
    # /collision,/diagnostics,/imu,/odom
    robot_list = ['sphero_wpp', 'sphero_ggw', 'sphero_wrb']
    parameter_topics = ['/parameter_descriptions', '/parameter_updates', '/move_base/cancel',\
    '/costmap_updates', '/global_costmap','/local_costmap', '/collision_object']
    sphero_sensor_topics = ['/collision','/diagnostics','/imu','/odom']
    check_resources_logger.info('--------')
    check_left_behind_robot_per_prop(output_published_topics, robot_list, topic_filtered_list+parameter_topics+sphero_sensor_topics)
    check_resources_logger.info('--------')
    check_prop_to_same_topic_from_differnt_nodes(output_published_topics, topic_filtered_list+parameter_topics+sphero_sensor_topics)
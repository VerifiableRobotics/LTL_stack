#! /usr/bin/env python
import yaml
import logging
import itertools
import argparse

import rosgraph.impl.graph
import rosservice
import rosnode

import controller_executor_logging
check_resources_logger = logging.getLogger("check_resources_logger")
import file_operations

# service queue
#http://answers.ros.org/question/11544/calling-a-ros-service-from-several-nodes-at-the-same-time/

# initialize graph
# graph = rosgraph.impl.graph.Graph()
# graph.set_master_stale(5.0)
# graph.set_node_stale(5.0)
# graph.update()

"""
if graph.srvs:
   print('Services:')
   for s in graph.srvs:
       print('  ' + s)

for node in graph.nn_nodes:
    print "node: {node}".format(node=node)
    service_names = rosservice.get_service_list(node=node)
    #service_type = rosservice.get_service_type(service_name)
    print "service_names: {service_names}".format(service_names=service_names)
    print "------------------------------------------------------------------------"
"""

def get_subscribed_and_published_topics(prop_to_ros_info):
    """
    This function takes in yaml file info about prop and ros node relation.
    And output the publishing and subcribing topics of each node.
    @param prop_to_ros_info: proposition to ros information
    @type  prop_to_ros_info: dict
    @return: two dictionaries about subscribed and published topics of each prop
    @rtype: dict, dict
    """

    #todo:
    # need to make this recursive

    # initialize graph
    graph = rosgraph.impl.graph.Graph()
    graph.set_master_stale(5.0)
    graph.set_node_stale(5.0)
    graph.update()

    subscribed_topics = dict((k,[]) for k in prop_to_ros_info.keys())
    published_topics = dict((k,[]) for k in prop_to_ros_info.keys())

    # check resource
    for prop, prop_info in prop_to_ros_info.iteritems():
        prefix = '/'+prop_info['node']
        check_resources_logger.log(4, prefix)
        check_resources_logger.log(4,'    Inbound:')
        for k in graph.nn_edges.edges_by_end.iterkeys():
            if k.startswith(prefix):
                for c in graph.nn_edges.edges_by_end[k]:
                    #subscribed_topics[prop].append(c.label)
                    subscribed_topics[prop].append(c)
                    check_resources_logger.log(4,'      ' + c.start + ' label = ' + c.label) # label is the topic

        check_resources_logger.log(4,'    Outbound:')
        for k in graph.nn_edges.edges_by_start.iterkeys():
            if k.startswith(prefix):
                for c in graph.nn_edges.edges_by_start[k]:
                    published_topics[prop].append(c)
                    check_resources_logger.log(4,'      ' + c.end + ' label = ' + c.label)

    check_resources_logger.debug('subscribed_topics: {0}'.format(subscribed_topics))
    check_resources_logger.debug('published_topics: {0}'.format(published_topics))

    return subscribed_topics, published_topics


def check_possible_concurrent_topic_access_dot_file(published_topics):
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
        check_resources_logger.debug(seed_prop)
        check_resources_logger.debug('seed_prop: {0}'.format((seed_prop[0], seed_prop[1])))
        check_resources_logger.debug('compare_prop: {0}'.format((compare_prop[0], compare_prop[1])))

        # find command elements excluding /rosout
        common_topics = list(set(seed_prop[1]) & set(compare_prop[1]) - set(['/rosout']))
        if common_topics:
            check_resources_logger.debug('common publishing topic(s): {0}'.format(common_topics))

            for topic in common_topics:
                if not topic in possible_concurrent_topic_access.keys():
                    possible_concurrent_topic_access[topic] = []

                # only add new element if it's not already there. (from combinations)
                possible_concurrent_topic_access[topic].append([seed_prop[0], compare_prop[0]])

    check_resources_logger.info('possible_concurrent_topic_access: {0}'.format(possible_concurrent_topic_access))
    return possible_concurrent_topic_access


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
        check_resources_logger.debug(seed_prop)
        check_resources_logger.debug('seed_prop: {0}'.format((seed_prop[0], [x.label for x in seed_prop[1]])))
        check_resources_logger.debug('compare_prop: {0}'.format((compare_prop[0], [x.label for x in compare_prop[1]])))

        # find command elements excluding /rosout
        common_topics = list(set([x.label for x in seed_prop[1]]) & set([x.label for x in compare_prop[1]]) - set(['/rosout']))
        if common_topics:
            check_resources_logger.debug('common publishing topic(s): {0}'.format(common_topics))

            for topic in common_topics:
                if not topic in possible_concurrent_topic_access.keys():
                    possible_concurrent_topic_access[topic] = []

                # only add new element if it's not already there. (from combinations)
                possible_concurrent_topic_access[topic].append([seed_prop[0], compare_prop[0]])

    check_resources_logger.info('possible_concurrent_topic_access: {0}'.format(possible_concurrent_topic_access))
    return possible_concurrent_topic_access

def check_possible_action_affected_sensors_dot_file(subscribed_topics, output_published_topics):
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



def check_possible_action_affected_sensors(subscribed_topics, output_list):
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
    check_resources_logger.debug('output_list: {0}'.format(output_list))

    for prop, prop_info_list in subscribed_topics.iteritems():
        for topic_info in prop_info_list: # enumerate all topic subscribing to
            check_resources_logger.debug('topic_info.start: {0}'.format(topic_info.start))

            if topic_info.start in ['/'+x for x in output_list]: # check if topic publisher is an output prop
                if not topic_info.start in possible_action_affected_sensors.keys():
                    possible_action_affected_sensors[topic_info.start] = []

                # now append which action affects what sensors
                possible_action_affected_sensors[topic_info.start].append(prop)

    check_resources_logger.info('possible_action_affected_sensors: {0}'.format(possible_action_affected_sensors))
    return possible_action_affected_sensors

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Check resouce usage in ROS")
    parser.add_argument('yaml_file', type=str, help='Path to yaml file')

    args, unknown = parser.parse_known_args()
    print args

    # load yaml file
    input_prop_to_ros_info, output_prop_to_ros_info = file_operations.loadYAMLFile(args.yaml_file)

    # load topics
    input_subscribed_topics, input_published_topics = get_subscribed_and_published_topics(input_prop_to_ros_info)
    output_subscribed_topics, output_published_topics = get_subscribed_and_published_topics(output_prop_to_ros_info)

    # check conflicts
    check_possible_concurrent_topic_access(output_published_topics)

    # check action affected sensors
    check_possible_action_affected_sensors(input_subscribed_topics, output_subscribed_topics.keys())
    # then compare with spec?


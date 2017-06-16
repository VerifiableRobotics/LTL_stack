#! /usr/bin/env python
import yaml
import argparse
import ntpath
import copy
import collections

import logging
import controller_executor_logging
setup_execution_logger = logging.getLogger("setup_execution_logger")

# define a compare function
compare = lambda x, y: collections.Counter(x) == collections.Counter(y)


def loadYAMLFile(yaml_file):
    with open(yaml_file, 'r') as stream:
        try:
             prop_to_ros_info = yaml.load(stream)
        except yaml.YAMLError as exc:
            print("ERROR: {0}".format(exc))
    return prop_to_ros_info['inputs'], prop_to_ros_info['outputs']

def create_launch_file(launch_file, example_name, input_prop_to_ros_info, output_prop_to_ros_info):
    with open(launch_file, 'w+') as f_launch_file:
        f_launch_file.write('<?xml version="1.0"?>\n<launch>\n\n')
        f_launch_file.write('\t<group ns="{0}">\n'.format(example_name))#ntpath.basename(launch_file).replace('.launch','')))

        # inputs
        f_launch_file.write('\t\t<!--inputs-->\n')
        f_launch_file.write('\t\t<group ns="inputs">\n')
        for prop, prop_info in input_prop_to_ros_info.iteritems():
            # check if parameters exist
            if 'parameters' in prop_info.keys():
                args_opt = " ".join(["--{key} {value}".format(key=key, value=value) for key, value in prop_info['parameters'].iteritems()])
            else:
                args_opt = ""

            args = '{node} {node_publish_topic} '.format(**prop_info)+args_opt
            f_launch_file.write(" ".join(['\t\t\t<node name="{node}" pkg="{pkg}" type="{filename}"'.format(**prop_info), \
                                               'args="{args}"'.format(args=args), \
                                        '/>\n']))
        f_launch_file.write('\t\t</group>\n')

        # outputs
        f_launch_file.write('\n\t\t<!--outputs-->\n')
        f_launch_file.write('\t\t<group ns="outputs">\n')
        for prop, prop_info in output_prop_to_ros_info.iteritems():
            # check if parameters exist
            if 'parameters' in prop_info.keys():
                args_opt = " ".join(["--{key} {value}".format(key=key, value=value) for key, value in prop_info['parameters'].iteritems()])
            else:
                args_opt = ""

            args = '{node} {node_subscribe_topic} '.format(**prop_info)+args_opt
            f_launch_file.write(" ".join(['\t\t\t<node name="{node}" pkg="{pkg}" type="{filename}"'.format(**prop_info), \
                                               'args="{args}"'.format(args=args), \
                                        '/>\n']))

        f_launch_file.write('\t\t</group>\n\t</group>\n</launch>\n')


#######################################
### Prepare for writing launch file ###
#######################################
def populate_namespace_dict_recursive(namespaces_to_node_name_dict, remaining_splited_list):
    if len(remaining_splited_list):
        if not remaining_splited_list[0] in namespaces_to_node_name_dict.keys():
            namespaces_to_node_name_dict[remaining_splited_list[0]] = {}

        populate_namespace_dict_recursive(namespaces_to_node_name_dict[remaining_splited_list[0]], remaining_splited_list[1:])

def prepare_populate_namespace_dict_recursive(prop, prop_info, namespaces_to_node_name_dict, namespace_chain_to_prop_dict):
    # get all namespaces
    splited_node = [x for x in prop_info['node'].split('/') if x]
    populate_namespace_dict_recursive(namespaces_to_node_name_dict, splited_node)

    prop_info['node'] = splited_node[-1] # replace info MAYBE?

    # save chain
    namespace_chain_to_prop_dict[frozenset(splited_node)] = prop


###########################
### Writing launch file ###
###########################
def create_launch_file_by_namespaces(launch_file, example_name, input_prop_to_ros_info, output_prop_to_ros_info):
    namespaces_to_node_name = {} # split a node's full name into namespaces and local name
    namespace_chain_to_prop = {} # save chain and map it to prop


    # group node names with namespaces
    if input_prop_to_ros_info:
        for prop, prop_info in input_prop_to_ros_info.iteritems():
            prop_info['node_original'] = prop_info['node']
            prepare_populate_namespace_dict_recursive(prop, prop_info, namespaces_to_node_name, namespace_chain_to_prop)

    # group node names with namespaces
    if output_prop_to_ros_info:
        for prop, prop_info in output_prop_to_ros_info.iteritems():

            if isinstance(prop_info, list): # if it's a one-output-to-many-nodes
                for prop_info_element in prop_info:
                    prop_info_element['node_original'] = prop_info_element['node']
                    prepare_populate_namespace_dict_recursive(prop, prop_info_element, namespaces_to_node_name, namespace_chain_to_prop)
            else:
                prop_info['node_original'] = prop_info['node']
                prepare_populate_namespace_dict_recursive(prop, prop_info, namespaces_to_node_name, namespace_chain_to_prop)


    print 'namespaces_to_node_name: {0}'.format(namespaces_to_node_name)
    print 'namespace_chain_to_prop: {0}'.format(namespace_chain_to_prop)


    with open(launch_file, 'w+') as f_launch_file:
        f_launch_file.write('<?xml version="1.0"?>\n<launch>\n\n')


        for namespace in namespaces_to_node_name.keys():
            write_launch_file_namespace_recursive(f_launch_file, namespace, namespaces_to_node_name, [], \
                namespace_chain_to_prop, input_prop_to_ros_info, output_prop_to_ros_info)

        f_launch_file.write('</launch>\n')

    f_launch_file.closed

def write_launch_file_output(prop_info, f_launch_file, namespace_chain_list):
    # check if parameters exist
    if 'parameters' in prop_info.keys():
        args_opt = " ".join(["--{key} {value}".format(key=key, value=value) for key, value in prop_info['parameters'].iteritems()])
    else:
        args_opt = ""

    args = '{node} {node_subscribe_topic} '.format(**prop_info)+args_opt
    f_launch_file.write(" ".join(['\t'*len(namespace_chain_list)+ \
                                       '<node name="{node}" pkg="{pkg}" type="{filename}"'.format(**prop_info), \
                                           'args="{args}"'.format(args=args), \
                                    '/>\n']))


def write_launch_file_namespace_recursive(f_launch_file, current_namespace, namespaces_to_node_name, namespace_chain_list, \
    namespace_chain_to_prop, input_prop_to_ros_info, output_prop_to_ros_info):

    namespace_chain_list.append(current_namespace) # save namespace in it
    if not namespaces_to_node_name[current_namespace]:
        # this is the node name!

        # get prop
        prop = namespace_chain_to_prop[frozenset(namespace_chain_list)]

        if prop in input_prop_to_ros_info.keys(): # input
            # check if parameters exist
            if 'parameters' in input_prop_to_ros_info[prop].keys():
                args_opt = " ".join(["--{key} {value}".format(key=key, value=value) for key, value in input_prop_to_ros_info[prop]['parameters'].iteritems()])
            else:
                args_opt = ""

            args = '{node} {node_publish_topic} '.format(**input_prop_to_ros_info[prop])+args_opt
            f_launch_file.write(" ".join(['\t'*len(namespace_chain_list)+ \
                                               '<node name="{node}" pkg="{pkg}" type="{filename}"'.format(**input_prop_to_ros_info[prop]), \
                                               'args="{args}"'.format(args=args), \
                                        '/>\n']))

        else: # output
            if isinstance(output_prop_to_ros_info[prop], list): # if it's a one-output-to-many-nodes
                for prop_info_element in output_prop_to_ros_info[prop]:
                    if compare(namespace_chain_list, prop_info_element['node_original'].strip('/').split('/')):
                        write_launch_file_output(prop_info_element, f_launch_file, namespace_chain_list)
            else:
                write_launch_file_output(output_prop_to_ros_info[prop], f_launch_file, namespace_chain_list)

    else:
        #print namespaces_to_node_name[current_namespace]
        f_launch_file.write('{0}<group ns="{1}">\n'.format('\t'*len(namespace_chain_list),current_namespace))
        for sub_namespace in namespaces_to_node_name[current_namespace]: # made into fn?
            write_launch_file_namespace_recursive(f_launch_file, sub_namespace, namespaces_to_node_name[current_namespace], copy.copy(namespace_chain_list), \
                namespace_chain_to_prop, input_prop_to_ros_info, output_prop_to_ros_info)

        f_launch_file.write('{0}</group>\n'.format('\t'*len(namespace_chain_list)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Create launch file")
    parser.add_argument('yaml_file', type=str, help='Path to yaml file')
    parser.add_argument('launch_file', type=str, help='Path to output launch file')
    parser.add_argument('example_name', type=str, help='Name of the example')

    args, unknown = parser.parse_known_args()
    print args

    input_prop_to_ros_info, output_prop_to_ros_info = loadYAMLFile(args.yaml_file)
    #create_launch_file(args.launch_file, args.example_name, input_prop_to_ros_info, output_prop_to_ros_info)
    create_launch_file_by_namespaces(args.launch_file, args.example_name, input_prop_to_ros_info, output_prop_to_ros_info)

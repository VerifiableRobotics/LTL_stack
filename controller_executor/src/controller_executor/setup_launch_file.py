#! /usr/bin/env python
"""
This file is used to auto-create launch file based on a LTLMoP example.
"""
import argparse
import logging
import sys
import ntpath
import os
import shutil
import xml.etree.ElementTree as ET


import file_operations
import controller_executor_logging
setup_execution_logger = logging.getLogger("setup_execution_logger")

from controller_executor import region_operations

def create_json_file(region_file, destination_folder):
    """
    This function calls LTLMoP regionToJSON
    """
    import lib.helperFns.regionToJSON

    # get name of region file
    region_filename = ntpath.basename(region_file)
    json_filename = region_filename.replace('.regions', '.json')

    # make directory if it does not exist
    if not os.path.exists(destination_folder+'/regions'):
        os.makedirs(destination_folder+'/regions')

    lib.helperFns.regionToJSON.region2json(region_file, destination_folder+'/regions/'+json_filename)

def create_png_files(region_file, destination_folder):
    import lib.helperFns.regionToPNG_combinations

    # get name of region file
    region_filename = ntpath.basename(region_file)
    png_filename = region_filename.replace('.regions', '.png')

    # make directory if it does not exist
    if not os.path.exists(destination_folder+'/regions'):
        os.makedirs(destination_folder+'/regions')

    # first create svg files
    svgFile_list = lib.helperFns.regionToPNG_combinations.region2svg(region_file, \
                        destination_folder+'/regions/'+png_filename)

    # now create png files
    for svgFile in svgFile_list:
        lib.helperFns.regionToPNG_combinations.svg2png(svgFile)


def create_proposition_launch_file(yaml_file, launch_file, example_name):
    # form proposition launch file from yaml file
    import file_operations
    input_prop_to_ros_info, output_prop_to_ros_info = file_operations.loadYAMLFile(yaml_file)
    #file_operations.create_launch_file(launch_file, example_name, input_prop_to_ros_info, output_prop_to_ros_info)
    file_operations.create_launch_file_by_namespaces(launch_file, example_name, input_prop_to_ros_info, output_prop_to_ros_info)


def update_arguments(tree_root_obj, example_name, yaml_file, slugsin_file):
    setup_execution_logger.log(2, "Updating arguments...")

    for arg in tree_root_obj.findall('arg'):
        if arg.attrib['name'] == "example_name":
            setup_execution_logger.log(2, "example_name: {0} {1}".format(arg.tag, arg.attrib))
            arg.attrib['default'] = example_name

        if arg.attrib['name'] == "slugsin_file":
            setup_execution_logger.log(2, "slugsin_file: {0} {1}".format(arg.tag, arg.attrib))
            arg.attrib['default'] = slugsin_file

        if arg.attrib['name'] == "yaml_file":
            setup_execution_logger.log(2, "yaml_file: {0} {1}".format(arg.tag, arg.attrib))
            arg.attrib['default'] = yaml_file

def create_executor_launch_file(controller_executor_dir, example_name, yaml_file, slugsin_file, launch_file, init_region):
    # create executor launch file

    # first load dummy example executor file
    tree = ET.parse(controller_executor_dir+'/launch/example_executor.launch')
    example_executor_file = tree.getroot()

    # then change arguments
    update_arguments(example_executor_file, example_name, yaml_file, slugsin_file)

    # update init props
    if init_region:
        for node in example_executor_file.iter('node'):
            if node.attrib['name'] == "executor":
                setup_execution_logger.debug("executor: {0} {1}".format(node.tag, node.attrib))
                node.attrib['args'] += " --init_props {0}".format(init_region+'_rc')

    # write to new file
    tree.write(launch_file, encoding='utf-8', xml_declaration=True)


def create_all_launch_file(controller_executor_dir, example_name, yaml_file, slugsin_file, launch_file):
    # create all launch file

    tree = ET.parse(controller_executor_dir+'/launch/example_all.launch')
    example_all_file = tree.getroot()

    # then change arguments
    update_arguments(example_all_file, example_name, yaml_file, slugsin_file)

    # then update include files
    setup_execution_logger.log(2, "Updating launch files...")
    for include_file in example_all_file.findall('include'):
        if "_background.launch" in include_file.attrib['file']:
            setup_execution_logger.log(2, "background: {0} {1}".format(include_file.tag, include_file.attrib))
            include_file.attrib['file'] = launch_file.replace('_all.launch', '_background.launch')

        if "_propositions.launch" in include_file.attrib['file']:
            setup_execution_logger.log(2, "propositions: {0} {1}".format(include_file.tag, include_file.attrib))
            include_file.attrib['file'] = launch_file.replace('_all.launch', '_propositions.launch')

        if "_executor.launch" in include_file.attrib['file']:
            setup_execution_logger.log(2, "executor: {0} {1}".format(include_file.tag, include_file.attrib))
            include_file.attrib['file'] = launch_file.replace('_all.launch', '_executor.launch')

    # write to new file
    tree.write(launch_file, encoding='utf-8', xml_declaration=True)


# TODO: load default xml for example?
def create_background_launch_file_placeholder(destination_folder, example_name):
    with open(destination_folder+'/'+example_name+'_background.launch', 'w+') as background_launch_obj:
        background_launch_obj.write('<?xml version="1.0"?>\n'+\
                                    '<launch>\n'+\
                                    '<!-- Please start the nesscary background nodes and launch files here.-->\n'+\
                                    '</launch>')
    background_launch_obj.closed



# setup initial position of the robot from region file?!
def create_background_gazebo_launch_file(controller_executor_dir, destination_folder, region_file, init_region, \
                                            rot, scale, x_trans, y_trans, launch_file):
    tree = ET.parse(controller_executor_dir+'/launch/example_background_gazebo_turtlebot.launch')
    gazebo_turtlebot_file = tree.getroot()

    # first figure out where do we want the robot to start
    # get name of region file
    region_filename = ntpath.basename(region_file)
    json_filename = region_filename.replace('.regions', '.json')

    json_file = destination_folder+'/regions/'+json_filename
    x_init, y_init = region_operations.find_point_in_region(json_file, init_region, rot, scale, x_trans, y_trans)

    # set initial position
    for arg in gazebo_turtlebot_file.findall('arg'):
        if arg.attrib['name'] == "x":
            setup_execution_logger.log(2, "x: {0} {1}".format(arg.tag, arg.attrib))
            arg.attrib['default'] = str(x_init)

        if arg.attrib['name'] == "y":
            setup_execution_logger.log(2, "y: {0} {1}".format(arg.tag, arg.attrib))
            arg.attrib['default'] = str(y_init)

        if arg.attrib['name'] == "delta_x":
            setup_execution_logger.log(2, "delta_x: {0} {1}".format(arg.tag, arg.attrib))
            arg.attrib['default'] = str(-1*x_init)

        if arg.attrib['name'] == "delta_y":
            setup_execution_logger.log(2, "delta_y: {0} {1}".format(arg.tag, arg.attrib))
            arg.attrib['default'] = str(-1*y_init)

    arg_str = "robot_current_region --model_name mobile_base --scale {scale} ".format(scale=scale)+\
              "--rot {rot} --json_file {json_file} ".format(rot=rot, json_file=json_file)+\
              "--x_trans {x_trans} --y_trans {y_trans}".format(x_trans=x_trans, y_trans=y_trans)
    #<node name="current_region" pkg="controller_executor" type="robot_location.py" args= />

    # add extra node for broadcasting current region
    current_region_obj = ET.Element('node')
    current_region_obj.attrib['name'] = "robot_current_region"
    current_region_obj.attrib['pkg'] = "controller_executor"
    current_region_obj.attrib['type'] = "robot_location.py"
    current_region_obj.attrib['args'] = arg_str
    gazebo_turtlebot_file.append(current_region_obj)

    # write to new file
    tree.write(launch_file, encoding='utf-8', xml_declaration=True)


def main(args):

    # add LTLMoP to system directory
    if args.LTLMoP_src_dir not in sys.path:
        sys.path.insert(0, args.LTLMoP_src_dir)
    setup_execution_logger.info('LTLMoP src folder: {0}'.format(args.LTLMoP_src_dir))

    #################################################################
    #### make a copy of the slugsin file into the example folder ####
    #################################################################
    if not os.path.isfile(args.destination_folder+'/'+args.example_name+'.slugsin') or \
    args.slugsin_file != args.destination_folder+'/'+args.example_name+'.slugsin':
    # check if file exists or we are copying from somewhere else
        shutil.copyfile(args.slugsin_file, args.destination_folder+'/'+args.example_name+'.slugsin')
        setup_execution_logger.info('Copying slugsin file to example directory')
    else:
        setup_execution_logger.info('Slugsin file already exists in example directory')

    ##############################################################
    #### make a copy of the yaml file into the example folder ####
    ##############################################################
    if not os.path.isfile(args.destination_folder+'/'+args.example_name+'.yaml') or \
    args.yaml_file != args.destination_folder+'/'+args.example_name+'.yaml':
    # check if file exists or we are copying from somewhere else
        shutil.copyfile(args.yaml_file, args.destination_folder+'/'+args.example_name+'.yaml')
        setup_execution_logger.info('Copying yaml file to example directory')
    else:
        setup_execution_logger.info('Yaml file already exists in example directory')


    ##############################################################
    #### Check if there are any regions/region file provided #####
    ##############################################################
    if args.region_file is not None and args.region_file != 'None':
        setup_execution_logger.info('Creating JSON files and PNG files...')

        # create json file from region file
        create_json_file(args.region_file, args.destination_folder)

        # create png files (in the middle svg files are created) from region file
        create_png_files(args.region_file, args.destination_folder)

        # remove svg files
        files = os.listdir(args.destination_folder+'/regions/')
        for file in files:
            if file.endswith(".svg"):
                os.remove(os.path.join(args.destination_folder+'/regions/',file))

    ############################################
    #### create launch file for propositions ####
    ############################################
    setup_execution_logger.info('Creating launch file for propositions...')
    create_proposition_launch_file(args.yaml_file, args.destination_folder+'/'+args.example_name+'_propositions.launch', args.example_name)


    ############################################
    #### create launch file for backgrounds ####
    ############################################
    setup_execution_logger.info('Creating launch file for background... to {0}'.format(args.destination_folder+'/'+args.example_name+'_background.launch'))

    if args.region_file is not None and args.region_file != 'None':
        # gazebo
        create_background_gazebo_launch_file(args.controller_executor_dir, args.destination_folder, args.region_file, args.init_region, \
                                                    args.rot, args.scale, args.x_trans, args.y_trans, \
                                                    args.destination_folder+'/'+args.example_name+'_background.launch')
    else:
        # dummy
        create_background_launch_file_placeholder(args.destination_folder, args.example_name)

    #########################################
    #### create launch file for executor ####
    #########################################
    setup_execution_logger.info('Creating launch file for executor... to {0}'.format(args.destination_folder+'/'+args.example_name+'_executor.launch'))
    create_executor_launch_file(args.controller_executor_dir, args.example_name, args.destination_folder+'/'+args.example_name+'.yaml', \
                                                    args.destination_folder+'/'+args.example_name+'.slugsin', \
                                                    args.destination_folder+'/'+args.example_name+'_executor.launch', args.init_region)


    ###########################################
    #### create launch file for everything ####
    ###########################################
    setup_execution_logger.info('Creating launch file that calls everthing... to {0}'.format(args.destination_folder+'/'+args.example_name+'_all.launch'))
    create_all_launch_file(args.controller_executor_dir, args.example_name, args.destination_folder+'/'+args.example_name+'.yaml', \
                                            args.destination_folder+'/'+args.example_name+'.slugsin', \
                                            args.destination_folder+'/'+args.example_name+'_all.launch')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This node subscribes pose info from gazebo and determine which region the robot is in.")
    parser.add_argument('--slugsin_file', type=str, help='Directory to example slugsin file', nargs='?', default='/home/catherine/LTLMoP/src/examples/firefighting_fastslow/firefighting.slugsin')
    parser.add_argument('--region_file', type=str, help='Directory to example region file', nargs='?', default='/home/catherine/LTLMoP/src/examples/firefighting_fastslow/floorplan.regions')
    parser.add_argument('--destination_folder', type=str, help='Directory to all files generated. No last backslash', nargs='?', default='/home/catherine/LTLROS_ws/src/controller_executor/examples/firefighting_test')
    parser.add_argument('--example_name', type=str, help='Name of the example in the destination folder.', nargs='?', default='firefighting_test')
    parser.add_argument('--yaml_file', type=str, help='Directory to yaml file.', nargs='?', default='/home/catherine/LTLROS_ws/src/controller_executor/examples/firefighting_test/firefighting_robot_location.yaml')
    parser.add_argument('--LTLMoP_src_dir', type=str, help='Directory to the LTLMoP src folder', nargs='?', default='/home/catherine/LTLMoP/src')
    parser.add_argument('--controller_executor_dir', type=str, help='Directory to the ROS controller executor folder', nargs='?', default='/home/catherine/LTLROS_ws/src/controller_executor')
    parser.add_argument('--init_region', type=str, help='Specify name of init region', nargs='?', default='porch')

    parser.add_argument('--rot', type=float, help='region rotation', nargs='?', const=0, default=0)
    parser.add_argument('--scale', type=float, help='region scale in both x and y direction. meter to pixel', nargs='?', const=0.01, default=0.01)
    parser.add_argument('--x_trans', type=float, help='region x translation', nargs='?', const=0, default=0)
    parser.add_argument('--y_trans', type=float, help='region y translation', nargs='?', const=0, default=0)


    args, unknown = parser.parse_known_args()
    setup_execution_logger.debug(args)

    main(args)


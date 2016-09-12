#! /usr/bin/env python
import yaml
import argparse
import ntpath

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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Create launch file")
    parser.add_argument('yaml_file', type=str, help='Path to yaml file')
    parser.add_argument('launch_file', type=str, help='Path to output launch file')
    parser.add_argument('example_name', type=str, help='Name of the example')

    args, unknown = parser.parse_known_args()
    print args

    input_prop_to_ros_info, output_prop_to_ros_info = loadYAMLFile(args.yaml_file)
    create_launch_file(args.launch_file, args.example_name, input_prop_to_ros_info, output_prop_to_ros_info)


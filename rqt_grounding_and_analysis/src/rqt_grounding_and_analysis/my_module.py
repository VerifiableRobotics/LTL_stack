import os
import rospy
import rospkg
import rosnode
import roslib.scriptutil as scriptutil
import socket
from functools import partial
import pydot
import ast
import getpass
import itertools
import copy
import rostopic
import StringIO
import yaml

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QTabWidget, QListWidget, QListWidgetItem, QGridLayout, QFormLayout
import python_qt_binding.QtGui as QtGui
import python_qt_binding.QtCore as QtCore

from controller_executor import file_operations

import check_resource_usage
import rosgraph_operations

import logging
import rqt_grounding_and_analysis_logging
gui_logger = logging.getLogger("gui_logger")

#command#
# rqt --standalone rqt_grounding_and_analysis

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            gui_logger.info('arguments: {0}'.format(args))
            gui_logger.info('unknowns: {0}'.format(unknowns))

        # Create QWidget
        self._widget = QtGui.QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_grounding_and_analysis'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._context = context
        ########### CUSTOM CODE ############

        self.example_name = "simple"
        #########################
        ####### MAPPING #########
        #########################
        # initialize dictionaries and list
        self._input_props = []
        self._output_props = []
        self._prop_node = {}
        self._prop_topic = {}

        # obtain vertical layout that contains grids for inputs and outputs
        self._verticalLayout_mapping = self._widget.findChild(QtGui.QVBoxLayout, name="verticalLayout_mapping")

        # obtain scroll area of mapping
        self._scrollAreaWidgetContents_Mapping = self._widget.findChild(QtGui.QWidget, name="scrollAreaWidgetContents_Mapping")
        self._gridLayout_mapping = self._widget.findChild(QtGui.QGridLayout, name="gridLayout_mapping")

        # obtain input and output grids for mapping
        self._gridLayout_input = self._widget.findChild(QtGui.QGridLayout, name="gridLayout_input")
        self._gridLayout_output = self._widget.findChild(QtGui.QGridLayout, name="gridLayout_output")

        # connect callback for pushButton load propositions
        self._pushButton_load_prop = self._widget.findChild(QtGui.QPushButton, name="pushButton_load_prop")
        self._pushButton_load_prop.clicked.connect(self.on_pushButton_load_prop_clicked)
        self._lineEdit_slugsin_file = self._widget.findChild(QtGui.QLineEdit, name="lineEdit_slugsin_file")

        # connect callback for pushButton load yaml mapping
        self._pushButton_load_yaml_mapping = self._widget.findChild(QtGui.QPushButton, name="pushButton_load_yaml_mapping")
        self._pushButton_load_yaml_mapping.clicked.connect(self.on_pushButton_load_yaml_mapping_clicked)
        self._lineEdit_load_yaml_mapping = self._widget.findChild(QtGui.QLineEdit, name="lineEdit_load_yaml_mapping")
        self._pushButton_load_yaml_mapping.setEnabled(False) #disable load mapping before props are loaded

        # connect callback for pushButton refresh topics and nodes
        self._pushButton_refresh = self._widget.findChild(QtGui.QPushButton, name="pushButton_refresh")
        self._pushButton_refresh.clicked.connect(self.on_pushButton_refresh_clicked)

        # save yaml file
        self._pushButton_save_yaml = self._widget.findChild(QtGui.QPushButton, name="pushButton_save_yaml")
        self._pushButton_save_yaml.clicked.connect(self.on_pushButton_save_yaml_clicked)

        # set up pernament list of node, topics to add to QComboBox of each prop
        self._mapping_file_node, self._mapping_file_topic = {}, {}

        ####################
        #### Analysis ######
        ####################
        self._tableView_same_topic= self._widget.findChild(QtGui.QTableView, name="tableView_same_topic")
        self._tableView_output_to_input= self._widget.findChild(QtGui.QTableView, name="tableView_output_to_input")

        self._verticalLayout_sub_analysis = self._widget.findChild(QtGui.QVBoxLayout, name="verticalLayout_sub_analysis")

        self._scrollAreaWidgetContents_Analysis = self._widget.findChild(QtGui.QWidget, name="scrollAreaWidgetContents_Analysis")

        self._listWidget_ltl = self._widget.findChild(QtGui.QListWidget, name="listWidget_ltl")

        # connect callback for pushButton analyze_conflicts
        self._pushButton_analyze_conflicts = self._widget.findChild(QtGui.QPushButton, name="pushButton_analyze_conflicts")
        self._pushButton_analyze_conflicts.clicked.connect(self.on_pushButton_analyze_conflicts_clicked)

        # connect callback for pushButton load graph from file
        self._pushButton_load_graph_from_file = self._widget.findChild(QtGui.QPushButton, name="pushButton_load_graph_from_file")
        self._pushButton_load_graph_from_file.clicked.connect(self.on_pushButton_load_graph_from_file_clicked)

        # connect callback for pushButton load graph from file
        self._pushButton_get_rqt_graph_snapshot = self._widget.findChild(QtGui.QPushButton, name="pushButton_get_rqt_graph_snapshot")
        self._pushButton_get_rqt_graph_snapshot.clicked.connect(self.on_pushButton_get_rqt_graph_snapshot_clicked)

        # disable analyze conflicts
        self._pushButton_analyze_conflicts.setEnabled(False)


        #### tests ###

    ###### FUNCTIONS ##########################################################################################################
    #####################################
    ###########  ANALYSIS ###############
    #####################################
    def on_pushButton_get_rqt_graph_snapshot_clicked(self):
        dot_data = rosgraph_operations.get_current_rqt_graph_to_dotcode()

        gui_logger.log(2, dot_data)

        # load dot file
        dot_graph =  pydot.graph_from_dot_data(dot_data)

        self.load_dot_graph(dot_graph)

        # bold text on button
        self._pushButton_analyze_conflicts.setEnabled(True)
        self._pushButton_get_rqt_graph_snapshot.setStyleSheet('QPushButton {background-color: #A3C1DA; color: red; font: bold;}')
        self._pushButton_load_graph_from_file.setStyleSheet("")

        rospy.loginfo("Getting a rosgraph snapshot.")


    def on_pushButton_load_graph_from_file_clicked(self):
        # second argument is file_type
        (dot_file, _) = QtGui.QFileDialog.getOpenFileName(caption="Open graph from file", directory="/", \
                             filter="DOT Graph (*.dot)")

        # load propositions from file
        with open(dot_file, 'r') as stream:
            text = stream.read()
        stream.closed

        dot_graph =  pydot.graph_from_dot_file(dot_file)

        # load dot file
        self.load_dot_graph(dot_graph)

        # bold text on button
        self._pushButton_analyze_conflicts.setEnabled(True)
        self._pushButton_load_graph_from_file.setStyleSheet('QPushButton {background-color: #A3C1DA; color: red; font: bold;}')
        self._pushButton_get_rqt_graph_snapshot.setStyleSheet("")

        rospy.loginfo("Getting rosgraph from a dot file.")


    def on_pushButton_analyze_conflicts_clicked(self):
        self.populate_table_same_topic()

        self.populate_ltl_list(self.suggested_ltl_list)

        self.populate_output_to_input()

        # reset tab size
        self._scrollAreaWidgetContents_Analysis.setMinimumSize(self._tableView_output_to_input.horizontalHeader().length()+20, \
                                                               self._tableView_same_topic.verticalHeader().length()+30+\
                                                               self._tableView_output_to_input.verticalHeader().length()+50+30*2)

        rospy.loginfo("Analyze conflicts.")


    def populate_ltl_list(self, suggested_ltl_list):
        self._listWidget_ltl.clear()

        for idx,ltl_str in enumerate(suggested_ltl_list):
            ltl_item = QtGui.QListWidgetItem(ltl_str)
            self._listWidget_ltl.insertItem(idx, ltl_item)


    def get_ltl_suggestion(self, output_full_list):
        full_ltl_list = []
        for idx, otuput_prop in enumerate(output_full_list):
            temp_list = copy.deepcopy(output_full_list)
            temp_list[idx] = "not "+temp_list[idx]
            full_ltl_list.append(temp_list)

        # everything can be false
        temp_list = copy.deepcopy(output_full_list)
        temp_list = ["not "+x for x in temp_list]
        full_ltl_list.append(temp_list)

        # form ltl string
        ltl_str = "always ("+ " or ".join("("+" and ".join(prop for prop in prop_list)+")" for prop_list in full_ltl_list)+")"

        return ltl_str


    def populate_table_same_topic(self):
        # populate table that show same topic

        same_topic_table_model = QtGui.QStandardItemModel(0,0) #initialize with 0 rows and 0 columns
        self._tableView_same_topic.setModel(same_topic_table_model)

        same_topic_table_model.setHorizontalHeaderItem(0, QtGui.QStandardItem("Topic"))
        same_topic_table_model.setHorizontalHeaderItem(1, QtGui.QStandardItem("Output"))
        same_topic_table_model.setHorizontalHeaderItem(2, QtGui.QStandardItem("Output-to-Topic Chain"))

        self._tableView_same_topic.horizontalHeader().resizeSection(0, 120) # Topic
        self._tableView_same_topic.horizontalHeader().resizeSection(1, 120) # Output
        #self._tableView_same_topic.resizeColumnsToContents()

        # check access
        same_output_dict = check_resource_usage.check_possible_concurrent_topic_access(\
            self.output_published_topics)

        gui_logger.log(1, 'self.published_graph: {0}'.format(self.published_graph))
        self.suggested_ltl_list = []

        gui_logger.log(1, 'same_output_dict:{0}'.format(same_output_dict))
        current_row_count = 0 # track no of rows
        for ros_topic, output_list in same_output_dict.iteritems():
            input_count = 0 # trak no of inputs for this output

            # set first column ros_topic
            same_topic_table_model.setItem(current_row_count,0,QtGui.QStandardItem(ros_topic))

            # convert two-item-list to a full list
            output_full_list = list(set(list(itertools.chain(*output_list))))
            gui_logger.log(2, 'output_full_list: {0}'.format(output_full_list))

            for output_prop in output_full_list:
                # set second column output_prop prop
                same_topic_table_model.setItem(current_row_count,1,QtGui.QStandardItem(output_prop))

                # chain
                #chain_str = check_resource_usage.chain_output(self.published_graph,\
                #    "n__{example_name}_outputs_{output_prop}".format(example_name=self.example_name, output_prop=output_prop),\
                #    "t_{ros_topic}".format(ros_topic=ros_topic.replace('/','_')),\
                #    self.prop_dot_to_real_name)
                chain_str = check_resource_usage.chain_output(self.published_graph,\
                    "n__"+"_".join([x for x in self.output_prop_to_ros_info[output_prop]['node'].split("/") if x]),\
                    "t_{ros_topic}".format(ros_topic=ros_topic.replace('/','_')),\
                    self.prop_dot_to_real_name)
                same_topic_table_model.setItem(current_row_count,2,QtGui.QStandardItem(chain_str))

                current_row_count += 1
                input_count += 1

            # set output to span rows
            self._tableView_same_topic.setSpan(current_row_count-input_count, 0, input_count, 1)

            # fomulate ltl suggestions
            ltl_str = self.get_ltl_suggestion(output_full_list)
            self.suggested_ltl_list.append(ltl_str)

        # resize tableview
        #self._tableView_same_topic.setWordWrap(True)
        #self._tableView_same_topic.resizeRowsToContents()
        #self._tableView_same_topic.horizontalHeader().resizeSection(2, 320) # Output
        self._tableView_same_topic.horizontalHeader().setStretchLastSection(True)
        self._tableView_same_topic.verticalHeader().setResizeMode(QtGui.QHeaderView.ResizeToContents)

        # resize tableview
        self._tableView_same_topic.setMaximumSize(self._tableView_same_topic.horizontalHeader().length(),\
                                                  self._tableView_same_topic.verticalHeader().length()+30)


        gui_logger.info("Suggested LTL: {0}".format(self.suggested_ltl_list))

    def populate_output_to_input(self):
        # TODO: Modify the size of table view to fit everything.

        output_to_input_table_model = QtGui.QStandardItemModel(0,0) #initialize with 0 rows and 0 columns
        self._tableView_output_to_input.setModel(output_to_input_table_model)

        output_to_input_table_model.setHorizontalHeaderItem(0, QtGui.QStandardItem("Output"))
        output_to_input_table_model.setHorizontalHeaderItem(1, QtGui.QStandardItem("Input"))
        output_to_input_table_model.setHorizontalHeaderItem(2, QtGui.QStandardItem("Output-to-input Chain"))

        self._tableView_output_to_input.horizontalHeader().resizeSection(0, 120)
        self._tableView_output_to_input.horizontalHeader().resizeSection(1, 120)
        #self._tableView_output_to_input.resizeColumnsToContents()

        output_to_input_dict = check_resource_usage.check_possible_action_affected_sensors(\
            self.input_subscribed_topics, self.output_published_topics)

        current_row_count = 0 # track no of rows
        for output_prop, input_list in output_to_input_dict.iteritems():
            input_count = 0 # trak no of inputs for this output

            # set first column output_prop
            output_to_input_table_model.setItem(current_row_count,0,QtGui.QStandardItem(output_prop))

            for input_prop in input_list:
                # set second column input prop
                output_to_input_table_model.setItem(current_row_count,1,QtGui.QStandardItem(input_prop))

                # chain
                #chain_str = check_resource_usage.chain_output(self.published_graph,\
                #    "n__{example_name}_outputs_{output_prop}".format(example_name=self.example_name, output_prop=output_prop),\
                #    "n__{example_name}_inputs_{input_prop}".format(example_name=self.example_name, input_prop=input_prop),\
                #    self.prop_dot_to_real_name)
                chain_str = check_resource_usage.chain_output(self.published_graph,\
                    "n__"+"_".join([x for x in self.output_prop_to_ros_info[output_prop]['node'].split("/") if x]),\
                    "n__"+"_".join([x for x in self.input_prop_to_ros_info[input_prop]['node'].split("/") if x]),\
                    self.prop_dot_to_real_name)
                output_to_input_table_model.setItem(current_row_count,2,QtGui.QStandardItem(chain_str))

                current_row_count += 1
                input_count += 1

            # set output to span rows
            self._tableView_output_to_input.setSpan(current_row_count-input_count, 0, input_count, 1)

        # resize tableview
        self._tableView_output_to_input.horizontalHeader().setStretchLastSection(True)
        self._tableView_output_to_input.verticalHeader().setResizeMode(QtGui.QHeaderView.ResizeToContents)

        # resize tableview
        self._tableView_output_to_input.setMaximumSize(self._tableView_output_to_input.horizontalHeader().length(),\
                                                       self._tableView_output_to_input.verticalHeader().length()+50)

    def load_dot_graph(self, dot_graph):
        # ---- simple ---- #
        #self.input_prop_to_ros_info, self.output_prop_to_ros_info = file_operations.loadYAMLFile(\
        #    '/home/{0}/LTLROS_ws/src/controller_executor/examples/simple/simple.yaml'.format(getpass.getuser()))

        self.input_prop_to_ros_info, self.output_prop_to_ros_info =  self.get_mapping_snapshot()

        ##################
        ##### NODES ######
        ##################
        #first grab all the nodes (both n__ and t__)
        nodes_dict = {}
        for subgraph_name, subgraph in dot_graph.obj_dict['subgraphs'].iteritems():
            for node_id, node in subgraph[0]['nodes'].iteritems():
                #if len(node) != 1:
                #    check_resources_logger.warning("Length of nodes is longer than 1!")
                nodes_dict[node_id] = node # to get info should do node[0]
                #else:
                #    nodes_dict[node_id] = node[0]

        # also join in obj_dict['nodes']
        nodes_dict.update(dot_graph.obj_dict["nodes"])

        #########################
        ## DOT TO REAL MAPPING ##
        #########################
        self.prop_dot_to_real_name, self.prop_real_to_dot_name = \
                check_resource_usage.get_prop_dot_name_real_name_mapping(nodes_dict)

        gui_logger.log(4, "self.prop_dot_to_real_name:\n {0}".format(str(self.prop_dot_to_real_name)))
        gui_logger.log(4, "self.prop_real_to_dot_name:\n {0}".format(str(self.prop_real_to_dot_name)))


        ##################
        ##### Edges ######
        ##################
        # get edges list
        edges_dict = dot_graph.obj_dict['edges']
        gui_logger.log(2, "Edges dict:\n {0}".format(str(edges_dict)))

        # separate into subscribe topics and publish topics
        self.input_subscribed_topics = {key: [] for key in self.input_prop_to_ros_info.keys()}
        self.input_published_topics = {key: [] for key in self.input_prop_to_ros_info.keys()}
        self.output_subscribed_topics = {key: [] for key in self.output_prop_to_ros_info.keys()}
        self.output_published_topics = {key: [] for key in self.output_prop_to_ros_info.keys()}

        # for storing dotnames
        self.input_subscribed_dotnames = {key: [] for key in self.input_prop_to_ros_info.keys()}
        self.input_published_dotnames = {key: [] for key in self.input_prop_to_ros_info.keys()}
        self.output_subscribed_dotnames = {key: [] for key in self.output_prop_to_ros_info.keys()}
        self.output_published_dotnames = {key: [] for key in self.output_prop_to_ros_info.keys()}

        self.published_graph = {}

        ####################################
        # filter some of the common topics #
        ####################################
        topic_filtered_list = ['/clock','/statistics', '/rosout']

        ###############
        ### inputs ####
        ###############
        for input_prop in self.input_prop_to_ros_info.keys():

            # rename prop to the dot file format
            #input_prop_dot_format = "n__"+self.example_name+'_inputs_'+input_prop.replace("/","_")
            input_prop_dot_format = "n__"+"_".join([x for x in self.input_prop_to_ros_info[input_prop]['node'].split("/") if x])
            gui_logger.log(1, "input_prop: {0} to {1}".format(input_prop, input_prop_dot_format))

            # recursive subscribed topics
            check_resource_usage.get_subscribed_topics(input_prop_dot_format, self.input_subscribed_dotnames[input_prop], \
                                  self.input_subscribed_topics[input_prop], edges_dict, nodes_dict, topic_filtered_list)

            # recursive published topics
            check_resource_usage.get_published_topics(input_prop_dot_format, self.input_published_dotnames[input_prop], \
                                 self.input_published_topics[input_prop], edges_dict, nodes_dict, topic_filtered_list)


        gui_logger.log(4, "self.input_subscribed_topics: {0}".format(str(self.input_subscribed_topics)))
        gui_logger.log(4, "self.input_published_topics: {0}".format(str(self.input_published_topics)))

        #rospy.loginfo("Published topics - bedroom_rc: {0}".format(self.input_published_topics['bedroom_rc']))

        ###############
        ### outputs ###
        ###############
        for output_prop in self.output_prop_to_ros_info.keys():

            # rename prop to the dot file format
            #output_prop_dot_format = "n__"+self.example_name+'_outputs_'+output_prop.replace("/","_")
            output_prop_dot_format = "n__"+"_".join([x for x in self.output_prop_to_ros_info[output_prop]['node'].split("/") if x])
            gui_logger.log(1, "input_prop: {0} to {1}".format(output_prop, output_prop_dot_format))

            # recursive subscribed topics
            check_resource_usage.get_subscribed_topics(output_prop_dot_format, self.output_subscribed_dotnames[output_prop], \
                                  self.output_subscribed_topics[output_prop], edges_dict, nodes_dict, topic_filtered_list)

            # recursive published topics
            check_resource_usage.get_published_topics(output_prop_dot_format, self.output_published_dotnames[output_prop], \
                                 self.output_published_topics[output_prop], edges_dict, nodes_dict, topic_filtered_list)


            check_resource_usage.get_published_graph(output_prop_dot_format, self.output_subscribed_dotnames[output_prop], \
                                    self.published_graph, edges_dict, nodes_dict, topic_filtered_list)
        #rospy.loginfo("Published topics - bedroom: {0}".format(self.output_published_topics['bedroom']))

    #####################################
    ###########  MAPPING ################
    #####################################
    def on_pushButton_save_yaml_clicked(self):
        # first get YAML file
        (yaml_file, _) = QtGui.QFileDialog.getSaveFileName(caption="Save mapping to yaml file", directory="/", \
                             filter="YAML file (*.yaml)")
        gui_logger.info("Writing to {0}".format(yaml_file))

        # open YAML file to write
        yamlHandle = open(yaml_file, 'w+')

        self.save_mapping(yamlHandle)

        rospy.loginfo("Saved mapping to yaml file.")

        # close file
        yamlHandle.close()


    def get_mapping_snapshot(self):
        # get a snapshot of the current mapping

        yamlHandle = StringIO.StringIO()

        self.save_mapping(yamlHandle)

        #gui_logger.debug(yamlHandle.getvalue())
        try:
            prop_to_ros_info = yaml.load(yamlHandle.getvalue())
        except yaml.YAMLError as exc:
            gui_logger.error("ERROR: {0}".format(exc))

        rospy.loginfo("Got a snapshot of the mapping.")
        return prop_to_ros_info['inputs'], prop_to_ros_info['outputs']


    def save_mapping(self, yamlHandle):

        # start with inputs
        yamlHandle.write('inputs:\n')

        for prop in self._input_props:
            self.write_prop_to_yaml(yamlHandle, prop)

        # start with outputs
        yamlHandle.write('outputs:\n')

        for prop in self._output_props:
            self.write_prop_to_yaml(yamlHandle, prop)


    def write_prop_to_yaml(self, yamlHandle, prop):
        # current prop
        yamlHandle.write('  {prop}:\n'.format(prop=prop))

        #node text box
        yamlHandle.write('    node: {0}\n'.format(self._prop_node[prop].currentText()))

        #topic for that prop
        if prop in self._input_props:
            yamlHandle.write('    node_publish_topic: {0}\n\n'.format(self._prop_topic[prop].currentText()))
        else:
            yamlHandle.write('    node_subscribe_topic: {0}\n\n'.format(self._prop_topic[prop].currentText()))


    def populate_grid(self):
        #############
        ## INPUTS ##
        ############
        # clear input grid
        for i in reversed(range(self._gridLayout_input.count())):
            widgetToRemove = self._gridLayout_input.itemAt(i).widget()
            # remove it from the layout list
            self._gridLayout_input.removeWidget(widgetToRemove)
            # remove it from the gui
            widgetToRemove.setParent(None)
            widgetToRemove.deleteLater()

        del self._gridLayout_input
        self._gridLayout_input = QtGui.QGridLayout()
        self._gridLayout_mapping.addLayout(self._gridLayout_input, 1, 0, 1, 0)
        gui_logger.log(2, "rowCount:{0}, columnCount: {1}".format(self._gridLayout_input.rowCount(), self._gridLayout_input.columnCount()))

        # for inputs
        for input_idx, input_prop in enumerate(self._input_props):
            self._prop_node[input_prop] = QtGui.QComboBox()
            self._prop_topic[input_prop] = QtGui.QComboBox()

            # fill up topic combo box after node combo box is selected
            self.connect(self._prop_node[input_prop], QtCore.SIGNAL("activated(const QString&)"), \
                            partial(self.refresh_topics,"input", input_prop, self._prop_node[input_prop]))

            # set to fixed size
            self._prop_node[input_prop].setMinimumHeight(23)
            self._prop_topic[input_prop].setMinimumHeight(23)

            self._gridLayout_input.addWidget(QtGui.QLabel(input_prop), input_idx, 0) # label
            self._gridLayout_input.addWidget(self._prop_node[input_prop], input_idx, 1) # node
            self._gridLayout_input.addWidget(self._prop_topic[input_prop], input_idx, 2)  # topic
            self._gridLayout_input.setRowMinimumHeight(input_idx, 23)

        gui_logger.log(2, "rowCount:{0}, columnCount: {1}".format(self._gridLayout_input.rowCount(), self._gridLayout_input.columnCount()))

        ##############
        ### OUTPUTS ##
        ##############
        # clear output grid
        for i in reversed(range(self._gridLayout_output.count())):
            widgetToRemove = self._gridLayout_output.itemAt(i).widget()
            # remove it from the layout list
            self._gridLayout_output.removeWidget(widgetToRemove)
            # remove it from the gui
            widgetToRemove.setParent(None)
            widgetToRemove.deleteLater()

        del self._gridLayout_output
        self._gridLayout_output = QtGui.QGridLayout()
        self._gridLayout_mapping.addLayout(self._gridLayout_output, 3, 0, 1, 0)
        gui_logger.log(2, "rowCount:{0}, columnCount: {1}".format(self._gridLayout_output.rowCount(), self._gridLayout_input.columnCount()))

        # for outputs
        for output_idx, output_prop in enumerate(self._output_props):
            self._prop_node[output_prop] = QtGui.QComboBox()
            self._prop_topic[output_prop] = QtGui.QComboBox()

            # fill up topic combo box after node combo box is selected
            self.connect(self._prop_node[output_prop], QtCore.SIGNAL("activated(const QString&)"),\
                         partial(self.refresh_topics, "output", output_prop, self._prop_node[output_prop]))

            # set to fixed size
            self._prop_node[output_prop].setMinimumHeight(23)
            self._prop_topic[output_prop].setMinimumHeight(23)

            self._gridLayout_output.addWidget(QtGui.QLabel(output_prop), output_idx, 0) # label
            self._gridLayout_output.addWidget(self._prop_node[output_prop], output_idx, 1) # node
            self._gridLayout_output.addWidget(self._prop_topic[output_prop], output_idx, 2)  # topic
            self._gridLayout_output.setRowMinimumHeight(output_idx, 23)

        gui_logger.log(2, "rowCount:{0}, columnCount: {1}".format(self._gridLayout_output.rowCount(), self._gridLayout_output.columnCount()))

        # adjust the size of the scroll area
        self._scrollAreaWidgetContents_Mapping.setMinimumSize(528, (self._gridLayout_input.rowCount() + self._gridLayout_output.rowCount()+2)*40)

        self._gridLayout_mapping.setRowStretch(1, self._gridLayout_input.rowCount())
        self._gridLayout_mapping.setRowStretch(4, self._gridLayout_output.rowCount())


    def refresh_nodes_and_topics(self):
        # first update nodes
        nodes_list = rosnode.get_node_names()

        for prop, node_combobox in self._prop_node.iteritems():

            # save previous node
            current_node = node_combobox.currentText()

            node_combobox.clear() # clear all options before updating
            node_combobox.addItems(nodes_list)

            # Add in permanent items from loading mapping
            if prop in self._mapping_file_node and \
            self._mapping_file_node[prop] not in nodes_list:
                node_combobox.addItem(self._mapping_file_node[prop])

            # Restore to old selected node
            if current_node in [node_combobox.itemText(i) for i in range(node_combobox.count())]:
                node_combobox.setCurrentIndex(node_combobox.findText(current_node))


            if prop in self._input_props:
                prop_type = "input"
            else:
                prop_type = "output"

            # now update topics
            self.refresh_topics(prop_type, prop, node_combobox)

        rospy.loginfo("Refreshed nodes and topics.")


    def refresh_topics(self, prop_type, prop, node_object):
        node_name = node_object.currentText()

        #gui_logger.debug('Refreshed topics for prop {0} and node {1}.'.format(prop, node_name))

        def _succeed(args):
            code, msg, val = args
            if code != 1:
                raise Exception("remote call failed: %s"%msg)
            return val

        def topic_type(t, pub_topics):
            matches = [t_type for t_name, t_type in pub_topics if t_name == t]
            if matches:
                return matches[0]
            return 'unknown type'

        master = scriptutil.get_master()
        ID = '/rosnode'
        # go through the master system state first
        try:
            state = _succeed(master.getSystemState(ID))
            pub_topics = _succeed(scriptutil.get_master().getPublishedTopics(ID, '/'))
        except socket.error:
            raise Exception("Unable to communicate with master!")

        pubs = [t for t, l in state[0] if node_name in l]
        subs = [t for t, l in state[1] if node_name in l]
        srvs = [t for t, l in state[2] if node_name in l]

        gui_logger.log(8,'pubs:' + str(pubs))
        gui_logger.log(8,'subs:' + str(subs))
        gui_logger.log(8,'srvs:' + str(srvs))
        gui_logger.log(8, 'pub_topics:' + str(pub_topics))


        if prop_type == "input": # publications
            topic_list = ["%s"%(l) for l in pubs if topic_type(l, pub_topics) =='std_msgs/Bool']

        elif prop_type == "output": #Subscriptions

            topic_list = ["%s"%(l) for l in subs if rostopic.get_topic_type(l)[0] =='std_msgs/Bool']
        else:
            rospy.logerr('This is not input or output?!')

        #gui_logger.debug('topic_list:' + str(topic_list))

        # save previous topic
        current_topic = self._prop_topic[prop].currentText()

        self._prop_topic[prop].clear()  # clear all options before updating
        self._prop_topic[prop].addItems(topic_list)

        # Add in permanent items from loading mapping
        if prop in self._mapping_file_topic and \
        self._mapping_file_topic[prop] not in topic_list:
            self._prop_topic[prop].addItem(self._mapping_file_topic[prop])

        # restore to old selected text
        if current_topic in [self._prop_topic[prop].itemText(i) for i in range(self._prop_topic[prop].count())]:
            self._prop_topic[prop].setCurrentIndex(self._prop_topic[prop].findText(current_topic))


    def on_pushButton_load_yaml_mapping_clicked(self):
        """
        extracts inputs and output props from slugsin file
        """
        # second argument is file_type
        (yaml_file, _) = QtGui.QFileDialog.getOpenFileName(caption="Open YAML mapping file", directory="/", \
                             filter="YAML File (*.yaml)")

        # set line text
        self._lineEdit_load_yaml_mapping.setText(yaml_file)

        # load mapping from file
        self.input_prop_to_ros_info, self.output_prop_to_ros_info = file_operations.loadYAMLFile(yaml_file)

        # set up pernament list of node, topics to add to QComboBox of each prop
        self._mapping_file_node, self._mapping_file_topic = {}, {}

        # inputs
        for input_prop in self.input_prop_to_ros_info:
            if input_prop in self._input_props: # first check if we have such propositions

                #check if node is there
                if not self.input_prop_to_ros_info[input_prop]['node'] in \
                [self._prop_node[input_prop].itemText(i) for i in range(self._prop_node[input_prop].count())]:
                    self._prop_node[input_prop].addItem(self.input_prop_to_ros_info[input_prop]['node'])

                # add this permanently to the QComboBox everytime it's refreshed.
                self._mapping_file_node[input_prop] = self.input_prop_to_ros_info[input_prop]['node']

                #check if topic is already in combo box
                if not self.input_prop_to_ros_info[input_prop]['node_publish_topic'] in \
                [self._prop_topic[input_prop].itemText(i) for i in range(self._prop_topic[input_prop].count())]:
                    self._prop_topic[input_prop].addItem(self.input_prop_to_ros_info[input_prop]['node_publish_topic'])

                # add this permanently to the QComboBox everytime it's refreshed.
                self._mapping_file_topic[input_prop] = self.input_prop_to_ros_info[input_prop]['node_publish_topic']

                # set current selected text to that from file
                self._prop_node[input_prop].setCurrentIndex(self._prop_node[input_prop].findText(self.input_prop_to_ros_info[input_prop]['node']))
                self._prop_topic[input_prop].setCurrentIndex(self._prop_topic[input_prop].findText(self.input_prop_to_ros_info[input_prop]['node_publish_topic']))

        # outputs
        for output_prop in self.output_prop_to_ros_info:
            if output_prop in self._output_props: # first check if we have such propositions

                #check if node is there
                if not self.output_prop_to_ros_info[output_prop]['node'] in \
                [self._prop_node[output_prop].itemText(i) for i in range(self._prop_node[output_prop].count())]:
                    self._prop_node[output_prop].addItem(self.output_prop_to_ros_info[output_prop]['node'])

                # add this permanently to the QComboBox everytime it's refreshed.
                self._mapping_file_node[output_prop] = self.output_prop_to_ros_info[output_prop]['node']

                #check if topic is already in combo box
                if not self.output_prop_to_ros_info[output_prop]['node_subscribe_topic'] in \
                [self._prop_topic[output_prop].itemText(i) for i in range(self._prop_topic[output_prop].count())]:
                    self._prop_topic[output_prop].addItem(self.output_prop_to_ros_info[output_prop]['node_subscribe_topic'])

                # add this permanently to the QComboBox everytime it's refreshed.
                self._mapping_file_topic[output_prop] = self.output_prop_to_ros_info[output_prop]['node_subscribe_topic']

                # set current selected text to that from file
                self._prop_node[output_prop].setCurrentIndex(self._prop_node[output_prop].findText(self.output_prop_to_ros_info[output_prop]['node']))
                self._prop_topic[output_prop].setCurrentIndex(self._prop_topic[output_prop].findText(self.output_prop_to_ros_info[output_prop]['node_subscribe_topic']))

        rospy.loginfo("Loaded Mapping.")



    def on_pushButton_load_prop_clicked(self):
        """
        extracts inputs and output props from slugsin file
        """
        # second argument is file_type
        (slugsin_file, _) = QtGui.QFileDialog.getOpenFileName(caption="Open slugsin file", directory="/", \
                             filter="Slugsin File (*.slugsin);; Structured Slugs File (*.structuredslugs)")

        # set line text
        self._lineEdit_slugsin_file.setText(slugsin_file)

        # load propositions from file
        with open(slugsin_file, 'r') as stream:
            text = stream.read()
        stream.closed

        # reset propositions
        self._input_props, self._output_props = [], []
        self._prop_node, self._prop_topic = {}, {}

        # reset file mapping memory
        self._mapping_file_node, self._mapping_file_topic = {}, {}

        # add propositions
        for line in text.split('\n'):
            # figure out what prop it is
            if line  == "[INPUT]":
                mode = "input"
                continue
            elif line == "[OUTPUT]":
                mode = "output"
                continue
            elif line in ["[ENV_TRANS]", "[ENV_INIT]", "[ENV_LIVENESS]", "[SYS_TRANS]", "[SYS_INIT]", "[SYS_LIVENESS]"]:
                break

            # add to list
            if line and mode == "input":
                self._input_props.append(line)
            elif line and mode == "output":
                self._output_props.append(line)

        # populate input and output grids
        self.populate_grid()

        # refresh nodes and topics
        self.refresh_nodes_and_topics()

        # now we allow user to load mapping
        self._pushButton_load_yaml_mapping.setEnabled(True)

        rospy.loginfo("Loaded propositions.")


    def on_pushButton_refresh_clicked(self):
        # refresh nodes and topics
        self.refresh_nodes_and_topics()


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
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

# extensions
from extension_analysis_multirobot import ExtensionAnalysisMultiRobot

import logging
import rqt_grounding_and_analysis_logging
gui_logger = logging.getLogger("gui_logger")

#command#
# rqt --standalone rqt_grounding_and_analysis

class PropMappingAndAnalysis(ExtensionAnalysisMultiRobot, Plugin):

    def __init__(self, context):
        super(PropMappingAndAnalysis, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MappingAndAnalysisPlugin')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_grounding_and_analysis'), 'resource', 'MappingAndAnalysisPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MappingAndAnalysisPluginUi')
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
        self._widget.resizeEvent = self.onResize

        #########################
        ####### MAPPING #########
        #########################
        # initialize dictionaries and list
        self._input_props = []
        self._output_props = []
        self._prop_node = {}
        self._prop_topic = {}
        self._prop_add_button = {}
        self._prop_label = {}
        self._prop_mapping_count = {}
        self._prop_gridLayout = {}

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

        self._tableView_same_topic.verticalHeader().setVisible(False)
        self._tableView_output_to_input.verticalHeader().setVisible(False)

        self._verticalLayout_sub_analysis = self._widget.findChild(QtGui.QVBoxLayout, name="verticalLayout_sub_analysis")

        self._scrollAreaWidgetContents_Analysis = self._widget.findChild(QtGui.QWidget, name="scrollAreaWidgetContents_Analysis")

        self._textEdit_ltl = self._widget.findChild(QtGui.QTextEdit, name="textEdit_ltl")

        # combo box for displaying ltl options
        self._comboBox_ltl_options = self._widget.findChild(QtGui.QComboBox, name='comboBox_ltl_options')
        # fill up ltl after combo box is selected
        self.connect(self._comboBox_ltl_options, QtCore.SIGNAL("activated(const QString&)"), \
                     partial(self.refresh_ltl_text_edit))
        self._comboBox_ltl_options.addItems(['Structured English','LTL'])  # add options

        # connect callback for pushButton analyze_conflicts
        self._pushButton_analyze_conflicts = self._widget.findChild(QtGui.QPushButton, name="pushButton_analyze_conflicts")
        self._pushButton_analyze_conflicts.clicked.connect(self.on_pushButton_analyze_conflicts_clicked)

        # connect callback for pushButton load graph from file
        self._pushButton_load_graph_from_file = self._widget.findChild(QtGui.QPushButton, name="pushButton_load_graph_from_file")
        self._pushButton_load_graph_from_file.clicked.connect(self.on_pushButton_load_graph_from_file_clicked)

        # connect callback for pushButton load graph from file
        self._pushButton_get_rqt_graph_snapshot = self._widget.findChild(QtGui.QPushButton, name="pushButton_get_rqt_graph_snapshot")
        self._pushButton_get_rqt_graph_snapshot.clicked.connect(self.on_pushButton_get_rqt_graph_snapshot_clicked)

        # combo box for displaying exclusion topics
        self._comboBox_select_exclusion_topic = self._widget.findChild(QtGui.QComboBox, name='comboBox_select_exclusion_topic')
        # fill up ltl after combo box is selected
        self.connect(self._comboBox_select_exclusion_topic, QtCore.SIGNAL("activated(const QString&)"), \
                     partial(self.refresh_exclusion_ltl_and_table_selection))

        self._tableView_same_topic.setSelectionMode(QtGui.QAbstractItemView.MultiSelection)   # set selection mode
        self.same_topic_table_model = None
        self.same_output_table_layout = {} #'rostopic': [first row, last row]

        # disable analyze conflicts
        self._pushButton_analyze_conflicts.setEnabled(False)

        # setup table for showing outputs that subscribe to different nodes publishing to the smae topic
        self._tableView_node_publish = self._widget.findChild(QtGui.QTableView, name="tableView_node_publish")
        self._tableView_node_publish.verticalHeader().setVisible(False)
        self._tableView_node_publish.setSelectionMode(QtGui.QAbstractItemView.MultiSelection)   # set selection mode

        self.prop_node_publish_layout = {}

        # setup extensions
        self.setup_extension()

        #### tests ###

    def setup_extension(self):
        super(PropMappingAndAnalysis, self).setup_extension()

    def refresh_exclusion_ltl_and_table_selection(self):
        #first check if there are rows and columns in the table:
        #gui_logger.debug("Same Topic Table Row count: {0}".format(self.same_topic_table_model.rowCount()))
        if self.same_topic_table_model.rowCount():
            # deselect old one before new ones
            self._tableView_same_topic.clearSelection()

            #highlight in table
            starting_row = self.same_output_table_layout[self._comboBox_select_exclusion_topic.currentText()][0]
            ending_row = self.same_output_table_layout[self._comboBox_select_exclusion_topic.currentText()][1]
            row_count = ending_row - starting_row
            gui_logger.log(4, self.same_output_table_layout[self._comboBox_select_exclusion_topic.currentText()])
            for idx in range(row_count):
                self._tableView_same_topic.selectRow(starting_row+idx)

            # populate ltl
            self.populate_ltl_text_edit(self._comboBox_select_exclusion_topic.currentIndex(), self.suggested_ltl_list, self.suggested_structured_eng_list)

            # set view current cell
            if self.same_topic_table_model:
                modelIndex =  self.same_topic_table_model.index(starting_row, 0, QtCore.QModelIndex())
                self._tableView_same_topic.scrollTo(modelIndex, QtGui.QAbstractItemView.PositionAtTop)

    ###### FUNCTIONS ##########################################################################################################
    #####################
    ##### RESIZE ########
    #####################
    def onResize(self, event):
        # mutual exclusions resize
        pass
        """
        self._tableView_same_topic.setMinimumSize(self._widget.width()-100, \
                                                  self._tableView_same_topic.verticalHeader().length()+50)
        self._tableView_same_topic.resizeRowsToContents()
        self._tableView_same_topic.verticalHeader().setResizeMode(QtGui.QHeaderView.ResizeToContents)
        self._tableView_same_topic.setMinimumSize(self._widget.width()-100, \
                                                  self._tableView_same_topic.verticalHeader().length()+50)

        # output to input resize
        self._tableView_output_to_input.setMinimumSize(self._widget.width()-100,\
                                                       self._tableView_output_to_input.verticalHeader().length()+50)
        self._tableView_output_to_input.verticalHeader().setResizeMode(QtGui.QHeaderView.ResizeToContents)
        self._tableView_output_to_input.resizeRowsToContents()
        self._tableView_output_to_input.setMinimumSize(self._widget.width()-100,\
                                                       self._tableView_output_to_input.verticalHeader().length()+50)
        gui_logger.log(2, 'length_tableView_same_topic:{0} length_tableView_output_to_input:{1}'.format(\
                                                        self._tableView_same_topic.verticalHeader().length(),\
                                                        self._tableView_output_to_input.verticalHeader().length()))

        # reset tab size
        length1 = self._tableView_same_topic.verticalHeader().length() if self._tableView_same_topic.verticalHeader().length() else 80
        length2 = self._tableView_output_to_input.verticalHeader().length() if self._tableView_output_to_input.verticalHeader().length() else 80
        self._scrollAreaWidgetContents_Analysis.setMinimumSize(self._widget.width()-60, \
                                                                100+ length1 + length2)

        gui_logger.log(2,'length_scrollAreaWidgetContents_Analysis:{0}'.format(self._tableView_same_topic.verticalHeader().length() +\
                                       self._tableView_output_to_input.verticalHeader().length()))
        """
    #####################################
    ###########  ANALYSIS ###############
    #####################################
    def on_pushButton_get_rqt_graph_snapshot_clicked(self):
        dot_data = rosgraph_operations.get_current_rqt_graph_to_dotcode()

        gui_logger.log(2, 'dot_data: {0}'.format(dot_data))

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

        self.populate_exlusion_combo_box()

        self.populate_ltl_text_edit(self._comboBox_select_exclusion_topic.currentIndex(), self.suggested_ltl_list, self.suggested_structured_eng_list)

        self.populate_output_to_input()

        # showing props with nodes publishing to the same topic
        prop_to_concurrent_node_publish = check_resource_usage.check_prop_to_same_topic_from_differnt_nodes(self.output_published_topics['exclude_props'], \
                  self.output_prop_topic_filtered_list+self.final_filtered_list+self.robot_sensor_topic_list)
        self.populate_prop_to_concurrent_node_publish(prop_to_concurrent_node_publish)

        # reset tab size
        length1 = self._tableView_same_topic.verticalHeader().length() if self._tableView_same_topic.verticalHeader().length() else 80
        length2 = self._tableView_output_to_input.verticalHeader().length() if self._tableView_output_to_input.verticalHeader().length() else 80
        #self._scrollAreaWidgetContents_Analysis.setMinimumSize(self._widget.width()-60, \
        #                                                        100+ length1 + length2)

        rospy.loginfo("Analyze conflicts.")

    def populate_exlusion_combo_box(self):
        self._comboBox_select_exclusion_topic.clear() # clear all options before updating

        self._comboBox_select_exclusion_topic.addItems(self.same_output_table_layout.keys()) # update

        self.refresh_exclusion_ltl_and_table_selection()

    def populate_ltl_text_edit(self, idx, suggested_ltl_list, suggested_structured_eng_list):
        if suggested_ltl_list:
            #self._listWidget_ltl.clear()
            #gui_logger.debug('idx: {0}, suggested_ltl_list: {1}'.format(idx, suggested_ltl_list[self._comboBox_select_exclusion_topic.currentText()]))
            #gui_logger.debug('Combo Box Size:{0}, suggested_ltl_list size:{1}'.format(self._comboBox_select_exclusion_topic.count(),\
            #                                                                            len(suggested_ltl_list)))
            #ltl_item = QtGui.QListWidgetItem(suggested_ltl_list[idx])
            #self._listWidget_ltl.insertItem(0, ltl_item)

            #ltl or structured eng
            if self._comboBox_ltl_options.currentText() == 'LTL':
                self._textEdit_ltl.clear()
                self._textEdit_ltl.setText(suggested_ltl_list[self._comboBox_select_exclusion_topic.currentText()])
            else:
                self._textEdit_ltl.clear()
                self._textEdit_ltl.setText(suggested_structured_eng_list[self._comboBox_select_exclusion_topic.currentText()])
        else:
            self._textEdit_ltl.clear()

    def refresh_ltl_text_edit(self):
        self.populate_ltl_text_edit(self._comboBox_select_exclusion_topic.currentIndex(), self.suggested_ltl_list, self.suggested_structured_eng_list)

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
        structured_eng_str = "always ("+ " or ".join("("+" and ".join(prop for prop in prop_list)+")" for prop_list in full_ltl_list)+")"
        ltl_str = "[] ("+ " | ".join("("+" & ".join(prop.replace('not', '!') for prop in prop_list)+")" for prop_list in full_ltl_list)+")"

        return structured_eng_str, ltl_str

    def populate_prop_to_concurrent_node_publish(self, prop_to_concurrent_node_publish):

        # populate table that show same topic
        self.item_model_node_publish = QtGui.QStandardItemModel(0,0) #initialize with 0 rows and 0 columns
        self._tableView_node_publish.setModel(self.item_model_node_publish)

        self.item_model_node_publish.setHorizontalHeaderItem(0, QtGui.QStandardItem("Output Proposition"))
        self.item_model_node_publish.setHorizontalHeaderItem(1, QtGui.QStandardItem("Topic"))
        self.item_model_node_publish.setHorizontalHeaderItem(2, QtGui.QStandardItem("Node"))
        self.item_model_node_publish.setHorizontalHeaderItem(3, QtGui.QStandardItem("One existing Chain"))

        self._tableView_node_publish.horizontalHeader().resizeSection(0, 150) # Prop
        self._tableView_node_publish.horizontalHeader().resizeSection(1, 150) # Topic
        self._tableView_node_publish.horizontalHeader().resizeSection(2, 120) # Robots

        current_row_count = 0 # track no of rows

        for prop, topic_list in prop_to_concurrent_node_publish.iteritems():

            prop_count = 0

            for ros_topic in topic_list:

                topic_count = 0 # trak no of inputs for this output

                # find node
                for mapping in self.output_prop_to_ros_info[prop]:

                    node_dot_name = self.prop_real_to_dot_name['n'][mapping['node']]

                    # map names to dot format
                    #chain_str =self.find_prop_to_topic_chain(node_dot_name, ros_topic, self.chain_prop_dot_to_topic_dot_published_dict)
                    chain_str =self.find_prop_to_topic_chain(node_dot_name, ros_topic, self.chain_topic_node_dotnames_published_dict)
                    if chain_str:
                        prop_item = QtGui.QStandardItem(prop)
                        # set first column to prop
                        self.item_model_node_publish.setItem(current_row_count,0,prop_item)
                        # set first column ros_topic (also break topic into parts to show everything in table)
                        self.item_model_node_publish.setItem(current_row_count,1,QtGui.QStandardItem(ros_topic.replace('/',' /').replace('_','_ ')))
                        self.item_model_node_publish.setItem(current_row_count,2,QtGui.QStandardItem(mapping['node']))
                        self.item_model_node_publish.setItem(current_row_count,3,QtGui.QStandardItem(chain_str))
                        gui_logger.log(2, 'chain_str: {0}'.format(chain_str))

                        # set alignment
                        prop_item.setTextAlignment(QtCore.Qt.AlignTop|QtCore.Qt.AlignHCenter)
                        current_row_count += 1
                        topic_count += 1
                        prop_count += 1

                if topic_count > 1:
                    self._tableView_node_publish.setSpan(current_row_count-topic_count, 1, topic_count, 1)

            if prop_count > 1:
                # set output to span rows
                self._tableView_node_publish.setSpan(current_row_count-prop_count, 0, prop_count, 1)

            # save starting row and ending row
            self.prop_node_publish_layout[prop] = [current_row_count-prop_count, current_row_count]

        # resize tableview
        self._tableView_node_publish.horizontalHeader().setStretchLastSection(True)
        self._tableView_node_publish.verticalHeader().setResizeMode(QtGui.QHeaderView.ResizeToContents)

    def clear_all_analysis(self):
        # clear same topic table view
        self.same_topic_table_model = QtGui.QStandardItemModel(0,0) #initialize with 0 rows and 0 columns
        self._tableView_same_topic.setModel(self.same_topic_table_model)

        # clear output to input table view
        output_to_input_table_model = QtGui.QStandardItemModel(0,0) #initialize with 0 rows and 0 columns
        self._tableView_output_to_input.setModel(output_to_input_table_model)

        # clear same node publish table view
        self.item_model_node_publish = QtGui.QStandardItemModel(0,0) #initialize with 0 rows and 0 columns
        self._tableView_node_publish.setModel(self.item_model_node_publish)

        self._textEdit_ltl.clear() # clear ltl suggestions
        self._comboBox_select_exclusion_topic.clear() # clear all options before updating


    def populate_table_same_topic(self):
        # populate table that show same topic
        self.same_topic_table_model = QtGui.QStandardItemModel(0,0) #initialize with 0 rows and 0 columns
        self._tableView_same_topic.setModel(self.same_topic_table_model)

        # clear combo box
        self.same_output_table_layout = {}

        # clear ltl lists
        self.suggested_ltl_list, self.suggested_structured_eng_list = {}, {}

        self.same_topic_table_model.setHorizontalHeaderItem(0, QtGui.QStandardItem("Topic"))
        self.same_topic_table_model.setHorizontalHeaderItem(1, QtGui.QStandardItem("Output"))
        self.same_topic_table_model.setHorizontalHeaderItem(2, QtGui.QStandardItem("Output-to-Topic Chain"))

        self._tableView_same_topic.horizontalHeader().resizeSection(0, 150) # Topic
        self._tableView_same_topic.horizontalHeader().resizeSection(1, 120) # Output

        # check access
        same_output_dict = check_resource_usage.check_possible_concurrent_topic_access(\
            self.output_published_topics['exclude_props'])

        gui_logger.log(1, 'self.published_graph: {0}'.format(self.published_graph))
        #self.suggested_structured_eng_list, self.suggested_ltl_list = {},{}

        gui_logger.log(1, 'same_output_dict:{0}'.format(same_output_dict))

        current_row_count = 0 # track no of rows
        for ros_topic, output_list in same_output_dict.iteritems():
            # remove topic that are in list
            if any(filtered_substring in ros_topic for filtered_substring in self.final_filtered_list):
                continue

            input_count = 0 # trak no of inputs for this output

            # set first column ros_topic (also break topic into parts to show everything in table)
            self.same_topic_table_model.setItem(current_row_count,0,QtGui.QStandardItem(ros_topic.replace('/',' /').replace('_','_ ')))

            # convert two-item-list to a full list
            output_full_list = list(set(list(itertools.chain(*output_list))))
            gui_logger.log(2, 'output_full_list: {0}'.format(output_full_list))

            for output_prop in output_full_list:
                # set second column output_prop prop
                self.same_topic_table_model.setItem(current_row_count,1,QtGui.QStandardItem(output_prop))

                if "action_topics" in ros_topic:
                    prop_to_topic_list = self.chain_topic_node_dotnames_published_dict['exclude_props'][output_prop][self.prop_real_to_dot_name['n'][ros_topic]]
                else:
                    prop_to_topic_list = self.chain_topic_node_dotnames_published_dict['exclude_props'][output_prop][self.prop_real_to_dot_name['t'][ros_topic]]
                chain_str_list = []
                # node and topic
                for prop_dot_name in prop_to_topic_list:
                    if prop_dot_name.startswith('n'): # node
                        chain_str_list.append('('+self.prop_dot_to_real_name[prop_dot_name]+')')
                    else: # topic
                        chain_str_list.append('['+self.prop_dot_to_real_name[prop_dot_name]+']')
                chain_str  = " -> ".join(chain_str_list)

                self.same_topic_table_model.setItem(current_row_count,2,QtGui.QStandardItem(chain_str))
                gui_logger.log(8, 'chain_str: {0}'.format(chain_str))

                current_row_count += 1
                input_count += 1

            # set output to span rows
            self._tableView_same_topic.setSpan(current_row_count-input_count, 0, input_count, 1)

            # save starting row and ending row
            self.same_output_table_layout[ros_topic] = [current_row_count-input_count, current_row_count]

            # fomulate ltl suggestions
            structured_eng_str, ltl_str = self.get_ltl_suggestion(output_full_list)
            self.suggested_structured_eng_list[ros_topic] = structured_eng_str
            self.suggested_ltl_list[ros_topic] = ltl_str
            gui_logger.log(6, 'ltl_str: {0}'.format(ltl_str))

        # resize tableview
        self._tableView_same_topic.horizontalHeader().setStretchLastSection(True)

        # resize tableview #self._tableView_same_topic.horizontalHeader().length()
        #self._tableView_same_topic.setMinimumSize(self._widget.width()-100, \
        #                                          self._tableView_same_topic.verticalHeader().length()+50)
        self._tableView_same_topic.verticalHeader().setResizeMode(QtGui.QHeaderView.ResizeToContents)


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

        #output_to_input_dict = check_resource_usage.check_possible_action_affected_sensors(\
        #    self.input_subscribed_topics['include_props'], self.output_published_topics['include_props'])
        output_to_input_dict = check_resource_usage.check_possible_action_affected_sensors(\
            self.input_prop_to_ros_info, self.output_prop_to_ros_info.keys(), self.prop_real_to_dot_name, \
            self.output_published_dotnames['include_props'])

        current_row_count = 0 # track no of rows
        for output_prop, input_list in output_to_input_dict.iteritems():
            input_count = 0 # trak no of inputs for this output

            # set first column output_prop
            output_to_input_table_model.setItem(current_row_count,0,QtGui.QStandardItem(output_prop))

            for input_prop in input_list:
                # set second column input prop
                output_to_input_table_model.setItem(current_row_count,1,QtGui.QStandardItem(input_prop))

                input_node_name = self.prop_real_to_dot_name['n'][self.input_prop_to_ros_info[input_prop]['node']]
                gui_logger.log(4,self.chain_topic_node_dotnames_published_dict['include_props'][output_prop])
                prop_to_node_list = self.chain_topic_node_dotnames_published_dict['include_props'][output_prop][input_node_name]
                chain_str_list = []
                # node and topic
                for prop_dot_name in prop_to_node_list:
                    if prop_dot_name.startswith('n'): # node
                        chain_str_list.append('('+self.prop_dot_to_real_name[prop_dot_name]+')')
                    else: # topic
                        chain_str_list.append('['+self.prop_dot_to_real_name[prop_dot_name]+']')
                chain_str  = " -> ".join(chain_str_list)

                output_to_input_table_model.setItem(current_row_count,2,QtGui.QStandardItem(chain_str))

                current_row_count += 1
                input_count += 1

            # set output to span rows
            self._tableView_output_to_input.setSpan(current_row_count-input_count, 0, input_count, 1)

        # resize tableview
        self._tableView_output_to_input.horizontalHeader().setStretchLastSection(True)

        # resize tableview # self._tableView_output_to_input.horizontalHeader().length()
        #self._tableView_output_to_input.setMinimumSize(self._widget.width()-100, \
        #                                          self._tableView_output_to_input.verticalHeader().length()+50)
        self._tableView_output_to_input.verticalHeader().setResizeMode(QtGui.QHeaderView.ResizeToContents)

    def load_dot_graph(self, dot_graph):
        # clear all tables
        self.clear_all_analysis()

        self.input_prop_to_ros_info, self.output_prop_to_ros_info =  self.get_mapping_snapshot()

        ##################  #################
        ##### NODES ######  ###### EDGES ####
        ##################  #################
        #first grab all the nodes (both n__ and t__)
        nodes_dict, edges_dict = {}, {}
        for subgraph_name, subgraph in dot_graph.obj_dict['subgraphs'].iteritems():
            #check_resources_logger.debug("subgraph_name: {0}, length of subgraph: {1}".format(subgraph_name, len(subgraph)))
            #check_resources_logger.log(8, "subgraph[0] keys: {0}".format(subgraph[0].keys()))

            # add in nodes
            for node_id, node in subgraph[0]['nodes'].iteritems():
                #if len(node) != 1:
                #    check_resources_logger.warning("Length of nodes is longer than 1!")
                nodes_dict[node_id] = node # to get info should do node[0]
                #else:
                #    nodes_dict[node_id] = node[0]

            # add in edges:
            for edge_id, edge in subgraph[0]['edges'].iteritems():
                edges_dict[edge_id] = edge
                #check_resources_logger.log(6, "edge_id: {0}, edge:{1}".format(edge_id, edge))


        #############################
        #### MORE NODES AND EDGES ###
        #############################
        # also join in obj_dict['nodes']
        nodes_dict.update(dot_graph.obj_dict["nodes"])
        gui_logger.log(2, "Nodes dict:\n {0}".format(str(nodes_dict)))

        #  also join in obj_dict['edges']
        edges_dict.update(dot_graph.obj_dict['edges'])
        gui_logger.log(2,"Edges dict:\n {0}".format(str(edges_dict)))

        #########################
        ## DOT TO REAL MAPPING ##
        #########################
        self.prop_dot_to_real_name, self.prop_real_to_dot_name = \
                check_resource_usage.get_prop_dot_name_real_name_mapping(nodes_dict)

        gui_logger.log(4, "self.prop_dot_to_real_name:\n {0}".format(str(self.prop_dot_to_real_name)))
        gui_logger.log(4, "self.prop_real_to_dot_name:\n {0}".format(str(self.prop_real_to_dot_name)))


        # separate into subscribe topics and publish topics
        self.input_subscribed_topics = {'include_props':{key: [] for key in self.input_prop_to_ros_info.keys()}, \
                                        'exclude_props':{key: [] for key in self.input_prop_to_ros_info.keys()}}
        self.input_published_topics = {'include_props':{key: [] for key in self.input_prop_to_ros_info.keys()}, \
                                       'exclude_props':{key: [] for key in self.input_prop_to_ros_info.keys()}}
        self.output_subscribed_topics = {'include_props':{key: [] for key in self.output_prop_to_ros_info.keys()}, \
                                         'exclude_props':{key: [] for key in self.output_prop_to_ros_info.keys()}}
        self.output_published_topics = {'include_props':{key: [] for key in self.output_prop_to_ros_info.keys()}, \
                                        'exclude_props':{key: [] for key in self.output_prop_to_ros_info.keys()}}
        # for storing dotnames
        self.input_subscribed_dotnames = {'include_props':{key: [] for key in self.input_prop_to_ros_info.keys()}, \
                                          'exclude_props':{key: [] for key in self.input_prop_to_ros_info.keys()}}
        self.input_published_dotnames = {'include_props':{key: [] for key in self.input_prop_to_ros_info.keys()}, \
                                        'exclude_props':{key: [] for key in self.input_prop_to_ros_info.keys()}}
        self.output_subscribed_dotnames = {'include_props':{key: [] for key in self.output_prop_to_ros_info.keys()}, \
                                        'exclude_props':{key: [] for key in self.output_prop_to_ros_info.keys()}}
        self.output_published_dotnames = {'include_props':{key: [] for key in self.output_prop_to_ros_info.keys()}, \
                                        'exclude_props':{key: [] for key in self.output_prop_to_ros_info.keys()}}

        self.chain_topic_node_dotnames_subscribed_dict = {'include_props':{key: {} for key in self.input_prop_to_ros_info.keys()+\
                                                                  self.output_prop_to_ros_info.keys()}, \
                                                          'exclude_props':{key: {} for key in self.input_prop_to_ros_info.keys()+\
                                                                  self.output_prop_to_ros_info.keys()}}
        self.chain_topic_node_dotnames_published_dict = {'include_props':{key: {} for key in self.input_prop_to_ros_info.keys()+\
                                                                  self.output_prop_to_ros_info.keys()}, \
                                                          'exclude_props':{key: {} for key in self.input_prop_to_ros_info.keys()+\
                                                                  self.output_prop_to_ros_info.keys()}}

        self.chain_prop_dot_to_topic_dot_subscribed_dict = {'include_props':{key: {} for key in self.input_prop_to_ros_info.keys()+\
                                                                  self.output_prop_to_ros_info.keys()}, \
                                                          'exclude_props':{key: {} for key in self.input_prop_to_ros_info.keys()+\
                                                                  self.output_prop_to_ros_info.keys()}}
        self.chain_prop_dot_to_topic_dot_published_dict = {'include_props':{key: {} for key in self.input_prop_to_ros_info.keys()+\
                                                                  self.output_prop_to_ros_info.keys()}, \
                                                          'exclude_props':{key: {} for key in self.input_prop_to_ros_info.keys()+\
                                                                  self.output_prop_to_ros_info.keys()}}

        self.published_graph = {}

        ####################################
        # filter some of the common topics #
        ####################################
        topic_filtered_list = ['/clock','/statistics', '/rosout', '/rviz', '/map', '/tf', '/tf_static', '/move_base/cancel']
        partial_topic_filtered_list = ['/rviz']

        # for same topic exclusions
        self.final_filtered_list = ['/parameter_updates', '/parameter_descriptions', '/costmap_updates','/bond', '/camera_info',\
        '/global_costmap','/local_costmap', '/collision_object']


        #modify topic filtered list to filter other proposition nodes
        prop_topic_filtered_list = topic_filtered_list + \
                                    [self.input_prop_to_ros_info[x]['node'] for x in self.input_prop_to_ros_info.keys()] + \
                                    [self.input_prop_to_ros_info[x]['node_publish_topic'] for x in self.input_prop_to_ros_info.keys()] + \
                                    [prop_info['node'] for prop_info_list in self.output_prop_to_ros_info.values() for prop_info in prop_info_list] + \
                                    [prop_info['node_subscribe_topic'] for prop_info_list in self.output_prop_to_ros_info.values() for prop_info in prop_info_list]

        self.output_prop_topic_filtered_list = topic_filtered_list + \
                                    [prop_info['node'] for prop_info_list in self.output_prop_to_ros_info.values() for prop_info in prop_info_list]


        ###############
        ### inputs ####
        ###############
        for input_prop in self.input_prop_to_ros_info.keys():

            # rename prop to the dot file format
            #input_prop_dot_format = "n__"+self.example_name+'_inputs_'+input_prop.replace("/","_")
            input_prop_dot_format = "n__"+"_".join([x for x in self.input_prop_to_ros_info[input_prop]['node'].split("/") if x])
            gui_logger.log(1, "input_prop: {0} to {1}".format(input_prop, input_prop_dot_format))

            # expand dict with prop_dot_format as well
            if input_prop_dot_format not in self.chain_topic_node_dotnames_published_dict['include_props'].keys():
                self.chain_topic_node_dotnames_published_dict['include_props'].update({input_prop_dot_format:{}})
            if input_prop_dot_format not in self.chain_topic_node_dotnames_published_dict['exclude_props'].keys():
                self.chain_topic_node_dotnames_published_dict['exclude_props'].update({input_prop_dot_format:{}})

            # recursive subscribed topics include props
            check_resource_usage.get_subscribed_topics(input_prop_dot_format, [], self.input_subscribed_dotnames['include_props'][input_prop], \
                                  self.input_subscribed_topics['include_props'][input_prop], edges_dict, nodes_dict, \
                                  [input_prop_dot_format], self.chain_topic_node_dotnames_subscribed_dict['include_props'], \
                                  input_prop, self.output_prop_topic_filtered_list, partial_topic_filtered_list)

            # recursive published topics include props
            check_resource_usage.get_published_topics(input_prop_dot_format, [], self.input_published_dotnames['include_props'][input_prop], \
                                 self.input_published_topics['include_props'][input_prop], edges_dict, nodes_dict, \
                                 [input_prop_dot_format], self.chain_topic_node_dotnames_published_dict['include_props'], \
                                 input_prop, input_prop_dot_format, self.output_prop_topic_filtered_list, partial_topic_filtered_list)

            # recursive subscribed topics exclude props
            check_resource_usage.get_subscribed_topics(input_prop_dot_format, [], self.input_subscribed_dotnames['exclude_props'][input_prop], \
                                  self.input_subscribed_topics['exclude_props'][input_prop], edges_dict, nodes_dict, \
                                  [input_prop_dot_format], self.chain_topic_node_dotnames_subscribed_dict['exclude_props'], \
                                  input_prop, prop_topic_filtered_list, partial_topic_filtered_list)

            # recursive published topics exclude props
            check_resource_usage.get_published_topics(input_prop_dot_format, [], self.input_published_dotnames['exclude_props'][input_prop], \
                                 self.input_published_topics['exclude_props'][input_prop], edges_dict, nodes_dict, \
                                 [input_prop_dot_format], self.chain_topic_node_dotnames_published_dict['exclude_props'], \
                                 input_prop, input_prop_dot_format, prop_topic_filtered_list, partial_topic_filtered_list)

        gui_logger.log(4, "self.input_subscribed_topics: {0}".format(str(self.input_subscribed_topics)))
        gui_logger.log(4, "self.input_published_topics: {0}".format(str(self.input_published_topics)))

        #rospy.loginfo("Published topics - bedroom_rc: {0}".format(self.input_published_topics['bedroom_rc']))

        ###############
        ### outputs ###
        ###############
        for output_prop in self.output_prop_to_ros_info.keys():

            # first check if it's a list
            output_prop_dot_format_list = []
            if isinstance(self.output_prop_to_ros_info[output_prop], list):
                for prop_info in self.output_prop_to_ros_info[output_prop]:
                    # rename prop to the dot file format
                    output_prop_dot_format_list.append("n__"+"_".join([x for x in prop_info['node'].split("/") if x]))
            else:
                # rename prop to the dot file format
                output_prop_dot_format_list.append("n__"+"_".join([x for x in self.output_prop_to_ros_info[output_prop]['node'].split("/") if x]))

            self.chain_prop_dot_to_topic_dot_subscribed_dict['exclude_props'].update(\
                dict(((output_prop_dot_format, {}) for output_prop_dot_format in output_prop_dot_format_list)))
            self.chain_prop_dot_to_topic_dot_published_dict['exclude_props'].update(\
                dict(((output_prop_dot_format, {}) for output_prop_dot_format in output_prop_dot_format_list)))
            self.chain_prop_dot_to_topic_dot_subscribed_dict['include_props'].update(\
                dict(((output_prop_dot_format, {}) for output_prop_dot_format in output_prop_dot_format_list)))
            self.chain_prop_dot_to_topic_dot_published_dict['include_props'].update(\
                dict(((output_prop_dot_format, {}) for output_prop_dot_format in output_prop_dot_format_list)))

            gui_logger.log(1, "output_prop: {0} to {1}".format(output_prop, output_prop_dot_format_list))

            for output_prop_dot_format in output_prop_dot_format_list:
                # expand dict with prop_dot_format as well
                if output_prop_dot_format not in self.chain_topic_node_dotnames_published_dict['include_props'].keys():
                    self.chain_topic_node_dotnames_published_dict['include_props'].update({output_prop_dot_format:{}})
                if output_prop_dot_format not in self.chain_topic_node_dotnames_published_dict['exclude_props'].keys():
                    self.chain_topic_node_dotnames_published_dict['exclude_props'].update({output_prop_dot_format:{}})

                temp_output_subscribed_dotnames_include_props = []
                temp_output_published_dotnames_include_props = []
                temp_output_subscribed_topics_include_props = []
                temp_output_published_topics_include_props = []
                temp_output_subscribed_dotnames_exclude_props = []
                temp_output_published_dotnames_exclude_props = []
                temp_output_subscribed_topics_exclude_props = []
                temp_output_published_topics_exclude_props = []

                # recursive subscribed topics include props
                check_resource_usage.get_subscribed_topics(output_prop_dot_format, [], temp_output_subscribed_dotnames_include_props, \
                                      temp_output_subscribed_topics_include_props, edges_dict, nodes_dict, \
                                      [output_prop_dot_format], self.chain_topic_node_dotnames_subscribed_dict['include_props'], \
                                      output_prop, [x for x in self.output_prop_topic_filtered_list if x != output_prop], partial_topic_filtered_list,\
                                      self.chain_prop_dot_to_topic_dot_subscribed_dict['include_props'])

                # recursive published topics include props
                check_resource_usage.get_published_topics(output_prop_dot_format, [], temp_output_published_dotnames_include_props, \
                                     temp_output_published_topics_include_props, edges_dict, nodes_dict, \
                                     [output_prop_dot_format], self.chain_topic_node_dotnames_published_dict['include_props'], \
                                     output_prop, output_prop_dot_format, [x for x in self.output_prop_topic_filtered_list if x != output_prop], partial_topic_filtered_list,\
                                     self.chain_prop_dot_to_topic_dot_published_dict['include_props'])

                # recursive subscribed topics exclude props
                check_resource_usage.get_subscribed_topics(output_prop_dot_format, [], temp_output_subscribed_dotnames_exclude_props, \
                                      temp_output_subscribed_topics_exclude_props, edges_dict, nodes_dict, \
                                      [output_prop_dot_format], self.chain_topic_node_dotnames_subscribed_dict['exclude_props'], \
                                      output_prop, prop_topic_filtered_list, partial_topic_filtered_list,\
                                      self.chain_prop_dot_to_topic_dot_subscribed_dict['include_props'])

                # recursive published topics exclude props
                check_resource_usage.get_published_topics(output_prop_dot_format, [], temp_output_published_dotnames_exclude_props, \
                                     temp_output_published_topics_exclude_props, edges_dict, nodes_dict, \
                                     [output_prop_dot_format], self.chain_topic_node_dotnames_published_dict['exclude_props'], \
                                     output_prop, output_prop_dot_format, prop_topic_filtered_list, partial_topic_filtered_list,\
                                     self.chain_prop_dot_to_topic_dot_published_dict['exclude_props'])

                self.output_subscribed_dotnames['include_props'][output_prop] += copy.deepcopy(temp_output_subscribed_dotnames_include_props)
                self.output_subscribed_dotnames['exclude_props'][output_prop] += copy.deepcopy(temp_output_subscribed_dotnames_exclude_props)
                self.output_published_dotnames['include_props'][output_prop] += copy.deepcopy(temp_output_published_dotnames_include_props)
                self.output_published_dotnames['exclude_props'][output_prop] += copy.deepcopy(temp_output_published_dotnames_exclude_props)
                self.output_subscribed_topics['include_props'][output_prop] += copy.deepcopy(temp_output_subscribed_topics_include_props)
                self.output_subscribed_topics['exclude_props'][output_prop] += copy.deepcopy(temp_output_subscribed_topics_exclude_props)
                self.output_published_topics['include_props'][output_prop] += copy.deepcopy(temp_output_published_topics_include_props)
                self.output_published_topics['exclude_props'][output_prop] += copy.deepcopy(temp_output_published_topics_exclude_props)


            #check_resource_usage.get_published_graph(output_prop_dot_format, self.output_subscribed_dotnames[output_prop], \
            #                        self.published_graph, edges_dict, nodes_dict, topic_filtered_list)

        #rospy.loginfo("Published topics - bedroom: {0}".format(self.output_published_topics['bedroom']))

        #gui_logger.log(4, 'self.chain_topic_node_dotnames_subscribed_dict: {0}'.format(self.chain_topic_node_dotnames_subscribed_dict))
        #gui_logger.log(4, 'self.chain_topic_node_dotnames_published_dict: {0}'.format(self.chain_topic_node_dotnames_published_dict))


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

        # change yaml directory in gui
        self._lineEdit_load_yaml_mapping.setText(yaml_file)

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
            self.write_prop_list_to_yaml(yamlHandle, prop)


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

    def write_prop_list_to_yaml(self, yamlHandle, prop):
        # current prop
        yamlHandle.write('  {prop}:\n'.format(prop=prop))

        for row in range(self._prop_gridLayout[prop].rowCount()):
            item_node = self._prop_gridLayout[prop].itemAtPosition(row,0)
            item_topic = self._prop_gridLayout[prop].itemAtPosition(row,1)
            if item_node and item_topic:
                #node text box
                yamlHandle.write('    -\n')
                yamlHandle.write('      node: {0}\n'.format(item_node.widget().text()))

                #topic for that prop
                if prop in self._input_props:
                    gui_logger.warning('write_prop_list_to_yaml: getting input props!')
                else:
                    yamlHandle.write('      node_subscribe_topic: {0}\n\n'.format(item_topic.widget().text()))

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
        # clear output prop grid
        for grid in self._prop_gridLayout.values():
            self.clear_grid(grid)

        # clear output grid
        self.clear_grid(self._gridLayout_output)
        del self._gridLayout_output

        self._gridLayout_output = QtGui.QGridLayout()
        self._gridLayout_mapping.addLayout(self._gridLayout_output, 3, 0, 1, 0)
        gui_logger.log(2, "rowCount:{0}, columnCount: {1}".format(self._gridLayout_output.rowCount(), self._gridLayout_input.columnCount()))

        # for outputs
        for output_idx, output_prop in enumerate(self._output_props):
            self._prop_node[output_prop] = QtGui.QComboBox()
            self._prop_topic[output_prop] = QtGui.QComboBox()
            self._prop_add_button[output_prop] = QtGui.QPushButton('+')
            self._prop_label[output_prop] = QtGui.QLabel(output_prop)
            self._prop_mapping_count[output_prop] = 0
            self._prop_gridLayout[output_prop] = QtGui.QGridLayout()

            # fill up topic combo box after node combo box is selected
            self.connect(self._prop_node[output_prop], QtCore.SIGNAL("activated(const QString&)"),\
                         partial(self.refresh_topics, "output", output_prop, self._prop_node[output_prop]))

            # add topic when + button is clicked
            self.connect(self._prop_add_button[output_prop], QtCore.SIGNAL("clicked()"),\
                         partial(self.on_pushButton_add_mapping, "output", output_prop, self._prop_label[output_prop]))

            # set to fixed size
            self._prop_node[output_prop].setMinimumHeight(23)
            self._prop_topic[output_prop].setMinimumHeight(23)

            self._gridLayout_output.addWidget(self._prop_label[output_prop], 2*output_idx, 0, QtCore.Qt.AlignTop) # label
            self._gridLayout_output.addWidget(self._prop_add_button[output_prop], 2*output_idx, 1, QtCore.Qt.AlignTop) # add mapping
            self._gridLayout_output.addLayout(self._prop_gridLayout[output_prop], 2*output_idx, 2, 1, 1) # add mapping
            self._gridLayout_output.setRowMinimumHeight(output_idx, 23)

            # add line
            line = QtGui.QFrame()
            line.setFrameShape(QtGui.QFrame.HLine)
            self._gridLayout_output.addWidget(line, 2*output_idx+1, 0, 1, 3)

            # set column stretch (prop_gridlayout)
            col_stretch_list = [5,5,0]
            for idx, stretch in enumerate(col_stretch_list):
                self._prop_gridLayout[output_prop].setColumnStretch(idx, stretch)

        # set column stretch (_gridLayout_output)
        col_stretch_list = [4,0,10]
        for idx, stretch in enumerate(col_stretch_list):
            self._gridLayout_output.setColumnStretch(idx, stretch)

        gui_logger.log(2, "rowCount:{0}, columnCount: {1}".format(self._gridLayout_output.rowCount(), self._gridLayout_output.columnCount()))

        # adjust the size of the scroll area !!!
        self._scrollAreaWidgetContents_Mapping.setMinimumSize(528, (self._gridLayout_input.rowCount() + self._gridLayout_output.rowCount()+2)*40)
        self._scrollAreaWidgetContents_Mapping.setMaximumSize(1000000, 1000000)


        self._gridLayout_mapping.setRowStretch(1, self._gridLayout_input.rowCount())
        self._gridLayout_mapping.setRowStretch(4, self._gridLayout_output.rowCount())

    def clear_grid(self, qt_grid_obj):
        # clear output prop grid
        for i in reversed(range(qt_grid_obj.count())):
            if isinstance(qt_grid_obj.itemAt(i), QtGui.QWidgetItem): # widget
                # remove it from the layout list
                widgetToRemove = qt_grid_obj.itemAt(i).widget()
                qt_grid_obj.removeWidget(widgetToRemove)
                # remove it from the gui
                widgetToRemove.setParent(None)
                widgetToRemove.deleteLater()

            else: # layout
                layout = qt_grid_obj.itemAt(i)
                del layout
        #del qt_grid_obj

    def on_pushButton_add_mapping(self, prop_type, prop, qt_label_obj):

        if prop_type == 'input':
            pass
        else:
            ok_button = QtGui.QPushButton('ok')

            # find label location
            self._prop_gridLayout[prop].addWidget(self._prop_node[prop], self._prop_mapping_count[prop], 0) # node
            self._prop_gridLayout[prop].addWidget(self._prop_topic[prop], self._prop_mapping_count[prop], 1)  # topic
            self._prop_gridLayout[prop].addWidget(ok_button, self._prop_mapping_count[prop], 2) # delete mapping
            self._prop_gridLayout[prop].addWidget(ok_button, self._prop_mapping_count[prop], 2) # delete mapping
            self._prop_gridLayout[prop].setRowMinimumHeight(self._prop_mapping_count[prop], 23)

            # change grid size
            self._gridLayout_mapping.setRowStretch(4, self._gridLayout_output.rowStretch(4)+1)
            self._gridLayout_mapping.update()

            # show combo boxes
            self._prop_node[prop].show()
            self._prop_topic[prop].show()

            # connect ok button
            self.connect(ok_button, QtCore.SIGNAL("clicked()"),\
                         partial(self.on_pushButton_confirm_mapping, prop, ok_button))

            # distable add mapping temporarily
            self._prop_add_button[prop].setEnabled(False)

    def on_pushButton_confirm_mapping(self, prop, ok_button):

        # increment mapping count
        self._prop_mapping_count[prop] += 1

         # find button location
        qt_button_idx = self._prop_gridLayout[prop].indexOf(ok_button)
        qt_button_location = self._prop_gridLayout[prop].getItemPosition(qt_button_idx) # row, col, rowspan, colspan
        idx = qt_button_location[0]

        # set mapping to static text
        node_label = QtGui.QLabel(self._prop_node[prop].currentText())
        topic_label = QtGui.QLabel(self._prop_topic[prop].currentText())
        delete_button = QtGui.QPushButton('Delete')

        # remove current widget
        self._prop_gridLayout[prop].removeWidget(self._prop_node[prop])
        self._prop_gridLayout[prop].removeWidget(self._prop_topic[prop])
        self._prop_gridLayout[prop].removeWidget(ok_button)
        ok_button.setParent(None)
        ok_button.deleteLater()

        # hide combo boxes
        self._prop_node[prop].hide()
        self._prop_topic[prop].hide()

        # add widget
        self._prop_gridLayout[prop].addWidget(node_label, idx, 0) # node
        self._prop_gridLayout[prop].addWidget(topic_label, idx, 1)  # topic
        self._prop_gridLayout[prop].addWidget(delete_button, idx, 2) # delete mapping

        # connect delete button
        self.connect(delete_button, QtCore.SIGNAL("clicked()"),\
                     partial(self.on_pushButton_delete_mapping, prop, delete_button))

        # distable add mapping temporarily
        self._prop_add_button[prop].setEnabled(True)

    def on_pushButton_delete_mapping(self, prop, delete_button):
        #decrement mapping count
        self._prop_mapping_count[prop] -= 1

        # find button location
        qt_button_idx = self._prop_gridLayout[prop].indexOf(delete_button)
        qt_button_location = self._prop_gridLayout[prop].getItemPosition(qt_button_idx) # row, col, rowspan, colspan
        idx = qt_button_location[0]

        # remove mapping
        for col_idx in [0,1,2]:
            widgetToRemove = self._prop_gridLayout[prop].itemAtPosition(idx,col_idx).widget()
            # remove it from the layout list
            self._prop_gridLayout[prop].removeWidget(widgetToRemove)
            # remove it from the gui
            widgetToRemove.setParent(None)
            widgetToRemove.deleteLater()

        self._gridLayout_mapping.setRowStretch(4, self._gridLayout_output.rowStretch(4)-1)


    def refresh_nodes_and_topics(self):
        # first update nodes
        nodes_list = rosnode.get_node_names()

        # filter nodes that does not contain boolean topics
        nodes_input_list, nodes_output_list = [], []
        for node_name in nodes_list:
            if self.check_if_boolean_topic_in_node('input', node_name)[0]:
                nodes_input_list.append(node_name)
            if self.check_if_boolean_topic_in_node('output', node_name)[0]:
                nodes_output_list.append(node_name)

        # populate combo box
        for prop, node_combobox in self._prop_node.iteritems():

            # save previous node
            current_node = node_combobox.currentText()

            node_combobox.clear() # clear all options before updating

            # check the type of prop
            if prop in self._input_props:
                prop_type = "input"
                node_combobox.addItems(nodes_input_list)

                # Add in permanent items from loading mapping
                if prop in self._mapping_file_node and \
                self._mapping_file_node[prop] not in nodes_list:
                    node_combobox.addItem(self._mapping_file_node[prop])

                # Restore to old selected node
                if current_node in [node_combobox.itemText(i) for i in range(node_combobox.count())]:
                    node_combobox.setCurrentIndex(node_combobox.findText(current_node))

            else:
                prop_type = "output"
                node_combobox.addItems(nodes_output_list)

            # now update topics
            self.refresh_topics(prop_type, prop, node_combobox)

        rospy.loginfo("Refreshed nodes and topics.")

    def check_if_boolean_topic_in_node(self, prop_type, node_name):
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

        return True if topic_list else False, topic_list


    def refresh_topics(self, prop_type, prop, node_object):
        node_name = node_object.currentText()

        # get topic_list
        _, topic_list = self.check_if_boolean_topic_in_node(prop_type, node_name)

        # save previous topic
        current_topic = self._prop_topic[prop].currentText()

        self._prop_topic[prop].clear()  # clear all options before updating
        self._prop_topic[prop].addItems(topic_list)

        if prop in self._input_props:
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
        # clear output prop grid
        for grid in self._prop_gridLayout.values():
            self.clear_grid(grid)
        for output_idx, output_prop in enumerate(self._output_props):
            self._prop_mapping_count[output_prop] = 0
            self._prop_gridLayout[output_prop] = QtGui.QGridLayout()
            self._gridLayout_output.addLayout(self._prop_gridLayout[output_prop], 2*output_idx, 2, 1, 1) # add mapping

        for output_prop in self.output_prop_to_ros_info:
            if output_prop in self._output_props: # first check if we have such propositions
                self._mapping_file_node[output_prop], self._mapping_file_topic[output_prop] = [], []

                if not isinstance(self.output_prop_to_ros_info[output_prop], list):
                    prop_info_list = [self.output_prop_to_ros_info[output_prop]]
                else:
                    prop_info_list = self.output_prop_to_ros_info[output_prop]

                for prop_info in prop_info_list:

                    # add this permanently to the QComboBox everytime it's refreshed.
                    self._mapping_file_node[output_prop].append(prop_info['node'])

                    # add this permanently to the QComboBox everytime it's refreshed.
                    self._mapping_file_topic[output_prop].append(prop_info['node_subscribe_topic'])

                    # add to grid layout
                    delete_button = QtGui.QPushButton('Delete')
                    self._prop_gridLayout[output_prop].addWidget(QtGui.QLabel(prop_info['node']), self._prop_mapping_count[output_prop], 0)
                    self._prop_gridLayout[output_prop].addWidget(QtGui.QLabel(prop_info['node_subscribe_topic']), self._prop_mapping_count[output_prop], 1)
                    self._prop_gridLayout[output_prop].addWidget(delete_button, self._prop_mapping_count[output_prop], 2)
                    self._prop_mapping_count[output_prop] += 1
                    self._prop_add_button[output_prop].setEnabled(True)

                    # connect delete button
                    self.connect(delete_button, QtCore.SIGNAL("clicked()"),\
                                 partial(self.on_pushButton_delete_mapping, output_prop, delete_button))


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
            if line and mode == "input" and not line.startswith('#'):
                self._input_props.append(line)
            elif line and mode == "output" and not line.startswith('#'):
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

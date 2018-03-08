import logging

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
import python_qt_binding.QtGui as QtGui
import python_qt_binding.QtCore as QtCore

import check_resource_usage

import rqt_grounding_and_analysis_logging
analysis_multirobot_logger = logging.getLogger("analysis_multirobot_logger")

HIGHLIGHT_COLOR = QtGui.QColor(255,204,204) #QtCore.Qt.red

class ExtensionAnalysisMultiRobot(Plugin):

    def __init__(self, context):
        super(ExtensionAnalysisMultiRobot, self).__init__(context)

    def setup_extension(self):
        # enable switch tab
        self._stackedWidget_analysis = self._widget.findChild(QtGui.QStackedWidget, name="stackedWidget_analysis")
        self._stackedWidget_analysis.setCurrentIndex(0)

        # connect callback for pushButton switch to multiple
        self._pushButton_switch_to_multiple = self._widget.findChild(QtGui.QPushButton, name="pushButton_switch_to_multiple")
        self._pushButton_switch_to_multiple.clicked.connect(self.on_pushButton_switch_to_multiple_clicked)

        # connect callback for pushButton switch to single
        self._pushButton_switch_to_single = self._widget.findChild(QtGui.QPushButton, name="pushButton_switch_to_single")
        self._pushButton_switch_to_single.clicked.connect(self.on_pushButton_switch_to_single_clicked)

        # table for topic to left behind robots
        self._tableView_topics_left_behind = self._widget.findChild(QtGui.QTableView, name="tableView_topics_left_behind")
        self._tableView_topics_left_behind.verticalHeader().setVisible(False)
        self._tableView_topics_left_behind.setSelectionMode(QtGui.QAbstractItemView.MultiSelection)   # set selection mode

        # table for prop to left behind robots
        self._tableView_props_left_behind = self._widget.findChild(QtGui.QTableView, name="tableView_props_left_behind")
        self._tableView_props_left_behind.verticalHeader().setVisible(False)
        self._tableView_props_left_behind.setSelectionMode(QtGui.QAbstractItemView.MultiSelection)   # set selection mode

        # connect callback for pushButton_analyz_multiple
        self._pushButton_analyz_multiple = self._widget.findChild(QtGui.QPushButton, name="pushButton_analyz_multiple")
        self._pushButton_analyz_multiple.clicked.connect(self.on_pushButton_analyz_multiple_clicked)

        # line edits
        self._lineEdit_robots = self._widget.findChild(QtGui.QLineEdit, name="lineEdit_robots")
        self._lineEdit_robot_sensor_topics = self._widget.findChild(QtGui.QLineEdit, name="lineEdit_robot_sensor_topics")

        # initialize dict
        self.topic_missing_robot_table_layout = {}
        self.prop_missing_robot_table_layout = {}
        self.robot_sensor_topic_list = []

    def on_pushButton_switch_to_multiple_clicked(self):
        self._stackedWidget_analysis.setCurrentIndex(1)

    def on_pushButton_switch_to_single_clicked(self):
        self._stackedWidget_analysis.setCurrentIndex(0)

    def on_pushButton_analyz_multiple_clicked(self):
        # read robot list
        robot_list = [x.replace(' ','').encode("ascii") for x in self._lineEdit_robots.text().strip(',').split(',')]

        # read robot sensor topic list
        self.robot_sensor_topic_list = [x.replace(' ','').encode("ascii") for x in self._lineEdit_robot_sensor_topics.text().strip(',').split(',')]

        if len(robot_list)>1:
            # analyze
            analysis_multirobot_logger.log(4, 'self.output_published_topics["include_props"]:{0}'.format(self.output_published_topics['include_props']))
            analysis_multirobot_logger.log(4, 'self.output_prop_topic_filtered_list:{0}'.format(self.output_prop_topic_filtered_list))
            analysis_multirobot_logger.log(4, 'self.final_filtered_list:{0}'.format(self.final_filtered_list))
            analysis_multirobot_logger.log(4, 'self.robot_sensor_topic_list:{0}'.format(self.robot_sensor_topic_list))
            analysis_multirobot_logger.log(4, 'robot_list:{0}'.format(robot_list))

            props_to_left_behind_robots, topic_to_left_behind_robots = \
                check_resource_usage.check_left_behind_robot_per_prop(self.output_published_topics['include_props'], \
                robot_list, self.output_prop_topic_filtered_list+self.final_filtered_list+self.robot_sensor_topic_list)

            analysis_multirobot_logger.log(8, 'topic_to_left_behind_robots:{0}'.format(topic_to_left_behind_robots))
            analysis_multirobot_logger.log(8, 'props_to_left_behind_robots:{0}'.format(props_to_left_behind_robots))

            # populate tables
            self.populate_topic_to_left_behind_robots(topic_to_left_behind_robots, robot_list)
            self.populate_prop_to_left_behind_robots(props_to_left_behind_robots, robot_list)

        else:
            msg = QtGui.QMessageBox()
            msg.setIcon(QtGui.QMessageBox.Warning)
            msg.setText("Please list at least two robots here.")
            msg.setWindowTitle("Warning!")
            msg.exec_()

    def find_prop_to_topic_chain(self, prop, ros_full_topic, chain_dict, exclude_props=True):
        chain_str = None

        if exclude_props:
            mode = 'exclude_props'
        else:
            mode = 'include_props'

        if "action_topics" in ros_full_topic:
            topic_dot_name = self.prop_real_to_dot_name['n'][ros_full_topic]
        else:
            topic_dot_name = self.prop_real_to_dot_name['t'][ros_full_topic]

        if topic_dot_name in chain_dict[mode][prop].keys():
            # find a random chain here?
            prop_to_topic_list = chain_dict[mode][prop][topic_dot_name]
            chain_str_list = []
            # node and topic
            for prop_dot_name in prop_to_topic_list:
                if prop_dot_name.startswith('n'): # node
                    chain_str_list.append('('+self.prop_dot_to_real_name[prop_dot_name]+')')
                else: # topic
                    chain_str_list.append('['+self.prop_dot_to_real_name[prop_dot_name]+']')
            chain_str  = " -> ".join(chain_str_list)
            analysis_multirobot_logger.log(6, 'chain_str: {0}'.format(chain_str))
        else:
            analysis_multirobot_logger.log(8, " Can't find {topic_dot_name} in {chain_dict}[{mode}][{prop}]".format(\
                    topic_dot_name=topic_dot_name, mode=mode, prop=prop, chain_dict=chain_dict))

        return chain_str

    def populate_topic_to_left_behind_robots(self, topic_to_left_behind_robots, robot_list):

        # populate table that show same topic
        self.item_model_topic_missing_robot = QtGui.QStandardItemModel(0,0) #initialize with 0 rows and 0 columns
        self._tableView_topics_left_behind.setModel(self.item_model_topic_missing_robot)

        # clear combo box
        self.same_output_table_layout = {}

        self.item_model_topic_missing_robot.setHorizontalHeaderItem(0, QtGui.QStandardItem("Topic"))
        self.item_model_topic_missing_robot.setHorizontalHeaderItem(1, QtGui.QStandardItem("Robots"))
        self.item_model_topic_missing_robot.setHorizontalHeaderItem(2, QtGui.QStandardItem("One existing Chain"))

        self._tableView_topics_left_behind.horizontalHeader().resizeSection(0, 150) # Topic
        self._tableView_topics_left_behind.horizontalHeader().resizeSection(1, 120) # Robots

        current_row_count = 0 # track no of rows
        for ros_topic, robot_info in topic_to_left_behind_robots.iteritems():

            input_count = 0 # trak no of inputs for this output

            # set first column ros_topic (also break topic into parts to show everything in table)
            self.item_model_topic_missing_robot.setItem(current_row_count,0,QtGui.QStandardItem('/'+ros_topic.replace('/',' /').replace('_','_ ')))

            # list robots
            for robot_name in robot_list:
                self.item_model_topic_missing_robot.setItem(current_row_count,1,QtGui.QStandardItem(robot_name))

                # check if this is the missing robot
                if robot_name not in robot_info['missing']:

                    # find a random chain
                    for output_prop in self.output_prop_to_ros_info.keys():

                        analysis_multirobot_logger.log(2, 'self.chain_topic_node_dotnames_published_dict["exclude_props"]: {0}'.format(\
                                                        self.chain_topic_node_dotnames_published_dict['exclude_props']))

                        # find the actual topic. ros_topic is just the last chunk
                        ros_full_topic = next(x for x in robot_info['topics'] if robot_name in x)
                        analysis_multirobot_logger.log(2, 'ros_full_topic: {0}'.format(ros_full_topic))

                        chain_str = self.find_prop_to_topic_chain(output_prop, ros_full_topic, self.chain_topic_node_dotnames_published_dict)
                        if chain_str:
                            self.item_model_topic_missing_robot.setItem(current_row_count,2,QtGui.QStandardItem(chain_str))
                            break
                else:
                    #change cell color
                    self.item_model_topic_missing_robot.setItem(current_row_count, 2, QtGui.QStandardItem())
                    self.item_model_topic_missing_robot.item(current_row_count, 1).setBackground(HIGHLIGHT_COLOR)
                    self.item_model_topic_missing_robot.item(current_row_count, 2).setBackground(HIGHLIGHT_COLOR)

                current_row_count += 1
                input_count += 1

            # set output to span rows
            self._tableView_topics_left_behind.setSpan(current_row_count-input_count, 0, input_count, 1)

            # save starting row and ending row
            self.topic_missing_robot_table_layout[ros_topic] = [current_row_count-input_count, current_row_count]

        # resize tableview
        self._tableView_topics_left_behind.horizontalHeader().setStretchLastSection(True)
        self._tableView_topics_left_behind.verticalHeader().setResizeMode(QtGui.QHeaderView.ResizeToContents)


    def populate_prop_to_left_behind_robots(self, prop_to_left_behind_robots, robot_list):

        # populate table that show same topic
        self.item_model_prop_missing_robot = QtGui.QStandardItemModel(0,0) #initialize with 0 rows and 0 columns
        self._tableView_props_left_behind.setModel(self.item_model_prop_missing_robot)

        # clear combo box
        self.same_output_table_layout = {}

        self.item_model_prop_missing_robot.setHorizontalHeaderItem(0, QtGui.QStandardItem("Output Proposition"))
        self.item_model_prop_missing_robot.setHorizontalHeaderItem(1, QtGui.QStandardItem("Topic"))
        self.item_model_prop_missing_robot.setHorizontalHeaderItem(2, QtGui.QStandardItem("Robots"))
        self.item_model_prop_missing_robot.setHorizontalHeaderItem(3, QtGui.QStandardItem("One existing Chain"))

        self._tableView_props_left_behind.horizontalHeader().resizeSection(0, 150) # Prop
        self._tableView_props_left_behind.horizontalHeader().resizeSection(1, 150) # Topic
        self._tableView_props_left_behind.horizontalHeader().resizeSection(2, 120) # Robots

        current_row_count = 0 # track no of rows

        for prop, prop_info in prop_to_left_behind_robots.iteritems():

            prop_count = 0

            # set first column to prop (also break topic into parts to show everything in table)
            self.item_model_prop_missing_robot.setItem(current_row_count,0,QtGui.QStandardItem(prop))

            for ros_topic, robot_info in prop_info.iteritems():

                topic_count = 0 # trak no of inputs for this output

                # set first column ros_topic (also break topic into parts to show everything in table)
                self.item_model_prop_missing_robot.setItem(current_row_count,1,QtGui.QStandardItem('/'+ros_topic.replace('/',' /').replace('_','_ ')))

                # list robots
                for robot_name in robot_list:
                    self.item_model_prop_missing_robot.setItem(current_row_count,2,QtGui.QStandardItem(robot_name))

                    # check if this is the missing robot
                    if robot_name not in robot_info['missing']:

                        # find a random chain
                        for output_prop in self.output_prop_to_ros_info.keys():

                            # find the actual topic. ros_topic is just the last chunk
                            ros_full_topic = next(x for x in robot_info['topics'] if robot_name in x)

                            chain_str = self.find_prop_to_topic_chain(output_prop, ros_full_topic, self.chain_topic_node_dotnames_published_dict)
                            if chain_str:
                                self.item_model_prop_missing_robot.setItem(current_row_count,3,QtGui.QStandardItem(chain_str))
                                break
                    else:
                        #change cell color
                        self.item_model_prop_missing_robot.setItem(current_row_count, 3, QtGui.QStandardItem())
                        self.item_model_prop_missing_robot.item(current_row_count, 2).setBackground(HIGHLIGHT_COLOR)
                        self.item_model_prop_missing_robot.item(current_row_count, 3).setBackground(HIGHLIGHT_COLOR)

                    current_row_count += 1
                    topic_count += 1
                    prop_count += 1

                # set output to span rows
                self._tableView_props_left_behind.setSpan(current_row_count-prop_count, 0, prop_count, 1)
                self._tableView_props_left_behind.setSpan(current_row_count-topic_count, 1, topic_count, 1)

                # save starting row and ending row
                self.prop_missing_robot_table_layout[ros_topic] = [current_row_count-prop_count, current_row_count]

        # resize tableview
        self._tableView_props_left_behind.horizontalHeader().setStretchLastSection(True)
        self._tableView_props_left_behind.verticalHeader().setResizeMode(QtGui.QHeaderView.ResizeToContents)


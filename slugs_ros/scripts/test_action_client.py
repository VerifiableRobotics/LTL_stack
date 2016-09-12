#! /usr/bin/env python

import rospy
import actionlib
import time
import argparse

import slugs_ros.msg, slugs_ros.srv

name = '/test_Slugs_action'

def feedback_fn(msg):
    print '-------------------------------------'
    print "We are getting feedback:\n" + str(msg)
    print '-------------------------------------'

def slugs_init_execution_string_client(inputs):
    rospy.wait_for_service(name+"/slugs_init_execution_string_service")
    try:
        slugs_init_execution = rospy.ServiceProxy(name+"/slugs_init_execution_string_service", slugs_ros.srv.SlugsInitExecutionString)
        resp1 = slugs_init_execution(inputs)
        return resp1.current_inputs_outputs
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def slugs_init_execution_array_client(key_list, value_list):
    request = slugs_ros.srv.SlugsInitExecutionArrayRequest()
    print request
    request.init_inputs_outputs_key_array = key_list
    request.init_inputs_outputs_value_array = value_list

    rospy.wait_for_service(name+"/slugs_init_execution_array_service")
    try:
        slugs_init_execution = rospy.ServiceProxy(name+"/slugs_init_execution_array_service", slugs_ros.srv.SlugsInitExecutionArray)
        resp1 = slugs_init_execution(request)
        return resp1.current_inputs_outputs_key_array, resp1.current_inputs_outputs_value_array
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def slugs_trans_execution_string_client(inputs):
    rospy.wait_for_service(name+"/slugs_trans_execution_string_service")
    try:
        slugs_trans_execution = rospy.ServiceProxy(name+"/slugs_trans_execution_string_service", slugs_ros.srv.SlugsTransExecutionString)
        resp1 = slugs_trans_execution(inputs)
        return resp1.current_inputs_outputs
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def slugs_trans_execution_array_client(key_list, value_list):
    request = slugs_ros.srv.SlugsTransExecutionArrayRequest()
    request.trans_inputs_key_array = key_list
    request.trans_inputs_value_array = value_list

    rospy.wait_for_service(name+"/slugs_trans_execution_array_service")
    try:
        slugs_trans_execution = rospy.ServiceProxy(name+"/slugs_trans_execution_array_service", slugs_ros.srv.SlugsTransExecutionArray)
        resp1 = slugs_trans_execution(request)
        return resp1.current_inputs_outputs_key_array, resp1.current_inputs_outputs_value_array
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def slugs_set_goal_client(inputs):
    rospy.wait_for_service(name+"/slugs_set_goal_service")
    try:
        slugs_trans_execution = rospy.ServiceProxy(name+"/slugs_set_goal_service", slugs_ros.srv.SlugsSetGoal)
        resp1 = slugs_trans_execution(inputs)
        return resp1.current_goal_id
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Trajectory Client.")
    parser.add_argument('ltl_filename', type=str, help='Specify .slugsin for now')

    args = parser.parse_args()

    rospy.init_node('test_action_client')
    client = actionlib.SimpleActionClient(name+"/slugs_synthesis_action", slugs_ros.msg.SlugsSynthesisAction)
    client.wait_for_server()

    goal = slugs_ros.msg.SlugsSynthesisGoal()
    goal.ltl_filename = args.ltl_filename
    #goal.output_filename = args.ltl_filename.replace('.slugsin','.aut')
    goal.options = ["--interactiveStrategy"]


    # Fill in the goal here
    client.send_goal(goal,feedback_cb=feedback_fn)
    print client.get_goal_status_text()
    client.wait_for_result()#rospy.Duration.from_sec(5.0))
    result = client.get_result()
    print result

    while not rospy.is_shutdown():
        ### test service #####
        print  "Init_state_string:" + slugs_init_execution_string_client("")

        init_key = ['person']
        init_value = [True]
        key, value = slugs_init_execution_array_client(init_key, init_value)
        print  "Init_state_key:" + str(key)
        print  "Init_state_value:" + str(value)

        print  "Init_state_string:" + slugs_init_execution_string_client("000000...")

        print  "Trans_state_string:" + str(slugs_trans_execution_string_client("00"))
        print  "Trans_state_string:" + str(slugs_trans_execution_string_client("10"))

        trans_key = ['person', 'hazardous_item']
        trans_value = [True, False]
        key, value = slugs_trans_execution_array_client(trans_key, trans_value)
        print  "Trans_state_key:" + str(key)
        print  "Trans_state_value:" + str(value)

        trans_value = [False, True]
        key, value = slugs_trans_execution_array_client(trans_key, trans_value)
        print  "Trans_state_key:" + str(key)
        print  "Trans_state_value:" + str(value)

        print  "Set current goal to:" + str(slugs_set_goal_client(0))

#!/usr/bin/env python
'''
Created on Jan 28, 2016

@author: cme
'''
#****************************************************************
# \file
#
# \note
# Copyright (c) 2016 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#*****************************************************************
#
# \note
# Project name: Care-O-bot
# \note
# ROS stack name: ipa_pars
# \note
# ROS package name: ipa_pars_main
#
# \author
# Author: Christian Ehrmann
# \author
# Supervised by: Richard Bormann
#
# \date Date of creation: 03.2016
#
# \brief
#
#
#*****************************************************************
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#****************************************************************/
import rospy
import actionlib
import subprocess

from ipa_pars_main.msg._PlanExecutorAction import *
from ipa_pars_main.msg._PlanSimulatorAction import *
from move_base_msgs.msg._MoveBaseAction import *
from move_base_msgs.msg import MoveBaseGoal
import simple_moveit_interface as smi
# from cob_perception_msgs.msg import Detection
# from cob_object_detection_msgs.msg import DetectObjectsActionGoal
# from cob_object_detection_msgs.msg import DetectObjectsAction

from simple_script_server import *
sss = simple_script_server()


import yaml
import tf
from yaml import load

from geometry_msgs.msg import Pose, PoseStamped

class PlanningExecutorServer(object):
    _feedback = ipa_pars_main.msg.PlanExecutorFeedback()
    _result = ipa_pars_main.msg.PlanExecutorResult()
    def __init__(self):
        rospy.loginfo("Initialize PlanExecutorServer ...")
        # read yaml file static knowledge base
        self._planSimulatorClient = actionlib.SimpleActionClient('plan_simulator_server', PlanSimulatorAction)
        rospy.logwarn("Waiting for PlanSimulatorServer to come available ...")
        self._planSimulatorClient.wait_for_server()
        rospy.logwarn("Server is online!")
        self._moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.logwarn("Waiting for MoveBaseActionServer to come available ...")
        self._moveBaseClient.wait_for_server()
        rospy.logwarn("Server is online!")
        #predefine positions for demonstration:
        self.gripper_preparation_left = [[-2.5,-1.36,1.35,0.06,0.75,0.7,0.57]]
        self.gripper_preparation_right = [[2.5,1.36,-1.35,-0.06,-0.75,-0.7,-0.57]]
        self.gripping_arm_left = [[-2.5,-1.56,1.35,0.06,1.25,0.7,0.57]]
        self.gripping_arm_right = [[2.5,1.56,-1.35,-0.06,-1.25,-0.7,-0.57]]
        self.gripping_gripper_left = [[0.3,0.3]]
        self.gripping_gripper_right = [[0.3,0.3]]
        self.gripper_right_open = [[0,0]]
        self.gripper_left_open = [[0,0]]
        self.carry_left = [[-2.8,-1.56,1.35,0.06,1.25,0.7,0.57]]
        self.carry_right = [[2.8,1.56,-1.35,-0.06,-1.25,-0.7,-0.57]]
        self.yamlfile_static_knowledge = self.load_static_knowledge_from_yaml()
        self.yamlfile_dynamic_knowledge = self.load_dynamic_knowledge_from_yaml()
        self.number_of_box = 1
        self.nothandledbefore = True
#         self.getTransformationForInit('base_link', 'map')
        self._as = actionlib.SimpleActionServer('plan_executor_server', ipa_pars_main.msg.PlanExecutorAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()


        rospy.loginfo("PlanExecutorServer running! Waiting for a new action list to execute ...")
        rospy.loginfo("PlanExecutorServer initialize finished")
    
#     def getTransformationForInit(self, target_frame, frame):
#         transform_available = False
#         while not transform_available:
#             try:
#                 (trans,rot) = self.listener.lookupTransform(target_frame, frame , rospy.Time(0))
#             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#                 rospy.logwarn("Waiting for transform...")
#                 if rospy.is_shutdown():
#                     break
# 
#                 rospy.sleep(0.5)
#                 continue
#             transform_available = True
#         rospy.loginfo("Init transformation from target "+target_frame+" to frame "+frame+" successfully received!")
        
        
    def getTargetPose(self, target_name):
        pose_is_set = False
        _new_pose = Pose()
        location_data = self.yamlfile_static_knowledge["location-data"]
        for squares in location_data:
            if (squares["name"] == target_name):
                point_coordinates = squares["center"]
                _new_pose.position.x = ((point_coordinates["Y"] - 379) * 0.05)
                _new_pose.position.y = ((point_coordinates["X"] - 388) * 0.05)
                _new_pose.position.z = point_coordinates["Z"] * 0.05
                _new_pose.orientation.x = 0
                _new_pose.orientation.y = 0
                _new_pose.orientation.z = 0
                _new_pose.orientation.w = 1
                pose_is_set = True

        if pose_is_set:
            return _new_pose
        else:
            rospy.logerr("For the location %s we found no data to set move-base parameters!" % target_name)
            return _new_pose

    def getTargetPosition(self, target_name):
        position = []
        location_data = self.yamlfile_static_knowledge["location-data"]
        for squares in location_data:
            if (squares["name"] == target_name):
                point_coordinates = squares["center"]
                print point_coordinates
                position.append((point_coordinates["Y"] - 380) * 0.05) # x-wert
                position.append((point_coordinates["X"] - 388) * (-0.05)) # y-wert
                # last parameter is angle not position!
                position.append(0.0)
        return position
        
    def execute_cb(self, goal):
        rospy.loginfo("Executing a new list of goals!")
        print goal.action_list
        last_goal = None
        
        for action_goal in goal.action_list:
            #print action_goal.data
            #split input
            split_input = action_goal.data.split( )
            if (split_input[0] == "move-robo-to"):
                print "========================================================"
                print "this is a move-base action call"
                print "robot should move to pose: "
                #print split_input[3]
#                 print self.getTargetPose(split_input[3])
                target_position = self.getTargetPosition(split_input[3])
                print "sending move command to robot"
                print target_position
                #sss.move("base",target_position)
                if split_input[3] == "room-10-square-5":
                    sss.move("base",[-3.6,-3.3,0.0])
                elif split_input[3] == "room-10-square-10":
                    sss.move("base",[-2.8,-3.4,0.0])
                elif split_input[3] == "room-9-square-4":
                    sss.move("base",[-1.6,-3.7,0.0])
                elif split_input[3] == "room-9-square-16":
                    sss.move("base",[-0.6,-3.7,0.0])
                elif split_input[3] == "room-9-square-28":
                    sss.move("base",[+0.4,-3.7,1.5])
                elif split_input[3] == "room-9-square-27":
                    sss.move("base",[0.4,-2.7,1.5])
                elif split_input[3] == "room-9-square-26":
                    sss.move("base",[0.4,-1.7,1.5])
                elif split_input[3] == "room-9-square-25":
                    if self.nothandledbefore:
                        print "exception handling because square can not be reached!"
                        print "add to knowledge base and start replanning!"
                        self.nothandledbefore = False
                        self.yamlfile_dynamic_knowledge["environment-data"].append({'center': {'Y': 999, 'X': 999, 'Z': 0}, 'type': 'phys-obj', 'properties': ['occupied','moveable','new'], 'name': name_of_new_object , 'location': name_of_location })
                        print self.yamlfile_dynamic_knowledge
                        new_yaml = yaml.dump(self.yamlfile_dynamic_knowledge)
                        rospy.loginfo("Plan Executor attained all goals")
                        self._result.dynamic_knowledge = new_yaml
                        self._result.replanning = True
                        self._result.success = False
                        self._as.set_succeeded(self._result, "replanning message sent")
                        break
#                 target_pose = PoseStamped()
#                 target_pose.header.stamp = rospy.Time.now()
#                 target_pose.header.frame_id = "map"
#                 target_pose.pose.position.x = target_position[0]
#                 target_pose.pose.position.y = target_position[1]
#                 target_pose.pose.position.z = 0.0
#                 target_pose.pose.orientation.x = 0.0
#                 target_pose.pose.orientation.y = 0.0
#                 target_pose.pose.orientation.z = 0.0
#                 target_pose.pose.orientation.w = 1.0
#                 target_goal = MoveBaseGoal(target_pose)
#                 # send the goal to MoveBaseAction server
#                 self._moveBaseClient.send_goal(target_goal)
#                 # check wheter the goal is reached in time (true if yes)
#                 rospy.loginfo("waiting to reach goal ...")
#                 is_in_time = self._moveBaseClient.wait_for_result(rospy.Duration.from_sec(30.0))
#
#                 goal = ipa_pars_main.msg.PlanSimulatorGoal()
#                 goal.action.data = action_goal.data
#                 self._planSimulatorClient.send_goal_and_wait(goal)
#                 result = self._planSimulatorClient.get_result()
#                 print "result of simulator call:"
#                 print result
#                 if not is_in_time:
# #                 if not result.success:
#                     print "++++++++++++++++++++++++++++++++++++++++++++++++++"
#                     print "started exception handling!!!!!!!!!!!!!!"
#                     print "recover to last valid position"
#                     goal = ipa_pars_main.msg.PlanSimulatorGoal()
#                     goal.action.data = last_goal.data
#                     self._planSimulatorClient.send_goal_and_wait(goal)
#                     result = self._planSimulatorClient.get_result()
#                     if result.success:
#                         print "look-at failed position"
#                         goal = ipa_pars_main.msg.PlanSimulatorGoal()
#                         goal.action.data = "look-at cob4-1 place "+last_goal.data.split()[3]+" "+action_goal.data.split()[3]
#                         self._planSimulatorClient.send_goal_and_wait(goal)
#                         result = self._planSimulatorClient.get_result()
#                         print "change dynamic knowledge about this object I just found!"
#                         name_of_new_object = "the-new-box-"+str(self.number_of_box)
#                         self.number_of_box += 1
#                         name_of_location = action_goal.data.split()[3]
#                         self.yamlfile_dynamic_knowledge["environment-data"].append({'center': {'Y': 999, 'X': 999, 'Z': 0}, 'type': 'phys-obj', 'properties': ['occupied','moveable','new'], 'name': name_of_new_object , 'location': name_of_location })
#                         #print self.yamlfile_dynamic_knowledge
#                         new_yaml = yaml.dump(self.yamlfile_dynamic_knowledge)
#                         print new_yaml
#                         rospy.loginfo("Plan Executor failed and will start replanning")
#                         self._result.replanning = True
#                         self._result.success = False
#                         self._as.set_succeeded(self._result, "perfect job")
#                     print "++++++++++++++++++++++++++++++++++++++++++++++++++"
                    
                #sss.move("base",target_position)
                #print "reached position"
#                 self.moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
# #                 print "waiting for move_base server ..."
# #                 self.moveBaseClient.wait_for_server()
#                 target_pose = PoseStamped()
#                 target_pose.header.stamp = rospy.Time.now()
#                 target_pose.header.frame_id = "map"
#                 target_pose.pose = self.getTargetPose(split_input[3])
#                 target_goal = MoveBaseGoal(target_pose)
#                 print "I would like to send this goal to move_base:"
#                 print target_goal
#                 self.moveBaseClient.send_goal(target_goal)
#                 print "sending goal to move_base: Waiting for result"
#                 result = self.moveBaseClient.wait_for_result(rospy.Duration.from_sec(30.0))
#                 print "result is"
#                 print result
                print "========================================================"
            if (split_input[0] == "look-at"):
                print "========================================================"
                print "this is a look-at action call"
                print "the robot should look-at position"
                print split_input[4]
                print "moving sensorring"
                #sss.move("sensorring",[[1.5]])
#                 goal = ipa_pars_main.msg.PlanSimulatorGoal()
#                 goal.action.data = action_goal.data
#                 self._planSimulatorClient.send_goal_and_wait(goal)
#                 result = self._planSimulatorClient.get_result()
#                 print "result of simulator call:"
#                 print result
#                 print "reached position"
#                 sss.say("sound", ["hello"])
#                 component_name = "arm_left"
#                 sss.move(component_name,["base_link", [0,0,0],[0,0,0]])
#                 config = smi.get_goal_from_server("arm", "home")
#                 print config
#                 success = smi.moveit_joint_goal("arm", config)
#                 print success
                print "========================================================"
            if (split_input[0] == "grip-it"):
                print "========================================================"
                print "this is a grip-it action call"
                print split_input[2] #what
                print split_input[5] #arm
#                 goal = ipa_pars_main.msg.PlanSimulatorGoal()
#                 goal.action.data = action_goal.data
#                 self._planSimulatorClient.send_goal_and_wait(goal)
#                 result = self._planSimulatorClient.get_result()
#                 print "result of simulator call:"
#                 print result
#                 arm = "arm_not_specified"
#                 if (split_input[5] == "arm-left"):
#                     arm = "arm_left"
#                 if (split_input[5] == "arm-right"):
#                     arm = "arm_right"
#                 sss.move("arm_left", self.gripper_preparation_left)
                #sss.move("arm_right", self.gripper_preparation_right)
                sss.move("arm_right", self.gripping_arm_right)
#                 sss.move("arm_left", self.gripping_arm_left)
                #sss.move("gripper_right", self.gripping_gripper_right)
#                 sss.move("gripper_left", self.gripping_gripper_left)
                sss.move("arm_right", self.carry_right)
                sss.move("arm_right","side")
                
                if split_input[2] == "the-box-2":
                    command = "rosservice call /gazebo/delete_model '{model_name: box5}'"
                    shell_output = subprocess.check_output([command],shell=True)
                    #self.write_to_logfile(shell_output)
                    print shell_output
                    command = "rosservice call /move_base/clear_costmaps"
                    shell_output = subprocess.check_output([command],shell=True)
                    #self.write_to_logfile(shell_output)
                    print shell_output
                    
                if split_input[4] == "room-9-square-4":
                    sss.move("base",[-3.1,-3.4,0.0])
                    sss.move("base",[-3.6,-3.3,0.7])
#                 sss.move("arm_left", self.carry_left)
#                 sss.say("sound", ["hello"])
#                 component_name = "arm_left"
#                 sss.move(component_name,["base_link", [0,0,0],[0,0,0]])
#                 config = smi.get_goal_from_server("arm", "home")
#                 print config
#                 success = smi.moveit_joint_goal("arm", config)
#                 print success
                print "========================================================"
            if (split_input[0] == "put-it"):
                print "========================================================"
                print "this is a put-it action call"
                print "the robot should put"
                print split_input[2] #what
                print split_input[4] #where
                print split_input[5] #arm
                obj_name = split_input[2]
#                 goal = ipa_pars_main.msg.PlanSimulatorGoal()
#                 goal.action.data = action_goal.data
#                 self._planSimulatorClient.send_goal_and_wait(goal)
#                 result = self._planSimulatorClient.get_result()
#                 print "result of simulator call:"
#                 print result
#                 sss.move("arm_left", self.gripping_gripper_left)
                sss.move("arm_right", self.gripping_arm_right)
                sss.move("gripper_right", self.gripper_right_open)
#                 sss.move("gripper_left", self.gripper_left_open)
#                 sss.move("arm_left", self.gripper_preparation_left)
                #sss.move("arm_right", self.gripper_preparation_right)
#                 sss.move("arm_left","side")
                
                if split_input[2] == "the-box-2":
                    command = "rosrun gazebo_ros spawn_model -file ~/git/catkin_ws/src/ipa_pars/planning_demo/scripts/the-box.sdf -sdf -model box5 -x -2.5 -y -2.5 -z 0.25"
                    shell_output = subprocess.check_output([command],shell=True)
                    #self.write_to_logfile(shell_output)
                    print shell_output
                
                if split_input[4] == "room-10-square-10":
                    sss.move("base",[-2.8,-3.7,0.0])
                   
                sss.move("arm_right","side")
                
                print "========================================================"
            if (split_input[0] == "deliver-to"):
                print "========================================================"
                print "this is a look-at action call"
#                 goal = ipa_pars_main.msg.PlanSimulatorGoal()
#                 goal.action.data = action_goal.data
#                 self._planSimulatorClient.send_goal_and_wait(goal)
#                 result = self._planSimulatorClient.get_result()
#                 print "result of simulator call:"
#                 print result
#                 sss.move("arm_left", self.gripping_gripper_left)
#                 sss.move("arm_right", self.gripping_gripper_right)
#                 sss.move("gripper_right", self.gripper_right_open)
#                 sss.move("gripper_left", self.gripper_left_open)
#                 sss.move("arm_left", self.gripper_preparation_left)
#                 sss.move("arm_right", self.gripper_preparation_right)
#                 sss.move("arm_left","side")
#                 sss.move("arm_right","side")
                print "========================================================"
            if (split_input[0] == "take"):
                print "========================================================"
                print "this is a take action call"
#                 goal = ipa_pars_main.msg.PlanSimulatorGoal()
#                 goal.action.data = action_goal.data
#                 self._planSimulatorClient.send_goal_and_wait(goal)
#                 result = self._planSimulatorClient.get_result()
#                 print "result of simulator call:"
#                 print result
#                 sss.move("arm_left", self.gripper_preparation_left)
#                 sss.move("arm_right", self.gripper_preparation_right)
#                 sss.move("arm_right", self.gripping_arm_right)
#                 sss.move("arm_left", self.gripping_arm_left)
#                 sss.move("gripper_right", self.gripping_gripper_right)
#                 sss.move("gripper_left", self.gripping_gripper_left)
#                 sss.move("arm_right", self.carry_right)
#                 sss.move("arm_left", self.carry_left)
                print "========================================================"
            last_goal = action_goal
            
        success = True
        self._result.success = True
        rospy.sleep(5)
        #===========================
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % 'planning_execution_server')
        if success:
            rospy.loginfo("Plan Executor attained all goals")
            self._as.set_succeeded(self._result, "perfect job")

    def load_static_knowledge_from_yaml(self):
        # beachte: YAMl verwendet nur dicts und lists
        # verwende die entsprechenden methoden richtig
        f = open("ipa_pars/knowledge/static-knowledge-base.yaml", 'r')
        yamlfile = load(f)
        f.close()
        return yamlfile
    
    def load_dynamic_knowledge_from_yaml(self):
        f = open("ipa_pars/knowledge/dynamic-knowledge-base.yaml", 'r')
        yamlfile = load(f)
        f.close()
        return yamlfile
        
if __name__ == '__main__':
    rospy.init_node('planning_executor_server_node', anonymous=False)
    pES = PlanningExecutorServer()
    rospy.spin()
        

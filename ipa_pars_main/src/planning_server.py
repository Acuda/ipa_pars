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
# \date Date of creation: 01.2016
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
import actionlib
import rospy
import sys
import cv2
from yaml import load
# from cv_bridge import CvBridge, CvBridgeError
from ipa_pars_main.msg._LogicPlanAction import *
from ipa_pars_main.msg._PlanSolverAction import *
from ipa_pars_main.msg._KnowledgeParserAction import *
from std_msgs.msg import String
#from std_msgs import String[]
# import numpy as np
# from sensor_msgs.msg._Image import Image
# import sensor_msgs.msg
# from map_analyzer.srv import MapAnalyzer
# from cob_srvs.srv._SetString import SetString
# from map_analyzer.srv._MapAnalyzer import MapAnalyzerResponse
# from ipa_pars_main.srv._PlanData import PlanData, PlanDataRequest



class PlanningServer(object):
    _feedback = ipa_pars_main.msg.LogicPlanFeedback()
    _result = ipa_pars_main.msg.LogicPlanResult()
    def __init__(self, path_to_domain, path_to_knowledge):
        rospy.loginfo("Initialize PlanningServer ...")
        self._path_to_domain = path_to_domain
        self._path_to_problem = path_to_domain
        self._path_to_knowledge = path_to_knowledge
        rospy.loginfo("path_to_knowledge: %s " % path_to_knowledge)
        rospy.loginfo("path_to_domain: %s" % path_to_domain)
        self._planSolverClient = actionlib.SimpleActionClient('planning_solver_server', PlanSolverAction)
        rospy.logwarn("Waiting for PlanSolverServer to come available ...")
        self._planSolverClient.wait_for_server()
        rospy.logwarn("PlanningSolverServer is online!")
        self._knowledgeParserClient = actionlib.SimpleActionClient('knowledge_parser_server', KnowledgeParserAction)
        rospy.logwarn("Waiting for KnowledgeParserServer to come available ...")
        self._knowledgeParserClient.wait_for_server()
        rospy.logwarn("KnowledgeParserServer is online!")
        self._as = actionlib.SimpleActionServer('planning_server', ipa_pars_main.msg.LogicPlanAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("PlanningServer running! Waiting for a new goal.")

    def execute_cb(self, goal):
        rospy.loginfo("Executing a new goal!")
        rospy.loginfo("GOAL: %s , %s, %s " % (str(goal.goal_type), str(goal.what), str(goal.where)))
        rospy.loginfo("in progress ...")
#         knowledge_goal = ipa_pars_main.msg.KnowledgeParserGoal()
#         knowledge_goal.static_knowledge = self.readKnowledgeBase("knowledge-base-static-test-01.yaml")
#         print knowledge_goal.static_knowledge
#         knowledge_goal.dynamic_knowledge = self.readKnowledgeBase("knowledge-base-dynamic-test-01.yaml")
#         rospy.loginfo("Sending goal to KnowledgeParserServer ...")
#         self._knowledgeParserClient.send_goal(knowledge_goal)
#         rospy.loginfo("Waiting for result ...")
#         self._knowledgeParserClient.wait_for_result()
#         result = self._knowledgeParserClient.get_result()
#         rospy.loginfo("Received the result from KnowledgeParserServer!")
#         rospy.loginfo(result)
        
        
        goal = ipa_pars_main.msg.PlanSolverGoal()
        problem_text = self.generate_debug_problem()
        goal.problem.data = problem_text
        domain_text = self.generate_debug_domain()
        goal.domain.data = domain_text
        
        rospy.loginfo("Sending goal to solver ...")
        self._planSolverClient.send_goal(goal)
        rospy.loginfo("Waiting for result ...")
        self._planSolverClient.wait_for_result()
        result = self._planSolverClient.get_result()
        rospy.loginfo("Received the result from Solver:")
        print result
        
        if not (result.success):
            rospy.loginfo("no valid plan found. I cannot solve this problem alone!")
            self._result.success = False
            self._as.set_aborted(self._result, "bad job")
        #rospy.loginfo(result)
#         r = rospy.Rate(1)
#         # change herer TODO:
#         #===========================
#         # robot stuff here
#         room_information = self.sendImageToMapAnalyzerServer()
        #room_information = MapAnalyzerResponse()
        #room_information.answer.data = "room-1 room-2\nroom-2 room-1 room-3 room-4 room-5 room-10\nroom-3 room-2\nroom-4 room-2\nroom-5 room-2\nroom-6 room-7 room-10 room-11\nroom-7 room-10 room-11\nroom-8 room-9 room-11\nroom-9 room-8\nroom-10 room-2 room-6 room-7\nroom-11 room-6 room-7 room-8"

        #controller_request = PlanDataRequest()
        #controller_request.goal_data.data = str(goal.goal_type)
        #controller_request.room_data.data = room_information.answer.data
        #controller_request.domain_data.data = self.generate_debug_domain()
        #answer0 = self.planning_controller_server(controller_request)
        #print answer0
        #==========================================================
        #answer1 = self.room_info_client(room_information.answer.data)
        #print answer1
        #planning_goal_text = str(goal.goal_type)+" "+str(goal.what)+" "+goal.where
        #answer2 = self.planning_goal_server(planning_goal_text)
        #print answer2
        #planning_domain_text = "this is my domain information"
        #answer3 = self.planning_domain_server(planning_domain_text)
        #print answer3
        #==============================================
        print "i am sleeping now"
        success = True
        rospy.sleep(5)
        #===========================
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % 'pars_server')
            success = False
        
        
        if success:
            self._result.success = True
            rospy.loginfo("Succeeded the Logic Plan")
            self._as.set_succeeded(self._result, "good job")

    def readKnowledgeBase(self, knowledge_yaml):
        listOfInput = []
        try:
            fileObject = open(self._path_to_knowledge+knowledge_yaml, "r")
            yamlfile = load(fileObject)
            fileObject.close()
        except IOError:
            rospy.loginfo("Reading %s base failed!" % knowledge_yaml)
        return yamlfile

        
    def generate_debug_domain(self):
        print "read input file"
        listOfInput = []
        try:
            fileObject = open(self._path_to_domain+"domain.pddl", "r")
            with fileObject as listOfText:
                listOfInput = listOfText.readlines()
            fileObject.close()
        except IOError:
            rospy.loginfo("open file failed or readLine error")
        StringOfObjects = str(" ").join(map(str, listOfInput))
        domain_text = StringOfObjects
        return domain_text

    
    def generate_debug_problem(self):
        print "read input file"
        listOfInput = []
        try:
            fileObject = open(self._path_to_problem+"problem.pddl", "r")
            with fileObject as listOfText:
                listOfInput = listOfText.readlines()
            fileObject.close()
        except IOError:
            rospy.loginfo("open file failed or readLine error")
        StringOfObjects = str(" ").join(map(str, listOfInput))
        problem_text = StringOfObjects
        return problem_text

if __name__ == '__main__':
    rospy.init_node('planning_server_node', anonymous=False)
    pS = PlanningServer(sys.argv[1], sys.argv[2])
    rospy.spin()

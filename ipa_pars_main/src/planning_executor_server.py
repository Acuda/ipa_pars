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

from ipa_pars_main.msg._PlanExecutorAction import *

import yaml
from yaml import load

from geometry_msgs.msg import Pose

class PlanningExecutorServer(object):
    _feedback = ipa_pars_main.msg.PlanExecutorFeedback()
    _result = ipa_pars_main.msg.PlanExecutorResult()
    def __init__(self, path_to_inputfile):
        rospy.loginfo("Initialize PlanExecutorServer ...")
        self.path_to_inputfile = path_to_inputfile
        rospy.loginfo(path_to_inputfile)
        self._as = actionlib.SimpleActionServer('plan_executor_server', ipa_pars_main.msg.PlanExecutorAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        # read yaml file static knowledge base
        self.load_params_from_yaml()

        rospy.loginfo("PlanExecutorServer running! Waiting for a new action list to execute ...")
        rospy.loginfo("MapAnalyzerServer initialize finished")
        
    def execute_cb(self, goal):
        rospy.loginfo("Executing a new list of goals!")
        print goal.action_list

        for action_goal in goal.action_list:
            #print action_goal.data
            #split input
            split_input = action_goal.data.split( )
            if (split_input[0] == "move-robo-to"):
                print "================================"
                print "this is a move-base action call"
                print "robot should move to"
                print split_input[3]
                print "================================"

            if (split_input[0] == "take"):
                print "================================"
                print "this is a take action call"
                print "the robot should take"
                print split_input[2]
                print "================================"
                
            if (split_input[0] == "look-at"):
                print "================================"
                print "this is a look-at action call"
                print "the robot should look-at"
                print split_input[2]
                print "================================"




        success = True
        self._result.success = True
        rospy.sleep(5)
        #===========================
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % 'map_analyzer_server')
        if success:
            rospy.loginfo("Plan Executor attained all goals")
            self._as.set_succeeded(self._result, "perfect job")

    def load_params_from_yaml(self):
        # beachte: YAMl verwendet nur dicts und lists
        # verwende die entsprechenden methoden richtig
        f = open(self.path_to_inputfile+"knowledge-base.yaml", 'r')
        yamlfile = load(f)
        f.close()
        print yamlfile
        print "------------------------"
        
if __name__ == '__main__':
    rospy.init_node('planning_executor_server_node', anonymous=False)
    pES = PlanningExecutorServer(sys.argv[1])
    rospy.spin()
        

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

from geometry_msgs.msg import Pose
from std_msgs.msg import String

class PlanningExecutorClient(object):
    def __init__(self):
        rospy.loginfo("Initialize PlanExecutorClient ...")
        rospy.loginfo("... starting plan_executor_server")
        self._planExecutorClient = actionlib.SimpleActionClient('planning_executor_server', PlanExecutorAction)
        rospy.logwarn("Waiting for PlanExecutorServer to come available ...")
        self._planExecutorClient.wait_for_server()
        rospy.logwarn("Server is online!")
        rospy.loginfo("PlanExecutorClient initialize finished")

    def sendGoal(self):
        goal = ipa_pars_main.msg.PlanExecutorGoal()
        #read goals for debug from file
        listOfInput = []
        try:
            fileObject = open("ipa_pars/output/sas_plan", "r")
            with fileObject as listOfText:
                listOfInput = listOfText.readlines()
            fileObject.close()
        except IOError:
            rospy.loginfo("open file failed or readLine error")
        
        print "this is the action list to send"
        #delete last element
        del listOfInput[-1:]
        print listOfInput

        listOfOutput = []
        for action_exe in listOfInput:
            new_action = String()
            new_action.data = action_exe.replace("(","").replace(")","")
            listOfOutput.append(new_action)
            
        print listOfOutput
        goal.action_list = listOfOutput
        rospy.loginfo("Send action list to PlanExecutorServer ...")
        self._planExecutorClient.send_goal(goal)
        rospy.loginfo("Waiting for result of PlanExecutorServer ...")
        self._planExecutorClient.wait_for_result()
        result = self._planExecutorClient.get_result()
        rospy.loginfo("Received a result from PlanExecutorServer!")
        print result


if __name__ == '__main__':
    rospy.init_node('planning_executor_client_node', anonymous=False)
    pEC = PlanningExecutorClient()
    pEC.sendGoal()
        

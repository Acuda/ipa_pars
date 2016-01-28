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
# ROS package name: planning_demo
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
import rospy
import actionlib
import ipa_pars_main.msg

class PlanningDemo(object):
    def __init__(self):
        rospy.loginfo("Initialize PlanningDemo ...")
        self._aclient = actionlib.SimpleActionClient('pars_server', ipa_pars_main.msg.LogicPlanAction)
        rospy.logwarn("Waiting for Action Server to come available ...")
        self._aclient.wait_for_server()
        rospy.logwarn("Server is online.")
        rospy.loginfo("... finished")
        
    def createGoal(self):
        rospy.loginfo("Creating Goal ...")
        goal = ipa_pars_main.msg.LogicPlanGoal()
        goal.goal_type = "FetchAndCarry"
        goal.what = "the-cake"
        goal.where = "the-kitchen"
        rospy.loginfo(goal.goal_type)
        rospy.loginfo(goal.what)
        rospy.loginfo(goal.where)
        rospy.loginfo(" ... finished!")
        return goal

    def sendGoalAndWait(self):
        goal = self.createGoal()
        rospy.loginfo("Sending goal ...")
        self._aclient.send_goal(goal)
        rospy.loginfo("Waiting for result ...")
        self._aclient.wait_for_result()
        solution = self._aclient.get_result()
        rospy.loginfo("Received the result:")
        rospy.loginfo(solution)
            
if __name__ == '__main__':
    rospy.init_node('planning_demo_client_node', anonymous=False)
    pD = PlanningDemo()
    try:
        pD.sendGoalAndWait()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
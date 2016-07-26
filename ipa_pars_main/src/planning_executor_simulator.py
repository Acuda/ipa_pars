#!/usr/bin/env python
'''
Created on Jun 30, 2016

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
# \date Date of creation: 06.2016
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

from ipa_pars_main.msg._PlanSimulatorAction import *

class PlanningSimulatorServer(object):
    _feedback = ipa_pars_main.msg.PlanSimulatorFeedback()
    _result = ipa_pars_main.msg.PlanSimulatorResult()
    def __init__(self):
        rospy.loginfo("Initialize PlanSimulatorServer ...")
        self._as = actionlib.SimpleActionServer('plan_simulator_server', ipa_pars_main.msg.PlanSimulatorAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()


        rospy.loginfo("PlanSimulatorServer running! Waiting for a new action list to execute ...")
        rospy.loginfo("PlanSimulatorServer initialize finished")

    def execute_cb(self, goal):
        rospy.loginfo("Executing a new goal!")
        print goal.action.data
        if (goal.action.data == "move-robo-to cob4-1 room-9-square-26 room-9-square-25\n"):
            rospy.logerr("Goal failed! Position not reached!")
            self._result.success = False
        else:
            self._result.success = True
        
        
        success = True
        #self._result.success = True
        rospy.sleep(2)
        #===========================
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % 'planning_simulator_server')
        if success:
            rospy.loginfo("Plan Simulator attained goal")
            self._as.set_succeeded(self._result, "great job")

if __name__ == '__main__':
    rospy.init_node('planning_executor_simulator_node', anonymous=False)
    pSS = PlanningSimulatorServer()
    rospy.spin()
        

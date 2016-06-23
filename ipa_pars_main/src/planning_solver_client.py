#!/usr/bin/env python
'''
Created on Jun 21, 2016

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
import sys
import os
import subprocess
import numpy as np

import actionlib

from ipa_pars_main.msg._PlanSolverAction import *
from std_msgs.msg import String

class PlanningSolverClient(object):
    def __init__(self):
        rospy.loginfo("Initialize PlanningSolverClient ...")
        self._planningSolverClient = actionlib.SimpleActionClient('planning_solver_server', PlanSolverAction)
        rospy.logwarn("Waiting for PlanningSolverServer to come available ...")
        self._planningSolverClient.wait_for_server()
        rospy.logwarn("Server is online!")
        rospy.loginfo("PlanningSolverClient initialize finished")
        
    def sendGoal(self):
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
        

    def generate_debug_domain(self):
        print "read input file"
        listOfInput = []
        try:
            fileObject = open("ipa_pars/pddl/domain.pddl", "r")
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
            fileObject = open("ipa_pars/pddl/problem.pddl", "r")
            with fileObject as listOfText:
                listOfInput = listOfText.readlines()
            fileObject.close()
        except IOError:
            rospy.loginfo("open file failed or readLine error")
        StringOfObjects = str(" ").join(map(str, listOfInput))
        problem_text = StringOfObjects
        return problem_text
    
if __name__ == '__main__':
    rospy.init_node('planning_solver_client_node', anonymous=False)
    pSC = PlanningSolverClient()
    pSC.sendGoal()

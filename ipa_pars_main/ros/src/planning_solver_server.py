#!/usr/bin/env python
'''
Created on Feb 04, 2016

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
# \date Date of creation: 02.2016
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

class PlanningSolverServer(object):
    _feedback = ipa_pars_main.msg.PlanSolverFeedback()
    _result = ipa_pars_main.msg.PlanSolverResult()
    def __init__(self):
        rospy.loginfo("Initialize PlanningSolverServer ...")
        self._as = actionlib.SimpleActionServer('planning_solver_server', ipa_pars_main.msg.PlanSolverAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("PlaningSolverServer running! Waiting for a new problem to solve ...")
        rospy.loginfo("PlanningSolverServer initialize finished")
        
    def execute_cb(self, goal):
        rospy.loginfo("=======================================================")
        rospy.loginfo("|                Solving a new problem!               |")
        rospy.loginfo("|             Received the following goal:            |")
        #print goal
        domain_text = goal.domain.data
        problem_text = goal.problem.data
#         self.save_domain_file(domain_text)
#         self.save_problem_file(problem_text)
        self.workOnPlan()
        actionsAsString = self.readPlanFile()
        if len(actionsAsString) < 1:
            rospy.loginfo("We found no solution for this task. No action plan!")
            #success = False
            self._result.success = False
            self._result.action_list = None
            self._as.set_aborted(self._result, "no action plan found")
            
        list_of_actions = actionsAsString.splitlines()
        del list_of_actions[-1] # delete the last element in the list
        newoutput = []
        for outputline in list_of_actions:
            newoutputline = String()
            newoutputline.data = outputline
            newoutput.append(newoutputline)
        success = True
        self._result.success = True
        self._result.action_list = newoutput
        #print self._result
        #rospy.sleep(5)
        #===========================
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % 'planning_solver_server')
        if success:
            rospy.loginfo("Plan Solver Server solved problems")
            self._as.set_succeeded(self._result, "perfect job")
        
#     def handle_domain_cb(self, domain_as_string):
#         print "domain_as_string"
#         print domain_as_string
#         # TODO: do some stuff
#         self.save_domain_file(domain_as_string.data)
#         response = SetStringResponse()
#         response.success = True
#         response.message = "ErrorAnswer"
#         
#         return response
#     
#     def handle_problem_cb(self, problem_as_string):
#         print "problem_as_string"
#         print problem_as_string
#         # TODO: do some stuff:
#         self.save_problem_file(problem_as_string.data)
#         
#         # TODO: check that domain AND problem files are created!
#         self.workOnPlan(self.path_to_outputfile)
#         plan_as_text = self.readPlanFile()
#         answer = self.execution_service_client(plan_as_text)
#         
#         print answer
#         
#         response = SetStringResponse()
#         response.success = True
#         response.message = "Error Answer"
#         return response

    def write_to_logfile(self, log_text):
        #print "save to logfile"
        try:
            if not os.path.isdir("ipa_pars/log"):
                os.mkdir("ipa_pars/log")
            with open("ipa_pars/log/planner-log.txt", "a") as myfile:
                myfile.seek(0)
                myfile.write(log_text)
            myfile.close()
            #rospy.loginfo("wrote logfile successfully!")
        except IOError:
            rospy.loginfo("writing logfile failed!")
        
        
#     def save_domain_file(self, domain_text):
#         #print "save domain file"
#         try:
#             with open(self.path_to_outputfile+"domain.pddl", "w") as myfile:
#                 myfile.seek(0)
#                 myfile.write(domain_text)
#             myfile.close()
#             #rospy.loginfo("wrote domain file successfully!")
#         except IOError:
#             rospy.loginfo("writing domain.pddl failed!")
#     
#     def save_problem_file(self, problem_text):
#         #print "save problem file"
#         try:
#             with open(self.path_to_outputfile+"problem.pddl", "w") as myfile:
#                 myfile.seek(0)
#                 myfile.write(problem_text)
#             myfile.close()
#             #rospy.loginfo("wrote problem file successfully!")
#         except IOError:
#             rospy.loginfo("writing problem.pddl failed!")

    def readPlanFile(self):
        #print "reading input file"
        listOfInput = []
        try:
            if os.path.isdir("ipa_pars/output/"):
                fileObject = open("ipa_pars/output/sas_plan", "r")
                with fileObject as listOfText:
                    listOfInput = listOfText.readlines()
                fileObject.close()
        except IOError:
            rospy.loginfo("open file failed or readline error in readPlanFile!")
            return listOfInput
            
        stringOfObjects = str(" ").join(map(str, listOfInput))
        return stringOfObjects

    def workOnPlan(self):
        rospy.loginfo("starting planner scripts: Using fast-downward and adp planner packages")
        self.deleteOldPlanFiles()
        translate_command = "bash ~/git/catkin_ws/src/ipa_pars/adp_planner/scripts/translate.bash"
        self.sendCommand(translate_command)
        preprocess_command = "bash ~/git/catkin_ws/src/ipa_pars/adp_planner/scripts/preprocess.bash"
        self.sendCommand(preprocess_command)
        fast_downward_command = "bash ~/git/catkin_ws/src/ipa_pars/adp_planner/scripts/fast-downward.bash"
        self.sendCommand(fast_downward_command)
        print "all command transmitted successfully!"
        print "script exit"
    
    #===========================================================================
    # sendCommand(command)
    # sending a bash command to shell
    # in this case just execute bash scripts to run translator, preprocessor, planner
    #===========================================================================
    def sendCommand(self, command):
        rospy.loginfo("sending command to shell")
        #TODO: Change commands. Send commands to shell directly and dont use bash scripts above!
        #print command
        shell_output = subprocess.check_output([command],shell=True)
        self.write_to_logfile(shell_output)
        print shell_output
        rospy.loginfo("done")
        
    def deleteOldPlanFiles(self):
        #print "deleting old plan file"
        try:
            if os.path.isdir("ipa_pars/output/"):
                os.remove("ipa_pars/output/output")
                os.remove("ipa_pars/output/output.sas")
                os.remove("ipa_pars/output/sas_plan")
            if os.path.isdir("ipa_pars/log/"):
                os.remove("ipa_pars/log/planner-log.txt")
            rospy.loginfo("old planner files successfully removed")
        except OSError:
            rospy.logerr("deleting old plan files failed!")
            
    
if __name__ == '__main__':
    rospy.init_node('planning_solver_server_node', anonymous=False)
    pSS = PlanningSolverServer()
    rospy.spin()

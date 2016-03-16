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
import cv2
import numpy as np
from sensor_msgs.msg import Image
#from sensor_msgs.msg._Image import Image

import actionlib

from cob_srvs.srv._SetString import SetString
from cob_srvs.srv._SetString import SetStringResponse


class PlanSolver(object):
    def __init__(self, path_to_outputfile):
        rospy.loginfo("Initialize PlanSolver ...")
        self.path_to_outputfile = path_to_outputfile
        rospy.loginfo(path_to_outputfile)
        self.domain_srv = rospy.Service('planning_solver_domain_server', SetString, self.handle_domain_cb)
        self.problem_srv = rospy.Service('planning_solver_problem_server', SetString, self.handle_problem_cb)
        
        rospy.logwarn("Waiting for planning_execution_service_server to come available ...")
        rospy.wait_for_service('planning_execution_service_server')
        rospy.logwarn("Server online!")
        try:
            self.execution_service_client = rospy.ServiceProxy('planning_execution_service_server', SetString)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.loginfo("PlanSolverServer is running, waiting for new problems to solve ...")
        rospy.loginfo("... finished")
        
    def handle_domain_cb(self, domain_as_string):
        print "domain_as_string"
        print domain_as_string
        # TODO: do some stuff
        self.save_domain_file(domain_as_string.data)
        response = SetStringResponse()
        response.success = True
        response.message = "ErrorAnswer"
        
        return response
    
    def handle_problem_cb(self, problem_as_string):
        print "problem_as_string"
        print problem_as_string
        # TODO: do some stuff:
        self.save_problem_file(problem_as_string.data)
        
        # TODO: check that domain AND problem files are created!
        self.workOnPlan(self.path_to_outputfile)
        plan_as_text = self.readPlanFile()
        answer = self.execution_service_client(plan_as_text)
        
        print answer
        
        response = SetStringResponse()
        response.success = True
        response.message = "Error Answer"
        return response
    
    def save_domain_file(self, domain_text):
        print "save domain file"
        try:
            with open(self.path_to_outputfile+"domain.pddl", "w") as myfile:
                myfile.seek(0)
                myfile.write(domain_text)
            myfile.close()
        except IOError:
            rospy.loginfo("writing domain.pddl failed!")
    
    def save_problem_file(self, problem_text):
        print "save problem file"
        try:
            with open(self.path_to_outputfile+"problem.pddl", "w") as myfile:
                myfile.seek(0)
                myfile.write(problem_text)
            myfile.close()
        except IOError:
            rospy.loginfo("writing problem.pddl failed!")

    def readPlanFile(self):
        print "reading input file"
        listOfInput = []
        try:
            fileObject = open(self.path_to_outputfile+"sas_plan", "r")
            with fileObject as listOfText:
                listOfInput = listOfText.readlines()
            fileObject.close()
        except IOError:
            rospy.loginfo("open file failed or readline error!")
            
        stringOfObjects = str(" ").join(map(str, listOfInput))
        return stringOfObjects

    def workOnPlan(self, path_to_plan):
        rospy.loginfo("starting planner scripts: Using fast-downward and adp planner packages")
        self.deleteOldPlanFiles(path_to_plan)
        translate_command = "bash ~/git/catkin_ws/src/ipa_pars/ipa_pars_main/scripts/translate.bash"
        self.sendCommand(translate_command)
        preprocess_command = "bash ~/git/catkin_ws/src/ipa_pars/ipa_pars_main/scripts/preprocess.bash"
        self.sendCommand(preprocess_command)
        fast_downward_command = "bash ~/git/catkin_ws/src/ipa_pars/ipa_pars_main/scripts/fast-downward.bash"
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
        print command
        shell_output = subprocess.check_output([command],shell=True)
        print shell_output
        rospy.loginfo("done")
        
    def deleteOldPlanFiles(self, path_to_files):
        print "deleting old plan file"
        try:
            os.remove(path_to_files+"output")
            os.remove(path_to_files+"output.sas")
            os.remove(path_to_files+"sas_plan")
            rospy.loginfo("old planner files successfully removed")
        except OSError:
            rospy.logerr("deleting old plan files failed!")
            
    
if __name__ == '__main__':
    rospy.init_node('planning_solver_node', anonymous=False)
    pS = PlanSolver(sys.argv[1])
    rospy.spin()

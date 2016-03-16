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
# ROS package name: planning_problem
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
import cv2
import numpy as np
from sensor_msgs.msg import Image
#from sensor_msgs.msg._Image import Image

import actionlib

from cob_srvs.srv._SetString import SetString
from cob_srvs.srv._SetString import SetStringResponse


class PlanProblemClass(object):
    def __init__(self, path_to_inputfile):
        rospy.loginfo("Initialize PlanProblemClass ...")
        self.path_to_inputfile = path_to_inputfile
        rospy.loginfo(path_to_inputfile)
        self.room_info_srv = rospy.Service('room_information_server', SetString, self.handle_info_cb)
        self.goal_srv = rospy.Service('planning_goal_server', SetString, self.handle_goal_cb)
        self.room_info = ""
        self.goal_info = ""
        self.linesAsList = ""
        self.goal_available = False
        self.room_info_available = False
        rospy.logwarn("Waiting for planning_solver_problem_server to come available ...")
        rospy.wait_for_service('planning_solver_problem_server')
        rospy.logwarn("Server online!")
        try:
            self.problem_solver_client = rospy.ServiceProxy('planning_solver_problem_server', SetString)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.loginfo("PlanProblemServer is running, waiting for new problems to plan ...")
        rospy.loginfo("... finished")
        
    def handle_info_cb(self, informations):
        print "informations"
        print informations
        #self.room_info = self.generate_debug_problem()
        #print self.room_info
        self.room_info = informations.data.splitlines()

        self.room_info_available = True


        response = SetStringResponse()
        response.success = True
        response.message = "ErrorAnswer"
        
        return response
    
    def generate_debug_problem(self):
        print "read input file"
        listOfInput = []
        try:
            fileObject = open(self.path_to_inputfile+"problem.pddl", "r")
            with fileObject as listOfText:
                listOfInput = listOfText.readlines()
            fileObject.close()
        except IOError:
            rospy.loginfo("open file failed or readLine error")
        StringOfObjects = str(" ").join(map(str, listOfInput))
        problem_text = StringOfObjects
        return problem_text
    
    def generate_debug_domain(self):
        print "read input file"
        listOfInput = []
        try:
            fileObject = open(self.path_to_inputfile+"domain.pddl", "r")
            with fileObject as listOfText:
                listOfInput = listOfText.readlines()
            fileObject.close()
        except IOError:
            rospy.loginfo("open file failed or readLine error")
        StringOfObjects = str(" ").join(map(str, listOfInput))
        domain_text = StringOfObjects
        return domain_text
        
    def handle_goal_cb(self, goal):
        print "goal"
        print goal
        self.goal_info = goal.data
        self.goal_available = True
        response = SetStringResponse()
        response.success = True
        response.message = "ErrorAnswer"
        return response
    
    def assembleListOfObjects(self, listOfInput):
        listOfObjects = []
        listOfObjects.append("\n \t\t ;;; available objects")
        listOfObjects.append("\n")
        listOfObjects.append("\n \t\t ;;; fixed locations")
        counter = 0
        for lines in listOfInput:
            
            pddlObject = lines.split(" ")[0]
            if counter == 0:
                listOfObjects.append("\n\t\t "+pddlObject)
                counter += 1
            else:
                listOfObjects.append(pddlObject)
                counter += 1
            if counter == 4:
                counter = 0

        listOfObjects.append("\n")
        listOfObjects.append("\n \t\t ;;; movable things")
        listOfObjects.append("\n")
        listOfObjects.append("\n\t\t the-cake")
        listOfObjects.append("\n\t\t cob4-1")
        StringOfObjects = str(" ").join(map(str, listOfObjects))

        return StringOfObjects

    def assembleListOfTransitions(self, listOfInput):
        listOfTransitions = []
        for lines in listOfInput:
            rooms = lines.split()
            mainRoom = rooms[0]
            listOfTransitions.append("\n \t\t ;;; transitions for "+mainRoom)
            for room in rooms:
                if not room == mainRoom:
                    listOfTransitions.append("\t\t(trans "+mainRoom+" "+room+")")
        StringOfTransitions = str("\n").join(map(str, listOfTransitions))
        return StringOfTransitions
    
    def assembleProblemFileText(self, listOfInput):
        problemName = "cob-test-problem-01"
        domainName = "cob-test-domain-01"
        StringOfObjects = self.assembleListOfObjects(listOfInput)
        StringOfTransitions = self.assembleListOfTransitions(listOfInput)
        listOfLines = []
        #====== 1 =======
        listOfLines.append("(define (problem "+problemName+")")
        #====== 2 =======
        listOfLines.append("\t(:domain "+domainName+")")
        #====== 3 =======
        listOfLines.append("\t(:objects "+StringOfObjects+")")
        #====== 4 =======
        listOfLines.append("\n")
        #====== 5 =======
        listOfLines.append("\t(:init  "+StringOfTransitions)
        #====== 6 =======
        listOfLines.append("\n\t ;;; hard coded definitions")
        listOfLines.append("\t\t(is-robo cob4-1)")
        listOfLines.append("\t\t(at the-cake room-9)")
        # test with two cake locations!
        listOfLines.append("\t\t(at the-cake room-8)")
        listOfLines.append("\t\t(at cob4-1 room-1)")
        listOfLines.append("\t)")
        listOfLines.append("\n")

        
        return listOfLines
    
    def appendGoalDefinition(self, listOfLines, goal_informations):
        print "translate the goal_informations to a goal pddl"
        print goal_informations
        
        listOfLines.append("\t;;; goal definition")
        listOfLines.append("\t(:goal (and (have cob4-1 the-cake) (at cob4-1 room-1)))")
        listOfLines.append(")")
        return listOfLines
        

    def run(self):
        r = rospy.Rate(1)
        #print self.goal_available
        #print self.goal_info
        #print self.room_info_available
        #print self.room_info
        
        if (self.goal_available) and (self.room_info_available):
            if not self.goal_info == "" and not self.room_info == "":
                linesAsList = self.assembleProblemFileText(self.room_info)
                linesAsList = self.appendGoalDefinition(linesAsList, self.goal_info)
                print "linesAsString"
                linesAsString = ("\n").join(linesAsList)
                print linesAsString
                answer = self.problem_solver_client(linesAsString)
                print answer
                #self.goal_available = False
                self.room_info_available = False
        r.sleep()
        
        
if __name__ == '__main__':
    rospy.init_node('planning_problem_node', anonymous=False)
    pPC = PlanProblemClass(sys.argv[1])
    while not rospy.is_shutdown():
        pPC.run()


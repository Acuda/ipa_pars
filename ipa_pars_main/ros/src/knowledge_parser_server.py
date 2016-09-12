#!/usr/bin/env python
'''
Created on Apr 15, 2016

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
import sys, os

import yaml
from yaml import load

from ipa_pars_main.msg._KnowledgeParserAction import *


class KnowledgeParserServer(object):
    _feedback = ipa_pars_main.msg.KnowledgeParserFeedback()
    _result = ipa_pars_main.msg.KnowledgeParserResult()
    def __init__(self):
        rospy.loginfo("Initialize KnowledgePaserServer ...")
#         self.path_to_inputfiles = path_to_inputfiles
#         self.yamlfile_static_knowledge = self.load_static_knowledge_from_yaml()
        self.goal_info = "static goal info"
        
        self._as = actionlib.SimpleActionServer('knowledge_parser_server', ipa_pars_main.msg.KnowledgeParserAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("KnowledgeParserServer initialize finished")
        rospy.loginfo("KnowledgeParserServer running waiting for knowledge to parse ...")

    def execute_cb(self, goal):
        rospy.loginfo("Received a new goal:")
        self.yamlfile_static_knowledge = yaml.load(goal.static_knowledge.data)
        self.yamlfile_dynamic_knowledge = yaml.load(goal.dynamic_knowledge.data)
        problem_text = self.createProblemPDDL()
        self._result.problem_pddl.data = problem_text
        self._result.domain_pddl.data = "empty test"
        rospy.loginfo("Succeeded the knowledge parsing")
        self._as.set_succeeded(self._result, "good job")
        
    def parseObjectInfoFromYaml(self):
        object_data = self.yamlfile_dynamic_knowledge["environment-data"]
        listOfObjects = []
        listOfAts = []
        listOfProps = []
        for obj in object_data:
            name_of_object = obj["name"]
            type_of_object = obj["type"]
            location_of_object = obj["location"]
            listOfObjects.append(name_of_object+" - "+type_of_object)
            #listOfLines.append("\t\t(at the-box-1 room-10-square-2)")
            listOfAts.append("\t\t(at "+name_of_object+" "+location_of_object+")")
            list_of_propterties = obj["properties"]
            for props in list_of_propterties:
                if props == "occupied":
                    listOfProps.append("\t\t(occupied "+location_of_object+")")
                if props == "moveable":
                    listOfProps.append("\t\t(moveable "+name_of_object+")")
            
        self.listOfObjectNames = listOfObjects
        self.listOfProps = listOfProps
        self.listOfAts = listOfAts
        print self.listOfObjectNames
        print self.listOfProps
        
    def parseLocationInfoFromYaml(self):
        location_data = self.yamlfile_static_knowledge["location-data"]
        listOfTransitions = []
        for squares in location_data:
            name_of_square = squares["name"]
            transitions = str(" ").join(map(str, squares["transitions"]))
            listOfTransitions.append(name_of_square+" "+transitions)
        return listOfTransitions
    
    def createProblemPDDL(self):
        listOfTransitions = self.parseLocationInfoFromYaml()
        self.parseObjectInfoFromYaml()
        #print listOfTransitions
        linesAsList = self.assembleProblemFileText(listOfTransitions)
        print linesAsList
        linesAsList = self.appendGoalDefinition(linesAsList, self.goal_info)
        linesAsString = ("\n").join(linesAsList)
        #print "++++++++++++++++++++++++++++++ TEST 1 +++++++++++++++++++++++"
        #print linesAsString
        #StringOfProblem = str(" ").join(map(str, linesAsString))
        problem_text = linesAsString
        self.save_problem_file(problem_text)
        return problem_text

    def save_problem_file(self, problem_text):
        print "save problem file"
        try:
            if not os.path.isdir("ipa_pars/pddl/"):
                os.mkdir("ipa_pars/pddl")
            with open("ipa_pars/pddl/problem.pddl", "w") as myfile:
                myfile.seek(0)
                myfile.write(problem_text)
            myfile.close()
        except IOError:
            rospy.loginfo("writing problem.pddl failed!")
            
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

        listOfObjects.append(" - room \n")
        listOfObjects.append("\n \t\t ;;; fixed things for interaction")
        listOfObjects.append("\n\t\t arm-left arm-right - gripper")
        #listOfObjects.append("\n \t\t the-boss - user")
        listOfObjects.append("\n")
        listOfObjects.append("\n \t\t ;;; movable things")
        listOfObjects.append("\n")
        #listOfObjects.append("\n\t\t the-cake - phys-obj")
        #listOfObjects.append("\n\t\t cob4-1 - robot")
        listOfObjects.append("\n\t\t")
        listOfObjects.append("\n\t\t ".join(self.listOfObjectNames))
        #listOfObjects.append("\n\t\t the-box-1 the-box-2 the-box-3 the-box-4 the-box-5 the-box-7 - phys-obj \n")
        StringOfObjects = str(" ").join(map(str, listOfObjects))

        return StringOfObjects
    

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
        #listOfLines.append("\t\t(at the-cake room-9)")
        # test with two cake locations!
        
        listOfLines.append("\t\t(neglected cob4-1)")
        listOfLines.append("\t\t ;;; gripper")
#         listOfLines.append("\t\t(which-gripper arm-left)")
        listOfLines.append("\t\t(which-gripper arm-right)")
#         listOfLines.append("\t\t(gripper-free arm-left)")
        listOfLines.append("\t\t(gripper-free arm-right)")
        
        listOfLines.append("\n\t ;;; definitions from dynamic knowledge")
        listOfLines.append(str("\n").join(self.listOfAts))
        listOfLines.append(str("\n").join(self.listOfProps))
        #listOfLines.append("\t\t(at the-cake room-10-square-10)")
        #listOfLines.append("\t\t(at cob4-1 room-10-square-4)")
        #listOfLines.append("\t\t(at the-box-1 room-10-square-2)")
        #listOfLines.append("\t\t(at the-box-2 room-10-square-22)")
        #listOfLines.append("\t\t(at the-box-3 room-10-square-13)")
        #listOfLines.append("\t\t(at the-box-4 room-10-square-36)")
        #listOfLines.append("\t\t(at the-box-5 room-10-square-27)")
        #listOfLines.append("\t\t(at the-box-6 room-10-square-3)")
        #listOfLines.append("\t\t(at the-box-7 room-10-square-9)")
        
        #listOfLines.append("\t\t(occupied room-10-square-2)")
        #listOfLines.append("\t\t(occupied room-10-square-22)")
        listOfLines.append("\t)")
        listOfLines.append("\n")

        
        return listOfLines

    def appendGoalDefinition(self, listOfLines, goal_informations):
        print "translate the goal_informations to a goal pddl"
        print goal_informations
        
        listOfLines.append("\t;;; goal definition")
        listOfLines.append("\t(:goal (and (have the-boss the-beer) ))")
        listOfLines.append(")")
        return listOfLines
    
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
    
#     def load_static_knowledge_from_yaml(self):
#         # beachte: YAMl verwendet nur dicts und lists
#         f = open(self.path_to_inputfiles+"roomcleaning/static-knowledge-base-onlyrooms.yaml", 'r')
#         yamlfile = load(f)
#         f.close()
#         return yamlfile
        
if __name__ == '__main__':
    rospy.init_node('knowledge_parser_server_node', anonymous=False)
    kPS = KnowledgeParserServer()
    rospy.spin()
    #kPS.createProblemPDDL()

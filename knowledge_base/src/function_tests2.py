#!/usr/bin/env python
'''
Created on Feb 22, 2016

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
# ROS package name: knowledge_base
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
import cv2
import sys
import numpy as np

from yaml import load
import yaml
import knowledge_base

class Location:
    def __init__(self, name, transitions, center, area_min_max):
        self.name = name
        self.transitions = transitions
        self.center = center
        self.area_min_max = area_min_max
    def __repr__(self):
        return "%s(name=%r, transitions=%r, center=%r, area_min_max=%r)" % (self.__class__.__name__, self.name, self.transitions, self.center, self.area_min_max)

class FunctionTest2(object):
    def __init__(self):
        rospy.loginfo("Initialize FunctionTest2 ...")
        self.path_to_knowledge_base = "/home/cme/git/catkin_ws/src/ipa_pars/knowledge_base/content/"
        print self.path_to_knowledge_base
        rospy.loginfo("... finished")

    
    def writeKnowledgeBase(self, knowledge_base_as_yaml_dict):
        with open(self.path_to_knowledge_base+"knowledge-base.yaml", 'w') as yaml_file:
            yaml_file.write( yaml.dump(knowledge_base_as_yaml_dict, default_flow_style=False))
        yaml_file.close()
        
    def load_params_from_yaml(self):
        # beachte: YAMl verwendet nur dicts und lists
        # verwende die entsprechenden methoden richtig
        f = open(self.path_to_knowledge_base+"knowledge-base-template.yaml", 'r')
        yamlfile = load(f)
        f.close()
        print yamlfile
        print "------------------------"
#         location_data = yamlfile['location-data']
#         for locations in location_data:
#             location_name = locations['name']
#             location_transitions = locations['transitions']
#             print location_name+": has transitions to:"
#             print location_transitions
            #room_name = locations[1]
            #print room_name
            #room_name = locations['name']
            #print room_name
        #location2 = location_data[1]
        #print location_data
        
#     def createKnowledgebase(self):
#         knowl_as_dict = {'Name': 'Zara', 'Age': 7, 'Class': 'First'};
#         loc1 = Location(name='room-1', transitions=['room-2','room-2'], center=3.5, area_min_max=402)
#         loc2 = Location(name='room-2', transitions=['room-1','room-3'], center=3.5, area_min_max=355)
#         listOfLocations = []
#         listOfLocations.append(loc1)
#         listOfLocations.append(loc2)
#         yaml_content = yaml.dump(listOfLocations, default_flow_style=False)
# #         yaml_content = yaml.dump(loc2, default_flow_style=False)
#         print yaml_content
#         with open(self.path_to_knowledge_base+"knowledge-base.yaml", 'w') as yaml_file:
#             yaml_file.write(yaml_content)
#         #self.writeKnowledgeBase(knowl_as_dict)
#         #self.load_params_from_yaml()
#         #self.load_params_from_yaml()

    def createKnowledgebase2(self):
        name = 'room-1'
        list_of_transitions = ['room-2', 'room-3', 'room-4']
        x = 234
        y = 567
        z = 789
        dict_of_center = {'X': x, 'Y': y, 'Z': z}
        dict_of_area = {'max': {'X': x, 'Y': y, 'Z': z}, 'min': {'X': x, 'Y': y, 'Z': z}}
        dict_of_loc = {'name': name, 'transitions': list_of_transitions, 'center': dict_of_center, 'min-max': dict_of_area}
        
        listOfLocs = []
        for i in range(0, 5, 1):
            listOfLocs.append(dict_of_loc.copy())
        yaml_content = yaml.dump(listOfLocs, default_flow_style=False)
        print yaml_content
        with open(self.path_to_knowledge_base+"knowledge-base.yaml", 'w') as yaml_file:
            yaml_file.write(yaml_content)
    
if __name__ == '__main__':
    rospy.init_node('function_test_2_node', anonymous=False)
    fT2 = FunctionTest2()
    fT2.createKnowledgebase2()
        

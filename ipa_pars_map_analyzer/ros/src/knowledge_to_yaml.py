#!/usr/bin/env python
'''
Created on Sept 05, 2016

@author: rmbce
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
# ROS package name: ipa_pars_map_analyzer
#
# \author
# Author: Christian Ehrmann
# \author
# Supervised by: Richard Bormann
#
# \date Date of creation: 09.2016
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
import os


from yaml import load
import yaml

from cob_srvs.srv import SetString

import ipa_pars_map_analyzer
from ipa_pars_map_analyzer.srv import KnowledgeToYaml
from ipa_pars_map_analyzer.srv._KnowledgeToYaml import KnowledgeToYamlRequest, KnowledgeToYamlResponse


class KnowledgeToYamlNode(object):
    def __init__(self):
        rospy.loginfo("Initialize KnowledgeToYaml ...")
        knowledge_to_yaml_service = rospy.Service('knowledge_to_yaml_service', KnowledgeToYaml, self.create_knowledge_yaml_structure)
        rospy.loginfo("KnowledgeToYamlService initialize finished")
    
    def create_knowledge_yaml_structure(self, srv_msg):
        #print srv_msg
        location_data = []
        for sqr in srv_msg.square_information:
            x_center = sqr.center.x
            y_center = - sqr.center.y
            z_center = sqr.center.z
            dict_of_center = {'X': x_center, 'Y': y_center, 'Z': z_center}
            name = "room-"+str(sqr.label.data/1000)+"-square-"+str(sqr.label.data%1000)
            list_of_transitions = []
            for trans in sqr.transitions:
                list_of_transitions.append("room-"+str(trans.data/1000)+"-square-"+str(trans.data%1000))
            list_of_properties = []
            if sqr.navigable.data:
                list_of_properties.append("navigable")
            dict_of_loc = {'name': name, 'transitions': list_of_transitions, 'center': dict_of_center, 'properties': list_of_properties}
            location_data.append(dict_of_loc)
        dict_of_locations = {'location-data': location_data}
        yaml_content = yaml.dump(dict_of_locations, default_flow_style=False)
        #print yaml_content
        self.save_static_knowledge_file(yaml_content)
        response = KnowledgeToYamlResponse()
        response.success = True;
        return response

    def save_static_knowledge_file(self, yaml_content):
        print "save static knowledge file ..."
        try:
            if not os.path.isdir("ipa_pars/knowledge/"):
                os.mkdir("ipa_pars/knowledge")
            with open("ipa_pars/knowledge/static-knowledge-from-map.yaml", "w") as yaml_file:
                yaml_file.write(yaml_content)
            yaml_file.close()
        except IOError:
            rospy.loginfo("writing static-knowledge.yaml failed!")
            
            
if __name__ == '__main__':
    rospy.init_node('knowledge_to_yaml_node', anonymous=False)
    kTY = KnowledgeToYamlNode()
    rospy.spin()
        

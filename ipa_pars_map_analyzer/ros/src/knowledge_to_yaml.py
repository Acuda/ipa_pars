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


class KnowledgeToYaml(object):
    def __init__(self):
        rospy.loginfo("Initialize KnowledgeToYaml ...")
        knowledge_to_yaml_service = rospy.Service('ipa_pars_map_knowledge_to_yaml', SetString, self.knowledge_dump)
        rospy.loginfo("KnowledgeToYamlService initialize finished")
    
    def knowledge_dump(self, msg):
        print "I am the yaml dumper and I received this"
        print msg.data
        self.save_static_knowledge_file(msg.data)

    def save_static_knowledge_file(self, yaml_content):
        print "save static knowledge file"
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
    kTY = KnowledgeToYaml()
    rospy.spin()
        

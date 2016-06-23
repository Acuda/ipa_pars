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
# \date Date of creation: 05.2016
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


class KnowledgeParserClient(object):
    def __init__(self):
        rospy.loginfo("Initialize KnowledgeParserClient ...")
        rospy.loginfo("... starting knowledge_parser_server")
        self._knowledgeParserClient = actionlib.SimpleActionClient('knowledge_parser_server', KnowledgeParserAction)
        rospy.logwarn("Waiting for KnowledgeParserServer to come available ...")
        self._knowledgeParserClient.wait_for_server()
        rospy.logwarn("Server is online!")
        rospy.loginfo("KnowledgeParserClient initialize finished")

        
    def sendGoal(self):
        knowledge_goal = ipa_pars_main.msg.KnowledgeParserGoal()
        knowledge_goal.static_knowledge.data = yaml.dump(self.readKnowledgeBase("static-knowledge-base.yaml"))
        print knowledge_goal.static_knowledge.data
        knowledge_goal.dynamic_knowledge.data = yaml.dump(self.readKnowledgeBase("dynamic-knowledge-base.yaml"))
        rospy.loginfo("Sending goal to KnowledgeParserServer ...")
        self._knowledgeParserClient.send_goal(knowledge_goal)
        rospy.loginfo("Waiting for result ...")
        self._knowledgeParserClient.wait_for_result()
        result = self._knowledgeParserClient.get_result()
        rospy.loginfo("Received the result from KnowledgeParserServer!")
        rospy.loginfo(result)
    
    def readKnowledgeBase(self, knowledge_yaml):
        listOfInput = []
        try:
            if os.path.isdir("ipa_pars/knowledge/"):
                fileObject = open("ipa_pars/knowledge/"+knowledge_yaml, "r")
                yamlfile = load(fileObject)
                fileObject.close()
                return yamlfile
        except IOError:
            rospy.loginfo("Reading %s base failed!" % knowledge_yaml)
        return None
    
        
if __name__ == '__main__':
    rospy.init_node('knowledge_parser_client_node', anonymous=False)
    kPC = KnowledgeParserClient()
    kPC.sendGoal()
    #kPS.createProblemPDDL()

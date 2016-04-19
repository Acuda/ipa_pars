#!/usr/bin/env python
'''
Created on Jan 28, 2016

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
# ROS package name: map_analyzer
#
# \author
# Author: Christian Ehrmann
# \author
# Supervised by: Richard Bormann
#
# \date Date of creation: 01.2016
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
import numpy as np
import cv

from map_analyzer.msg._MapAnalyzerAction import *

from sensor_msgs.msg._Image import Image
import color_utils_cme


from cv_bridge import CvBridge, CvBridgeError
import actionlib
# Important notice: class and/or filename must not be same package name!


from geometry_msgs.msg import Pose
from cv2 import CV_8U


class KnowledgeExtractorServer(object):
    _feedback = map_analyzer.msg.MapAnalyzerFeedback()
    _result = map_analyzer.msg.MapAnalyzerResult()
    def __init__(self):
        rospy.loginfo("Initialize KnowledgeExtractorServer ...")
        rospy.loginfo("... starting room_segmentation_client")
        
        self.bridge = CvBridge()
        
        self._as = actionlib.SimpleActionServer('knowledge_extractor_server', map_analyzer.msg.MapAnalyzerAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        
        rospy.loginfo("KnowledgeExtractorServer running! Waiting for a new map to analyze ...")
        rospy.loginfo("KnowledgeExtractorServer initialize finished")
        
    def execute_cb(self, goal):
        rospy.loginfo("Extracting knowledge from a new map!")
        rospy.loginfo("header");
        print goal.input_map.header
        rospy.loginfo("goal in progress ...")
        r = rospy.Rate(1)

        received_map_as_cvimg = self.bridge.imgmsg_to_cv2(goal.input_map).copy()

        
        print "i am sleeping now"
        success = True
        self._result = "my yaml file text is here from knowledgeExtractor"
        rospy.sleep(5)
        #===========================
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % 'map_analyzer_server')
            result = "there is no yaml file text here"
        r.sleep()
        if success:
            self._result.static_knowledge.data = result
            rospy.loginfo("Produced fundamental static knowledge")
            self._as.set_succeeded(self._result, "good job")

        
if __name__ == '__main__':
    rospy.init_node('knowledge_extractor_server_node', anonymous=False)
    kES = KnowledgeExtractorServer()
    rospy.spin()
        
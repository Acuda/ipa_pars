#!/usr/bin/env python
'''
Created on Feb 02, 2016

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
import actionlib
import rospy

import cv2
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from map_analyzer.msg._MapAnalyzerAction import *
from geometry_msgs.msg._Pose import Pose
import map_analyzer

class MapAnalyzerClient(object):
    def __init__(self, path_to_map):
        rospy.loginfo("Initialize MapAnalyzerClient ...")
        self.path_to_map = path_to_map+"lab_ipa4.png"
        rospy.loginfo(path_to_map)
        rospy.loginfo("... loading map from file")
        self.input_map = cv2.imread(self.path_to_map, cv2.IMREAD_GRAYSCALE)
        rospy.loginfo("converting map to sensor_msg ...")
        self.bridge = CvBridge()
        self.output_map = self.bridge.cv2_to_imgmsg(self.input_map, "mono8")
        #self.output_map = self.bridge.cv2_to_imgmsg(self.input_map, "bgr8")
        rospy.loginfo("... done!")
        self._aclient = actionlib.SimpleActionClient('map_analyzer_server', map_analyzer.msg.MapAnalyzerAction)
        rospy.logwarn("Waiting for MapAnalyzerServer to come available ...")
        self._aclient.wait_for_server()
        rospy.logwarn("Server is online.")
        rospy.loginfo("Initialize MapAnalyzerClient finished")

    def sendGoalAndWait(self):
        rospy.loginfo("Create goal ...")
        output_img = Image()
        output_img.header.stamp = rospy.Time.now()
        output_img.header.frame_id = "demo_map_init_frame"
        output_img.height = self.input_map.shape[0]
        output_img.width = self.input_map.shape[1]
        output_img.encoding = "mono8"
        #output_img.encoding = "rgb8"
        output_img.is_bigendian = False
        output_img.step = output_img.width * 3
        output_img.data = self.output_map.data

        goal = map_analyzer.msg.MapAnalyzerGoal()
        goal.input_map = output_img
        goal.map_resolution = 0.05
        goal.map_origin = Pose()
        goal.map_origin.position.x = 0.0
        goal.map_origin.position.y = 0.0
        goal.map_origin.position.z = 0.0
        goal.map_origin.orientation.x = 0.0
        goal.map_origin.orientation.y = 0.0
        goal.map_origin.orientation.z = 0.0
        goal.map_origin.orientation.w = 1.0

        rospy.loginfo("Sending goal ...")
        self._aclient.send_goal(goal)
        rospy.loginfo("Waiting for result ...")
        self._aclient.wait_for_result()
        result = self._aclient.get_result()
        rospy.loginfo("Received the result:")
        rospy.loginfo(result)

if __name__ == '__main__':
    rospy.init_node('demo_map_init_client_node', anonymous=False)
    mAC = MapAnalyzerClient(sys.argv[1])
    try:
        mAC.sendGoalAndWait()
    except rospy.ROSInterruptException:
        print "demo_map_init_client_node was interrupted!"
        

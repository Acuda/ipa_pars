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
# ROS package name: ipa_pars_main
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
import actionlib
import rospy
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ipa_pars_main.msg._LogicPlanAction import *

from sensor_msgs.msg._Image import Image


class ParsServer(object):
    _feedback = ipa_pars_main.msg.LogicPlanFeedback()
    _result = ipa_pars_main.msg.LogicPlanResult()
    def __init__(self, path_to_map):
        rospy.loginfo("Initialize ParsServer ...")
        self._path_to_map = path_to_map
        rospy.loginfo("path_to_map: %s" % path_to_map)
        rospy.loginfo("Reading map from disk ...")
        self.input_map = cv2.imread(self._path_to_map, cv2.IMREAD_COLOR)
        rospy.loginfo("converting map to sensor_msg ...")
        self.bridge = CvBridge()
        self.output_map = self.bridge.cv2_to_imgmsg(self.input_map, "bgr8")
        rospy.loginfo("... done!")
        rospy.logwarn("Waiting for map_analyzer_service_server to come available ...")
        rospy.wait_for_service('map_analyzer_service_server')
        rospy.logwarn("Server online!")
        try:
            self.serviceClient = rospy.ServiceProxy('map_analyzer_service_server', Image)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self._as = actionlib.SimpleActionServer('pars_server', ipa_pars_main.msg.LogicPlanAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        
        rospy.loginfo("ParsServer running! Waiting for a new goal.")

    def execute_cb(self, goal):
        rospy.loginfo("Executing a new goal!")
        rospy.loginfo("GOAL: %s , %s, %s " % (str(goal.goal_type), str(goal.what), str(goal.where)))
        rospy.loginfo("in progress ...")
        r = rospy.Rate(1)
        # change herer TODO:
        #===========================
        # robot stuff here
        success = self.sendImageToMapAnalyzerServer()
        #===========================
        
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % 'pars_server')
            success = False
            
        
        r.sleep()
        
        if success:
            self._result.success = True
            rospy.loginfo("Succeeded the Logic Plan")
            self._as.set_succeeded(self._result, "good job")
            
    def sendImageToMapAnalyzerServer(self):
        answer = self.serviceClient(self.img_as_message)
        print "answer"
        print answer
        return answer

if __name__ == '__main__':
    rospy.init_node('planning_server', anonymous=False)
    pARS = ParsServer(sys.argv[1])
    rospy.spin()

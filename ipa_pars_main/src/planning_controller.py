#!/usr/bin/env python
'''
Created on Feb 10, 2016

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
import cv2
import numpy as np
from sensor_msgs.msg import Image
#from sensor_msgs.msg._Image import Image

import actionlib

from cob_srvs.srv._SetString import SetString
from cob_srvs.srv._SetString import SetStringResponse

from ipa_pars_main.srv._PlanData import PlanData, PlanDataResponse, PlanDataRequest

class PlanningController(object):
    def __init__(self):
        rospy.loginfo("Initialize PlanningController ...")
        self.controller_srv = rospy.Service('planning_controller_server', PlanData, self.handle_controller_cb)
        rospy.logwarn("Waiting for room_information_server to come available ...")
        rospy.wait_for_service('room_information_server')
        rospy.logwarn("Server online!")
        rospy.logwarn("Waiting for planning_goal_server to come available ...")
        rospy.wait_for_service('planning_goal_server')
        rospy.logwarn("Server online!")
        rospy.logwarn("Waiting for planning_domain_server to come available ...")
        rospy.wait_for_service('planning_domain_server')
        rospy.logwarn("Server online!")
        try:
            self.room_info_client = rospy.ServiceProxy('room_information_server', SetString)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        try:
            self.planning_goal_server = rospy.ServiceProxy('planning_goal_server', SetString)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        try:
            self.planning_domain_server = rospy.ServiceProxy('planning_domain_server', SetString)
        except rospy.ServiceException, e:
            print "Service call faield: %s"%e
        rospy.loginfo("PlanningController is running, waiting for new problems to control ...")
        rospy.loginfo("... finished")
        
    def handle_controller_cb(self, controller_info):
        print "controller_info"
        print controller_info
        #==========================================================
        answer1 = self.room_info_client(controller_info.room_data.data)
        print answer1
        answer2 = self.planning_goal_server(controller_info.goal_data.data)
        print answer2
        answer3 = self.planning_domain_server(controller_info.domain_data.data)
        print answer3
        #==============================================
        
        #answer = self.domain_server_client("whatever")
        
        #print answer
        response = PlanDataResponse()
        response.answer.data = "Antwort vom Controller"
        
        return response

if __name__ == '__main__':
    rospy.init_node('planning_controller_node', anonymous=False)
    pC = PlanningController()
    rospy.spin()
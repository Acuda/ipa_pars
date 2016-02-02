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
# ROS package name: map_publisher
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
from sensor_msgs.msg import Image
#from sensor_msgs.msg._Image import Image


from cv_bridge import CvBridge, CvBridgeError
import actionlib

from map_analyzer.srv import MapAnalyzer
from map_analyzer.srv._MapAnalyzer import MapAnalyzerResponse, MapAnalyzerRequest



class MapPublisher(object):
    def __init__(self):
        rospy.loginfo("Initialize MapPublisher ...")
        rospy.loginfo("... starting map_analyzer_publisher")
        self.map_srvs = rospy.Service('map_publisher_server', MapAnalyzer, self.handle_map_cb)
        self.map_img_pub = rospy.Publisher('map_status', Image, queue_size=1)
        self.img_map = Image()

        rospy.loginfo("... finished")
        
    def handle_map_cb(self, input_map):
        print "print recieved map header:"
        print input_map.map.header
        
        response = MapAnalyzerResponse()
        self.img_map = input_map.map
        response.answer.data = "saftige Map Antwort!"
        
        return response
    
    def run(self):
        self.map_img_pub.publish(self.img_map)
        r = rospy.Rate(10)
        r.sleep()
        
if __name__ == '__main__':
    rospy.init_node('map_publisher_node', anonymous=False)
    mP = MapPublisher()
    while not rospy.is_shutdown():
        mP.run()
        
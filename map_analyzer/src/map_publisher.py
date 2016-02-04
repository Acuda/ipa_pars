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
import numpy as np
from sensor_msgs.msg import Image
#from sensor_msgs.msg._Image import Image


from cv_bridge import CvBridge, CvBridgeError
import actionlib

from map_analyzer.srv import MapAnalyzer
from map_analyzer.srv._MapAnalyzer import MapAnalyzerResponse, MapAnalyzerRequest

import color_utils_cme

class MapPublisher(object):
    def __init__(self):
        rospy.loginfo("Initialize MapPublisher ...")
        rospy.loginfo("... starting map_analyzer_publisher")
        self.map_srvs = rospy.Service('map_publisher_server', MapAnalyzer, self.handle_map_cb)
        self.map_img_pub = rospy.Publisher('map_status', Image, queue_size=1)
        self.img_map = Image()
        self.bridge = CvBridge()
        #for image conversion:
        self.colorlist = []
        self.colorScale = 10
        self.colorlist = color_utils_cme.randomColor(self.colorScale)
        self.colorlist = color_utils_cme.shuffle_list(self.colorlist)
        self.usedColors = []
        rospy.loginfo("... finished")
        
    def handle_map_cb(self, input_map):
        print "print recieved map header:"
        print input_map.map.header
        print "encoding"
        print input_map.map.encoding
        response = MapAnalyzerResponse()
        if input_map.map.encoding == "32SC1":
            self.img_map = self.encodeImage(input_map.map)
        else:
            self.img_map = input_map.map
            
        response.answer.data = "MapPublisher received a new map!"
        
        return response
    
    '''
    ###########################################################################
    #
    #                    image encoding format 32SC1 to BGR
    #
    ###########################################################################
    #
    #    definition of message type:
    #
    #    the action server need a map as a input image to segment it,
    #    format 32SC1, room labels from 1 to N,
    #    room with label i -> access to room_information_in_pixel[i-1]
    # sensor_msgs/Image segmented_map
    #    the resolution of the segmented map in [meter/cell]
    # float32 map_resolution
    #    the origin of the segmented map in [meter]
    # geometry_msgs/Pose map_origin
    #    for the following data: value in pixel can be obtained when
    #    the value of [return_format_in_pixel] from goal definition is true
    #    room data (min/max coordinates, center coordinates) measured in pixels
    #    for the following data: value in meter can be obtained when the value
    #    of [return_format_in_meter] from goal definition is true
    # ipa_room_segmentation/RoomInformation[] room_information_in_pixel
    #    room data (min/max coordinates, center coordinates) measured in meters
    # ipa_room_segmentation/RoomInformation[] room_information_in_meter
    '''
    def encodeImage(self, img_msg):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough").copy()
        cv_enc_img = np.zeros((cv_img.shape[0], cv_img.shape[1] , 3), np.uint8) # BGR
        
        listOfDifColors = []
        for w in range (0, cv_img.shape[1], 1):
            for h in range(0, cv_img.shape[0], 1):
                colorvalue = cv_img[h][w]
                if colorvalue not in listOfDifColors:
                    listOfDifColors.append(colorvalue)
        print listOfDifColors
        print "colorade labes:"
        listOfColLab = []
        for lab in listOfDifColors:
            labcol = []
            labcol.append(lab)
            if lab == 65280:
                col = [255,255,255]
            elif lab == 0:
                col = [0,0,0]
            else:
                col = self.colorlist.pop()
            labcol.append(col)
            listOfColLab.append(labcol)
        print listOfColLab
        
        pix_col = [255,255,255]
        for w in range (0, cv_img.shape[1], 1):
            for h in range (0, cv_img.shape[0], 1):
                value = cv_img[h][w]
                #print "listOfColLab index value:"
                
                for item in listOfColLab:
                    if value == item[0]:
                        pix_col = item[1]
                    
                #print listOfColLab.index(value)
                #pix_col = listOfColLab[listOfColLab[0].index(value)][:]
                cv_enc_img[h,w,:] = pix_col
        cv_enc_img_msg = self.bridge.cv2_to_imgmsg(cv_enc_img, "bgr8")
        return cv_enc_img_msg
    
    def run(self):
        self.map_img_pub.publish(self.img_map)
        r = rospy.Rate(10)
        r.sleep()
        
if __name__ == '__main__':
    rospy.init_node('map_publisher_node', anonymous=False)
    mP = MapPublisher()
    while not rospy.is_shutdown():
        mP.run()
        
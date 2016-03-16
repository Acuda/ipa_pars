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
import roslib
import cv2
import sys
import colorsys
import numpy as np
from sensor_msgs.msg import Image
#from sensor_msgs.msg._Image import Image


from cv_bridge import CvBridge, CvBridgeError
import actionlib

from map_analyzer.srv import MapAnalyzer
from map_analyzer.srv._MapAnalyzer import MapAnalyzerResponse, MapAnalyzerRequest

import color_utils_cme

class MapPublisher(object):
    def __init__(self, path_to_logfile):
        rospy.loginfo("Initialize MapPublisher ...")
        self.path_to_logfile = path_to_logfile
        rospy.loginfo(path_to_logfile)
        rospy.loginfo("... starting map_analyzer_publisher")
        self.map_srvs = rospy.Service('map_publisher_server', MapAnalyzer, self.handle_map_cb)
        self.map_img_pub = rospy.Publisher('map_status', Image, queue_size=1)
        self.img_map = Image()
        self.bridge = CvBridge()
        #for image conversion:
        self.colorlist = []
        self.colorScale = 10
        self.colorlist = color_utils_cme.listColor(self.colorScale)
        #self.colorlist = color_utils_cme.shuffle_list(self.colorlist)
        self.usedColors = []
        self.counter = 0
        print "produceIndividualColors"
        (listOfRoomCol, listOfSquareCol) = self.produceRoomAndSquareColors(10)
        self.listOfRoomCol = listOfRoomCol
        self.listOfSquareCol = listOfSquareCol
        print "listOfRoomCol"
        print listOfRoomCol
        print "listOfSquareCol"
        print listOfSquareCol
        
        rospy.loginfo("... finished")
        
        
        
    def drawWhiteLines(self, img):
        for w in range (0, img.shape[1], 1):
            for h in range (0, img.shape[0], 1):
                if not img[h,w] == 0:
                    if not img[h,w] == img[h,w+1]:
                        if not img[h,w+1] == 0:
                            img[h,w] = 65530
                    if not img[h,w] == img[h+1,w]:
                        if not img[h+1,w] == 0:
                            img[h,w] = 65530
        return img
        
    def handle_map_cb(self, input_map):
        print "print recieved map header:"
        print input_map.map.header
        print "encoding"
        print input_map.map.encoding
        response = MapAnalyzerResponse()
        if input_map.map.encoding == "32SC1":
            self.img_map = self.encodeImage(input_map.map)
            self.saveLogImage(self.img_map)
        elif input_map.map.encoding == "16UC1":
            self.img_map = self.encodeImage(input_map.map)
            self.saveLogImage(self.img_map)
        else:
            self.img_map = input_map.map
            self.saveLogImage(self.img_map)
            
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
        
        cv_img = self.drawWhiteLines(cv_img)
        listOfDifColors = []
        for w in range (0, cv_img.shape[1], 1):
            for h in range(0, cv_img.shape[0], 1):
                colorvalue = cv_img[h][w]
                if colorvalue not in listOfDifColors:
                    listOfDifColors.append(colorvalue)
        print listOfDifColors
        
        listOfColWithoutError = []
        for colors in listOfDifColors:
            if colors < 65500:
                listOfColWithoutError.append(colors)
#         cv_enc_img[np.where(cv_img==0)] = [0,0,0]
#         listOfDifColors.remove(0)
#         if 65501 in listOfDifColors:
#             cv_enc_img[np.where(cv_img==65501)] = [0,0,0]
#             listOfDifColors.remove(65501)
#         if 65502 in listOfDifColors:
#             cv_enc_img[np.where(cv_img==65502)] = [0,0,0]
#             listOfDifColors.remove(65502)
#         if 65503 in listOfDifColors:
#             cv_enc_img[np.where(cv_img==65503)] = [0,0,0]
#             listOfDifColors.remove(65503)
#         if 65280 in listOfDifColors:
#             cv_enc_img[np.where(cv_img==65280)] = [0,0,0]
#             listOfDifColors.remove(65280)
        if len(listOfDifColors) == 2:
            #cv_enc_img[np.where(cv_img==0)] = [0,0,0]
            cv_enc_img[np.where(cv_img==255)] = [255,255,255]
            cv_enc_img_msg = self.bridge.cv2_to_imgmsg(cv_enc_img, "bgr8")
            return cv_enc_img_msg

        elif max(listOfColWithoutError) < 1500:
            print listOfDifColors
            print "erkannt zahl zwischen 0 - 1500"
            highestcol = max(listOfColWithoutError)
            print "highestcol = %d" % highestcol
            colordivisor = 1.0 / float(highestcol) 
            print colordivisor
            listOfColLab = []
            for lab in listOfDifColors:
                labcol = []
                labcol.append(lab)
                if lab == 0:
                    col = [0,0,0]
                elif lab == 65501:
                    col = [0,0,0]
                elif lab == 65502:
                    col = [0,0,0]
                elif lab == 65503:
                    col = [0,0,0]
                else:
                    col_h = float(lab) * colordivisor
                    col_s = 1
                    col_v = 0.8
                    print "lab = %f to color = %f, %f, %f" % (lab, col_h, col_s, col_v)
                    color_in_dec = colorsys.hsv_to_rgb(col_h, col_s, col_v)
                    print "as dec = %f , %f , %f " % (color_in_dec[0], color_in_dec[1], color_in_dec[2])
                    col = [int(round(color_in_dec[0] * 255)),int(round(color_in_dec[1] * 255)),int(round(color_in_dec[2] * 255))]
                    print "which is rgb"
                    print col
                labcol.append(col)
                listOfColLab.append(labcol)
        else:
            print "erkannt zahl groesser 1500"
            highestcol = (max(listOfColWithoutError) / 1000)
            print "highestcol = %d" % highestcol
            colordivisor = 1.0 / float(highestcol) 
            print colordivisor

            listOfColLab = []
            for lab in listOfDifColors:
                labcol = []
                labcol.append(lab)
                if lab == 0:
                    col = [0,0,0]
                elif lab == 65501:
                    col = [0,0,0]
                elif lab == 65502:
                    col = [0,0,0]
                elif lab == 65503:
                    col = [0,0,0]
                elif lab == 65530:
                    col = [255,255,255]
                else:
                    col_h = (float(lab) / 1000) * colordivisor
                    col_v = 1
                    col_s = 1
#                     col_v = float(lab) / 10000
#                     col_s = float(lab) / 10000
                    print "lab = %f to color = %f, %f, %f" % (lab, col_h, col_s, col_v)
                    color_in_dec = colorsys.hsv_to_rgb(col_h, col_s, col_v)
                    print "as dec = %f , %f , %f " % (color_in_dec[0], color_in_dec[1], color_in_dec[2])
                    col = [int(round(color_in_dec[0] * 255)),int(round(color_in_dec[1] * 255)),int(round(color_in_dec[2] * 255))]
                    print "which is rgb"
                    print col
                labcol.append(col)
                listOfColLab.append(labcol)
#         listOfRooms = []
#         for roomcol in listOfDifColors:
#             val = (roomcol // 1000)
#             if (val < 65):
#                 listOfRooms.append(val)
#         highestCol = max(listOfRooms)
#         divis = highestCol // 360
        
        
#         print "colorade labes:"
#         listOfColLab = []
#         for lab in listOfDifColors:
#             labcol = []
#             labcol.append(lab)
#             if lab == 65280:
#                 col = [255,255,255]
#             elif lab == 0:
#                 col = [0,0,0]
#             else:
#                 col = self.colorlist.pop()
# #                 col = self.listOfRoomCol.pop()
# #                 col_value = listOfDifColors.pop()
# #                 print "das label ist"
# #                 print col_value
# #                 col_h = ((col_value // 1000) * divis)
# #                 col_s = ((col_value % 1000) / 100)
# #                 col_v = 1
# #                 print "errechnete hsv:"
# #                 print "%d , %d , %d " % (col_h, col_s, col_v)
# #                 col = colorsys.hsv_to_rgb(col_h, col_s, col_v)
# #                 print "errechnete RGB Farbe ist:"
# #                 print col
#                 
#             labcol.append(col)
#             listOfColLab.append(labcol)
#         print listOfColLab
        
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

#     def createHSVcolor(self, label):
#         h_div = label // 1000
#         
        
    def saveLogImage(self, img):
        self.counter += 1
        path_to_logimage = self.path_to_logfile+"logimg"+str(self.counter)+".png"
        print "++++++++++++++++++ try to save image +++++++++++++++++++++++"
        print path_to_logimage
        newImg = self.bridge.imgmsg_to_cv2(img, "bgr8")
        try:
            # write image as png [0] no compression --> [10] full compression
            cv2.imwrite(path_to_logimage, newImg, [cv2.IMWRITE_PNG_COMPRESSION, 10])
            print "write loginfo successful"
        except:
            e = sys.exc_info()[0]
            print e

        
    def produceRoomAndSquareColors(self, number_of_rooms):
        listOfRoomColors = []
        listOfRoomColorsAsInt = []
        listOfSquareColors = []
        listOfSquareColorsAsInt = []
        divisor = 65000 / (number_of_rooms)
        for i in range (divisor, 65001, divisor):
            color = []
            color.append(i // 256 // 256 % 256)
            color.append(i // 256 % 256)
            color.append(i % 256)
            listOfRoomColors.append(color)
            listOfRoomColorsAsInt.append(i)
        #colInt = int('%02x%02x%02x' % (color[0], color[1], color[2]), 16)
        #mostimportantColor = (mostimportantColorInt[0][0] // 256 // 256 % 256, mostimportantColorInt[0][0] // 256 % 256, mostimportantColorInt[0][0] % 256)
        # producte Square Colors:
        # schaetze anzahl quadrate grob auf max 20
        squares_div = divisor / 20
        for k in range(0, len(listOfRoomColorsAsInt)-1, 1):
            for l in range(listOfRoomColorsAsInt[k], listOfRoomColorsAsInt[k+1]-squares_div, squares_div):
                color = []
                color.append(l // 256 // 256 % 256)
                color.append(l // 256 % 256)
                color.append(l % 256)
                listOfSquareColors.append(color)
                listOfSquareColorsAsInt.append(l)
        return (listOfRoomColors, listOfSquareColors)
        

    def run(self):
        self.map_img_pub.publish(self.img_map)
        r = rospy.Rate(10)
        r.sleep()
        
if __name__ == '__main__':
    rospy.init_node('map_publisher_node', anonymous=False)
    mP = MapPublisher(sys.argv[1])
    while not rospy.is_shutdown():
        mP.run()
        
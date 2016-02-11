#!/usr/bin/env python
'''
Created on Feb 04, 2016

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
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
#from sensor_msgs.msg._Image import Image


from cv_bridge import CvBridge, CvBridgeError
import actionlib

from map_analyzer.srv import MapAnalyzer
from map_analyzer.srv._MapAnalyzer import MapAnalyzerResponse, MapAnalyzerRequest
import sensor_msgs
from gi.overrides.Gdk import Color


class MapTesselation(object):
    def __init__(self):
        rospy.loginfo("Initialize MapTesselation ...")
        self.map_srvs = rospy.Service('map_tesselation_service_server', MapAnalyzer, self.handle_map_cb)
        
        rospy.logwarn("Waiting for map_publisher_service_server to come available ...")
        rospy.wait_for_service('map_publisher_server')
        rospy.logwarn("Server online!")
        try:
            self.serviceMapPublisherClient = rospy.ServiceProxy('map_publisher_server', MapAnalyzer)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        self.bridge = CvBridge()
        self.numbpix = 0
        # change alogrithm to one that fits best to the rooms?
        self.squaresize = 60
        rospy.loginfo("... finished")
        
    def handle_map_cb(self, input_map):
        print "print recieved map header:"
        print input_map.map.header
        print "encoding"
        print input_map.map.encoding
        cv_img = self.bridge.imgmsg_to_cv2(input_map.map, desired_encoding="passthrough").copy()
        cv_img = self.tesselateMap(cv_img)
        # create new sensor_msgs/Image:
        output = Image()
        output.header = input_map.map.header
        output.encoding = input_map.map.encoding
        output.height = input_map.map.height
        output.width = input_map.map.width
        cv_enc_img_msg = self.bridge.cv2_to_imgmsg(cv_img)
        
        answer = self.serviceMapPublisherClient(cv_enc_img_msg)
        print answer
        
        response = MapAnalyzerResponse()
        response.answer.data = "MapPublisher received a new map!"
        return response

    def tesselateMap(self, map_img):
        newMap = map_img
        print "deleteWhitePixel"
        newMap = self.deleteWhitePixels(newMap)
        print "makeListOfAreasAndPixels and delete"
        newMap = self.makeListOfAreasAndPixels(newMap)
        # room preparation done
        print "room preparation done"
        print "send to publisher"
        cv_enc_img_msg = self.bridge.cv2_to_imgmsg(newMap)
        
        answer = self.serviceMapPublisherClient(cv_enc_img_msg)
        print answer
        print "draw squares"
        newMap = self.createSquares(newMap)
        print "fill squares"
        newMap = self.fillSquares(newMap)
        print "deleteSquaresHorizontal"
        newMap = self.deleteSquaresHorizontal(newMap)
        print "deleteSuqaresVertical"
        newMap = self.deleteSquaresVertical(newMap)
        print "mergeSmallAreas"
        #print listOfAreasAndPix
        #newMap = self.deleteDoubleAreas(newMap)
        #newMap = self.correctSmallAreas(map_img)
        #for list_of_mapcolors do -->
#         newMap = self.tesselateRooms(newMap, 9) # 14
#         newMap = self.createSquares(newMap)
#         newMap = self.fillSquares(newMap)
#         newMap = self.deleteSquaresVertical(newMap)
#         newMap = self.deleteSquaresHorizontal(newMap)
#         listOfSmallAreas = self.findSmallAreas(newMap)
#         print "listOfSmallAreas"
#         print listOfSmallAreas
#         newMap = self.mergeSmallAreas(newMap, listOfSmallAreas)
        
                 
        return newMap


    def deleteWhitePixels(self, map_img):
        for w in range (0, map_img.shape[1], 1):
            for h in range (0, map_img.shape[0], 1):
                if map_img[h][w] == 65280:
                    map_img[h][w] = 0
        return map_img
    
    def makeListOfAreasAndPixels(self, map_img):
        listOfAreasAndPixels = []
        listOfAlreadyFinished = []
        # dont check black pixels
        listOfAlreadyFinished.append(0)
        map_copy = map_img.copy()
        for w in range (0, map_copy.shape[1], 1):
            for h in range (0, map_copy.shape[0], 1):
                if not map_copy[h][w] in listOfAlreadyFinished:
                    print "found region with color = %d" % map_copy[h][w]
                    #listOfAlreadyFinished.append(map_copy[h][w])
                    listOfAlreadyFinished.append(map_copy[h][w]+1000)
                    pixelcounter = 0
                    (map_copy, pixelcounter) = self.floodfill4(h, w, map_copy[h][w], map_copy[h][w]+1000, map_copy, pixelcounter)
                    if pixelcounter < 500:
                        print "deleting area because its too small"
                        (map_img, pixelcounter) = self.floodfill4(h, w, map_img[h][w], 0, map_img, pixelcounter)
                    listOfAreasAndPixels.append((map_copy[h][w], pixelcounter))
        
        print listOfAreasAndPixels
        return map_img

#     def mergeSmallAreas(self, map_img, listOfAreas):
#         newMap = map_img
#         while not (len(listOfAreas) == 0): # list empty
#             (color, pixelcount) = listOfAreas.pop()
#             print "color"
#             print color
#             print "pixelcount"
#             print pixelcount
#             if pixelcount < 200:
#                 for w in range (0, newMap.shape[1], 1):
#                     for h in range (0, newMap.shape[0], 1):
#                         if newMap[h][w] == color+1000:
#                             stackOfNeighbours = self.floodfindNeighbours(h, w, color, 1, newMap)
#                             print "color"
#                             print color
#                             print "has stack of Neighbours:"
#                             print stackOfNeighbours
#                             listOfOccasions = []
#                             for item in stackOfNeighbours:
#                                 listOfOccasions.append(stackOfNeighbours.count(item))
#                             newColor = stackOfNeighbours[listOfOccasions.index(max(listOfOccasions))]
#                             newMap = self.floodfill4(h, w, color, newColor, newMap)
#                 
#         return newMap
#         
    def deleteSquaresVertical(self, map_img):
        newMap = map_img
        for w in range (0, newMap.shape[1], 1):
            for h in range (0, newMap.shape[0], 1):
                if newMap[h][w] == 65220:
#                     if not newMap[h-1][w] == 0 or newMap[h-1][w] == 65220:
#                         newMap[h][w] = newMap[h-1][w]
#                     elif not newMap[h+1][w] == 0 or newMap[h+1][w] == 65220:
#                         newMap[h][w] = newMap[h+1][w]
                    if not newMap[h][w+1] == 0 or newMap[h][w+1] == 65220:
                        newMap[h][w] = newMap[h][w+1]
                    elif not newMap[h][w-1] == 0 or newMap[h][w-1] == 65220:
                        newMap[h][w] = newMap[h][w-1]
        return newMap
#     
    def deleteSquaresHorizontal(self, map_img):
        newMap = map_img
        for w in range (0, newMap.shape[1], 1):
            for h in range (0, newMap.shape[0], 1):
                if newMap[h][w] == 65221:
                    if not newMap[h-1][w] == 0 or newMap[h-1][w] == 65221:
                        newMap[h][w] = newMap[h-1][w]
                    elif not newMap[h+1][w] == 0 or newMap[h+1][w] == 65221:
                        newMap[h][w] = newMap[h+1][w]
#                     elif not newMap[h][w+1] == 0 or newMap[h][w+1] == 65221:
#                         newMap[h][w] = newMap[h][w+1]
#                     elif not newMap[h][w-1] == 0 or newMap[h][w-1] == 65221:
#                         newMap[h][w] = newMap[h][w-1]
        return newMap
# 
#     
    def fillSquares(self, map_img):
        newMap = map_img
        color = 0
        counter = 1
        workedOnColors = []
        for w in range (0, newMap.shape[1], 1):
            for h in range (0, newMap.shape[0], 1):
                if not newMap[h][w] == 0:
                    if not newMap[h][w] == 65220 and not newMap[h][w] == 65221:
                        if color == 0:
                            color = newMap[h][w] * 1000
                        if not newMap[h][w] in workedOnColors:
                            newValue = color + counter
                            counter += 1
                            oldValue = newMap[h][w]
                            workedOnColors.append(newValue)
                            pixelcounter = 0
                            (newMap, pixelcounter) = self.floodfill4(h, w, oldValue, newValue, newMap, pixelcounter)
        return newMap
#     
    def createSquares(self, room_img):
        for w in range (0, room_img.shape[1], 1):
            for h in range (0, room_img.shape[0], self.squaresize):
                if not room_img[h][w] == 0:
                    room_img[h][w] = 65221
         
        for w in range (0, room_img.shape[1],  self.squaresize):
            for h in range (0, room_img.shape[0], 1):
                if not room_img[h][w] == 0:
                    room_img[h][w] = 65220
         
        return room_img
#     
#     def findSmallAreas(self, map_img):
#         newMap = map_img.copy()
#         listOfCol = []
#         listOfArea = []
#         workedOnColors = []
#         listOfColAndArea = []
#         for w in range (0, newMap.shape[1], 1):
#             for h in range (0, newMap.shape[0], 1):
#                 if not newMap[h][w] == 0:
#                     if not newMap[h][w] in listOfCol:
#                         if not newMap[h][w] in workedOnColors:
#                             listOfCol.append(newMap[h][w])
#                             oldValue = newMap[h][w]
#                             newValue = oldValue + 1000
#                             workedOnColors.append(newValue)
#                             self.numbpix = 0
#                             self.floodfill4(h, w, oldValue, newValue, newMap)
#                             listOfArea.append(self.numbpix)
#                             stack = []
#                             stack.append(oldValue)
#                             stack.append(self.numbpix)
#                             listOfColAndArea.append(stack)
#         return listOfColAndArea
#     
#     def correctSmallAreas(self, map_img):
#         newMap = map_img
#         for w in range (0, newMap.shape[1], 1):
#             for h in range (0, newMap.shape[0], 1):
#                 if not newMap[h][w] == 0:
#                     if newMap[h][w] == 65280:
#                         newMap[h][w] = 0
#                     elif (newMap[h+1][w] == 0) and (newMap[h-1][w] == 0) and (newMap[h][w+1] == 0) and (newMap[h][w-1] == 0):
#                         newMap[h][w] = 0
#         listOfColorValues = self.makeListOfColorValues(newMap)
#         colorcounter = max(listOfColorValues)
#         print "colorcounter"
#         print colorcounter
#         max_perc = newMap.shape[1] * newMap.shape[0]
#         workedOnColors = []
#         percentagecounter = 0
#         highestcolor = colorcounter
#         for w in range (0, newMap.shape[1], 1):
#             for h in range (0, newMap.shape[0], 1):
#                 percentagecounter +=1
#                 print percentagecounter
#                 if not percentagecounter == 0:
#                     print str(100 / (max_perc / (percentagecounter)))+"%"
#                 if not newMap[h][w] == 0:
#                     if not newMap[h][w] in workedOnColors:
#                         workedOnColors.append(newMap[h][w])
#                         print workedOnColors
#                         oldColor = newMap[h][w]
#                         self.numbpix = 0
#                         colorcounter += 1
#                         workedOnColors.append(colorcounter)
#                         print "start floodfill"
#                         print "on pixel"
#                         print h
#                         print w
#                         print "oldcolor"
#                         print oldColor
#                         print colorcounter
#                         newMap = self.floodfill4(h, w, oldColor, colorcounter, newMap)
#                         if self.numbpix < 50:
#                             print "pix is to low refill black"
#                             newMap = self.floodfill4(h, w, colorcounter, 0, newMap)
#         # this is possibly a bad idea. What if two regions with the same color exist?
#         # here the second one will be deleted. However it could be the bigger one!
#         newMap[newMap<highestcolor+1]=0
#         return newMap
# 
#     def tesselateRooms(self, map_img, room_color):
#         cv_img = map_img
#         for w in range (0, map_img.shape[1], 1):
#             for h in range(0, map_img.shape[0], 1):
#                 if map_img[h][w] == room_color:
#                     cv_img[h][w] = room_color
#                 else:
#                     cv_img[h][w] = 0
#         return cv_img
#         
#     def makeListOfColorValues(self, map_img):
#         listOfDifColors = []
#         for w in range (0, map_img.shape[1], 1):
#             for h in range(0, map_img.shape[0], 1):
#                 colorvalue = map_img[h][w]
#                 if colorvalue not in listOfDifColors:
#                     listOfDifColors.append(colorvalue)
#         print listOfDifColors
#         return listOfDifColors
    
    def floodfill4(self, x, y, oldValue, newValue, img, pixelcounter):
        stack = []
        innerstack = []
        innerstack.append(x)
        innerstack.append(y)
        stack.append(innerstack)
        while not (len(stack) == 0): # list empty
            (x, y) = stack.pop()
            if img[x][y] == oldValue:
                img[x][y] = newValue
                pixelcounter += 1
                innerstack = []
                innerstack.append(x)
                innerstack.append(y+1)
                stack.append(innerstack)
                innerstack = []
                innerstack.append(x)
                innerstack.append(y-1)
                stack.append(innerstack)
                innerstack = []
                innerstack.append(x+1)
                innerstack.append(y)
                stack.append(innerstack)
                innerstack = []
                innerstack.append(x-1)
                innerstack.append(y)
                stack.append(innerstack)
        return (img, pixelcounter)
    
#     def floodfindNeighbours(self, x, y, oldValue, newValue, img):
#         newImg = img.copy()
#         stackOfNeighbours = []
#         stack = []
#         innerstack = []
#         innerstack.append(x)
#         innerstack.append(y)
#         stack.append(innerstack)
#         while not (len(stack) == 0): # list empty
#             (x, y) = stack.pop()
#             if img[x][y] == oldValue:
#                 img[x][y] = newValue
#                 self.numbpix += 1
#                 innerstack = []
#                 innerstack.append(x)
#                 innerstack.append(y+1)
#                 stack.append(innerstack)
#                 innerstack = []
#                 innerstack.append(x)
#                 innerstack.append(y-1)
#                 stack.append(innerstack)
#                 innerstack = []
#                 innerstack.append(x+1)
#                 innerstack.append(y)
#                 stack.append(innerstack)
#                 innerstack = []
#                 innerstack.append(x-1)
#                 innerstack.append(y)
#                 stack.append(innerstack)
#             else:
#                 if not img[x][y] == newValue:
#                     stackOfNeighbours.append(img[x][y])
#         return stackOfNeighbours

if __name__ == '__main__':
    rospy.init_node('map_tesselation_node', anonymous=False)
    mT = MapTesselation()
    rospy.spin()

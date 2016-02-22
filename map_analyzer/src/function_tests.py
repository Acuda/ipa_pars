#!/usr/bin/env python
'''
Created on Feb 19, 2016

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
import sys
import numpy as np
from sensor_msgs.msg import Image
#from sensor_msgs.msg._Image import Image
from PIL import Image

from cv_bridge import CvBridge, CvBridgeError

import sensor_msgs


class PicFuncTests(object):
    def __init__(self, path_to_map):
        rospy.loginfo("Initialize FunctionTestNode ...")
        self.path_to_map = path_to_map
        rospy.loginfo(self.path_to_map)
        self.bridge = CvBridge()
        self.img_map = cv2.imread(self.path_to_map, cv2.IMREAD_GRAYSCALE)
        rospy.loginfo("... finished")
        
    def workOnPic(self):
        print "working on pic"
        cv2.imshow("output", self.img_map)
        cv2.waitKey(10)
        print "make binary Image"
        #self.img_map = self.deleteErrosInMap(self.img_map)
        img = self.binarImage(self.img_map)
        cv2.imshow("output", img)
        cv2.waitKey(0)
        print img[50,50]
        img[50,50] = 255
        print img[50,50]
        print "makeListOfPixels"
        img = self.makeListOfAreasAndPixels(img)
        cv2.imshow("output", img)
        cv2.waitKey(0)
        print "ready"
    
    def binarImage(self, img):
        print type(img)
        n_img = np.asarray(img)
        print type(n_img)
        n_img[np.where(n_img>127)]=255
        n_img[np.where(n_img<128)]=0
        #n_img[np.where(n_img<128)]=255
        print n_img
        #img_proc = Image.fromarray(n_img.astype(np.uint8))
        #img = newMap.copy()
#         for w in range (0, n_img.shape[1], 1):
#             for h in range (0, n_img.shape[0], 1):
#                 if n_img[h,w] > 127:
#                     n_img[h,w] = 255
#                 else:
#                     n_img[h,w] = 0
        return n_img
        

    def makeListOfAreasAndPixels(self, img):
        listOfAreasAndPixels = []
        listOfAlreadyFinished = []
        # dont check black pixels
        color = 1
        listOfAlreadyFinished.append(0)
        for w in range (0, img.shape[1], 1):
            for h in range (0, img.shape[0], 1):
                if not img[h,w] in listOfAlreadyFinished:
                    print "found region with color = %d" % img[h,w]
                    #listOfAlreadyFinished.append(map_copy[h][w])
                    pixelcounter = 0
                    oldColor = img[h,w]
                    (img, pixelcounter) = self.floodfill4(h, w, oldColor, color, img, pixelcounter)
#                     if pixelcounter < 500:
#                         print "deleting area because its too small"
#                         pixelcounter = 0
#                         (img, pixelcounter) = self.floodfill4(h, w, color, 0, img, pixelcounter)
                    listOfAlreadyFinished.append(color)
                    listOfAreasAndPixels.append((color, pixelcounter))
                    color += 1
        print listOfAreasAndPixels
        for area in listOfAreasAndPixels:
            if area[1] > 5000:
                img[np.where(img==area[0])] = 255
            else:
                img[np.where(img==area[0])] = 0
        
        return img

    def floodfill4(self, x, y, oldValue, newValue, img, pixelcounter):
        stack = []
        innerstack = []
        innerstack.append(x)
        innerstack.append(y)
        stack.append(innerstack)
        while not (len(stack) == 0): # list empty
            (x, y) = stack.pop()
            if img[x,y] == oldValue:
                img[x,y] = newValue
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

if __name__ == '__main__':
    rospy.init_node('picture_function_tests_node', anonymous=False)
    pFT = PicFuncTests(sys.argv[1])
    pFT.workOnPic()

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

from ipa_pars_map_analyzer.msg._ParsMapKnowledgeAction import *
from sensor_msgs.msg._Image import Image

from yaml import load
import yaml

from geometry_msgs.msg import Point32

from cv_bridge import CvBridge, CvBridgeError
import actionlib
# Important notice: class and/or filename must not be same package name!


from geometry_msgs.msg import Pose
from cv2 import CV_8U


class ParsKnowledgeExtractorServer(object):
    _feedback = ipa_pars_map_analyzer.msg.ParsMapKnowledgeFeedback()
    _result = ipa_pars_map_analyzer.msg.ParsMapKnowledgeResult()
    def __init__(self):
        rospy.loginfo("Initialize ParsKnowledgeExtractorServer ...")
        self.bridge = CvBridge()
        
        self._as = actionlib.SimpleActionServer('ipa_pars_map_knowledge_extractor_server', ipa_pars_map_analyzer.msg.ParsMapKnowledgeAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        
        rospy.loginfo("ParsKnowledgeExtractorServer running! Waiting for a new map to analyze ...")
        rospy.loginfo("ParsKnowledgeExtractorServer initialize finished")
        
    def execute_cb(self, goal):
        rospy.loginfo("Extracting knowledge from a new map!")
#         rospy.loginfo("header");
#         print goal.input_map.header
        rospy.loginfo("goal in progress ...")
        r = rospy.Rate(1)

        received_map_as_cvimg = self.bridge.imgmsg_to_cv2(goal.input_map).copy()
        listOfSquares = goal.square_information
        
#         listOfAreas = self.debugmakeListOfColors(received_map_as_cvimg)
#         listOfBalance = self.calcBalancePoints(received_map_as_cvimg, listOfAreas)
        print "listOfBalancePoints"
        print listOfBalance
        roomBalancePoints = []
        
        for room in listOfBalance:
            roomBalancePoint = Point32()
            roomBalancePoint.x = room[1][0]
            roomBalancePoint.y = room[1][1]
            roomBalancePoint.z = 0
            print "added BalancePoint to list"
            print roomBalancePoint
            roomBalancePoints.append(roomBalancePoint)
            
        listOfBalancePoints = roomBalancePoints
#         listOfBalancePoints = goal.balance_points
#         listOfTransitions = self.getListOfTransitions(received_map_as_cvimg)
        (listOfTransitions_asstring, listOfTransitionsAsList) = self.getListOfTransitions(received_map_as_cvimg)
        print "============================== List of Room transitions ========================="
        print listOfTransitions_asstring
        yaml_content = listOfTransitions_asstring
        
        if (len(listOfTransitionsAsList)) > 20:
            #print yaml_content
            with open("ipa_pars/knowledge/static-knowledge-base0.yaml", 'w') as yaml_file:
                yaml_file.write(yaml_content)
        else:
            with open("ipa_pars/knowledge/static-knowledge-base0-justrooms.yaml", 'w') as yaml_file:
                yaml_file.write(yaml_content)
        
        
        index = 1
        listOfLocs = []
        print "lenth of ListOfAreas"
        print len(listOfAreas)
        print "first and last area"
        print listOfAreas[0]
        print listOfAreas[len(listOfAreas)-1]
        print "length of TranisitionsList"
        print len(listOfTransitionsAsList)
        print "length of listOfBalancePoints"
        print len(listOfBalancePoints)
        for room in listOfTransitionsAsList:
            
            name = room[0]
#             i = int(name.split('-')[1])-1
            list_of_transitions = room[1]
            x_center = listOfBalancePoints[index].x
            y_center = listOfBalancePoints[index].y
            z_center = listOfBalancePoints[index].z
            index += 1
            dict_of_center = {'X': x_center, 'Y': y_center, 'Z': z_center}
#             area_x_max = map_seg.room_information_in_pixel[i].room_min_max.points[1].x
#             area_y_max = map_seg.room_information_in_pixel[i].room_min_max.points[1].y
#             area_z_max = map_seg.room_information_in_pixel[i].room_min_max.points[1].z
#             area_x_min = map_seg.room_information_in_pixel[i].room_min_max.points[0].x
#             area_y_min = map_seg.room_information_in_pixel[i].room_min_max.points[0].y
#             area_z_min = map_seg.room_information_in_pixel[i].room_min_max.points[0].z
#             dict_of_area = {'max': {'X': area_x_max, 'Y': area_y_max, 'Z': area_z_max}, 'min': {'X': area_x_min, 'Y': area_y_min, 'Z': area_z_min}}
            dict_of_loc = {'name': name, 'transitions': list_of_transitions, 'center': dict_of_center}
            listOfLocs.append(dict_of_loc)
        dict_of_locations = {'location-data': listOfLocs}
        yaml_content = yaml.dump(dict_of_locations, default_flow_style=False)
        print "this is the list of received balance points in knowledge extractor"
        print yaml_content
        if (len(listOfTransitionsAsList) > 20):
            self.save_static_knowledge_file(yaml_content)
        else:
            with open("ipa_pars/knowledge/static-knowledge-base-onlyrooms.yaml", 'w') as yaml_file:
                yaml_file.write(yaml_content)
            
            
        
        print "i am sleeping now"
        success = True
        self._result.static_knowledge.data = listOfTransitions_asstring
        
        
        rospy.sleep(5)
        #===========================
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % 'map_analyzer_server')
        r.sleep()
        if success:
            rospy.loginfo("Produced fundamental static knowledge")
            self._as.set_succeeded(self._result, "static knowledge ready")

    def debugmakeListOfColors(self, map_img):
        listOfColors = []
        for w in range (0, map_img.shape[1], 1):
            for h in range (0, map_img.shape[0], 1):
                if not map_img[h,w] in listOfColors:
                    listOfColors.append(map_img[h,w])
        return listOfColors
    
    def getListOfTransitions(self, cv_img):
        cv_img[cv_img==65280]=0
        listOfRooms = []
        listOfRoomInformations = []
        
        for w in range (0, cv_img.shape[1], 1):
            for h in range (0, cv_img.shape[0], 1):
                if not cv_img[h][w] == 0:
                    if not cv_img[h][w] == 65280:
                        if not cv_img[h][w] in listOfRooms:
                            pixelcounter = 0
                            newValue = np.max(cv_img)+1
                            oldValue = cv_img[h][w]
                            (cv_img, pixelcounter, listOfTransitions) = self.floodfill4Neighbours2(h, w, oldValue, newValue, cv_img, pixelcounter)
                            (cv_img, pixelcounter, listOfSaft) = self.floodfill4Neighbours2(h, w, newValue, oldValue, cv_img, pixelcounter)
                            print "added room with number= %d" % oldValue
                            print "this room has transitions to"
                            print listOfTransitions
                            listOfRooms.append(oldValue)
                            listOfSingleTransitions = []
                            for trans in listOfTransitions:
                                if trans not in listOfSingleTransitions:
                                    listOfSingleTransitions.append(trans)
                            listOfRoomInformations.append((oldValue, listOfSingleTransitions))
        listOfTransAsStrings = []
        print "listOfSingleTransitions"
        print listOfRoomInformations
        if (len(listOfRoomInformations)) > 20:
        # translate to stringlist:
            for (rooms, transitions) in listOfRoomInformations:
                trans_as_string = []
                for trans in transitions:
                    trans_as_string.append("room-"+str(trans/1000)+"-square-"+str(trans%1000))
                listOfTransAsStrings.append(("room-"+str(rooms/1000)+"-square-"+str(rooms%1000) , trans_as_string))
        else:
            for (rooms, transitions) in listOfRoomInformations:
                trans_as_string = []
                for trans in transitions:
                    trans_as_string.append("room-"+str(trans))
                listOfTransAsStrings.append(("room-"+str(rooms), trans_as_string))
        # translate to string:
        stringOfTransitions = ""
        for (room, transi) in listOfTransAsStrings:
            stringOfTransitions += room+" "
            for tra in transi:
                stringOfTransitions += tra+" "
            stringOfTransitions += "\n"
        #stringOfTransitions = str(" ").join(map(str, listOfInput))
        return (stringOfTransitions, listOfTransAsStrings)

    def calcBalancePoints(self, img, listOfAreas):
        
        listOfBalancePoints = []
        #print listOfAreas
        for color in listOfAreas:
            listOfPointInfo = []
            print "Schwerpunkt berechnen fuer=",color
            listOfX = []
            listOfY = []
            for w in range(0, img.shape[1], 1):
                for h in range(0, img.shape[0], 1):
                    if (img[h,w] == color):
                        listOfX.append(h)
                        listOfY.append(w)
            mx = 0
            my = 0
            for x in listOfX:
                mx = mx+x
                
            for y in listOfY:
                my = my+y
            
            xs = mx/len(listOfX)
            ys = my/len(listOfY)
            #mark red
            #img[xs,ys] = 65525
            coordinateXYZ = []
            coordinateXYZ.append(xs)
            coordinateXYZ.append(ys)
            coordinateXYZ.append(0)
            listOfPointInfo.append(color)
            listOfPointInfo.append(coordinateXYZ)
            listOfBalancePoints.append(listOfPointInfo)
#             for room in listOfTransitions:
#                 if (room[1][0] == color[0]) and (room[1][1] == color[1]) and (room[1][2] == color[2]):
#                     room.append(coordinateXY)
        return listOfBalancePoints
    
    
    def floodfill4Neighbours2(self, x, y, oldValue, newValue, img, pixelcounter):
        stack = []
        innerstack = []
        neighborColorList = []
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
            # if the color is not black save the color in a list
            if not (img[x][y] == 0) and not (img[x][y] == oldValue) and not (img[x][y] == newValue):
                neighborColorList.append(img[x][y])
                
        return (img, pixelcounter, neighborColorList)
    
    def save_static_knowledge_file(self, yaml_content):
        print "save static knowledge file"
        try:
            if not os.path.isdir("ipa_pars/knowledge/"):
                os.mkdir("ipa_pars/knowledge")
            with open("ipa_pars/knowledge/static-knowledge-from-map.yaml", "w") as yaml_file:
                yaml_file.write(yaml_content)
            yaml_file.close()
        except IOError:
            rospy.loginfo("writing static-knowledge.yaml failed!")
            
            
if __name__ == '__main__':
    rospy.init_node('knowledge_extractor_server_node', anonymous=False)
    parsKES = ParsKnowledgeExtractorServer()
    rospy.spin()
        

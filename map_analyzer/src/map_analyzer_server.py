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
from map_analyzer.msg._MapTesselationAction import *
from map_analyzer.msg._MapKnowledgeAction import *

from sensor_msgs.msg import *
from sensor_msgs.msg._Image import Image
import color_utils_cme


from cv_bridge import CvBridge, CvBridgeError
import actionlib
# Important notice: class and/or filename must not be same package name!
#from map_analyzer.srv import MapAnalyzer
#from map_analyzer.srv._MapAnalyzer import MapAnalyzerResponse, MapAnalyzerRequest

#from map_analyzer.srv import RoomTesselation
#from map_analyzer.srv._RoomTesselation import RoomTesselationResponse, RoomTesselationRequest

from map_analyzer.srv import MapLogger
from map_analyzer.srv._MapLogger import MapLoggerResponse, MapLoggerRequest
import ipa_room_segmentation
from ipa_room_segmentation.msg._MapSegmentationAction import *

#from knowledge_base.srv import MapSeg
#from knowledge_base.srv._MapSeg import MapSegResponse, MapSegRequest

from geometry_msgs.msg import Pose
from cv2 import CV_8U



class MapAnalyzerServer(object):
    _feedback = map_analyzer.msg.MapAnalyzerFeedback()
    _result = map_analyzer.msg.MapAnalyzerResult()
    def __init__(self):
        rospy.loginfo("Initialize MapAnalyzerServer ...")
        rospy.loginfo("... starting room_segmentation_client")
        self._roomsegclient = actionlib.SimpleActionClient('room_segmentation_server', MapSegmentationAction)
        rospy.logwarn("Waiting for Segmentation Server to come available ...")
        self._roomsegclient.wait_for_server()
        rospy.logwarn("Server is online!")
        rospy.loginfo("... starting room_tesselation_client")
        self._roomtesclient = actionlib.SimpleActionClient('room_tesselation_server', MapTesselationAction)
        rospy.logwarn("Waiting for Tesselation Server to come available ...")
        self._roomtesclient.wait_for_server()
        rospy.logwarn("Server is online!")
        rospy.loginfo("... starting knowledge_extractor_client")
        self._knowledgeExtractorClient = actionlib.SimpleActionClient('knowledge_extractor_server', MapKnowledgeAction)
        rospy.logwarn("Waiting for KnowledgeExtractorServer to come available ...")
        self._knowledgeExtractorClient.wait_for_server()
        rospy.logwarn("Server is online!")
        
        rospy.logwarn("Waiting for MapStatusLoggerServiceServer to come available ...")
        rospy.wait_for_service('map_status_logger')
        rospy.loginfo("MapStatusLoggerServiceServer online!")
        rospy.loginfo("Creating MapLogger Service Client ...")
        try:
            self.serviceMapLoggerClient = rospy.ServiceProxy('map_status_logger', MapLogger)
        except rospy.ServiceException, e:
            print "MapStatusLoggerServiceCall failed! %s" %e
#        rospy.loginfo("... starting map_analyzer_service_server")
#         self.map_srvs = rospy.Service('map_analyzer_service_server', MapAnalyzer, self.handle_map_cb)
#         rospy.logwarn("Waiting for map_publisher_service_server to come available ...")
#         rospy.wait_for_service('map_publisher_server')
#         rospy.logwarn("Server online!")
#         rospy.logwarn("Waiting for knowledge_segmented_map_server to come available ...")
#         rospy.wait_for_service('knowledge_segmented_map_server')
#         rospy.logwarn("Server online!")
#         try:
#             self.serviceMapPublisherClient = rospy.ServiceProxy('map_publisher_server', MapAnalyzer)
#         except rospy.ServiceException, e:
#             print "Service call failed: %s"%e
#         rospy.logwarn("Waiting for map_tesselation_service_server to come available ...")
#         rospy.wait_for_service('map_tesselation_service_server')
#         rospy.logwarn("Server online!")
#         try:
#             self.serviceMapTesselationClient = rospy.ServiceProxy('map_tesselation_service_server', RoomTesselation)
#         except rospy.ServiceException, e:
#             print "Service call failed: %s"%e
#         try:
#             self.serviceKnowledgeSegmentedMapPublisherClient = rospy.ServiceProxy('knowledge_segmented_map_server', MapSeg)
#         except rospy.ServiceException, e:
#             print "Service call failed: %s"%e
#         rospy.loginfo("generating object instances")
        self.bridge = CvBridge()
        
        self._as = actionlib.SimpleActionServer('map_analyzer_server', map_analyzer.msg.MapAnalyzerAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        
        rospy.loginfo("MapAnalyzerServer running! Waiting for a new map to analyze ...")
        rospy.loginfo("MapAnalyzerServer initialize finished")
        
    def execute_cb(self, goal):
        rospy.loginfo("Analyzing a new map!")
        rospy.loginfo("header");
        print goal.input_map.header
        rospy.loginfo("goal in progress ...")
        r = rospy.Rate(1)
        # 1
        
        # Delete Errors:
        received_map_as_bgr = self.bridge.imgmsg_to_cv2(goal.input_map).copy()
        received_map_as_bgr = self.deleteErrorsInMap(received_map_as_bgr)
        received_map_as_imgmsg = self.bridge.cv2_to_imgmsg(received_map_as_bgr, encoding="bgr8")
        self._feedback.status = 1
#         response = self.serviceMapLoggerClient(received_map_as_imgmsg)
#         print response
        # goal to MapSegmentationServer
        segmented_map_response = self.useRoomSegmentation(received_map_as_imgmsg)
        print "we received a segmented map:"
        print "its resolution is:"
        print segmented_map_response.map_resolution
        self._feedback.status = 2
#         response = self.serviceMapLoggerClient(segmented_map_response.segmented_map)
#         print response
        # goal to MapTesselationServer
        tesselated_map_response = self.useRoomTesselation(segmented_map_response)
        print "this should be a resolution"
        print tesselated_map_response.map_resolution
#         listOfBalancePoints = tesselated_map_response.balance_points
        self._feedback.status = 3
#         response = self.serviceMapLoggerClient(tesselated_map_response.tesselated_map)
#         print response
        # concatenate rooms and squares to new numbers
        squaresAndRoomsMap = self.transposeSquaresToRooms(segmented_map_response.segmented_map, tesselated_map_response.tesselated_map)
        map_np = self.getRealSquaresRoomNames(squaresAndRoomsMap)
        cv_map = self.bridge.cv2_to_imgmsg(map_np, encoding="mono16")
        self._feedback.status = 4
#         response = self.serviceMapLoggerClient(map_np)
#         print response
        # send map to KnowledgeExtractorServer
        knowledge_result = self.useKnowledgeExtractor(cv_map)
        print "this should be a text in yaml format"
        self._feedback.status = 5

        
        print "i am sleeping now"
        success = True
        self._result.static_knowledge.data = knowledge_result.static_knowledge.data
        rospy.sleep(5)
        #===========================
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % 'map_analyzer_server')
        r.sleep()
        if success:
            rospy.loginfo("Map Analyzer produced static knowledge!")
            self._as.set_succeeded(self._result, "good job")

    def deleteErrorsInMap(self, img):
        for w in range (0, img.shape[1], 1):
            for h in range (0, img.shape[0], 1):
                if not img[h,w][0] == 0:
                    if not img[h,w][0] == 254:
                        pixelcounter = 0
                        (img, pixelcounter) = self.floodfill4ColorsForErrorDeletion(h, w, 255, 254, img, pixelcounter)
                        if pixelcounter < 500:
                            pixelcounter = 0
                            (img, pixelcounter) = self.floodfill4ColorsForErrorDeletion(h, w, 254, 0, img, pixelcounter)
        img[np.where(img==254)] = 255
        return img
    
    def floodfill4ColorsForErrorDeletion(self, x, y, oldValue, newValue, img, pixelcounter):
        stack = []
        innerstack = []
        innerstack.append(x)
        innerstack.append(y)
        stack.append(innerstack)
        while not (len(stack) == 0): # list empty
            (x, y) = stack.pop()
            if img[x,y][0] == oldValue:
                img[x,y][0] = newValue
                img[x,y][1] = newValue
                img[x,y][2] = newValue
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
    
    
    def debugmakeListOfColors(self, map_img):
        listOfColors = []
        for w in range (0, map_img.shape[1], 1):
            for h in range (0, map_img.shape[0], 1):
                if not map_img[h,w] in listOfColors:
                    listOfColors.append(map_img[h,w])
        return listOfColors
    
    def deleteErrorsInMap2(self, img):
        for w in range (0, img.shape[1], 1):
            for h in range (0, img.shape[0], 1):
                if not img[h,w] == 0:
                    if not img[h,w] == 254:
                        pixelcounter = 0
                        (img, pixelcounter) = self.floodfill4(h, w, 255, 254, img, pixelcounter)
                        if pixelcounter < 500:
                            pixelcounter = 0
                            (img, pixelcounter) = self.floodfill4(h, w, 254, 0, img, pixelcounter)
        img[np.where(img==254)] = 255
        return img
        
    def handle_map_cb(self, input_map):
        print "print recieved map header:"
        print input_map.map.header
        # first image input_map display in publisher
        answer = self.serviceMapPublisherClient(input_map)
        print "answer of pass through publisher call"
        print answer
        print "------------- read out type of image"
        print type(input_map.map)
        print input_map.map.encoding
#         cv_img = self.bridge.imgmsg_to_cv2(input_map.map, desired_encoding="mono8").copy()
#         print "deleting errors"
#         cv_img = self.deleteErrorsInMap(cv_img)
        #print "show image"
#         cv2.imshow("output", cv_img)
#         cv2.waitKey(0)
        #output_map = self.bridge.cv2_to_imgmsg(cv_img)
        segmented_map_response = self.useRoomSegmentation(input_map.map)
        print "we received a segmented map:"
        print "its resolution is:"
        print segmented_map_response.map_resolution
         
        request5 = MapSegRequest()
        request5.segmented_map = segmented_map_response.segmented_map
        request5.map_resolution = segmented_map_response.map_resolution
        request5.map_origin = segmented_map_response.map_origin
        request5.room_information_in_pixel = segmented_map_response.room_information_in_pixel
        request5.room_information_in_meter = segmented_map_response.room_information_in_meter
        answer5 = self.serviceKnowledgeSegmentedMapPublisherClient(request5)
        print answer5
#         col_map = self.convertSegmentedMap(segmented_map_response)
#         answer2 = self.serviceMapPublisherClient(segmented_map_response.segmented_map)
#         print answer2
        #print "listOfTransitions after room_segmentation"
        #listOfTransitions = self.getListOfTransitions(segmented_map_response.segmented_map)
        #print "============================== List of Room transitions ========================="
        #print listOfTransitions
        
        print "send map to tesselation server"
        answer3 = self.serviceMapTesselationClient(segmented_map_response.segmented_map)
        #request = RoomTesselationRequest()
        #request.room_map = input_map
#         answer3 = self.serviceMapTesselationClient(input_map.map)
        print answer3.tesselated_rooms.encoding
        
        print "transposeSquaresToRooms:"
        squaresAndRoomsMap = self.transposeSquaresToRooms(segmented_map_response.segmented_map, answer3.tesselated_rooms)
        print "ready transposing"
        listOfcol = self.debugmakeListOfColors(squaresAndRoomsMap)
        print "listOfTransposedColors"
        print listOfcol
        output_map = self.bridge.cv2_to_imgmsg(squaresAndRoomsMap)
        answer7  = self.serviceMapPublisherClient(output_map)
        print "newSquaresNames"
        map = self.getRealSquaresRoomNames(squaresAndRoomsMap)
        
        #print "listOfTransitions after room_tesselation"
        #print "map encoding after tesselation"
        #print input_map.map.encoding
        
        #listOfTransitions = self.getListOfTransitions(answer3.tesselated_rooms)
        #print "============================== List of Room transitions ========================="
        #print listOfTransitions
        
        response = MapAnalyzerResponse()
        #response.answer.data = listOfTransitions
        return response
    
    def transposeSquaresToRooms(self, segmented_map, tesselated_map):
        cv_img_segmented_map = self.bridge.imgmsg_to_cv2(segmented_map, desired_encoding="passthrough").copy()
        cv_img_tesselated_map = self.bridge.imgmsg_to_cv2(tesselated_map, desired_encoding="passthrough").copy()
#         squaresAndRoomMap = np.zeros((cv_img_segmented_map.shape[0], cv_img_segmented_map.shape[1] , 1), np.uint16) # BGR
        squaresAndRoomMap = np.zeros((cv_img_segmented_map.shape[0], cv_img_segmented_map.shape[1]), np.uint16) # BGR
        for w in range (0, squaresAndRoomMap.shape[1], 1):
            for h in range (0, squaresAndRoomMap.shape[0], 1):
                if not cv_img_segmented_map[h,w] == 0:
                    squaresAndRoomMap[h,w] = cv_img_tesselated_map[h,w]*12 + 1000 * cv_img_segmented_map[h,w]
                else:
                    squaresAndRoomMap[h,w] = 0 
        
        return squaresAndRoomMap
                    

    def getRealSquaresRoomNames(self, squaresAndRoomMap):
        mapwithnicenames = np.zeros((squaresAndRoomMap.shape[0], squaresAndRoomMap.shape[1]), np.uint16)
        listOfColors = self.debugmakeListOfColors(squaresAndRoomMap)
        print "listOfColors before RealSquares"
        print listOfColors
        listOfColors.remove(0)
        listOfColors.sort()
        counter = 1
        room = 0
        newListOfRoomSquareTuples = []
        while not (len(listOfColors) == 0): # list empty
            newTuple = []
            romNbr = listOfColors.pop(0)
            newTuple.append(romNbr)
            
            currentRoom = romNbr / 1000
            if not currentRoom == room:
                room = currentRoom
                counter = 1
            newRoomNumber = currentRoom * 1000 + counter
            newTuple.append(newRoomNumber)
            newListOfRoomSquareTuples.append(newTuple)
            counter += 1
        
        print "listOfColors after RealSquares"
        print newListOfRoomSquareTuples
#         listOfColorsAndRoomNumbers = []
#         counterOfRoomNumber = 0
        for w in range (0, squaresAndRoomMap.shape[1], 1):
            for h in range (0, squaresAndRoomMap.shape[0], 1):
                if not squaresAndRoomMap[h,w] == 0:
                    oldValue = squaresAndRoomMap[h,w]
                    for element in newListOfRoomSquareTuples:
                        if element[0] == oldValue:
                            newValue = element[1]
                            break
                    #newValue = newListOfRoomSquareTuples[newListOfRoomSquareTuples.index(oldValue)][1]
                    mapwithnicenames[h,w] = newValue
        newList = self.debugmakeListOfColors(mapwithnicenames)
        print newList
        return mapwithnicenames
    
    def getListOfTransitions(self, map_img):
        cv_img = self.bridge.imgmsg_to_cv2(map_img, desired_encoding="passthrough").copy()
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
        # translate to stringlist:
        for (rooms, transitions) in listOfRoomInformations:
            trans_as_string = []
            for trans in transitions:
                trans_as_string.append("room-"+str(trans))
            listOfTransAsStrings.append(("room-"+str(rooms) , trans_as_string))
        
        # translate to string:
        stringOfTransitions = ""
        for (room, transi) in listOfTransAsStrings:
            stringOfTransitions += room+" "
            for tra in transi:
                stringOfTransitions += tra+" "
            stringOfTransitions += "\n"
        #stringOfTransitions = str(" ").join(map(str, listOfInput))
        return stringOfTransitions
    
    def convertSegmentedMap(self, seg_map_as_index_map):

        print "*************************infromation about incoming segmented map!*********************************"
        print "resolution:"
        print seg_map_as_index_map.map_resolution
        print "map_origin"
        print seg_map_as_index_map.map_origin
        print "RoomInformation in pixels"
        print seg_map_as_index_map.room_information_in_pixel
        print "RoomInformation in meter"
        print seg_map_as_index_map.room_information_in_meter
        print "segmented map in 32SC1"
        print "type"
        print type(seg_map_as_index_map.segmented_map)
        #cvmap = cv2.cv.fromarray(seg_map_as_index_map.segmented_map)
#         cv_img = self.bridge.imgmsg_to_cv2(seg_map_as_index_map.segmented_map, desired_encoding="passthrough").copy()
#         cv_col_img = numpy.zeros((cv_img.shape[1], cv_img.shape[0] , 3), numpy.uint8) # BGR
#         
#         listOfDifColors = []
#         for w in range (0, cv_img.shape[1], 1):
#             for h in range(0, cv_img.shape[0], 1):
#                 colorvalue = cv_img[h][w]
#                 if colorvalue not in listOfDifColors:
#                     listOfDifColors.append(colorvalue)
#         print listOfDifColors
#         print "colorade labes:"
#         listOfColLab = []
#         for lab in listOfDifColors:
#             labcol = []
#             labcol.append(lab)
#             col = self.colorlist.pop()
#             labcol.append(col)
#             listOfColLab.append(labcol)
#         print listOfColLab
#         
#         for w in range (0, cv_col_img.shape[1], 1):
#             for h in range (0, cv_col_img.shape[0], 1):
#                 labelnbr = cv_img[h][w]
#                 if item in listOfColLab
#                 colorpix = listOfColLab.
#                 cv_col_img[h,w,:] = colorpix
#         cv2.imshow("newimg", cv_col_img)
#         cv2.waitKey()
        
        #cv_col_img = cv2.copyMakeBorder(cv_img, 5,5,5,5,cv2.BORDER_CONSTANT,value=(0,0,0))
#         cv_img[cv_img==numpy.max(cv_img)]=10 #alle maximalwert auf -1 setzen
#         cv2.imshow("output",cv_img)
#         cv2.waitKey()
#         cv2.destroyAllWindows()
        #color = []
        #cv_img[h,w,:] = [0,1,2] #bgr farben zuweisen zu cv_img[hoehe,breite,farbe als array]
        
        
        #cv2.imshow("output", cv_img)
        #cv2.waitKey()
        #cvconverted = self.bridge.imgmsg_to_cv2(seg_map_as_index_map.segmented_map, "CV_32S")
        #col_map = self.bridge.imgmsg_to_cv2(seg_map, desired_encoding="passthrough")
        output_msg = MapAnalyzerRequest()
        #output_msg.map = seg_map_as_index_map
        # convertion in room_segmentation_server Mat!
        
        # TODO: here!
        
        print "information about my output!"
        print output_msg.map.header
        print output_msg.map.height
        print output_msg.map.width
        print output_msg.map.encoding
        print output_msg.map.is_bigendian
        print output_msg.map.step
        return output_msg
    
    def useKnowledgeExtractor(self, cv_img):
        goal = map_analyzer.msg.MapKnowledgeGoal()
        goal.input_map = cv_img
        goal.input_map.header.stamp = rospy.Time.now()
        goal.input_map.header.frame_id = "mymapframe"
#         goal.balance_points = listOfPoints
#         goal.map_resolution = tesselated_map_response.map_resolution
#         goal.map_origin = tesselated_map_response.map_origin
        rospy.loginfo("Send goal to KnowledgeExtractorServer ...")
        self._knowledgeExtractorClient.send_goal(goal)
        rospy.loginfo("Waiting for result of KnowledgeExtractorServer ...")
        self._knowledgeExtractorClient.wait_for_result()
        result = self._knowledgeExtractorClient.get_result()
        rospy.loginfo("Received a result from KnowledegeExtractorServer!")
        return result
        
    def useRoomTesselation(self, seg_map):
        goal = map_analyzer.msg.MapTesselationGoal()
        goal.input_map = seg_map.segmented_map
        goal.map_resolution = seg_map.map_resolution
        goal.map_origin = seg_map.map_origin
        rospy.loginfo("Send goal to MapTesselationServer ...")
        self._roomtesclient.send_goal(goal)
        rospy.loginfo("Waiting for result of MapTesselationServer ...")
        self._roomtesclient.wait_for_result()
        result = self._roomtesclient.get_result()
        rospy.loginfo("Received a result from RoomTesselationServer!")
        
        return result
        
    def useRoomSegmentation(self, in_map):
        goal = ipa_room_segmentation.msg.MapSegmentationGoal()
        #goal.input_map.header.seq = 1
        goal.input_map.header.stamp = rospy.Time.now()
        goal.input_map.header.frame_id = "mymapframe"
        #goal.input_map = in_map.map
        goal.input_map = in_map
        # TODO: change this lines to use every map!
        goal.map_resolution = 0.05
        pose = Pose()
        pose.position.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0
        goal.map_origin = pose
        goal.return_format_in_pixel = True
        goal.return_format_in_meter = False
        goal.room_segmentation_algorithm = 2
        
        print "header and size of pic before room_segmentation"
        print goal.input_map.header
        print "width %d height %d = " % (goal.input_map.width , goal.input_map.height)
        
        rospy.loginfo("Wait for goal!")
        self._roomsegclient.send_goal(goal)
        rospy.loginfo("Waiting for result of MapSegmentationServer")
        self._roomsegclient.wait_for_result()
        solution = self._roomsegclient.get_result()
        rospy.loginfo("Received a result:")
        
        return solution
    
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
    
    '''
    TODO: Delete this: This is just for debugging!
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
        
if __name__ == '__main__':
    rospy.init_node('map_analyzer_server_node', anonymous=False)
    mAS = MapAnalyzerServer()
    rospy.spin()
        
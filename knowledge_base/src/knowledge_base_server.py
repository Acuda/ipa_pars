#!/usr/bin/env python
'''
Created on Feb 22, 2016

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
# ROS package name: knowledge_base
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
import numpy as np
from sensor_msgs.msg import Image
#from sensor_msgs.msg._Image import Image
from rosparam import upload_params
from yaml import load
import yaml

from cv_bridge import CvBridge, CvBridgeError

from knowledge_base.srv import MapSeg
from knowledge_base.srv._MapSeg import MapSegResponse, MapSegRequest

from map_analyzer.srv import MapAnalyzer
from map_analyzer.srv._MapAnalyzer import MapAnalyzerResponse, MapAnalyzerRequest


class KnowledgeBaseServer(object):
    def __init__(self, path_to_knowledge_base):
        rospy.loginfo("Initialize KnowledgeBaseServer ...")
        self.path_to_knowledge_base = path_to_knowledge_base
        rospy.loginfo(path_to_knowledge_base)
        rospy.loginfo("... creating room_segmentation_publisher")
        self.room_seg_pub = rospy.Publisher('segmented_map', Image, queue_size=1)
        rospy.loginfo("... creating tesselated_map_publisher")
        self.tesselated_map_pub = rospy.Publisher('tesselated_map', Image, queue_size=1)
        self.segmented_map_srvs = rospy.Service('knowledge_segmented_map_server', MapSeg, self.handle_segmented_map_cb)
        self.segmented_map = Image()
        self.tesselated_map = Image()
        self.img_map = Image()
        self.bridge = CvBridge()
        #for image conversion:

        rospy.loginfo("... finished")
        
    def handle_segmented_map_cb(self, map_seg):
        print "print recieved segmented_map:"
        print "header"
        print map_seg.segmented_map.header
        print "encoding"
        print map_seg.segmented_map.encoding
        print "room info in meter"
        print map_seg.room_information_in_meter
        print "room info in pixel"
        print map_seg.room_information_in_pixel
        print "room points"
        print map_seg.map_origin
        print "map resolution"
        print map_seg.map_resolution
        self.segmented_map = map_seg.segmented_map
#         self.load_params_from_yaml()
        print "listOfTransitions after room_segmentation"
        (listOfTransitions_asstring, listOfTransitionsAsList) = self.getListOfTransitions(map_seg.segmented_map)
        print "============================== List of Room transitions made in knowledge base ========================="
        print listOfTransitions_asstring
        print "writing Transitions and rooms to new knowledge-base"
        #self.writeKnowledgeBase(listOfTransitions)
        
        response = MapSegResponse()
        response.success = True
        # concatenate informations from room_seg and transitions to yaml file:
        print "------------------------------------------------"
        for room in listOfTransitionsAsList:
            print room
        print "------------------------------------------------"
        
        listOfLocs = []
        for room in listOfTransitionsAsList:
            name = room[0]
            i = int(name.split('-')[1])-1
            list_of_transitions = room[1]
            x_center = map_seg.room_information_in_pixel[i].room_center.x
            y_center = map_seg.room_information_in_pixel[i].room_center.y
            z_center = map_seg.room_information_in_pixel[i].room_center.z
            dict_of_center = {'X': x_center, 'Y': y_center, 'Z': z_center}
            area_x_max = map_seg.room_information_in_pixel[i].room_min_max.points[1].x
            area_y_max = map_seg.room_information_in_pixel[i].room_min_max.points[1].y
            area_z_max = map_seg.room_information_in_pixel[i].room_min_max.points[1].z
            area_x_min = map_seg.room_information_in_pixel[i].room_min_max.points[0].x
            area_y_min = map_seg.room_information_in_pixel[i].room_min_max.points[0].y
            area_z_min = map_seg.room_information_in_pixel[i].room_min_max.points[0].z
            dict_of_area = {'max': {'X': area_x_max, 'Y': area_y_max, 'Z': area_z_max}, 'min': {'X': area_x_min, 'Y': area_y_min, 'Z': area_z_min}}
            dict_of_loc = {'name': name, 'transitions': list_of_transitions, 'center': dict_of_center, 'min-max': dict_of_area}
            listOfLocs.append(dict_of_loc)
        dict_of_locations = {'location-data': listOfLocs}
        yaml_content = yaml.dump(dict_of_locations, default_flow_style=False)
        print yaml_content
        with open(self.path_to_knowledge_base+"knowledge-base.yaml", 'w') as yaml_file:
            yaml_file.write(yaml_content)
        return response
    
    
    def writeKnowledgeBase(self, knowledge_base_as_yaml_dict):
        with open(self.path_to_knowledge_base, 'w') as yaml_file:
            yaml_file.write( yaml.dump(knowledge_base_as_yaml_dict, default_flow_style=False))
        yaml_file.close()
        
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
    
    
    def load_params_from_yaml(self):
        # beachte: YAMl verwendet nur dicts und lists
        # verwende die entsprechenden methoden richtig
        f = open(self.path_to_knowledge_base, 'r')
        yamlfile = load(f)
        f.close()
        print yamlfile
        print "------------------------"
        location_data = yamlfile['location-data']
        for locations in location_data:
            location_name = locations['name']
            location_transitions = locations['transitions']
            print location_name+": has transitions to:"
            print location_transitions
            #room_name = locations[1]
            #print room_name
            #room_name = locations['name']
            #print room_name
        #location2 = location_data[1]
        #print location_data
        #print location1
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
        return (stringOfTransitions, listOfTransAsStrings)
        

    def run(self):
        self.room_seg_pub.publish(self.segmented_map)
        r = rospy.Rate(10)
        r.sleep()
        
if __name__ == '__main__':
    rospy.init_node('knowledge_base_server_node', anonymous=False)
    kBS = KnowledgeBaseServer(sys.argv[1])
    while not rospy.is_shutdown():
        kBS.run()
        

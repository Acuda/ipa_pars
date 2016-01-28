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
import rospy
import actionlib
import sys
import ipa_pars_main.msg

class ParsServer(object):
    _feedback = ipa_pars_main.msg.LogicPlanFeedback()
    _result = ipa_pars_main.msg.LogicPlanResult()
    def __init__(self, path_to_map):
        rospy.loginfo("Initialize ParsServer ...")
        self._path_to_map = path_to_map
        rospy.loginfo("path_to_map: %s" % path_to_map)
        self._as = actionlib.SimpleActionServer('pars_server', ipa_pars_main.msg.LogicPlanAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        
        rospy.loginfo("ParsServer running! Waiting for a new goal.")

    def execute_cb(self, goal):
        rospy.loginfo("Executing a new goal!")
        rospy.loginfo("GOAL: %s , %s, %s " % (str(goal.goal_type), str(goal.what), str(goal.where)))
        rospy.loginfo("in progress ...")
        r = rospy.Rate(1)
        # change herer TODO:
        success = True
        #===========================
        # robot stuff here
        #===========================
        
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % 'pars_server')
            success = False
            
        
        r.sleep()
        
        if success:
            self._result.success = True
            rospy.loginfo("Succeeded the Logic Plan")
            self._as.set_succeeded(self._result, "good job")
            

if __name__ == '__main__':
    rospy.init_node('planning_server', anonymous=False)
    pARS = ParsServer(sys.argv[1])
    rospy.spin()

/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2016 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: ipa_pars
 * \note
 *   ROS package name: ipa_pars_map_analyzer
 *
 * \author
 *   Author: Christian Ehrmann, email: Christian.Ehrmann@ipa.fraunhofer.de
 *
 * \date Date of creation: July, 2016
 *
 * \brief
 *   This class provides a map analysis tool for the IPA planning and
	 reasoning system.
 *
 ****************************************************************/
#include <ros/ros.h>
#include <math.h>

#import <opencv/cv.h>
#import <cv_bridge/cv_bridge.h>
#import <iostream>

#include <ipa_pars_map_analyzer/ipa_pars_map_analyzer_client.h>
#include <ipa_pars_map_analyzer/ParsMapAnalyzerAction.h>

bool ParsMapAnalyzerClient::initialize(std::string path_to_map)
{
	ROS_INFO("initializing ParsMapAnalyzerClient ...");
	// create NodeHandle
	ros::NodeHandle nh_;
	_path_to_map = path_to_map.append("lab_ipa4.png");
	ROS_INFO("path to input map is %s",_path_to_map);
	ROS_INFO("loading map from file");
	_input_map =
	actionlib::SimpleActionClient<ipa_pars_map_analyzer::ParsMapAnalyzerAction> ac_("map_analyzer_server", true);
	
	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac_->waitForServer(); //will wait for infinite time

	return true;
}

void ParsMapAnalyzerClient::run()
{
	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	ipa_pars_map_analyzer::ParsMapAnalyzerGoal goal;
	ac_->sendGoal(goal);

}

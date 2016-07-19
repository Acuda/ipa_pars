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
 *   This class provides map analysis for knowledge creation for the ipa
 *   planning and reasoning system
 *
 ****************************************************************/
#ifndef IPA_PARS_MAP_ANALYZER_H
#define IPA_PARS_MAP_ANALYZER_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa_pars_map_analyzer/ParsMapAnalyzerAction.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>

#include <boost/thread/mutex.hpp>

typedef actionlib::SimpleActionClient<ipa_pars_map_analyzer::ParsMapAnalyzerAction> SAC_ParsMapAnalyzerAction_t;

class ParsMapAnalyzerClient
{
public:

	// constructor
	ParsMapAnalyzerClient(std::string path_to_map)
{
	std::string _path_to_map = path_to_map;
}
	// destructor
	~ParsMapAnalyzerClient(void)
	{

	}

	bool initialize();
	void run();

	void sendGoal();


private:
	std::string _path_to_map;
	sensor_msgs::Image _input_map;
	boost::shared_ptr<SAC_ParsMapAnalyzerAction_t> ac_;
};

#endif

/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2016 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: ipa_pars
 * \note
 * ROS package name: ipa_pars_map_analyzer
 *
 * \author
 * Author: Christian Ehrmann
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 07.2016
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef IPA_PARS_IPA_PARS_MAP_ANALYZER_ROS_INCLUDE_IPA_PARS_MAP_ANALYZER_IPA_PARS_MAP_TESSELATION_SERVER_H_
#define IPA_PARS_IPA_PARS_MAP_ANALYZER_ROS_INCLUDE_IPA_PARS_MAP_ANALYZER_IPA_PARS_MAP_TESSELATION_SERVER_H_


#include "ros/ros.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <actionlib/server/simple_action_server.h>

#include <ipa_pars_map_analyzer/ParsMapTesselationAction.h>

class ParsMapTesselationServer
{
protected:

	//This is the execution function used by action server
	void execute_map_tesselation_server(const ipa_pars_map_analyzer::ParsMapTesselationGoalConstPtr &goal);

	//Tesselation function
	void tesselate_map(const cv::Mat& map_to_tesselate, cv::Mat& tesselated_map, std::vector<int>& labelcount);

	//!!Important!!
	// define the Nodehandle before the action server, or else the server won't start
	//
	ros::NodeHandle node_handle_;
	actionlib::SimpleActionServer<ipa_pars_map_analyzer::ParsMapTesselationAction> ipa_pars_map_tesselation_server_;

public:
	//initialize the action-server
	ParsMapTesselationServer(ros::NodeHandle nh, std::string name_of_the_action);

	//Default destructor for the class
	~ParsMapTesselationServer(void)
	{
	}
};



#endif /* IPA_PARS_IPA_PARS_MAP_ANALYZER_ROS_INCLUDE_IPA_PARS_MAP_ANALYZER_IPA_PARS_MAP_TESSELATION_SERVER_H_ */

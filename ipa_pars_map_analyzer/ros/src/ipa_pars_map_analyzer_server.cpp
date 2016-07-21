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

#include <ipa_pars_map_analyzer/ipa_pars_map_analyzer_server.h>

#include <ros/package.h>

ParsMapAnalyzerServer::ParsMapAnalyzerServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	ipa_pars_map_analyzer_server_(node_handle_, name_of_the_action, boost::bind(&ParsMapAnalyzerServer::execute_map_analyzer_server, this, _1), false)
{
	//Start action server
	ipa_pars_map_analyzer_server_.start();
}

void ParsMapAnalyzerServer::execute_map_analyzer_server(const ipa_pars_map_analyzer::ParsMapAnalyzerGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****MapAnalyzer action server*****");
	ROS_INFO("map resolution is : %f", goal->map_resolution);

	//converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat original_img = cv_ptr_obj->image;

	//set the resolution and the limits for the actual goal and the Map origin
	const float map_resolution = goal->map_resolution;
	const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);

	//make non-white pixels black
	for (int y = 0; y < original_img.rows; y++)
	{
		for (int x = 0; x < original_img.cols; x++)
		{
			//find not reachable regions and make them black
			if (original_img.at<unsigned char>(y, x) < 250)
			{
				original_img.at<unsigned char>(y, x) = 0;
			}
			//else make it white
			else
			{
				original_img.at<unsigned char>(y, x) = 255;
			}
		}
	}

	cv::Mat disp = original_img.clone();
	cv::imshow("mapanalyzer", disp);
	cv::waitKey();

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ipa_pars_map_analyzer_server");

	ros::NodeHandle nh;

	ParsMapAnalyzerServer analyzerAlgorithmObj(nh, ros::this_node::getName());
	ROS_INFO("Action Server for map_analyzer has been initialized......");
	ros::spin();

	return 0;
}

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

#include <ipa_pars_map_analyzer/ipa_pars_map_tesselation_server.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "highgui.h"
#include <stdlib.h>
#include <stdio.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa_pars_map_analyzer/ParsMapTesselationAction.h>

#include "std_msgs/Int32MultiArray.h"

#include <sstream>


ParsMapTesselationServer::ParsMapTesselationServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	ipa_pars_map_tesselation_server_(node_handle_, name_of_the_action, boost::bind(&ParsMapTesselationServer::execute_map_tesselation_server, this, _1), false)
{
	//Start action server
	ipa_pars_map_tesselation_server_.start();
}


void ParsMapTesselationServer::execute_map_tesselation_server(const ipa_pars_map_analyzer::ParsMapTesselationGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****ParsMapTesselation action server*****");
	ROS_INFO("map resolution is : %f", goal->map_resolution);

	//converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::TYPE_32SC1);
	cv::Mat original_img = cv_ptr_obj->image;

	//set the resolution and the limits for the actual goal and the Map origin
	const float map_resolution = goal->map_resolution;
	const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);

	// output
	ipa_pars_map_analyzer::ParsMapTesselationResult map_tesselation_action_result_;
	//cv::Mat tesselated_map;
	cv_bridge::CvImage  cv_img;
	cv_img.header.stamp = ros::Time::now();
	cv_img.encoding = "32SC1";
	cv::Mat output_img = original_img.clone();
	cv::Mat corrected_img = original_img.clone();

	// delete errors
	for (int y = 0; y < original_img.rows; ++y)
	{
		for (int x = 0; x < original_img.cols; ++x)
		{
			const int label = original_img.at<int>(y,x);
			if (label == 65280)
				corrected_img.at<int>(y,x) = 0;
		}
	}
	std::vector <int> labelcount;
	cv::Mat tesselated_img;
	ParsMapTesselationServer::tesselate_map(corrected_img, tesselated_img, labelcount);


	output_img.convertTo(tesselated_img, CV_32SC1, 256, 0);
	cv_img.image = output_img;
	tesselated_img.convertTo(tesselated_img, CV_8U);
	cv::cvtColor(tesselated_img, tesselated_img, CV_GRAY2BGR);
	cv::imshow("tesselationbefore", tesselated_img);
	cv_img.toImageMsg(map_tesselation_action_result_.tesselated_map);

	map_tesselation_action_result_.map_resolution = goal->map_resolution;
	map_tesselation_action_result_.map_origin = goal->map_origin;
	ROS_INFO_STREAM("labelcount.size() = " << labelcount.size());
//	long arrayOfInts[labelcount.size()];
//	for (int i = 0; i< labelcount.size(); ++i)
//	{
//		arrayOfInts[i] = labelcount.at(i);
//	}
	map_tesselation_action_result_.labels.data = labelcount;
	ROS_INFO_STREAM("labels.data = " << map_tesselation_action_result_.labels.data.size());
	ipa_pars_map_tesselation_server_.setSucceeded(map_tesselation_action_result_);

}



void ParsMapTesselationServer::tesselate_map(const cv::Mat& map_to_tesselate, cv::Mat& tesselated_map, std::vector<int>& labelcount)
{
	cv::Mat temporary_map_to_work = map_to_tesselate.clone();
	// create squares
	std::vector<int> labelcounter;
	int countery = 0;
	int label = 1;
	int counterx = 0;
	int cols = 0;
	bool firstround = true;
	labelcounter.push_back(label);
	ROS_INFO("map rows = %u", map_to_tesselate.rows);
	ROS_INFO("map cols = %u", map_to_tesselate.cols);

	for (int y = 0; y < map_to_tesselate.rows; ++y)
	{
		countery++;

		for (int x = 0; x < map_to_tesselate.cols; ++x)
		{
			counterx++;

			if (countery > 19)
			{
				countery = 0;
				firstround = true;
				cols++;
				label = 1 + cols * 100;
				labelcounter.push_back(label);
			}

			if (counterx > 19)
			{
				counterx = 0;
				label++;
				if (firstround)
				{
					labelcounter.push_back(label);
				}

			}

			// label all pixels that are not black
			if (map_to_tesselate.at<int>(x,y) != 0)
			{
				//ROS_INFO("I just marked a label = %u", label);
				temporary_map_to_work.at<int>(x,y) = static_cast<int>(label);
			}
		}
		counterx = 0;
		firstround = false;


	}

	std::string output;
	ROS_INFO_STREAM("labelcounter.size = " << labelcounter.size());
//	for (int i = 0; i<labelcounter.size()-1; i++)
//	{
//		std::ostringstream convert;
//		convert << labelcounter[i];
//		output.append(convert.str());
//	}

	ROS_INFO_STREAM("list of labels " << output.c_str());
	tesselated_map = temporary_map_to_work.clone();
	labelcount = labelcounter;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ipa_pars_map_analyzer_server");

	ros::NodeHandle nh;

	ParsMapTesselationServer tesselationAlgorithmObj(nh, ros::this_node::getName());
	ROS_INFO("Action Server for map_tesselation has been initialized......");
	ros::spin();

	return 0;
}

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
#include <ipa_room_segmentation/MapSegmentationAction.h>
#include <ipa_pars_map_analyzer/ParsMapTesselationAction.h>

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
	ROS_INFO("*****ParsMapAnalyzer action server*****");
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

	//erode map
	cv::Mat erode_img = original_img.clone();
	int erosion_type = cv::MORPH_RECT;
	int erosion_size = 8;
	cv::Mat element = getStructuringElement( erosion_type,
	                                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
	                                       cv::Point( erosion_size, erosion_size ) );
	cv::erode(original_img,erode_img,element);
	cv::Mat disp_orig = original_img.clone();
	cv::Mat disp_erode = erode_img.clone();

//	//make non-white pixels black
//	for (int y = 0; y < erode_img.rows; y++)
//	{
//		for (int x = 0; x < erode_img.cols; x++)
//		{
//			//find not reachable regions and make them black
//			if (erode_img.at<unsigned char>(y, x) < 250)
//			{
//				erode_img.at<unsigned char>(y, x) = 0;
//			}
//			//else make it white
//			else
//			{
//				erode_img.at<unsigned char>(y, x) = 255;
//			}
//		}
//	}

	cv::imshow("mapanalyzer_orig", disp_orig);
	cv::imshow("mapanalyzer_erode", disp_erode);

	//send map to ipa_room_segmentation_server
	sensor_msgs::Image labeling;
	cv_bridge::CvImage cv_image;
	cv_image.encoding = "mono8";
	cv_image.image = erode_img;
	cv_image.toImageMsg(labeling);

	actionlib::SimpleActionClient<ipa_room_segmentation::MapSegmentationAction> seg_ac("room_segmentation_server",true);

	seg_ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	ipa_room_segmentation::MapSegmentationGoal seg_goal;
	seg_goal.input_map = labeling;
	seg_goal.map_origin.position.x = 0;
	seg_goal.map_origin.position.y = 0;
	seg_goal.map_resolution = 0.05;
	seg_goal.return_format_in_meter = false;
	seg_goal.return_format_in_pixel = true;
	seg_goal.room_segmentation_algorithm = 2; //Distance Segmentation
	seg_goal.robot_radius = 0.3;
	seg_ac.sendGoal(seg_goal);

	//wait for the action to return
	bool finished_before_timeout = seg_ac.waitForResult(ros::Duration(300.0));

	if (finished_before_timeout)
	{
		ROS_INFO("Finished successfully!");
		ipa_room_segmentation::MapSegmentationResultConstPtr result_seg = seg_ac.getResult();

		// display
		cv_bridge::CvImagePtr cv_ptr_obj;
		cv_ptr_obj = cv_bridge::toCvCopy(result_seg->segmented_map, sensor_msgs::image_encodings::TYPE_32SC1);
		cv::Mat segmented_map = cv_ptr_obj->image;
		cv::Mat colour_segmented_map = segmented_map.clone();
		colour_segmented_map.convertTo(colour_segmented_map, CV_8U);
		cv::cvtColor(colour_segmented_map, colour_segmented_map, CV_GRAY2BGR);
		for(size_t i = 1; i <= result_seg->room_information_in_pixel.size(); ++i)
		{
			//choose random color for each room
			int blue = (rand() % 250) + 1;
			int green = (rand() % 250) + 1;
			int red = (rand() % 250) + 1;
			for(size_t u = 0; u < segmented_map.rows; ++u)
			{
				for(size_t v = 0; v < segmented_map.cols; ++v)
				{
					if(segmented_map.at<int>(u,v) == i)
					{
						colour_segmented_map.at<cv::Vec3b>(u,v)[0] = blue;
						colour_segmented_map.at<cv::Vec3b>(u,v)[1] = green;
						colour_segmented_map.at<cv::Vec3b>(u,v)[2] = red;
					}
				}
			}
		}
		cv::imshow("segmentation", colour_segmented_map);

		ROS_INFO("the important flag 1 --------------------");
		// map tesselation:
		cv::Mat map_to_tesselate = erode_img.clone();
		cv::Mat tesselated_map = map_to_tesselate.clone();
		tesselated_map.convertTo(tesselated_map, CV_32SC1);


//		//send map to ipa_pars_map_tesselation_server
		sensor_msgs::Image tess_labeling;
		cv_bridge::CvImage cv_image_tess;
		//cv_image_tess.encoding = "32SC1";
		cv_image_tess.encoding = "mono8";
		cv_image_tess.image = erode_img;
		cv_image_tess.toImageMsg(labeling);

		actionlib::SimpleActionClient<ipa_pars_map_analyzer::ParsMapTesselationAction> tess_ac("ipa_pars_map_tesselation_server",true);

		tess_ac.waitForServer(); //will wait for infinite time

		ROS_INFO("Action server started, sending goal.");
		// send a goal to the action
		ipa_pars_map_analyzer::ParsMapTesselationGoal tess_goal;
		tess_goal.input_map = labeling;
		tess_goal.map_origin.position.x = 0;
		tess_goal.map_origin.position.y = 0;
		tess_goal.map_resolution = 0.05;
		tess_ac.sendGoal(tess_goal);

		//wait for the action to return
		bool finished_before_timeout = tess_ac.waitForResult(ros::Duration(300.0));

		if (finished_before_timeout)
		{
			ROS_INFO("Finished successfully!");
			ipa_pars_map_analyzer::ParsMapTesselationResultConstPtr result_tess = tess_ac.getResult();

			// display
			cv_bridge::CvImagePtr cv_ptr_obj2;
			cv_ptr_obj2 = cv_bridge::toCvCopy(result_tess->tesselated_map, sensor_msgs::image_encodings::TYPE_32SC1);

			cv::Mat tesselated_map = cv_ptr_obj2->image;
			cv::Mat colour_tesselated_map = tesselated_map.clone();
			colour_tesselated_map.convertTo(colour_tesselated_map, CV_8U);
			cv::cvtColor(colour_tesselated_map, colour_tesselated_map, CV_GRAY2BGR);
//			cv::imshow("teswithoutcolour", colour_tesselated_map);
			ROS_INFO_STREAM("For coloring: Labels.data.size() = " << result_tess->labels.data.size());
			for(int i = 1; i <= result_tess->labels.data.size(); ++i)
			{
				ROS_INFO("label = %u",i);
				//choose random color for each room
				int blue = (rand() % 250) + 1;
				int green = (rand() % 250) + 1;
				int red = (rand() % 250) + 1;
				for(int u = 0; u < tesselated_map.rows; ++u)
				{
					for(int v = 0; v < tesselated_map.cols; ++v)
					{
						if(tesselated_map.at<int>(u,v) == i)
						{
//							ROS_INFO("Coloring label = %u", i);
							colour_tesselated_map.at<cv::Vec3b>(u,v)[0] = blue;
							colour_tesselated_map.at<cv::Vec3b>(u,v)[1] = green;
							colour_tesselated_map.at<cv::Vec3b>(u,v)[2] = red;
						}
					}
				}
			}
			cv::imshow("tesselation", colour_tesselated_map);
		}
	}

	ipa_pars_map_analyzer::ParsMapAnalyzerResult map_analyzer_action_result_;
	map_analyzer_action_result_.static_knowledge.data = "test_output";
	ipa_pars_map_analyzer_server_.setSucceeded(map_analyzer_action_result_);
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

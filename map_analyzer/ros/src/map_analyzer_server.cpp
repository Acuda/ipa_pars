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
 * ROS package name: map_analyzer
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

#include <map_analyzer/map_analyzer_server.h>
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
#include <ipa_building_msgs/MapSegmentationAction.h>
#include <map_analyzer/ParsMapTesselationAction.h>
#include <map_analyzer/ParsMapKnowledgeAction.h>

#include <map_analyzer/KnowledgeToYaml.h>

#include <map_analyzer/SquareInformation.h>
#include <std_msgs/Bool.h>

// for vector unique
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <cctype>

ParsMapAnalyzerServer::ParsMapAnalyzerServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	map_analyzer_server_(node_handle_, name_of_the_action, boost::bind(&ParsMapAnalyzerServer::execute_map_analyzer_server, this, _1), false)
{
	//Start action server
	map_analyzer_server_.start();

}

bool ParsMapAnalyzerServer::initialize()
{
	knowledgeToYamlClient_ = node_handle_.serviceClient<map_analyzer::KnowledgeToYaml>("knowledge_to_yaml_service");
	knowledgeToYamlClient_.waitForExistence(); //infinte time
	ROS_INFO("/map_analyzer_server ... initialized");
	return true;
}

void ParsMapAnalyzerServer::execute_map_analyzer_server(const map_analyzer::ParsMapAnalyzerGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****ParsMapAnalyzer action server*****");

	// empty till segmentation is over:
	std::vector<map_analyzer::SquareInformation> sqr_info;

	// todo send this to initialize:
	ROS_INFO("Create room square colors");
	std::vector<cv::Vec3b> room_colors;
	ParsMapAnalyzerServer::createRoomColors(room_colors);

	//converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat original_img = cv_ptr_obj->image;

	//set the resolution and the limits for the actual goal and the Map origin
	const double map_resolution = goal->map_resolution;
//	const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);
//	const float yaw = goal->map_origin.position.z;
	const double robot_radius = goal->robot_radius.data;
	std::vector<double> map_origin;
	map_origin.push_back(goal->map_origin.position.x);
	map_origin.push_back(goal->map_origin.position.y);
	map_origin.push_back(goal->map_origin.position.z);


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

	//erode map for error elimination
	cv::Mat erode_img = original_img.clone();
	int erosion_type = cv::MORPH_RECT;
	int erosion_size = 1;
	cv::Mat element = getStructuringElement( erosion_type,
	                                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
	                                       cv::Point( erosion_size, erosion_size ) );
	cv::erode(original_img,erode_img,element);
	cv::Mat disp_orig = original_img.clone();
	cv::Mat disp_erode = erode_img.clone();
	disp_orig.convertTo(disp_orig, CV_8U);
	displayMapAsImage(disp_erode, disp_erode, room_colors, sqr_info, 0, map_resolution, map_origin);


	// erode map to offset with robot radius:
	cv::Mat offset_img = erode_img.clone();
	int offset_erosion_type = cv::MORPH_ELLIPSE;
	int offset_size = (int) 2*robot_radius / 0.05;
	cv::Mat offset_element = getStructuringElement( offset_erosion_type,
											cv::Size(offset_size, offset_size),
											cv::Point(-1, -1));
	cv::erode(erode_img, offset_img, offset_element);
//	cv::Mat displ_offset = offset_img.clone();
	cv::Mat intersection = erode_img.clone();
	for (int r = 0; r < offset_img.rows; r++)
	{
		for (int c = 0; c < offset_img.cols; c++)
		{
			if (offset_img.at<unsigned char>(r,c) != erode_img.at<unsigned char>(r,c))
			{
				intersection.at<unsigned char>(r,c) = 1;
			}
		}
	}
//	displ_offset.convertTo(offset_img, CV_8U);
	displayMapAsImage(intersection,intersection, room_colors, sqr_info, 1, map_resolution, map_origin);



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

//	cv::imshow("orig", disp_orig);
//	cv::imshow("erode", disp_erode);

	//send map to ipa_room_segmentation_server
	sensor_msgs::Image labeling;
	cv_bridge::CvImage cv_image;
	cv_image.encoding = "mono8";
	cv_image.image = erode_img;
	cv_image.toImageMsg(labeling);

	actionlib::SimpleActionClient<ipa_building_msgs::MapSegmentationAction> seg_ac("room_segmentation_server",true);

	seg_ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
//	ipa_room_segmentation::MapSegmentationGoal seg_goal;
	ipa_building_msgs::MapSegmentationGoal seg_goal;
	seg_goal.input_map = labeling;
	seg_goal.map_origin.position.x = goal->map_origin.position.x;
	seg_goal.map_origin.position.y = goal->map_origin.position.y;
	seg_goal.map_origin.position.z = goal->map_origin.position.z;
	seg_goal.map_resolution = goal->map_resolution;
	seg_goal.robot_radius = goal->robot_radius.data;
	seg_goal.return_format_in_meter = false;
	seg_goal.return_format_in_pixel = true;
//	seg_goal.room_segmentation_algorithm = 2; //Distance Segmentation
	seg_ac.sendGoal(seg_goal);

	//wait for the action to return
	bool finished_before_timeout = seg_ac.waitForResult(ros::Duration(300.0));

	if (finished_before_timeout)
	{
		ROS_INFO("Finished successfully!");
		ipa_building_msgs::MapSegmentationResultConstPtr result_seg = seg_ac.getResult();

//		// display
		cv_bridge::CvImagePtr cv_ptr_obj;
		cv_ptr_obj = cv_bridge::toCvCopy(result_seg->segmented_map, sensor_msgs::image_encodings::TYPE_32SC1);
		cv::Mat segmented_map = cv_ptr_obj->image;
//		cv::Mat colour_segmented_map = segmented_map.clone();
//		colour_segmented_map.convertTo(colour_segmented_map, CV_8U);
//		cv::cvtColor(colour_segmented_map, colour_segmented_map, CV_GRAY2BGR);
//		for(int i = 1; i <= result_seg->room_information_in_pixel.size(); ++i)
//		{
//			//choose random color for each room
//			int blue = (rand() % 250) + 1;
//			int green = (rand() % 250) + 1;
//			int red = (rand() % 250) + 1;
//			for(size_t u = 0; u < segmented_map.rows; ++u)
//			{
//				for(size_t v = 0; v < segmented_map.cols; ++v)
//				{
//					if(segmented_map.at<int>(u,v) == i)
//					{
//						colour_segmented_map.at<cv::Vec3b>(u,v)[0] = blue;
//						colour_segmented_map.at<cv::Vec3b>(u,v)[1] = green;
//						colour_segmented_map.at<cv::Vec3b>(u,v)[2] = red;
//					}
//				}
//			}
//		}
//		cv::imshow("segmentation", colour_segmented_map);
		//display new
		displayMapAsImage(segmented_map, intersection,room_colors, sqr_info,0, map_resolution, map_origin);

//		ROS_INFO("the important flag 1 --------------------");
		cv::Mat concatenated_image = erode_img.clone();
		concatenated_image.convertTo(concatenated_image, CV_32SC1, 256, 0);
		for (int k = 1; k <= result_seg->room_information_in_pixel.size(); k++)
		{


			cv::Mat segmented_map_each_room = original_img.clone();
			segmented_map_each_room.convertTo(segmented_map_each_room, CV_8U);
//			cv::cvtColor(segmented_map_each_room, segmented_map_each_room, CV_8U2GRAY);
			for (int u = 0; u < segmented_map_each_room.rows; u++)
			{
				for (int v = 0; v < segmented_map_each_room.cols; v++)
				{
					if (segmented_map.at<int>(u,v) == k)
					{
						segmented_map_each_room.at<unsigned char>(u,v) = 255;
					}
					else
					{
						segmented_map_each_room.at<unsigned char>(u,v) = 0;
					}
				}
			}
//			cv::imshow("saft", segmented_map_each_room);

			//debug

			sensor_msgs::Image singleRoomTesselation;
			cv_bridge::CvImage cv_image_singleRoom;
			cv_image_singleRoom.encoding = "mono8";
			cv_image_singleRoom.image = segmented_map_each_room;
			cv_image_singleRoom.toImageMsg(singleRoomTesselation);
			actionlib::SimpleActionClient<map_analyzer::ParsMapTesselationAction> single_room_tess_ac("map_tesselation_server",true);

			single_room_tess_ac.waitForServer(); //will wait for infinite time

			ROS_INFO("Action server started, sending goal.");
			// send a goal to the action
			map_analyzer::ParsMapTesselationGoal single_tess_goal;
			single_tess_goal.input_map = singleRoomTesselation;
			single_tess_goal.map_origin.position.x = goal->map_origin.position.x;
			single_tess_goal.map_origin.position.y = goal->map_origin.position.y;
			single_tess_goal.map_origin.position.z = goal->map_origin.position.z;
			single_tess_goal.robot_radius.data = goal->robot_radius.data;
			single_tess_goal.map_resolution = goal->map_resolution;
			single_room_tess_ac.sendGoal(single_tess_goal);


			//wait for the action to return
			bool finished_before_timeout = single_room_tess_ac.waitForResult(ros::Duration(300.0));

			if (finished_before_timeout)
			{
				ROS_INFO("Finished successfully!");
				map_analyzer::ParsMapTesselationResultConstPtr result_single_room_tess = single_room_tess_ac.getResult();
				// display
				cv_bridge::CvImagePtr cv_ptr_obj2;
				cv_ptr_obj2 = cv_bridge::toCvCopy(result_single_room_tess->tesselated_map, sensor_msgs::image_encodings::TYPE_32SC1);

				cv::Mat tesselated_map = cv_ptr_obj2->image;
//				cv::Mat colour_tesselated_map = tesselated_map.clone();
//				colour_tesselated_map.convertTo(colour_tesselated_map, CV_8U);
//				cv::cvtColor(colour_tesselated_map, colour_tesselated_map, CV_GRAY2BGR);
//
//				//cv::imshow("teswithoutcolour", colour_tesselated_map);
//				std::vector<int> labels = result_single_room_tess->labels.data;
//				ROS_INFO_STREAM("For coloring: Labels.data.size() = " << result_single_room_tess->labels.data.size());
//				for(int i = 1; i <= result_single_room_tess->labels.data.size(); ++i)
//				{
//	//				ROS_INFO("label = %u",i);
//					//choose random color for each room
//					int blue = (rand() % 250) + 1;
//					int green = (rand() % 250) + 1;
//					int red = (rand() % 250) + 1;
//					for(int u = 0; u < tesselated_map.rows; ++u)
//					{
//						for(int v = 0; v < tesselated_map.cols; ++v)
//						{
//	//						if(tesselated_map.at<int>(u,v) == result_tess->labels.data.at(i-1))
//							if(tesselated_map.at<int>(u,v) == i)
//							{
//	//							ROS_INFO("Coloring label = %u", i);
//								colour_tesselated_map.at<cv::Vec3b>(u,v)[0] = blue;
//								colour_tesselated_map.at<cv::Vec3b>(u,v)[1] = green;
//								colour_tesselated_map.at<cv::Vec3b>(u,v)[2] = red;
//	//							labels.erase(std::remove(labels.begin(), labels.end(), i), labels.end());
//							}
//						}
//
//					}
//	//				cv::imshow("output", colour_tesselated_map);
//	//				cv::waitKey(1);
//	//				std::vector<int>& vec = myNumbers; // use shorter name
//
//	//				labels.erase(i);
//
//				}

	//			ROS_INFO_STREAM("Amount of elements in list (should be zero) = "<<labels.size());
//				std::stringstream ss;
//				ss << "window_name" << k;
//				std::string window_name = ss.str();
//				ROS_INFO("Name would be %s", window_name.c_str());
//				cv::imshow(window_name, colour_tesselated_map);

//				ROS_INFO("Debugging calculation = %u", k * 1000 + tesselated_map.at<int>(150,150));
//				concatenated_image.convertTo(concatenated_image, CV_8U);
//				cv::cvtColor(concatenated_image, concatenated_image, CV_8U2GRAY);
				for (int u = 0; u < concatenated_image.rows; u++)
				{
					for (int v = 0; v < concatenated_image.cols; v++)
					{
						if (tesselated_map.at<int>(u,v) != 0)
						{
							concatenated_image.at<int>(u,v) = (k * 1000 + tesselated_map.at<int>(u,v));

//							if ((k*100 + tesselated_map.at<int>(u,v) < 50000) && (k*100 + tesselated_map.at<int>(u,v) > 900))
//							{
//								concatenated_image.at<int>(u,v) = (k * 1000 + tesselated_map.at<int>(u,v));
//							}

						}
					}
				}

			}
		}

	 //delete errors
		for (int y = 0; y < concatenated_image.rows; ++y)
		{
			for (int x = 0; x < concatenated_image.cols; ++x)
			{
				const int label = concatenated_image.at<int>(y,x);
				if (label == 65280)
					concatenated_image.at<int>(y,x) = 0;
			}
		}
		std::vector <int> reallabelcount;
		reallabelcount.clear();
		for (int y = 0; y < concatenated_image.rows; y++)
		{
			for ( int x = 0; x< concatenated_image.cols; x++)
			{
				if (concatenated_image.at<int>(y,x) != 0)
				{
	//				counter++;
					int label = concatenated_image.at<int>(y,x);
	//				ROS_INFO("label = %u", label);
					addElementNotInVec(reallabelcount, label);
	//				tesselated_map.at<int>(y,x) = tesselated_only.at<int>(y,x);
//					concatenated_image.at<int>(y,x) = label;
				}
	//			else
	//			{
	//				tesselated_map.at<int>(y,x) = 0;
	//			}
			}
		}

		// for debug
//		for (int i=0 ; i < reallabelcount.size(); i++)
//		{
//			ROS_INFO("these labels are here: %u", reallabelcount.at(i));
//		}



		//debug:
//		ROS_INFO("balancePoints.size = %lu", balancePoints.size());


		// display
//		cv::Mat colour_tesselated_map = concatenated_image.clone();
//		colour_tesselated_map.convertTo(colour_tesselated_map, CV_8U);
//		cv::cvtColor(colour_tesselated_map, colour_tesselated_map, CV_GRAY2BGR);
//		for(int i = 0; i < reallabelcount.size(); ++i)
//		{
////				ROS_INFO("label = %u",i);
//			//choose random color for each room
//			int blue = (rand() % 250) + 1;
//			int green = (rand() % 250) + 1;
//			int red = (rand() % 250) + 1;
//
//
//			for(int u = 0; u < concatenated_image.rows; ++u)
//			{
//				for(int v = 0; v < concatenated_image.cols; ++v)
//				{
////						if(tesselated_map.at<int>(u,v) == result_tess->labels.data.at(i-1))
//					if(concatenated_image.at<int>(u,v) == reallabelcount.at(i))
//					{
////							ROS_INFO("Coloring label = %u", i);
//						colour_tesselated_map.at<cv::Vec3b>(u,v)[0] = blue;
//						colour_tesselated_map.at<cv::Vec3b>(u,v)[1] = green;
//						colour_tesselated_map.at<cv::Vec3b>(u,v)[2] = red;
//
////						colour_tesselated_map.at<cv::Vec3b>(u,v)[0] = room_colors.at(i)[0];
////						colour_tesselated_map.at<cv::Vec3b>(u,v)[1] = room_colors.at(i)[1];
////						colour_tesselated_map.at<cv::Vec3b>(u,v)[2] = room_colors.at(i)[2];
////							labels.erase(std::remove(labels.begin(), labels.end(), i), labels.end());
//					}
//				}
//
//			}
////			ROS_INFO("Drawing balance point for label %u", i);
////			ROS_INFO("On location %u, %u", balancePoints.at(i).at(0), balancePoints.at(i).at(1));
////			ROS_INFO("On location %u, %u", balancePoints.at(5).at(0), balancePoints.at(5).at(1));
////			colour_tesselated_map.at<cv::Vec3b>(balancePoints.at(i).at(1), balancePoints.at(i).at(0)) = cv::Vec3b(255,255,255);
////				cv::imshow("output", colour_tesselated_map);
////				cv::waitKey(1);
////				std::vector<int>& vec = myNumbers; // use shorter name
//
////				labels.erase(i);
//
//		}
//		cv::imshow("tesselation", colour_tesselated_map);

		displayMapAsImage(concatenated_image,intersection, room_colors, sqr_info,0, map_resolution, map_origin);

		//concat mit robot_radius
//		cv::Mat concat_with_rad = concatenated_image.clone();
//		for (int r = 0; r < concat_with_rad.rows; r++)
//		{
//			for (int c = 0; c < concat_with_rad.cols; c++)
//			{
//				if (concat_with_rad.at<int>(r,c) != 0 && concat_with_rad.at<int>(r,c) != 65535 && concat_with_rad<int> (r,c) != 65534 && intersection.at<int>(r,c) == 1)
//				{
//					concat_with_rad.at<int>(r,c) = 65532;
//				}
//
//			}
//		}

		// map tesselation:
//		cv::Mat map_to_tesselate = erode_img.clone();
//		cv::Mat tesselated_map = map_to_tesselate.clone();
//		tesselated_map.convertTo(tesselated_map, CV_32SC1);


//		//send map to ipa_pars_map_tesselation_server
//		sensor_msgs::Image tess_labeling;
//		cv_bridge::CvImage cv_image_tess;
//		//cv_image_tess.encoding = "32SC1";
//		cv_image_tess.encoding = "mono8";ROS_INFO("--------------------------------------------------------");
//		cv_image_tess.image = erode_img;
//		cv_image_tess.toImageMsg(tess_labeling);
//
//		actionlib::SimpleActionClient<map_analyzer::ParsMapTesselationAction> tess_ac("map_tesselation_server",true);
//
//		tess_ac.waitForServer(); //will wait for infinite time
//
//		ROS_INFO("Action server started, sending goal.");
//		// send a goal to the action
//		map_analyzer::ParsMapTesselationGoal tess_goal;
//		tess_goal.input_map = tess_labeling;
//		tess_goal.map_origin.position.x = 0;
//		tess_goal.map_origin.position.y = 0;
//		tess_goal.map_resolution = 0.05;
//		tess_ac.sendGoal(tess_goal);
//
//		//wait for the action to return
//		bool finished_before_timeout2 = tess_ac.waitForResult(ros::Duration(300.0));
//
//		if (finished_before_timeout2)
//		{
//			ROS_INFO("Finished successfully!");
//			map_analyzer::ParsMapTesselationResultConstPtr result_tess = tess_ac.getResult();
//
//			// display
//			cv_bridge::CvImagePtr cv_ptr_obj2;
//			cv_ptr_obj2 = cv_bridge::toCvCopy(result_tess->tesselated_map, sensor_msgs::image_encodings::TYPE_32SC1);
//
//			cv::Mat tesselated_map = cv_ptr_obj2->image;
//			cv::Mat colour_tesselated_map = tesselated_map.clone();
//			colour_tesselated_map.convertTo(colour_tesselated_map, CV_8U);
//			cv::cvtColor(colour_tesselated_map, colour_tesselated_map, CV_GRAY2BGR);
//			//cv::imshow("teswithoutcolour", colour_tesselated_map);
//			std::vector<int> labels = result_tess->labels.data;
////			ROS_INFO_STREAM("For coloring: Labels.data.size() = " << result_tess->labels.data.size());
//			for(size_t i = 1; i <= result_tess->labels.data.size(); ++i)
//			{
////				ROS_INFO("label = %u",i);
//				//choose random color for each room
//				int blue = (rand() % 250) + 1;
//				int green = (rand() % 250) + 1;
//				int red = (rand() % 250) + 1;
//				for(int u = 0; u < tesselated_map.rows; ++u)
//				{
//					for(int v = 0; v < tesselated_map.cols; ++v)
//					{
////						if(tesselated_map.at<int>(u,v) == result_tess->labels.data.at(i-1))
//						if(tesselated_map.at<int>(u,v) == i)
//						{
////							ROS_INFO("Coloring label = %u", i);
//							colour_tesselated_map.at<cv::Vec3b>(u,v)[0] = blue;
//							colour_tesselated_map.at<cv::Vec3b>(u,v)[1] = green;
//							colour_tesselated_map.at<cv::Vec3b>(u,v)[2] = red;
////							labels.erase(std::remove(labels.begin(), labels.end(), i), labels.end());
//						}
//					}
//
//				}
////				cv::imshow("output", colour_tesselated_map);
////				cv::waitKey(1);
////				std::vector<int>& vec = myNumbers; // use shorter name
//
////				labels.erase(i);
//
//
//			}
//
////			ROS_INFO_STREAM("Amount of elements in list (should be zero) = "<<labels.size());
//			cv::imshow("tesselation", colour_tesselated_map);
//			cv::waitKey(1);

//		}
//			// concatenate tesselated and segmented map
//			cv::Mat concatenated_map = erode_img.clone();
//			// for every room
//			for(size_t i = 1; i <= result_seg->room_information_in_pixel.size(); ++i)
//			{
//				//tesselated_map (32SC1)
//				for (int y = 0; y < erode_img.rows; y++)
//				{
//					for (int x = 0; x < erode_img.cols; x++)
//					{
//						concatenated_map.at<int>(y,x) = segmented_map.at<int>(y,x) * 1000 + tesselated_map.at<int>(y,x);
//					}
//				}
//			}



		// build SquareInformation Messages
//		std::vector<map_analyzer::SquareInformation> sqr_info_vec;
//		for(int i = 0; i < reallabelcount.size(); ++i)
//		{
//			map_analyzer::SquareInformation square;
//			square.label.data = reallabelcount.at(i);
//			square.center.x = balancePoints.at(i).at(0);
//			square.center.y = balancePoints.at(i).at(1);
//			square.center.z = 0.0;
//			square.transitions = vec_of_transitions.at(i);
//			sqr_info_vec.push_back(square);
//		}ROS_INFO("--------------------------------------------------------");


		actionlib::SimpleActionClient<map_analyzer::ParsMapKnowledgeAction> knowledge_ac("map_knowledge_extractor_server",true);

		knowledge_ac.waitForServer(); //will wait for infinite time

		ROS_INFO("Action server started, sending goal.");
		// send a goal to the action
		map_analyzer::ParsMapKnowledgeGoal knowledge_extractor_goal;
		sensor_msgs::Image labeled_map;
		cv_bridge::CvImage cv_image_concatened;
		cv_image_concatened.encoding = "32SC1";
		cv_image_concatened.image = concatenated_image;
		cv_image_concatened.toImageMsg(labeled_map);

		std::vector<std_msgs::Int32> list_of_labels;

		for (int i = 0; i < reallabelcount.size(); i++)
		{
			std_msgs::Int32 label;
			label.data = reallabelcount.at(i);
			list_of_labels.push_back(label);
		}
		knowledge_extractor_goal.input_map = labeled_map;
		knowledge_extractor_goal.map_resolution = goal->map_resolution;
		knowledge_extractor_goal.map_origin.position.x = goal->map_origin.position.x;
		knowledge_extractor_goal.map_origin.position.y = goal->map_origin.position.y;
		knowledge_extractor_goal.map_origin.position.z = goal->map_origin.position.z;
		knowledge_extractor_goal.robot_radius.data = goal->robot_radius.data;
		knowledge_ac.sendGoal(knowledge_extractor_goal);


		//wait for the action to return
		bool finished_before_timeout = knowledge_ac.waitForResult(ros::Duration(300.0));

		if (finished_before_timeout)
		{
			ROS_INFO("Finished successfully!");
			map_analyzer::ParsMapKnowledgeResultConstPtr result_knowledge = knowledge_ac.getResult();
			// display
			ROS_INFO("The produced square information is:");
			std::vector<map_analyzer::SquareInformation> sqr_info = result_knowledge->square_information;
			ROS_INFO_STREAM("The size of the given square information vector is" << sqr_info.size());

			// display with names and balancePoints
			displayMapAsImage(concatenated_image, intersection, room_colors, sqr_info, 0, map_resolution, map_origin);

			// display balance points, intersection and transitions but no names:
			displayMapAsImage(concatenated_image, intersection, room_colors, sqr_info, 1, map_resolution, map_origin);

			//add properties navigable to square_information!
			for (int i = 0; i < sqr_info.size(); i++)
			{
				int row = (intersection.rows - ((sqr_info.at(i).center.y + goal->map_origin.position.y)) / map_resolution);
				int col = (sqr_info.at(i).center.x + goal->map_origin.position.x) / map_resolution;
				if (intersection.at<unsigned char>(row,col) != 0 && intersection.at<unsigned char>(row,col) != 1)
				{
					sqr_info.at(i).navigable.data = true;
				}
			}

			// sending to yaml dumper
			map_analyzer::KnowledgeToYaml knowledge_srv;
			knowledge_srv.request.square_information = sqr_info;
//			map_analyzer::KnowledgeToYamlResponse resp;
//			knowledgeToYamlClient.waitForExistence();
			knowledgeToYamlClient_.call(knowledge_srv);

			if (knowledge_srv.response.success)
			{
				ROS_INFO("The answer is true");
			}
			else
			{
				ROS_INFO("The answer is false");
			}

		}


	}






	map_analyzer::ParsMapAnalyzerResult map_analyzer_action_result_;
	map_analyzer_action_result_.static_knowledge.data = "test_output";
	map_analyzer_server_.setSucceeded(map_analyzer_action_result_);
	cv::waitKey();
}

void ParsMapAnalyzerServer::addElementNotInVec(std::vector<int> &reallabelcount, int label)
{
	if (std::find(reallabelcount.begin(), reallabelcount.end(), label) != reallabelcount.end())
	{

	}
	else
	{
//		ROS_INFO_STREAM("Adding label =" << label);
		reallabelcount.push_back(label);
	}
}


void ParsMapAnalyzerServer::createRoomColors(std::vector<cv::Vec3b> &room_colors)
{
	for (int blue = 50; blue < 255; blue += 50)
	{
		for (int green = 50; green < 255; green += 50)
		{
			for (int red = 50; red < 255; red += 50)
			{
				cv::Vec3b color;
				color[0] = blue;
				color[1] = green;
				color[2] = red;
				room_colors.push_back(color);
			}
		}
	}

//	ROS_INFO("Created %d colors for room squares segmenation", (int)room_colors.size());
}

void ParsMapAnalyzerServer::displayMapAsImage(cv::Mat &map, cv::Mat &map_with_rob_rad, std::vector<cv::Vec3b> &room_colors,  std::vector<map_analyzer::SquareInformation> &sqr_info, int printtype, double map_resolution, std::vector<double> map_origin)
{
	// for writing debug images:
	std::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(100);


	std::vector<int> labels;
	int image_type = map.type();
	ROS_INFO("This is the Image Type you try to paint %u ", image_type);
	if (map.type() == 0)
	{
		for (int r = 0; r < map.rows; ++r)
		{
			for ( int c = 0; c < map.cols; ++c)
			{
				if (map.at<unsigned char>(r,c) != 0)
				{
					int label = map.at<unsigned char>(r,c);
					addElementNotInVec(labels, label);
				}
			}
		}
	}
	else
	{
		for (int r = 0; r < map.rows; ++r)
		{
			for ( int c = 0; c < map.cols; ++c)
			{
				if (map.at<int>(r,c) != 0)
				{
					int label = map.at<int>(r,c);
					addElementNotInVec(labels, label);
				}
			}
		}
	}
//	ROS_INFO("--------------------------------------------------------");
//	ROS_INFO_STREAM("THE LABELS.size() is "<< labels.size());
//	ROS_INFO("--------------------------------------------------------");

	cv::Mat colored_map = map.clone();
	colored_map.convertTo(colored_map, CV_8U);
	cv::cvtColor(colored_map, colored_map, CV_GRAY2BGR);
	if (labels.size() < 3 && map.type() == 0 && sqr_info.size() == 0) // unlabeled map
	{
		ROS_INFO("Printing with printtype %u", printtype);
		if (printtype == 0)
		{
			for (int r = 0; r < colored_map.rows; ++r)
			{
				for (int c = 0; c < colored_map.cols; ++c)
				{
					if (map.at<unsigned char>(r,c) == 0)
					{
						colored_map.at<cv::Vec3b>(r,c)[0] = 0;
						colored_map.at<cv::Vec3b>(r,c)[1] = 0;
						colored_map.at<cv::Vec3b>(r,c)[2] = 0;
					}
					else
					{
						colored_map.at<cv::Vec3b>(r,c)[0] = 255;
						colored_map.at<cv::Vec3b>(r,c)[1] = 255;
						colored_map.at<cv::Vec3b>(r,c)[2] = 255;
					}
				}
			}
			cv::imshow("input_map_as_image", colored_map);
			cv::imwrite("ipa_pars/log/input_map_as_image.jpeg", colored_map, compression_params);
		}
		else if (printtype == 1)
		{
			for (int r = 0; r < colored_map.rows; ++r)
			{
				for (int c = 0; c < colored_map.cols; ++c)
				{
					if (map.at<unsigned char>(r,c) == 0)
					{
						colored_map.at<cv::Vec3b>(r,c)[0] = 0;
						colored_map.at<cv::Vec3b>(r,c)[1] = 0;
						colored_map.at<cv::Vec3b>(r,c)[2] = 0;
					}
					else if (map.at<unsigned char>(r,c) == 1)
					{
						colored_map.at<cv::Vec3b>(r,c)[0] = 255;
						colored_map.at<cv::Vec3b>(r,c)[1] = 0;
						colored_map.at<cv::Vec3b>(r,c)[2] = 0;
					}
					else
					{
						colored_map.at<cv::Vec3b>(r,c)[0] = 255;
						colored_map.at<cv::Vec3b>(r,c)[1] = 255;
						colored_map.at<cv::Vec3b>(r,c)[2] = 255;
					}
				}
			}
			cv::imshow("intersection_map_as_image", colored_map);
			cv::imwrite("ipa_pars/log/intersection_map_as_image.jpeg", colored_map, compression_params);
		}



	}
	else if (labels.size() < 125 && map.type() == 4 && sqr_info.size() == 0) // segmented map
	{
		for (int i = 0; i < labels.size(); ++i)
		{
			for (int r = 0; r < colored_map.rows; ++r)
			{
				for (int c = 0; c < colored_map.cols; ++c)
				{
					if (map.at<int>(r,c) == i)
					{
						cv::Vec3b color = room_colors.at(i);
						colored_map.at<cv::Vec3b>(r,c)[0] = color[0];
						colored_map.at<cv::Vec3b>(r,c)[1] = color[1];
						colored_map.at<cv::Vec3b>(r,c)[2] = color[2];
					}

				}
			}
		}
		for (int r = 0; r < colored_map.rows; ++r)
		{
			for (int c = 0; c < colored_map.cols; ++c)
			{
				if (map.at<int>(r,c) == 0)
				{
					colored_map.at<cv::Vec3b>(r,c)[0] = 0;
					colored_map.at<cv::Vec3b>(r,c)[1] = 0;
					colored_map.at<cv::Vec3b>(r,c)[2] = 0;
				}

			}
		}
		cv::imshow("segmented_map_as_image", colored_map);
		cv::imwrite("ipa_pars/log/segmented_map_as_image.jpeg", colored_map, compression_params);
	}
	else if (labels.size() > 125 && map.type() == 4 && sqr_info.size() == 0) // tesselated_map
	{
		// extract rooms:
		std::vector<int> room_labels;
		for (int i = 0; i < labels.size(); ++i)
		{
			addElementNotInVec(room_labels, labels.at(i));
		}

		// color rooms:
		for (int j = 0; j < room_labels.size(); ++j)
		{
			for (int r = 0; r < colored_map.rows; ++r)
			{
				for (int c = 0; c < colored_map.cols; ++c)
				{
					if ((int)(map.at<int>(r,c) / 1000) == j)
					{
						cv::Vec3b color = room_colors.at(j);
						colored_map.at<cv::Vec3b>(r,c)[0] = color[0];
						colored_map.at<cv::Vec3b>(r,c)[1] = color[1];
						colored_map.at<cv::Vec3b>(r,c)[2] = color[2];
					}
				}
			}
		}

		for (int r = 0; r < colored_map.rows; ++r)
		{
			for (int c = 0; c < colored_map.cols; ++c)
			{
				if (map.at<int>(r,c) == 0)
				{
					colored_map.at<cv::Vec3b>(r,c)[0] = 0;
					colored_map.at<cv::Vec3b>(r,c)[1] = 0;
					colored_map.at<cv::Vec3b>(r,c)[2] = 0;
				}

			}
		}

		// color square edges
		for (int r = 0; r < colored_map.rows; ++r)
		{
			for (int c = 0; c < colored_map.cols; ++c)
			{
				if (map.at<int>(r,c) != 0)
				{
					if ((map.at<int>(r,c) != map.at<int>(r,c+1)) && (map.at<int>(r,c+1) != 0))
					{
						colored_map.at<cv::Vec3b>(r,c)[0] = 255;
						colored_map.at<cv::Vec3b>(r,c)[1] = 255;
						colored_map.at<cv::Vec3b>(r,c)[2] = 255;
					}
					else if ((map.at<int>(r,c) != map.at<int>(r+1,c)) && (map.at<int>(r+1,c) != 0))
					{
						colored_map.at<cv::Vec3b>(r,c)[0] = 255;
						colored_map.at<cv::Vec3b>(r,c)[1] = 255;
						colored_map.at<cv::Vec3b>(r,c)[2] = 255;
					}
				}
			}
		}
		cv::imshow("tesselated_map_as_image", colored_map);
		cv::imwrite("ipa_pars/log/tesselated_map_as_image.jpeg", colored_map, compression_params);
	}
	else if (sqr_info.size() > 0)
	{
		if (printtype == 0)
		{
			// extract rooms:
					std::vector<int> room_labels;
					for (int i = 0; i < labels.size(); ++i)
					{
						addElementNotInVec(room_labels, labels.at(i));
					}

					// color rooms:
					for (int j = 0; j < room_labels.size(); ++j)
					{
						for (int r = 0; r < colored_map.rows; ++r)
						{
							for (int c = 0; c < colored_map.cols; ++c)
							{
								if ((int)(map.at<int>(r,c) / 1000) == j)
								{
									cv::Vec3b color = room_colors.at(j);
									colored_map.at<cv::Vec3b>(r,c)[0] = color[0];
									colored_map.at<cv::Vec3b>(r,c)[1] = color[1];
									colored_map.at<cv::Vec3b>(r,c)[2] = color[2];
								}
							}
						}
					}

					for (int r = 0; r < colored_map.rows; ++r)
					{
						for (int c = 0; c < colored_map.cols; ++c)
						{
							if (map.at<int>(r,c) == 0)
							{
								colored_map.at<cv::Vec3b>(r,c)[0] = 0;
								colored_map.at<cv::Vec3b>(r,c)[1] = 0;
								colored_map.at<cv::Vec3b>(r,c)[2] = 0;
							}

						}
					}

					// color square edges
					for (int r = 0; r < colored_map.rows; ++r)
					{
						for (int c = 0; c < colored_map.cols; ++c)
						{
							if (map.at<int>(r,c) != 0)
							{
								if ((map.at<int>(r,c) != map.at<int>(r,c+1)) && (map.at<int>(r,c+1) != 0))
								{
									colored_map.at<cv::Vec3b>(r,c)[0] = 255;
									colored_map.at<cv::Vec3b>(r,c)[1] = 255;
									colored_map.at<cv::Vec3b>(r,c)[2] = 255;
								}
								else if ((map.at<int>(r,c) != map.at<int>(r+1,c)) && (map.at<int>(r+1,c) != 0))
								{
									colored_map.at<cv::Vec3b>(r,c)[0] = 255;
									colored_map.at<cv::Vec3b>(r,c)[1] = 255;
									colored_map.at<cv::Vec3b>(r,c)[2] = 255;
								}
							}
						}
					}



					// put SquareNumbers on BalancePoint Spots
					for (int c = 0; c < sqr_info.size(); c++)
					{
						std::string room_number = boost::lexical_cast<std::string>( (int)sqr_info.at(c).label.data / 1000);
						std::string square_number = boost::lexical_cast<std::string>( (int) sqr_info.at(c).label.data % 1000);
						std::string sqr_numb = room_number+"-"+square_number;
//						cv::putText(colored_map, sqr_numb , cv::Point( (int) ((sqr_info.at(c).center.x + map_origin.at(0)) / map_resolution ) -10, (int) colored_map.rows - ((sqr_info.at(c).center.y+ map_origin.at(1)) / map_resolution)+10 ), CV_FONT_HERSHEY_SIMPLEX, 0.2f,
//						        cv::Scalar(255, 255, 255), 0.1, 8, false);
						//  and transformation in pixels
						int balance_x = (int) ((sqr_info.at(c).center.x ) / map_resolution);
						int balance_y = (int) ((sqr_info.at(c).center.y ) / map_resolution);
						// transformed with
						double yaw = map_origin.at(2);
						//transform back meins clowise!
						yaw = -yaw;
						// x y changed because transformation invers!
						int trans_x = balance_x * cos(yaw) + balance_y * sin(yaw);
						int trans_y = (-1) * balance_x * sin(yaw) + balance_y * cos(yaw);
						// transform to upper left corner (x = cols; y = rows!);
						int translat_x = trans_x + map_origin.at(0)/map_resolution;
						int translat_y = trans_y + map_origin.at(1)/map_resolution;
						int corn_x = translat_x;
						int corn_y = colored_map.rows - translat_y;
						colored_map.at<cv::Vec3b>(corn_y,corn_x)[0] = 255;
						colored_map.at<cv::Vec3b>(corn_y, corn_x)[1] = 255;
						colored_map.at<cv::Vec3b>(corn_y, corn_x)[2] = 51;

						cv::putText(colored_map, sqr_numb , cv::Point( corn_x - 8, corn_y + 5), CV_FONT_HERSHEY_SIMPLEX, 0.2f,cv::Scalar(255, 255, 255), 0.1, 8, false);
					}

					// drawArrow:
					double length = 2 / map_resolution; //40 px in general
					double arrowLength = 0.25 * length;
					double arrowAngle = 2.62; // 30 degrees as arrow angle;
					double zero_x = map_origin.at(0) / map_resolution;
					double zero_y = colored_map.rows - (map_origin.at(1) / map_resolution);
					double yaw = map_origin.at(2);
					// x axis:
					cv::line(colored_map, cv::Point(zero_x,zero_y), cv::Point(zero_x + (int) (cos(yaw)*length), zero_y - (int) (sin(yaw) * length)),cv::Scalar(255,0,0),1,CV_AA,0);
					cv::line(colored_map, cv::Point(zero_x + (int) (cos(yaw)*length), zero_y - (int) (sin(yaw) * length)), cv::Point(zero_x + (int) (cos(yaw)*length) + (int) (cos(yaw + arrowAngle) * arrowLength) , zero_y - (int) (sin(yaw) * length) - (int) (sin(yaw+arrowAngle) * arrowLength)),cv::Scalar(255,0,0),1,CV_AA,0);
					cv::line(colored_map, cv::Point(zero_x + (int) (cos(yaw)*length), zero_y - (int) (sin(yaw) * length)), cv::Point(zero_x + (int) (cos(yaw)*length) + (int) (cos(yaw - arrowAngle) * arrowLength) , zero_y - (int) (sin(yaw) * length) - (int) (sin(yaw-arrowAngle) * arrowLength)),cv::Scalar(255,0,0),1,CV_AA,0);
					// y axis
					yaw = yaw + 1.57;
					cv::line(colored_map, cv::Point(zero_x,zero_y), cv::Point(zero_x + (int) (cos(yaw)*length), zero_y - (int) (sin(yaw) * length)),cv::Scalar(255,0,0),1,CV_AA,0);
					cv::line(colored_map, cv::Point(zero_x + (int) (cos(yaw)*length), zero_y - (int) (sin(yaw) * length)), cv::Point(zero_x + (int) (cos(yaw)*length) + (int) (cos(yaw + arrowAngle) * arrowLength) , zero_y - (int) (sin(yaw) * length) - (int) (sin(yaw+arrowAngle) * arrowLength)),cv::Scalar(255,0,0),1,CV_AA,0);
					cv::line(colored_map, cv::Point(zero_x + (int) (cos(yaw)*length), zero_y - (int) (sin(yaw) * length)), cv::Point(zero_x + (int) (cos(yaw)*length) + (int) (cos(yaw - arrowAngle) * arrowLength) , zero_y - (int) (sin(yaw) * length) - (int) (sin(yaw-arrowAngle) * arrowLength)),cv::Scalar(255,0,0),1,CV_AA,0);

//					//put map Origin and axis
//					//y - axis
//					int x_origin_in_px = map_origin.at(0) / map_resolution;
//					int y_origin_in_px = colored_map.rows - (map_origin.at(1) / map_resolution);
//					cv::line(colored_map, cv::Point(x_origin_in_px,y_origin_in_px),cv::Point(x_origin_in_px,y_origin_in_px - (2/map_resolution)),cv::Scalar(255,0,0),1,CV_AA,0);
//					cv::line(colored_map, cv::Point(x_origin_in_px,y_origin_in_px - (2/map_resolution)),cv::Point(x_origin_in_px + (0.15 / map_resolution ),y_origin_in_px - (2/map_resolution) + (0.2/map_resolution)),cv::Scalar(255,0,0),1,CV_AA,0);
//					cv::line(colored_map, cv::Point(x_origin_in_px,y_origin_in_px - (2/map_resolution)),cv::Point(x_origin_in_px - (0.15 / map_resolution ),y_origin_in_px - (2/map_resolution) + (0.2/map_resolution)),cv::Scalar(255,0,0),1,CV_AA,0);
////					// x
//					cv::line(colored_map, cv::Point(x_origin_in_px,y_origin_in_px),cv::Point(424,y_origin_in_px),cv::Scalar(255,0,0),1,CV_AA,0);
//					cv::line(colored_map, cv::Point(x_origin_in_px + (2/map_resolution),y_origin_in_px),cv::Point(x_origin_in_px + (2/map_resolution) - (0.2/map_resolution),y_origin_in_px + (0.15 / map_resolution )),cv::Scalar(255,0,0),1,CV_AA,0);
//					cv::line(colored_map, cv::Point(x_origin_in_px + (2/map_resolution),y_origin_in_px),cv::Point(x_origin_in_px + (2/map_resolution) - (0.2/map_resolution),y_origin_in_px - (0.15 / map_resolution )),cv::Scalar(255,0,0),1,CV_AA,0);

					cv::imshow("map_with_square_info_as_image", colored_map);
					cv::imwrite("ipa_pars/log/map_with_square_info_as_image.jpeg", colored_map, compression_params);
		}
		else if (printtype == 1)
		{
			// extract rooms:
			std::vector<int> room_labels;
			for (int i = 0; i < labels.size(); ++i)
			{
				addElementNotInVec(room_labels, labels.at(i));
			}

			// color rooms:
			for (int j = 0; j < room_labels.size(); ++j)
			{
				for (int r = 0; r < colored_map.rows; ++r)
				{
					for (int c = 0; c < colored_map.cols; ++c)
					{
						if ((int)(map.at<int>(r,c) / 1000) == j)
						{
							cv::Vec3b color = room_colors.at(j);
							colored_map.at<cv::Vec3b>(r,c)[0] = color[0];
							colored_map.at<cv::Vec3b>(r,c)[1] = color[1];
							colored_map.at<cv::Vec3b>(r,c)[2] = color[2];
						}
					}
				}
			}

			for (int r = 0; r < colored_map.rows; ++r)
			{
				for (int c = 0; c < colored_map.cols; ++c)
				{
					if (map.at<int>(r,c) == 0)
					{
						colored_map.at<cv::Vec3b>(r,c)[0] = 0;
						colored_map.at<cv::Vec3b>(r,c)[1] = 0;
						colored_map.at<cv::Vec3b>(r,c)[2] = 0;
					}

				}
			}

			// color robo radius
			for (int r = 0; r < colored_map.rows; ++r)
			{
				for (int c = 0; c < colored_map.cols; ++c)
				{
					if (map_with_rob_rad.at<unsigned char>(r,c) == 1)
					{
						colored_map.at<cv::Vec3b>(r,c)[0] = 255;
						colored_map.at<cv::Vec3b>(r,c)[1] = 0;
						colored_map.at<cv::Vec3b>(r,c)[2] = 0;
					}
				}
			}

			// color square edges
			for (int r = 0; r < colored_map.rows; ++r)
			{
				for (int c = 0; c < colored_map.cols; ++c)
				{
					if (map.at<int>(r,c) != 0)
					{
						if ((map.at<int>(r,c) != map.at<int>(r,c+1)) && (map.at<int>(r,c+1) != 0))
						{
							colored_map.at<cv::Vec3b>(r,c)[0] = 255;
							colored_map.at<cv::Vec3b>(r,c)[1] = 255;
							colored_map.at<cv::Vec3b>(r,c)[2] = 255;
						}
						else if ((map.at<int>(r,c) != map.at<int>(r+1,c)) && (map.at<int>(r+1,c) != 0))
						{
							colored_map.at<cv::Vec3b>(r,c)[0] = 255;
							colored_map.at<cv::Vec3b>(r,c)[1] = 255;
							colored_map.at<cv::Vec3b>(r,c)[2] = 255;
						}
					}
				}
			}



//			// put SquareNumbers on BalancePoint Spots
			for (int c = 0; c < sqr_info.size(); c++)
			{
				int balance_x = (int) ((sqr_info.at(c).center.x ) / map_resolution);
				int balance_y = (int) ((sqr_info.at(c).center.y ) / map_resolution);
				// transformed with
				double yaw = map_origin.at(2);
				//transform back meins clowise!
				yaw = -yaw;
				// x y changed because transformation invers!
				int trans_x = balance_x * cos(yaw) + balance_y * sin(yaw);
				int trans_y = (-1) * balance_x * sin(yaw) + balance_y * cos(yaw);
				// transform to upper left corner (x = cols; y = rows!);
				int translat_x = trans_x + map_origin.at(0)/map_resolution;
				int translat_y = trans_y + map_origin.at(1)/map_resolution;
				int corn_x = translat_x;
				int corn_y = colored_map.rows - translat_y;
				colored_map.at<cv::Vec3b>(corn_y,corn_x)[0] = 255;
				colored_map.at<cv::Vec3b>(corn_y, corn_x)[1] = 255;
				colored_map.at<cv::Vec3b>(corn_y, corn_x)[2] = 51;
			}
//
			//put map Origin and axis
			//y - axis
			// transform origin translation
//			int x_origin_in_px = map_origin.at(0) / map_resolution;
//			int y_origin_in_px = colored_map.rows - (map_origin.at(1) / map_resolution);

			// transform rotation:
//			cv::Point point_before_rot;
//			point_before_rot = (x_origin_in_px,y_origin_in_px);
//			cv::Point point_after_rot;

			//todo: drawKS Function. origin(x,y,yaw) + map_resolution

			// drawArrow:
			double length = 2 / map_resolution; //40 px in general
			double arrowLength = 0.25 * length;
			double arrowAngle = 2.62; // 30 degrees as arrow angle;
			double zero_x = map_origin.at(0) / map_resolution;
			double zero_y = colored_map.rows - (map_origin.at(1) / map_resolution);
			double yaw = map_origin.at(2);
			// x axis:
			cv::line(colored_map, cv::Point(zero_x,zero_y), cv::Point(zero_x + (int) (cos(yaw)*length), zero_y - (int) (sin(yaw) * length)),cv::Scalar(255,0,0),1,CV_AA,0);
			cv::line(colored_map, cv::Point(zero_x + (int) (cos(yaw)*length), zero_y - (int) (sin(yaw) * length)), cv::Point(zero_x + (int) (cos(yaw)*length) + (int) (cos(yaw + arrowAngle) * arrowLength) , zero_y - (int) (sin(yaw) * length) - (int) (sin(yaw+arrowAngle) * arrowLength)),cv::Scalar(255,0,0),1,CV_AA,0);
			cv::line(colored_map, cv::Point(zero_x + (int) (cos(yaw)*length), zero_y - (int) (sin(yaw) * length)), cv::Point(zero_x + (int) (cos(yaw)*length) + (int) (cos(yaw - arrowAngle) * arrowLength) , zero_y - (int) (sin(yaw) * length) - (int) (sin(yaw-arrowAngle) * arrowLength)),cv::Scalar(255,0,0),1,CV_AA,0);
			// y axis
			yaw = yaw + 1.57;
			cv::line(colored_map, cv::Point(zero_x,zero_y), cv::Point(zero_x + (int) (cos(yaw)*length), zero_y - (int) (sin(yaw) * length)),cv::Scalar(255,0,0),1,CV_AA,0);
			cv::line(colored_map, cv::Point(zero_x + (int) (cos(yaw)*length), zero_y - (int) (sin(yaw) * length)), cv::Point(zero_x + (int) (cos(yaw)*length) + (int) (cos(yaw + arrowAngle) * arrowLength) , zero_y - (int) (sin(yaw) * length) - (int) (sin(yaw+arrowAngle) * arrowLength)),cv::Scalar(255,0,0),1,CV_AA,0);
			cv::line(colored_map, cv::Point(zero_x + (int) (cos(yaw)*length), zero_y - (int) (sin(yaw) * length)), cv::Point(zero_x + (int) (cos(yaw)*length) + (int) (cos(yaw - arrowAngle) * arrowLength) , zero_y - (int) (sin(yaw) * length) - (int) (sin(yaw-arrowAngle) * arrowLength)),cv::Scalar(255,0,0),1,CV_AA,0);

//			cv::line(colored_map, cv::Point(x_origin_in_px,y_origin_in_px), cv::Point(x_origin_in_px * cos(-yaw) + (y_origin_in_px - (2/map_resolution)) * sin(-yaw), -x_origin_in_px * sin(-yaw) + (y_origin_in_px - (2/map_resolution)) * cos(-yaw)), cv::Scalar(255,0,0),1,CV_AA,0);
//			cv::line(colored_map, cv::Point(x_origin_in_px,y_origin_in_px - (2/map_resolution)),cv::Point(x_origin_in_px + (0.15 / map_resolution ),y_origin_in_px - (2/map_resolution) + (0.2/map_resolution)),cv::Scalar(255,0,0),1,CV_AA,0);
//			cv::line(colored_map, cv::Point(x_origin_in_px,y_origin_in_px - (2/map_resolution)),cv::Point(x_origin_in_px - (0.15 / map_resolution ),y_origin_in_px - (2/map_resolution) + (0.2/map_resolution)),cv::Scalar(255,0,0),1,CV_AA,0);
////			// x
//			cv::line(colored_map, cv::Point(x_origin_in_px,y_origin_in_px),cv::Point(424,y_origin_in_px),cv::Scalar(255,0,0),1,CV_AA,0);
//			cv::line(colored_map, cv::Point(x_origin_in_px + (2/map_resolution),y_origin_in_px),cv::Point(x_origin_in_px + (2/map_resolution) - (0.2/map_resolution),y_origin_in_px + (0.15 / map_resolution )),cv::Scalar(255,0,0),1,CV_AA,0);
//			cv::line(colored_map, cv::Point(x_origin_in_px + (2/map_resolution),y_origin_in_px),cv::Point(x_origin_in_px + (2/map_resolution) - (0.2/map_resolution),y_origin_in_px - (0.15 / map_resolution )),cv::Scalar(255,0,0),1,CV_AA,0);

//			cv::line(colored_map, cv::Point(x_origin_in_px,y_origin_in_px),cv::Point(x_origin_in_px,y_origin_in_px - (2/map_resolution)),cv::Scalar(255,0,0),1,CV_AA,0);
//			cv::line(colored_map, cv::Point(x_origin_in_px,y_origin_in_px - (2/map_resolution)),cv::Point(x_origin_in_px + (0.15 / map_resolution ),y_origin_in_px - (2/map_resolution) + (0.2/map_resolution)),cv::Scalar(255,0,0),1,CV_AA,0);
//			cv::line(colored_map, cv::Point(x_origin_in_px,y_origin_in_px - (2/map_resolution)),cv::Point(x_origin_in_px - (0.15 / map_resolution ),y_origin_in_px - (2/map_resolution) + (0.2/map_resolution)),cv::Scalar(255,0,0),1,CV_AA,0);
////			// x
//			cv::line(colored_map, cv::Point(x_origin_in_px,y_origin_in_px),cv::Point(424,y_origin_in_px),cv::Scalar(255,0,0),1,CV_AA,0);
//			cv::line(colored_map, cv::Point(x_origin_in_px + (2/map_resolution),y_origin_in_px),cv::Point(x_origin_in_px + (2/map_resolution) - (0.2/map_resolution),y_origin_in_px + (0.15 / map_resolution )),cv::Scalar(255,0,0),1,CV_AA,0);
//			cv::line(colored_map, cv::Point(x_origin_in_px + (2/map_resolution),y_origin_in_px),cv::Point(x_origin_in_px + (2/map_resolution) - (0.2/map_resolution),y_origin_in_px - (0.15 / map_resolution )),cv::Scalar(255,0,0),1,CV_AA,0);

			cv::imshow("map_with_squares_and_robo_radius_as_image", colored_map);
			cv::imwrite("ipa_pars/log/map_with_squares_and_robo_radius_as_image.jpeg", colored_map, compression_params);
		}

	}
	else
	{
		ROS_ERROR("The map you try to display has a wrong format or to many rooms!");
		ROS_ERROR("Format should be of cv::type 0 or 4; amount of rooms possible 1-65");
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_analyzer_server");

	ros::NodeHandle nh;

	ParsMapAnalyzerServer analyzerAlgorithmObj(nh, ros::this_node::getName());
	if (!analyzerAlgorithmObj.initialize())
	{
		ROS_ERROR("Failed to inizialize map_analyzer");
		return -1;
	}
	else
	{
		ROS_INFO("Action Server for map_analyzer has been initialized......");
	}

	ros::spin();

	return 0;
}

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
#include <ipa_pars_map_analyzer/ParsMapKnowledgeAction.h>

#include <ipa_pars_map_analyzer/KnowledgeToYaml.h>

// for vector unique
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <cctype>

ParsMapAnalyzerServer::ParsMapAnalyzerServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	ipa_pars_map_analyzer_server_(node_handle_, name_of_the_action, boost::bind(&ParsMapAnalyzerServer::execute_map_analyzer_server, this, _1), false)
{
	//Start action server
	ipa_pars_map_analyzer_server_.start();

}

bool ParsMapAnalyzerServer::initialize()
{
	knowledgeToYamlClient_ = node_handle_.serviceClient<ipa_pars_map_analyzer::KnowledgeToYaml>("knowledge_to_yaml_service");
	knowledgeToYamlClient_.waitForExistence(); //infinte time
	ROS_INFO("/map_analyzer_server ... initialized");
	return true;
}

void ParsMapAnalyzerServer::execute_map_analyzer_server(const ipa_pars_map_analyzer::ParsMapAnalyzerGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****ParsMapAnalyzer action server*****");
	ROS_INFO("map resolution is : %f", goal->map_resolution);

	// empty till segmentation is over:
	std::vector<ipa_pars_map_analyzer::SquareInformation> sqr_info;

	// todo send this to initialize:
	ROS_INFO("Create room square colors");
	std::vector<cv::Vec3b> room_colors;
	ParsMapAnalyzerServer::createRoomColors(room_colors);

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
	int erosion_size = 1;
	cv::Mat element = getStructuringElement( erosion_type,
	                                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
	                                       cv::Point( erosion_size, erosion_size ) );
	cv::erode(original_img,erode_img,element);
	cv::Mat disp_orig = original_img.clone();
	cv::Mat disp_erode = erode_img.clone();
	disp_orig.convertTo(disp_orig, CV_8U);
	displayMapAsImage(disp_erode, room_colors, sqr_info);
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

	actionlib::SimpleActionClient<ipa_room_segmentation::MapSegmentationAction> seg_ac("room_segmentation_server",true);

	seg_ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	ipa_room_segmentation::MapSegmentationGoal seg_goal;
	seg_goal.input_map = labeling;
	seg_goal.map_origin.position.x = goal->map_origin.position.x;
	seg_goal.map_origin.position.y = goal->map_origin.position.y;
	seg_goal.map_resolution = goal->map_resolution;
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
		displayMapAsImage(segmented_map, room_colors, sqr_info);

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
			actionlib::SimpleActionClient<ipa_pars_map_analyzer::ParsMapTesselationAction> single_room_tess_ac("ipa_pars_map_tesselation_server",true);

			single_room_tess_ac.waitForServer(); //will wait for infinite time

			ROS_INFO("Action server started, sending goal.");
			// send a goal to the action
			ipa_pars_map_analyzer::ParsMapTesselationGoal single_tess_goal;
			single_tess_goal.input_map = singleRoomTesselation;
			single_tess_goal.map_origin.position.x = goal->map_origin.position.x;
			single_tess_goal.map_origin.position.y = goal->map_origin.position.y;
			single_tess_goal.map_resolution = goal->map_resolution;
			single_room_tess_ac.sendGoal(single_tess_goal);


			//wait for the action to return
			bool finished_before_timeout = single_room_tess_ac.waitForResult(ros::Duration(300.0));

			if (finished_before_timeout)
			{
				ROS_INFO("Finished successfully!");
				ipa_pars_map_analyzer::ParsMapTesselationResultConstPtr result_single_room_tess = single_room_tess_ac.getResult();
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

//							if ((k*100 + tesselated_map.at<int>(u,v) < 50000) && (k*100 + tesselated_map.at<int>(u,v) > 900)) //TODO:
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

		displayMapAsImage(concatenated_image, room_colors, sqr_info);
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
//		actionlib::SimpleActionClient<ipa_pars_map_analyzer::ParsMapTesselationAction> tess_ac("ipa_pars_map_tesselation_server",true);
//
//		tess_ac.waitForServer(); //will wait for infinite time
//
//		ROS_INFO("Action server started, sending goal.");
//		// send a goal to the action
//		ipa_pars_map_analyzer::ParsMapTesselationGoal tess_goal;
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
//			ipa_pars_map_analyzer::ParsMapTesselationResultConstPtr result_tess = tess_ac.getResult();
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
//		std::vector<ipa_pars_map_analyzer::SquareInformation> sqr_info_vec;
//		for(int i = 0; i < reallabelcount.size(); ++i)
//		{
//			ipa_pars_map_analyzer::SquareInformation square;
//			square.label.data = reallabelcount.at(i);
//			square.center.x = balancePoints.at(i).at(0);
//			square.center.y = balancePoints.at(i).at(1);
//			square.center.z = 0.0;
//			square.transitions = vec_of_transitions.at(i);
//			sqr_info_vec.push_back(square);
//		}ROS_INFO("--------------------------------------------------------");


		actionlib::SimpleActionClient<ipa_pars_map_analyzer::ParsMapKnowledgeAction> knowledge_ac("ipa_pars_map_knowledge_extractor_server",true);

		knowledge_ac.waitForServer(); //will wait for infinite time

		ROS_INFO("Action server started, sending goal.");
		// send a goal to the action
		ipa_pars_map_analyzer::ParsMapKnowledgeGoal knowledge_extractor_goal;
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
//		knowledge_extractor_goal.labels = list_of_labels;

		knowledge_ac.sendGoal(knowledge_extractor_goal);


		//wait for the action to return
		bool finished_before_timeout = knowledge_ac.waitForResult(ros::Duration(300.0));

		if (finished_before_timeout)
		{
			ROS_INFO("Finished successfully!");
			ipa_pars_map_analyzer::ParsMapKnowledgeResultConstPtr result_knowledge = knowledge_ac.getResult();
			// display
			ROS_INFO("The produced square information is:");
			std::vector<ipa_pars_map_analyzer::SquareInformation> sqr_info = result_knowledge->square_information;
			ROS_INFO_STREAM("The size of the given square information vector is" << sqr_info.size());

			// display with names and balancePoints
			displayMapAsImage(concatenated_image, room_colors, sqr_info);
			// sending to yaml dumper
			ipa_pars_map_analyzer::KnowledgeToYaml knowledge_srv;
			knowledge_srv.request.square_information = sqr_info;
//			ipa_pars_map_analyzer::KnowledgeToYamlResponse resp;
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






	ipa_pars_map_analyzer::ParsMapAnalyzerResult map_analyzer_action_result_;
	map_analyzer_action_result_.static_knowledge.data = "test_output";
	ipa_pars_map_analyzer_server_.setSucceeded(map_analyzer_action_result_);
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

	ROS_INFO("Created %d colors for room squares segmenation", (int)room_colors.size());
}

void ParsMapAnalyzerServer::displayMapAsImage(cv::Mat &map, std::vector<cv::Vec3b> &room_colors,  std::vector<ipa_pars_map_analyzer::SquareInformation> &sqr_info)
{
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
	ROS_INFO("--------------------------------------------------------");
	ROS_INFO_STREAM("THE LABELS.size() is "<< labels.size());
	ROS_INFO("--------------------------------------------------------");

	cv::Mat colored_map = map.clone();
	colored_map.convertTo(colored_map, CV_8U);
	cv::cvtColor(colored_map, colored_map, CV_GRAY2BGR);
	if (labels.size() < 2 && map.type() == 0 && sqr_info.size() == 0) // unlabeled map
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
	}
	else if (sqr_info.size() > 0)
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
			cv::putText(colored_map, sqr_numb , cv::Point( (int) ((sqr_info.at(c).center.x + 19.2) / 0.05 ) -10, (int) ((sqr_info.at(c).center.y+19.2) / 0.05)+10 ), CV_FONT_HERSHEY_SIMPLEX, 0.2f,
			        cv::Scalar(255, 255, 255), 0.1, 8, false);
			colored_map.at<cv::Vec3b>(((sqr_info.at(c).center.y + 19.2) / 0.05 ), ((sqr_info.at(c).center.x + 19.2) / 0.05 ))[0] = 255;
			colored_map.at<cv::Vec3b>(((sqr_info.at(c).center.y + 19.2) / 0.05 ), ((sqr_info.at(c).center.x + 19.2) / 0.05 ))[1] = 255;
			colored_map.at<cv::Vec3b>(((sqr_info.at(c).center.y + 19.2) / 0.05 ), ((sqr_info.at(c).center.x + 19.2) / 0.05 ))[2] = 51;
		}
		cv::imshow("map_with_square_info_as_image", colored_map);
	}
	else
	{
		ROS_ERROR("The map you try to display has a wrong format or to many rooms!");
		ROS_ERROR("Format should be of cv::type 0 or 4; amount of rooms possible 1-65");
	}


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ipa_pars_map_analyzer_server");

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

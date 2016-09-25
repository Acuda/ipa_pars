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

#include <ipa_pars_map_analyzer/ipa_pars_map_knowledge_extractor_server.h>
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
//#include <algorithm> // for std::sort
//#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa_pars_map_analyzer/ParsMapKnowledgeAction.h>

#include <ipa_pars_map_analyzer/SquareInformation.h>

// round
#include <sstream>
//#include <ipa_room_segmentation/meanshift2d.h>


ParsMapKnowledgeExtractorServer::ParsMapKnowledgeExtractorServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	ipa_pars_map_knowledge_extractor_server_(node_handle_, name_of_the_action, boost::bind(&ParsMapKnowledgeExtractorServer::execute_map_knowledge_extractor_server, this, _1), false)
{
	//Start action server
	ipa_pars_map_knowledge_extractor_server_.start();
}


void ParsMapKnowledgeExtractorServer::execute_map_knowledge_extractor_server(const ipa_pars_map_analyzer::ParsMapKnowledgeGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****ParsMapKnowledgeExtractor action server*****");
//	ROS_INFO("map resolution is : %f", goal->map_resolution);


	//converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::TYPE_32SC1);
	cv::Mat input_img = cv_ptr_obj->image;

	//set the resolution and the limits for the actual goal and the Map origin
	const float map_resolution = goal->map_resolution;
	const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);
	const float yaw = goal->map_origin.position.z;
	const float robot_radius = goal->robot_radius.data;

	std::vector<int> reallabelcount;
	for (int r = 0; r < input_img.rows; ++r)
	{
		for ( int c = 0; c < input_img.cols; ++c)
		{
			if (input_img.at<int>(r,c) != 0)
			{
				int label = input_img.at<int>(r,c);
				addElementNotInVec(reallabelcount, label);
			}
		}
	}

	// rotaton from lower-left corner to upper left corner (here) because of x,y in for loop.
	// origin is given from the lower-left corner of the map.
	// so x -> x ; y -> map.rows - y

	std::vector< std::vector<double> > balancePoints;
	std::vector<double> balancePointXY;
	for (int i = 0; i < reallabelcount.size(); i++)
	{
		int counterx = 0;
		int countery = 0;
		int weightx = 0;
		int weighty = 0;
		double xs = 0;
		double ys = 0;
		balancePointXY.clear();
		for (int y = 0; y < input_img.rows; y++)
		{
			for (int x = 0; x < input_img.cols; x++)
			{
				if (input_img.at<int>(y,x) == reallabelcount.at(i))
				{
					weightx += x;
					weighty += y;
					counterx++;
					countery++;
				}
			}
		}
		if (counterx > 0 && countery > 0)
		{
			xs = weightx / counterx;
			ys = weighty / countery;

		}
		balancePointXY.push_back(xs);
		balancePointXY.push_back(ys);
		balancePoints.push_back(balancePointXY);
	}

//	for (int i = 0; i < reallabelcount.size(); i++)
//	{
//		for (int r = 0; r < input_img.rows; ++r)
//		{
//			for (int c; c < input_img.cols; ++c)
//			{
//
//			}
//		}
//	}

//	MeanShift2D ms;
//	cv::Vec2d room_center = ms.findRoomCenter(input_img, room_cells, map_resolution);
	// get list of transitions:

	std::vector< std::vector<int> > vec_of_transitions;

	for (int i = 0; i < reallabelcount.size(); i++)
	{
		std::vector<int> transitions;
		std::vector<int> neighborcounter;
		transitions.push_back(reallabelcount.at(i));
		for (int y = 0; y < input_img.rows; y++)
		{
			for ( int x = 0; x< input_img.cols; x++)
			{
				if (input_img.at<int>(y,x) == reallabelcount.at(i))
				{
					if ((input_img.at<int>(y+1,x) != 0) && ( input_img.at<int>(y+1,x) != input_img.at<int>(y,x)))
					{
						neighborcounter.push_back((input_img.at<int>(y+1,x)));
					}
					else if ((input_img.at<int>(y-1,x) != 0) && ( input_img.at<int>(y-1,x) != input_img.at<int>(y,x)))
					{
						neighborcounter.push_back(input_img.at<int>(y-1,x));
					}
					else if ((input_img.at<int>(y,x+1) != 0) && ( input_img.at<int>(y,x+1) != input_img.at<int>(y,x)))
					{
						neighborcounter.push_back(input_img.at<int>(y,x+1));
					}
					else if ((input_img.at<int>(y,x-1) != 0) && ( input_img.at<int>(y,x-1) != input_img.at<int>(y,x)))
					{
						neighborcounter.push_back(input_img.at<int>(y,x-1));
					}
				}
			}
		}

		std::sort(neighborcounter.begin(), neighborcounter.end());
		std::vector<int>::iterator last = std::unique(neighborcounter.begin(), neighborcounter.end());
		neighborcounter.erase(last, neighborcounter.end());
		for (int t = 0; t < neighborcounter.size(); t++)
		{
			transitions.push_back(neighborcounter.at(t));

		}
		vec_of_transitions.push_back(transitions);
	}



	for (int t = 0; t < vec_of_transitions.size(); t++)
	{
		for (int l = 1; l < vec_of_transitions.at(t).size(); l++)
		{
//			ROS_INFO("Transitions of label %d found are %d", vec_of_transitions.at(t).at(0), vec_of_transitions.at(t).at(l) );
		}
	}

	std::vector<ipa_pars_map_analyzer::SquareInformation> vec_square_info;
	for (int t = 0; t < vec_of_transitions.size(); t++)
	{
		ipa_pars_map_analyzer::SquareInformation square_info;
		square_info.label.data = vec_of_transitions.at(t).at(0);
		for (int l = 1; l < vec_of_transitions.at(t).size(); l++)
		{
			std_msgs::Int32 trans;
			trans.data = vec_of_transitions.at(t).at(l);
			square_info.transitions.push_back(trans);
		}
		// map origin
		double converted_center_x = round ( 100 * (balancePoints.at(t).at(0) * goal->map_resolution - goal->map_origin.position.x)) / 100;
		double converted_center_y = round (100 * ((input_img.rows - balancePoints.at(t).at(1)) * goal->map_resolution - goal->map_origin.position.y)) / 100;
		square_info.center.x = converted_center_x;
		square_info.center.y = converted_center_y;
		square_info.center.z = 0.0; // now without rotation in lower left corner (0|0);
		vec_square_info.push_back(square_info);
	}





	// display
//	cv::Mat colour_extracted_map = input_img.clone();
//	colour_extracted_map.convertTo(colour_extracted_map, CV_8U);
//	cv::cvtColor(colour_extracted_map, colour_extracted_map, CV_GRAY2BGR);

//	cv::Mat color_img(input_img.rows, input_img.cols, CV_8U);
////	color_img.convertTo(color_img, color_img, CV_8U);
//	cv::cvtColor(color_img, color_img, CV_GRAY2BGR);

//	ROS_INFO_STREAM("realllabelcount is =" << reallabelcount.size());
//	for(int i = 0; i < reallabelcount.size(); ++i)
//	{
//	//				ROS_INFO("label = %u",i);
//		//choose random color for each room
//		int blue = (rand() % 250) + 1;
//		int green = (rand() % 250) + 1;
//		int red = (rand() % 250) + 1;
//		for(int u = 0; u < input_img.rows; ++u)
//		{
//			for(int v = 0; v < input_img.cols; ++v)
//			{
//	//						if(tesselated_map.at<int>(u,v) == result_tess->labels.data.at(i-1))
//				if(input_img.at<int>(u,v) == reallabelcount.at(i))
//				{
////					ROS_INFO("Coloring label = %u", i);
//					colour_extracted_map.at<cv::Vec3b>(u,v)[0] = blue;
//					colour_extracted_map.at<cv::Vec3b>(u,v)[1] = green;
//					colour_extracted_map.at<cv::Vec3b>(u,v)[2] = red;
//	//							labels.erase(std::remove(labels.begin(), labels.end(), i), labels.end());
//				}
//			}
//
//		}
//	//			ROS_INFO("Drawing balance point for label %u", i);
//	//			ROS_INFO("On location %u, %u", balancePoints.at(i).at(0), balancePoints.at(i).at(1));
//	//			ROS_INFO("On location %u, %u", balancePoints.at(5).at(0), balancePoints.at(5).at(1));
//		colour_extracted_map.at<cv::Vec3b>(balancePoints.at(i).at(1), balancePoints.at(i).at(0)) = cv::Vec3b(255,255,255);
//	//				cv::imshow("output", colour_tesselated_map);
//	//				cv::waitKey(1);
//	//				std::vector<int>& vec = myNumbers; // use shorter name
//
//	//				labels.erase(i);
//
//	}
//	ROS_INFO("Trying to show the image");
//	cv::imshow("ready", colour_extracted_map);
//	cv::waitKey();



















	// output
	ipa_pars_map_analyzer::ParsMapKnowledgeResult map_knowledge_extractor_action_result_;
	map_knowledge_extractor_action_result_.square_information = vec_square_info;
	ipa_pars_map_knowledge_extractor_server_.setSucceeded(map_knowledge_extractor_action_result_);

}

void ParsMapKnowledgeExtractorServer::addElementNotInVec(std::vector<int> &reallabelcount, int label)
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ipa_pars_map_knowledge_extractor_server");

	ros::NodeHandle nh;

	ParsMapKnowledgeExtractorServer knowledgeExtractorAlgorithmObj(nh, ros::this_node::getName());

	ROS_INFO("Action Server for map_knowledge_extractor has been initialized......");

	ros::spin();
	return 0;
}

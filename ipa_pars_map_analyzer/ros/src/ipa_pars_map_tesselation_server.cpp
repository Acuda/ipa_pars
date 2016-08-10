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
//#include <algorithm> // for std::sort
//#include <vector>

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
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
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

	 //delete errors
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
	corrected_img.convertTo(corrected_img, CV_32SC1, 256, 0);
	ParsMapTesselationServer::tesselate_map(corrected_img, tesselated_img, labelcount);

	//output_img.convertTo(tesselated_img, CV_32SC1, 256, 0);
	cv::Mat outputt_img;
//	tesselated_img.convertTo(outputt_img, CV_32SC1, 256, 0);
	//cv_img.image = output_img;
	cv_img.image = tesselated_img;
	//tesselated_img.convertTo(tesselated_img, CV_8U);
	//cv::cvtColor(tesselated_img, tesselated_img, CV_GRAY2BGR);
	//cv::imshow("tesselationbefore", tesselated_img);
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
	// estimate amount of colors needed
//	int amount = int (ceil(((map_to_tesselate.cols * map_to_tesselate.rows) / (20*20)))); // TODO: gridsize 20!
//    unsigned int maxcolors = 255*255*255;
//	unsigned int color = amount; // first color

	int label = 1;
	std::vector<int> vecOfColors;
	cv::Mat tesselated_only = map_to_tesselate.clone();
	tesselated_map = map_to_tesselate.clone();
	//  divide in rectangles:
	for (int y = 0; y < map_to_tesselate.rows-20; y += 20)
	{
		for (int x = 0; x < map_to_tesselate.cols-20; x += 20)
		{
//			ROS_INFO("I paint a rectangle with the label %u", label);
			// for every cell (rectangle)
			for (unsigned int u = 0; u < 20; u++)
			{
				for (unsigned int v = 0; v < 20; v++)
				{
					tesselated_only.at<int>(y+u,x+v) = label;
				}
			}
//			unsigned char blue   = (color & 0xff0000) >> 16;
//			unsigned char green = (color & 0x00ff00) >> 8;
//			unsigned char red  = (color & 0x0000ff);
//			ROS_INFO("new rectangle printed %u %u %u", blue, green, red);
//			cv::rectangle(tesselated_only, cv::Point(x,y),cv::Point(x+20,y+20),cv::Scalar(blue,green,red), -1);
//			vecOfColors.push_back(label);
			label++;
//			color += amount;
//			if (color > (maxcolors - amount))
//			{
//				break;
//			}
		}
	}
	// just for debug!
//	ROS_INFO("channels of image is = %u", tesselated_map.channels());

//	labelcount = vecOfColors;
//	ROS_INFO_STREAM("Labelcount before is = " << labelcount.size());
	std::vector <int> reallabelcount;
	for (int y = 0; y < map_to_tesselate.rows; y++)
	{
		for ( int x = 0; x< map_to_tesselate.cols; x++)
		{
			if (map_to_tesselate.at<int>(y,x) != 0)
			{
//				counter++;
				int label = tesselated_only.at<int>(y,x);
//				ROS_INFO("label = %u", label);
				addElementNotInVec(reallabelcount, label);
//				tesselated_map.at<int>(y,x) = tesselated_only.at<int>(y,x);
				tesselated_map.at<int>(y,x) = label;
			}
//			else
//			{
//				tesselated_map.at<int>(y,x) = 0;
//			}
		}
	}

	std::vector<int> new_labels;
	int counter = 1;
	for (int i = 0; i < reallabelcount.size() ; ++i)
	{
		new_labels.push_back(counter);
		counter++;
	}

	for (int i = 0; i < new_labels.size(); i++)
	{
		for (int y = 0; y < map_to_tesselate.rows; y++)
		{
			for ( int x = 0; x< map_to_tesselate.cols; x++)
			{
				if (tesselated_map.at<int>(y,x) == reallabelcount.at(i))
				{
					tesselated_map.at<int>(y,x) = new_labels.at(i);
				}
			}
		}

	}

	labelcount = new_labels;
	// count ares and neighbours
	for (int i = 0; i < new_labels.size(); i++)
	{
//		ROS_INFO_STREAM("I am checking label = " << new_labels.at(i));
		int pixelcounter = 0;
		// calculate area size
		for (int y = 0; y < tesselated_map.rows; y++)
		{
			for ( int x = 0; x< tesselated_map.cols; x++)
			{
//				ROS_INFO("tesselated_map.at<int>(y,x) = %u",tesselated_map.at<int>(y,x));
//				ROS_INFO("reallabelcount.at(i) = %u", new_labels.at(i));
				if (tesselated_map.at<int>(y,x) == new_labels.at(i))
				{

//					ROS_INFO("Pixelcounter = %u", pixelcounter);
					pixelcounter++;
				}
			}
		}



		// if area size is small than half of a normal rectangle:
		if (pixelcounter < 200) // should be 200
		{
//			ROS_INFO("Found a area smaller than 200 pixels");
//			ROS_INFO_STREAM("The area is label = "<< new_labels.at(i));
			std::vector<int> neighborcounter;
			for (int y = 0; y < tesselated_map.rows; y++)
			{
				for ( int x = 0; x< tesselated_map.cols; x++)
				{
					if (tesselated_map.at<int>(y,x) == new_labels.at(i))
					{
						if ((tesselated_map.at<int>(y+1,x) != 0) && ( tesselated_map.at<int>(y+1,x) != tesselated_map.at<int>(y,x)))
						{
							neighborcounter.push_back((tesselated_map.at<int>(y+1,x)));
						}
						else if ((tesselated_map.at<int>(y-1,x) != 0) && ( tesselated_map.at<int>(y-1,x) != tesselated_map.at<int>(y,x)))
						{
							neighborcounter.push_back(tesselated_map.at<int>(y-1,x));
						}
						else if ((tesselated_map.at<int>(y,x+1) != 0) && ( tesselated_map.at<int>(y,x+1) != tesselated_map.at<int>(y,x)))
						{
							neighborcounter.push_back(tesselated_map.at<int>(y,x+1));
						}
						else if ((tesselated_map.at<int>(y,x-1) != 0) && ( tesselated_map.at<int>(y,x-1) != tesselated_map.at<int>(y,x)))
						{
							neighborcounter.push_back(tesselated_map.at<int>(y,x-1));
						}
					}
				}
			}

			int max = 0;
			int most_common = -1;
			std::map<int,int> m;
			for (std::vector<int>::iterator vi = neighborcounter.begin(); vi != neighborcounter.end(); vi++) {
			  m[*vi]++;
			  if (m[*vi] > max) {
			    max = m[*vi];
			    most_common = *vi;
			  }
			}

			if (most_common != -1)
			{
				for (int y = 0; y < tesselated_map.rows; y++)
				{
					for ( int x = 0; x< tesselated_map.cols; x++)
					{
						if (tesselated_map.at<int>(y,x) == new_labels.at(i))
						{
							tesselated_map.at<int>(y,x) = most_common;
						}

					}
				}
//				ROS_INFO_STREAM("Painted it in ="<< most_common);
			}
			neighborcounter.clear();
		}
	}

	std::vector <int> endlabelcount;
	for (int y = 0; y < tesselated_map.rows; y++)
	{
		for ( int x = 0; x< tesselated_map.cols; x++)
		{
			if (tesselated_map.at<int>(y,x) != 0)
			{
//				counter++;
				int label = tesselated_map.at<int>(y,x);
//				ROS_INFO("label = %u", label);
				addElementNotInVec(endlabelcount, label);
//				tesselated_map.at<int>(y,x) = tesselated_only.at<int>(y,x);
//				tesselated_map.at<int>(y,x) = label;
			}
//			else
//			{
//				tesselated_map.at<int>(y,x) = 0;
//			}
		}
	}

	std::vector<int> end_labels;
	int labelcounter = 1;
	for (int i = 0; i < endlabelcount.size() ; ++i)
	{
		end_labels.push_back(labelcounter);
		labelcounter++;
	}

	for (int i = 0; i < end_labels.size(); i++)
	{
		for (int y = 0; y < map_to_tesselate.rows; y++)
		{
			for ( int x = 0; x< map_to_tesselate.cols; x++)
			{
				if (tesselated_map.at<int>(y,x) == endlabelcount.at(i))
				{
					tesselated_map.at<int>(y,x) = end_labels.at(i);
				}
			}
		}

	}

	labelcount = end_labels;
	// for debug
	for (int i=0 ; i < labelcount.size(); i++)
	{
//		ROS_INFO("these labels are here: %u", labelcount.at(i));
	}
//	ROS_INFO("counter is = %u", counter);
//	labelcount = vecOfColors;
//	labelcount = reallabelcount;
//	ROS_INFO_STREAM("reallabelcount after = " << labelcount.size());
//	tesselated_map = tesselated_only;








}

void ParsMapTesselationServer::addElementNotInVec(std::vector<int> &reallabelcount, int label)
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
	ros::init(argc, argv, "ipa_pars_map_analyzer_server");

	ros::NodeHandle nh;

	ParsMapTesselationServer tesselationAlgorithmObj(nh, ros::this_node::getName());
	ROS_INFO("Action Server for map_tesselation has been initialized......");
	ros::spin();

	return 0;
}

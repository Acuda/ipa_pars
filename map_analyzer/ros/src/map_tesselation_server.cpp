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

#include <map_analyzer/map_tesselation_server.h>
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
#include <map_analyzer/ParsMapTesselationAction.h>

#include "std_msgs/Int32MultiArray.h"

#include <sstream>



ParsMapTesselationServer::ParsMapTesselationServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	map_tesselation_server_(node_handle_, name_of_the_action, boost::bind(&ParsMapTesselationServer::execute_map_tesselation_server, this, _1), false)
{
	//Start action server
	map_tesselation_server_.start();
}


void ParsMapTesselationServer::execute_map_tesselation_server(const map_analyzer::ParsMapTesselationGoalConstPtr &goal)
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
	map_analyzer::ParsMapTesselationResult map_tesselation_action_result_;
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
	map_tesselation_server_.setSucceeded(map_tesselation_action_result_);

}



void ParsMapTesselationServer::tesselate_map(const cv::Mat& map_to_tesselate, cv::Mat& tesselated_map, std::vector<int>& labelcount)
{
	// estimate amount of colors needed
//	int amount = int (ceil(((map_to_tesselate.cols * map_to_tesselate.rows) / (20*20)))); // TODO: gridsize 20!
//    unsigned int maxcolors = 255*255*255;
//	unsigned int color = amount; // first color

//	int label = 1;
//	std::vector<int> vecOfColors;
	cv::Mat tesselated_only = map_to_tesselate.clone();
//	tesselated_map = map_to_tesselate.clone();
//	//  divide in rectangles:
//	for (int y = 0; y < map_to_tesselate.rows-20; y += 20)
//	{
//		for (int x = 0; x < map_to_tesselate.cols-20; x += 20)
//		{
////			ROS_INFO("I paint a rectangle with the label %u", label);
//			// for every cell (rectangle)
//			for (unsigned int u = 0; u < 20; u++)
//			{
//				for (unsigned int v = 0; v < 20; v++)
//				{
//					tesselated_only.at<int>(y+u,x+v) = label;
//				}
//			}
////			unsigned char blue   = (color & 0xff0000) >> 16;
////			unsigned char green = (color & 0x00ff00) >> 8;
////			unsigned char red  = (color & 0x0000ff);
////			ROS_INFO("new rectangle printed %u %u %u", blue, green, red);
////			cv::rectangle(tesselated_only, cv::Point(x,y),cv::Point(x+20,y+20),cv::Scalar(blue,green,red), -1);
////			vecOfColors.push_back(label);
//			label++;
////			color += amount;
////			if (color > (maxcolors - amount))
////			{
////				break;
////			}
//		}
//	}
//	// just for debug!
////	ROS_INFO("channels of image is = %u", tesselated_map.channels());
//
//	labelcount = vecOfColors;
//	ROS_INFO_STREAM("Labelcount before is = " << labelcount.size());
//	std::vector <int> reallabelcount;
//	for (int y = 0; y < map_to_tesselate.rows; y++)
//	{
//		for ( int x = 0; x< map_to_tesselate.cols; x++)
//		{
//			if (map_to_tesselate.at<int>(y,x) != 0)
//			{
////				counter++;
//				int newlabel = tesselated_only.at<int>(y,x);
////				int labelleft = map_to_tesselate.at<int>(y,x-1); //
////				int labelabove = map_to_tesselate.at<int>(y-1,x); //
////				ROS_INFO("label = %u", label);
//				addElementNotInVec(reallabelcount, newlabel);
////				tesselated_map.at<int>(y,x) = tesselated_only.at<int>(y,x);
//				tesselated_map.at<int>(y,x) = newlabel;
//			}
////			else
////			{
////				tesselated_map.at<int>(y,x) = 0;
////			}
//		}
//	}
//	ROS_INFO("Image rows x cols = %u x %u", tesselated_only.rows, tesselated_only.cols);
		int grid_size = 20;
//		ROS_INFO("grid_size = %u", grid_size);
		int global_label_count = 0;

	//	for (int r = 0; r < erode_map.rows; r+=grid_size)
	//	{
	//		for (int c = 0; c < erode_map.cols; c+=grid_size)
	//		{

		for (int r = 0; r < tesselated_only.rows; r+=grid_size)
			{
				for (int c = 0; c < tesselated_only.cols; c+=grid_size)
				{
				int width = grid_size;
				int height = grid_size;

				if (c+grid_size > tesselated_only.cols)
				{
					width = tesselated_only.cols - c;
				}
				if (r+grid_size > tesselated_only.rows)
				{
					height = tesselated_only.rows - r;
				}
				int label = global_label_count;
				// select square as region of interest (roi)
	//			ROS_INFO("ROI is Rect (%u,%u,%u,%u)",c,r,width,height);
				cv::Rect roi_square = cv::Rect(c,r,width,height);
				cv::Mat map_roi = tesselated_only(roi_square);
	//			ROS_INFO("ROI size is rows %u x cols %u ", map_roi.rows, map_roi.cols);

				cv::Mat map_roi_borders = map_roi.clone();
				global_label_count++;

	//			cv::imshow("map_roi_bordersbefore",map_roi_borders);
				//make black border with offset around roi
				int bordercolour = 65534;
				int xoffset = 10;
				int yoffset = 10;
				cv::Mat map_roi_offset(map_roi_borders.rows+2*xoffset, map_roi_borders.cols+2*yoffset, map_roi_borders.type(), bordercolour);
				cv::Mat roi_tmp(map_roi_offset(cvRect(xoffset, yoffset,map_roi_borders.cols, map_roi_borders.rows)));
				map_roi_borders.copyTo(roi_tmp);
				map_roi_borders = map_roi_offset.clone();

	//			cv::copyMakeBorder( map_roi, map_roi_copy, 2, 2, 2, 2, cv::BORDER_CONSTANT, 0 );

	//			cv::imshow("map_roi_borders", map_roi_borders);
	//			cv::waitKey(500);
	//			label++;
	//			cv::Mat mask;
	//			cv::inRange(map_roi, 65535, 65535, mask);
	//			map_roi.setTo(label, mask);
	//		    cv::imshow("roi_with_offset", map_roi);
	//		    cv::waitKey(1);

	//			for (int x = 0; x < map_roi.rows; x++)
	//			{
	//				for (int y = 0; y < map_roi.cols; y++)
	//				{
	//					ROS_INFO("working on row %u, col %u", x, y);
	//					// todo warum falsch rum?
	//					map_roi.at<int>(x,y) = label;
	//				}
	//			}

	//			int label = 0;
				std::map<int, std::vector<int> > lookup_label_map;
				std::map<int, std::vector<int> >::iterator it;

				for (int x = xoffset; x < (map_roi_borders.rows-xoffset); x++)
				{
	//				ROS_INFO("label is = %u", label);
	//				label++;
					for (int y = yoffset; y < (map_roi_borders.cols-yoffset); y++)
					{
	//					// label left
						if (map_roi_borders.at<int>(x,y-1) == 0 || map_roi_borders.at<int>(x,y-1) == 65534 || map_roi_borders.at<int>(x,y-1) == 65535)
						{
							label++;
						}
	//					ROS_INFO("Trying to change a pixel in map_roi_borders at cols %u, rows %u", y,x);
	//					ROS_INFO("Which is at position col %u, row %u in map_roi", y-yoffset,x-xoffset);
	//					ROS_INFO("label is = %u", label);
						if (map_roi_borders.at<int>(x,y) != 0 && map_roi_borders.at<int>(x,y) != 65534)
						{
	//						ROS_INFO("Trying to change a pixel in map_roi at %u %u", y-yoffset,x-xoffset);
							map_roi_borders.at<int>(x,y) = label;

							//label above
							if (map_roi_borders.at<int>(x-1,y) != 0 && map_roi_borders.at<int>(x-1,y) < label)
							{
	//							ROS_INFO("The label above (%u) is smaller then this label %u", map_roi_borders.at<int>(x-1,y), label);
								std::vector<int> label_expression;
	//							map_roi_borders.at<int>(x,y) = map_roi_borders.at<int>(x,y-1);
								// if label doesnt exist in map

								it = lookup_label_map.find(label);
								if (it != lookup_label_map.end())
								{
	//								ROS_INFO("They key_label %u is already in the map", label);
									// key is in the map, add label to vector
									label_expression = lookup_label_map.at(label);
									if (std::find(label_expression.begin(), label_expression.end(), map_roi_borders.at<int> (x-1,y)) != label_expression.end())
									{

									}
									else
									{
	//									ROS_INFO("Adding new label to list = %u", map_roi_borders.at<int> (x-1,y));
										label_expression.push_back(map_roi_borders.at<int> (x-1,y));
										lookup_label_map.at(label) = label_expression;
									}

								}
								else
								{
									// key is not in the map
	//								ROS_INFO("The key is not in the map: %u adding it now", label);
									label_expression.push_back(map_roi_borders.at<int> (x-1,y));
									lookup_label_map.insert ( std::pair<int,std::vector<int> >( label, label_expression ));
								}
							}
						}
					}
				}

				int oldlabel_key = 25;
				int newlabel = 1;
				while( !lookup_label_map.empty())
				{
					// get highest keyvalue (label)
					oldlabel_key = lookup_label_map.rbegin()->first;
					std::vector<int> newkey_vec = lookup_label_map[oldlabel_key];
	//				ROS_INFO("Working on map_key %u = ",oldlabel_key);
					newlabel = newkey_vec.at(0);
					cv::Mat mask;
					cv::inRange(map_roi_borders, oldlabel_key, oldlabel_key, mask);
					map_roi_borders.setTo(newlabel, mask);
					if (newkey_vec.size() > 1)
					{
	//					ROS_INFO("this message should be there only twice!");
						for (int i = 1; i < newkey_vec.size(); i++)
						{
							cv::Mat mask;
							cv::inRange(map_roi_borders, newkey_vec.at(i), newkey_vec.at(i), mask);
							map_roi_borders.setTo(newlabel, mask);
							std::vector<int> new_vec_pointer;
							std::vector<int> label_vec;
							// point to new label!
							if (lookup_label_map.find(newkey_vec.at(i)) == lookup_label_map.end())
							{
								//not found
								break;
							}
							else
							{
								new_vec_pointer = lookup_label_map[newkey_vec.at(i)];
							}

							if (lookup_label_map.find(newlabel) == lookup_label_map.end())
							{
								//not found
								break;
							}
							else
							{
								label_vec = lookup_label_map[newlabel];
							}

							// get vec to change
							//transfer all elements to new key_label
							if (new_vec_pointer.size() > 0)
							{
	//							ROS_INFO_STREAM("new_vec pointer size = " << new_vec_pointer.size());
	//							ROS_INFO("new vec pointer.at(0) = %u", new_vec_pointer.at(0));
								for (int p = 0; p < new_vec_pointer.size(); p++)
								{
	//								label_vec.push_back(new_vec_pointer.at(p));
									if (std::find(label_vec.begin(), label_vec.end(), new_vec_pointer.at(p)) != label_vec.end())
									{

									}
									else
									{
										label_vec.push_back(new_vec_pointer.at(p));
									}

								}
	//							ROS_INFO(" changed point from %u --> %u", label_vec.at(0), newlabel);
								lookup_label_map[newlabel] = label_vec;

	//							ROS_INFO("changed pointer from %u --> %u", new_vec_pointer.at(p), newlabel);
							}

						}
					}


					lookup_label_map.erase(oldlabel_key);
	//				newlabel = lookup_label_map[oldlabel_key];
	//				 todo als multimap oder als map aus int und vector of int!
	//				 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*

				}


				// find list of labels:
				std::vector<int> list_of_labels;
				for (int r = 0; r<map_roi_borders.rows; r++)
				{
					for (int c = 0; c<map_roi_borders.cols; c++)
					{
						int label = map_roi_borders.at<int>(r,c);
						if (std::find(list_of_labels.begin(), list_of_labels.end(), label) != list_of_labels.end())
						{

						}
						else
						{
							list_of_labels.push_back(label);
						}
					}
				}

				std::map<int,int> renew_labels;
				for (int i = 0; i < list_of_labels.size(); i++)
				{
					int lab = list_of_labels.at(i);
					if (lab != 0 && lab != 65535 && lab != 65534)
					{
						renew_labels.insert( std::pair<int,int> ( lab, global_label_count));
						global_label_count++;

					}
				}

				// change labels to global counter:
				while( !renew_labels.empty())
				{
					int first_key = renew_labels.begin()->first;
					cv::Mat mask;
					cv::inRange(map_roi_borders, first_key, first_key, mask);
					map_roi_borders.setTo(renew_labels[first_key], mask);
					renew_labels.erase(first_key);
				}
	//			// get highest keyvalue (label)
	//			ROS_INFO("Flag 2");
	//			oldlabel = lookup_label_map.rbegin()->first;
	//			ROS_INFO("Flag 3");
	//			newlabel = lookup_label_map[oldlabel];
	//			ROS_INFO("Flag 4");
	//			cv::Mat mask;
	//			ROS_INFO("Flag 5");
	//			cv::inRange(map_roi_borders, oldlabel, oldlabel, mask);
	//			ROS_INFO("Flag 6");
	//			map_roi_borders.setTo(newlabel, mask);
	//			ROS_INFO("Flag 7");
	//			lookup_label_map.erase(oldlabel);
	//			ROS_INFO("replaced %u --> %u", oldlabel, newlabel);



				// roi back:
				cv::Rect roi_square_back = cv::Rect(xoffset,yoffset,width,height);
				cv::Mat roi_back = map_roi_borders(roi_square_back);
	//			cv::imshow("roi_bacK",roi_back);
				roi_back.copyTo(map_roi);

	//			ROS_INFO("content of lookup_label_map = ");
	//			for (const auto &p : lookup_label_map) {
	//			    std::cout << "m[" << p.first << "] = " << p.second << '\n';
	//			}
	//			cv::imshow("roi_with_offset", map_roi);
	//			cv::waitKey();
			}
		}

//		ROS_INFO("Flag output created!");
		cv::Mat output = tesselated_only.clone();

	//for debug:
	std::vector <int> reallabelcount;


	for (int r = 0; r < output.rows; r++)
	{
		for (int c = 0; c < output.cols; c++)
		{
			if (output.at<int>(r,c) != 0)
			{
				addElementNotInVec(reallabelcount, output.at<int>(r,c));
			}
		}
	}
//	ROS_INFO_STREAM("the reallabelcount = "<< reallabelcount.size());

	std::vector<int> new_labels;
	int counter = 1;
	for (int i = 0; i < reallabelcount.size() ; ++i)
	{
		new_labels.push_back(counter);
		counter++;
	}

	for (int i = 0; i < new_labels.size(); i++)
	{
		for (int y = 0; y < output.rows; y++)
		{
			for ( int x = 0; x< output.cols; x++)
			{
				if (output.at<int>(y,x) == reallabelcount.at(i))
				{
					output.at<int>(y,x) = new_labels.at(i);
				}
			}
		}

	}

//	ROS_INFO("label renew was successfull");

	labelcount = new_labels;
	// count ares and neighbours
	for (int i = 0; i < new_labels.size(); i++)
	{
//		ROS_INFO_STREAM("I am checking label = " << new_labels.at(i));
		int pixelcounter = 0;
		// calculate area size
		for (int y = 0; y < output.rows; y++)
		{
			for ( int x = 0; x< output.cols; x++)
			{
//				ROS_INFO("tesselated_map.at<int>(y,x) = %u",tesselated_map.at<int>(y,x));
//				ROS_INFO("reallabelcount.at(i) = %u", new_labels.at(i));
				if (output.at<int>(y,x) == new_labels.at(i))
				{

//					ROS_INFO("Pixelcounter = %u", pixelcounter);
					pixelcounter++;
				}
			}
		}

//		ROS_INFO("pixelcounting was successfull");


		// if area size is small than half of a normal rectangle:
		if (pixelcounter < 200) // should be 200
		{
//			ROS_INFO("Found a area smaller than 200 pixels");
//			ROS_INFO_STREAM("The area is label = "<< new_labels.at(i));
			std::vector<int> neighborcounter;
			for (int y = 0; y < output.rows; y++)
			{
				for ( int x = 0; x< output.cols; x++)
				{
					if (output.at<int>(y,x) == new_labels.at(i))
					{
						if ((output.at<int>(y+1,x) != 0) && ( output.at<int>(y+1,x) != output.at<int>(y,x)))
						{
							neighborcounter.push_back((output.at<int>(y+1,x)));
						}
						else if ((output.at<int>(y-1,x) != 0) && ( output.at<int>(y-1,x) != output.at<int>(y,x)))
						{
							neighborcounter.push_back(output.at<int>(y-1,x));
						}
						else if ((output.at<int>(y,x+1) != 0) && ( output.at<int>(y,x+1) != output.at<int>(y,x)))
						{
							neighborcounter.push_back(output.at<int>(y,x+1));
						}
						else if ((output.at<int>(y,x-1) != 0) && ( output.at<int>(y,x-1) != output.at<int>(y,x)))
						{
							neighborcounter.push_back(output.at<int>(y,x-1));
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
				for (int y = 0; y < output.rows; y++)
				{
					for ( int x = 0; x< output.cols; x++)
					{
						if (output.at<int>(y,x) == new_labels.at(i))
						{
							output.at<int>(y,x) = most_common;
						}

					}
				}
//				ROS_INFO_STREAM("Painted it in ="<< most_common);
			}
			neighborcounter.clear();
		}
	}

	std::vector <int> endlabelcount;
	for (int y = 0; y < output.rows; y++)
	{
		for ( int x = 0; x< output.cols; x++)
		{
			if (output.at<int>(y,x) != 0)
			{
//				counter++;
				int label = output.at<int>(y,x);
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
		for (int y = 0; y < output.rows; y++)
		{
			for ( int x = 0; x< output.cols; x++)
			{
				if (output.at<int>(y,x) == endlabelcount.at(i))
				{
					output.at<int>(y,x) = end_labels.at(i);
				}
			}
		}

	}

	labelcount = end_labels;
	// for debug
//	for (int i=0 ; i < labelcount.size(); i++)
//	{
////		ROS_INFO("these labels are here: %u", labelcount.at(i));
//	}
//	ROS_INFO("counter is = %u", counter);
//	labelcount = vecOfColors;
//	labelcount = reallabelcount;
//	ROS_INFO_STREAM("reallabelcount after = " << labelcount.size());
	tesselated_map = output;

//	// for debug
//	for (int i = 0; i < end_labels.size(); i++)
//	{
//		int pix_counter = 0;
//		for (int y = 0; y < tesselated_map.rows; y++)
//		{
//			for (int x = 0; x < tesselated_map.cols; x++)
//			{
//				if (tesselated_map.at<int>(y,x) == end_labels.at(i))
//				{
//					pix_counter++;
//				}
//			}
//		}
//		ROS_INFO("The label %u has an amount of %u pixels", end_labels.at(i), pix_counter);
//	}








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
	ros::init(argc, argv, "map_analyzer_server");

	ros::NodeHandle nh;

	ParsMapTesselationServer tesselationAlgorithmObj(nh, ros::this_node::getName());
	ROS_INFO("Action Server for map_tesselation has been initialized......");
	ros::spin();

	return 0;
}

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
 *   This class provides a map analysis tool for the IPA planning and
	 reasoning system.
 *
 ****************************************************************/
#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>

#include <ipa_pars_map_analyzer/ParsMapAnalyzerAction.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ipa_pars_cv_test_file");
	std::string image_filename = ros::package::getPath("ipa_pars_map_analyzer") + "/common/files/test_maps/lab_ipa4.png";
	cv::Mat map = cv::imread(image_filename.c_str(), 0);
	cv::Mat new_map = map.clone();
	cv::Mat colour_map = map.clone();
	cv::cvtColor(new_map, new_map, CV_GRAY2BGR);
	cv::cvtColor(colour_map, colour_map, CV_GRAY2BGR);

	for (int y = 0; y < map.rows; y++)
	{
		for (int x = 0; x < map.cols; x++)
		{
			//find not reachable regions and make them black
			if (map.at<unsigned char>(y, x) < 250)
			{
				map.at<unsigned char>(y, x) = 0;
			}
			//else make it white
			else
			{
				map.at<unsigned char>(y, x) = 255;
			}
		}
	}
	ROS_INFO("image channels= %u", map.channels());
	//make non-white pixels black

//	for (int y = 0; y < map.rows-20; y += 20)
//	{
//		for (int x = 0; x < map.cols-20; x += 20)
//		{
//			cv::rectangle(new_map,cv::Point(x,y),cv::Point(x+20,y+20),cv::Scalar(((rand() % 250) + 1),((rand() % 250) + 1), ((rand() % 250) + 1)), -1);
//		}
//	}
//	int b,g,r;
//	for (int y = 0; y < map.rows; y++)
//	{
//		for (int x = 0; x < map.cols; x++)
//		{
////
//			if (map.at<unsigned char>(y,x) != 0)
//			{
//				b = new_map.at<cv::Vec3b>(y,x)[0];
//				g = new_map.at<cv::Vec3b>(y,x)[1];
//				r = new_map.at<cv::Vec3b>(y,x)[2];
//				colour_map.at<cv::Vec3b>(y,x) = cv::Vec3b(b,g,r);
//			}
//		}
//	}

	double map_resolution = 0.05;
	double origin_x = 19.2;
	double origin_y = 19.2;
	double yaw = 1.57;


	// drawArrow:
	double length = 2 / map_resolution; //40 px in general
	double arrowLength = 0.1 * length;
	double arrowAngle = 0.52; // 30 degrees as arrow angle;
	double input_x = origin_x / map_resolution;
	double input_y = colour_map.rows - (origin_y / map_resolution);
	cv::line(colour_map, cv::Point(input_x,input_y), cv::Point(input_x+length, input_y),cv::Scalar(255,0,0),1,CV_AA,0);
	ROS_INFO("input_y = %f", input_y);
	ROS_INFO("arrowLength = %f", arrowLength);
	ROS_INFO("sin(yaw) = %f", sin(yaw));
	ROS_INFO("cos(yaw) = %f", cos(yaw));
	ROS_INFO("my calculation = %f ", input_y + arrowLength * (tan(yaw)));
	cv::line(colour_map, cv::Point(input_x+ length,input_y), cv::Point(input_x+length - arrowLength, input_y + arrowLength * (tan(yaw))),cv::Scalar(255,0,0),1,CV_AA,0);
//	cv::line(colour_map, cv::Point(input_x+length,input_y), cv::Point((input_x + 0.1 * length) * cos(yaw) , (input_y + 0.1 * length) * -1 *  sin(yaw) ),cv::Scalar(255,0,0),1,CV_AA,0);
//	cv::line(colored_map, cv::Point(x_origin_in_px,y_origin_in_px), cv::Point(x_origin_in_px * cos(-yaw) + (y_origin_in_px - (2/map_resolution)) * sin(-yaw), -x_origin_in_px * sin(-yaw) + (y_origin_in_px - (2/map_resolution)) * cos(-yaw)), cv::Scalar(255,0,0),1,CV_AA,0);

//	cv::imshow("testoutput_map", new_map);
	cv::imshow("testoutput_color", colour_map);
	cv::waitKey();
	//exit
	return 0;
}

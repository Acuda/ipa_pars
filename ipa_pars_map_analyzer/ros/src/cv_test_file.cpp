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
	cv::Mat new_map = cv::Mat::zeros( map.size() , map.type() );
	cv::Mat colour_map = cv::Mat::zeros( map.size() , map.type() );
	cv::cvtColor(colour_map, colour_map, CV_GRAY2BGR);

	//make non-white pixels black
	int label = 1;
	int countery = 0;
	int counterx = 0;
	int thickness = 10;
	int linetype = 8;
	int blue = (rand() % 250) + 1;
	int green = (rand() % 250) + 1;
	int red = (rand() % 250) + 1;
	for (int y = 0; y < new_map.rows-10; y = y + 20)
	{
		for (int x = 0; x < new_map.cols-10; x + 20)
		{
			cv::line(colour_map,cv::Point(y,x),cv::Point(y,x+30),cv::Scalar(((rand() % 250) + 1),((rand() % 250) + 1), ((rand() % 250) + 1)), thickness, linetype, 0);

			new_map.at<int>(y,x) = label;
//			colour_map.at<cv::Vec3b>(y,x)[0] = blue;
//			colour_map.at<cv::Vec3b>(y,x)[1] = green;
//			colour_map.at<cv::Vec3b>(y,x)[2] = red;
		}
		//counterx = 0;
	}

	cv::imshow("testoutput_map", new_map);
	cv::imshow("testoutput_color", colour_map);
	cv::waitKey();
	//exit
	return 0;
}

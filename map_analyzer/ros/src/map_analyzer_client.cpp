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
 *   ROS package name: map_analyzer
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

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <map_analyzer/ParsMapAnalyzerAction.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_analyzer_client");
	ros::NodeHandle nh_;
	std::string map_name;
	double map_resolution;
	double robot_radius;
	std::vector<double> map_origin;
	nh_.getParam("map_name", map_name);
	nh_.getParam("map_resolution", map_resolution);
	nh_.getParam("robot_radius", robot_radius);
	nh_.getParam("map_origin", map_origin);
	std::string image_filename = ros::package::getPath("map_analyzer") + "/common/files/test_maps/" + map_name;
	cv::Mat map = cv::imread(image_filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	sensor_msgs::Image output_img;
	cv_bridge::CvImage cv_image;
	ROS_INFO("img.type() = %u", map.type());
	ROS_INFO("img.channels() = %u",map.channels());
	//	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "mono8";
	cv_image.image = map;
	cv_image.toImageMsg(output_img);
	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<map_analyzer::ParsMapAnalyzerAction> ac("map_analyzer_server", true);
	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time
	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	map_analyzer::ParsMapAnalyzerGoal goal;
	goal.input_map = output_img;

	// origin : The 2-D pose of the lower-left pixel in the map, as (x, y, yaw)
	// with yaw as counterclockwise rotation (yaw=0 means no rotation).
	goal.map_origin.position.x = map_origin.at(0);
	goal.map_origin.position.y = map_origin.at(1);
	goal.map_origin.position.z = map_origin.at(2);
	goal.map_resolution = map_resolution;
	goal.robot_radius.data = robot_radius;
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration());
	if (finished_before_timeout)
	{
		ROS_INFO("Finished successfully!");
		map_analyzer::ParsMapAnalyzerResultConstPtr result_knowledge = ac.getResult();
		// display
		ROS_INFO("%s", result_knowledge->static_knowledge.data.c_str());
	}

	//exit
	return 0;
}

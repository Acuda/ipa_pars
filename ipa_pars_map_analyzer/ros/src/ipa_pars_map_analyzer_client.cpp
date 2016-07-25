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

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <ipa_pars_map_analyzer/ParsMapAnalyzerAction.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ipa_pars_map_analyzer_client");
	std::string image_filename = ros::package::getPath("ipa_pars_map_analyzer") + "/common/files/test_maps/lab_ipa4.png";
	cv::Mat map = cv::imread(image_filename.c_str(), 0);
	sensor_msgs::Image output_img;
	cv_bridge::CvImage cv_image;
	//	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "mono8";
	cv_image.image = map;
	cv_image.toImageMsg(output_img);
	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<ipa_pars_map_analyzer::ParsMapAnalyzerAction> ac("ipa_pars_map_analyzer_server", true);
	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time
	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	ipa_pars_map_analyzer::ParsMapAnalyzerGoal goal;
	goal.input_map = output_img;
	goal.map_origin.position.x = 0;
	goal.map_origin.position.y = 0;
	goal.map_resolution = 0.05;
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration());
	if (finished_before_timeout)
	{
		ROS_INFO("Finished successfully!");
		ipa_pars_map_analyzer::ParsMapAnalyzerResultConstPtr result_knowledge = ac.getResult();
		// display
		ROS_INFO("%s", result_knowledge->static_knowledge.data.c_str());
	}

	//exit
	return 0;
}

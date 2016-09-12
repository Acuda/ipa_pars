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
	cv::Mat original_img = map.clone();
	cv::Mat new_map = map.clone();

	cv::cvtColor(new_map, new_map, CV_GRAY2BGR);


	//make non-white pixels black
	for (int y = 0; y < map.rows; y++)
	{
		for (int x = 0; x < map.cols; x++)
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
	int erosion_type = cv::MORPH_RECT;
	int erosion_size = 1;
	cv::Mat element = getStructuringElement( erosion_type,
										   cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
										   cv::Point( erosion_size, erosion_size ) );
	cv::erode(original_img,new_map,element);

	ROS_INFO("image channels= %u", new_map.channels());
	cv::Mat new_map_vert = new_map.clone();


	cv::Mat output = new_map.clone();
	cv::Mat input = new_map.clone();
	unsigned char label = 1;
//	unsigned char highest_label = 1;
	int grid_size = 200;

	for (int r = 0; r < input.rows; r+= grid_size)
	{
		ROS_INFO("r = %u", r);
		for (int c = 0; c < input.cols; c+= grid_size)
		{
			int endc = grid_size;
			int endr = grid_size;
			if (r+grid_size > input.rows)
			{
				endr = input.rows-1 - r;
			}
			if (c+grid_size > input.cols)
			{
				endc = input.cols-1 - c;
			}
			cv::Rect roi_square = cv::Rect(c,r, endc, endr);
			cv::Mat image_roi = input(roi_square);
			image_roi.setTo(label);
//			cv::Mat mask;
//			cv::Rect rect;
//			mask.create(image_roi.rows+2, image_roi.cols+2, CV_8UC1);
//			cv::floodFill(image_roi, mask, cv::Point(0,0) , cv::Scalar(label), &rect, cv::Scalar(0),
//					cv::Scalar(0), 4);
			label++;
		}
	}


//	int factor_x = floor(input.cols / grid_size);
//	int factor_y = floor(input.rows / grid_size);
//	for (int y = 0; y < factor_y * grid_size; y+= grid_size)
//	{
//		for (int x = 0; x < factor_x * grid_size; x+= grid_size)
//		{
//			cv::Rect region_of_interest = cv::Rect(x, y, grid_size, grid_size);
//			cv::Mat image_roi = input(region_of_interest);
//
//			if (image_roi.at<unsigned char> (y,x) != 0)
//			{
//				cv::floodFill(output, image_roi, cv::Point(0,0),  4 | cv::FLOODFILL_MASK_ONLY | ( label << 8 ));
//				label++;
//			}
//
//		}
//	}
//			std::vector< std::vector< unsigned char> > labels_to_equalize;
//			for (int k=0; k < image_roi.rows; k++)
//			{
//				for (int l=0; l < image_roi.cols; l++)
//				{
//					if (image_roi.at<unsigned char>(l,k) != 0)
//					{
//						if (image_roi.at<unsigned char>(l,k-1) == 0)
//						{
//							label++;
//							highest_label = label;
//						}
//						image_roi.at<unsigned char>(l,k) = label;
//
//						if (image_roi.at<unsigned char>(l,k) != image_roi.at<unsigned char>(l-1,k) && image_roi.at<unsigned char> (l-1,k) != 0 )
//						{
//							std::vector<unsigned char> equal_labels;
//							equal_labels.push_back(image_roi.at<unsigned char>(l,k));
//							equal_labels.push_back(image_roi.at<unsigned char>(l-1,k));
//							labels_to_equalize.push_back(equal_labels);
//						}
//
//						if (image_roi.at<unsigned char> (l,k-1) != image_roi.at<unsigned char> (l,k-1) && image_roi.at<unsigned char> (l,k-1) != 0 )
//						{
//							std::vector<unsigned char> equal_labels;
//							equal_labels.push_back(image_roi.at<unsigned char>(l,k));
//							equal_labels.push_back(image_roi.at<unsigned char>(l,k-1));
//							labels_to_equalize.push_back(equal_labels);
//						}
//					}
//				}
//			}
//
//			for (int z = 0; z < labels_to_equalize.size(); z++)
//			{
//				for (int r = 0; r < image_roi.rows; r++)
//				{
//					for (int c = 0; c < image_roi.cols; c++)
//					{
//						if (image_roi.at<unsigned char>(r,c) == labels_to_equalize.at(z).at(1))
//						{
//							image_roi.at<unsigned char>(r,c) = labels_to_equalize.at(z).at(0);
//						}
//						else if (labels_to_equalize.at(z).at(0))
//						{
//
//						}
//					}
//				}
//			}
//
//			image_roi.copyTo(output(region_of_interest));
//			labels_to_equalize.clear();
//
//		}
//		label = highest_label;
//	}

	std::vector<int> list_of_labels;
	for (int r = 0; r < output.rows; ++r)
	{
		for ( int c = 0; c < output.cols; ++c)
		{
			if (output.at<int>(r,c) != 0)
			{
				int label = output.at<int>(r,c);
				if (std::find(list_of_labels.begin(), list_of_labels.end(), label) != list_of_labels.end())
				{

				}
				else
				{
			//		ROS_INFO_STREAM("Adding label =" << label);
					list_of_labels.push_back(label);
				}
			}
		}
	}

//	unsigned char first_label_in_row = 1;
//	unsigned char last_label_in_row = 0;
//	bool label_in_work = false;
//	int counterx = 0;
//	int countery = 0;
//	for (int y = 0; y < new_map.rows; y++)
//	{
//		for (int x = 0; x < new_map.cols; x++)
//		{
//			if (counterx>100)
//			{
//				label++;
//				counterx = 0;
//				label_in_work = false;
//			}
//			if (x == map.cols-1)
//			{
//				last_label_in_row = label;
//				label = first_label_in_row;
//				counterx = 0;
//				label_in_work = false;
//			}
//			if (countery > 100)
//			{
//				first_label_in_row = last_label_in_row + 1;
//				label = first_label_in_row;
//				countery = 0;
//				label_in_work = false;
//			}
//
//			counterx++;
//
//			if (new_map.at<unsigned char>(y,x) != 0)
//			{
//				if (label_in_work)
//				{
//					// pixel links
//					if (new_map.at<unsigned char>(y,x-1) == 0 || (new_map.at<unsigned char>(y,x-1) == 250))
//					{
//						new_map.at<unsigned char>(y,x) = 250;
//					}
////					else if (new_map.at<unsigned char>(y-1,x) == 0 || (new_map.at<unsigned char>(y-1,x) == 250))
////					{
////						new_map.at<unsigned char>(y,x) = 250;
////					}
//					else
//					{
//						new_map.at<unsigned char>(y,x) = label;
//					}
//				}
//				else
//				{
//					new_map.at<unsigned char>(y,x) = label;
//					label_in_work = true;
//				}
//
//				if (std::find(list_of_labels.begin(), list_of_labels.end(), label) != list_of_labels.end())
//				{
//
//				}
//				else
//				{
//					list_of_labels.push_back(label);
//				}
//			}
//		}
//		countery++;
//	}

	//display
	cv::Mat colour_map = new_map.clone();
	cv::cvtColor(colour_map, colour_map, CV_GRAY2BGR);
	ROS_INFO_STREAM("list_of_labels.size() "<< list_of_labels.size());
	for (int l = 0; l < list_of_labels.size(); l++)
	{
		//choose random color for each room
		int blue = (rand() % 250) + 1;
		int green = (rand() % 250) + 1;
		int red = (rand() % 250) + 1;
		for (int y = 0; y < output.rows; y++)
		{
			for (int x = 0; x < output.cols; x++)
			{
				if (output.at<unsigned char>(y,x) ==  list_of_labels.at(l))
				{
					colour_map.at<cv::Vec3b>(y,x) = cv::Vec3b(blue,green,red);
				}
				else if (output.at<unsigned char>(y,x) == 250)
				{
					colour_map.at<cv::Vec3b>(y,x) = cv::Vec3b(255,255,255);
				}
			}
		}
	}

	cv::imshow("testoutput_color", colour_map);


//	//same vertical
//	std::vector<unsigned char> list_of_labels_vert;
//
//	unsigned char label_vert = 1;
//	unsigned char first_label_in_row_vert = 1;
//	unsigned char last_label_in_row_vert = 0;
////	bool label_in_work_vert = false;
//	std::vector<bool> label_in_work_vert;
//	int counterx_vert = 0;
//	int countery_vert = 0;
//	for (int y = 0; y < new_map_vert.rows; y++)
//	{
//		for (int x = 0; x < new_map_vert.cols; x++)
//		{
//			if (counterx_vert>100)
//			{
//				label_vert++;
//				label_in_work_vert.push_back(false);
//				counterx_vert = 0;
//				label_in_work_vert = false;
//			}
//			if (x == new_map_vert.cols-1)
//			{
//				last_label_in_row_vert = label_vert;
//				label_vert = first_label_in_row_vert;
//				counterx_vert = 0;
//				label_in_work_vert = false;
//			}
//			if (countery_vert > 100)
//			{
//				first_label_in_row_vert = last_label_in_row_vert + 1;
//				label_vert = first_label_in_row_vert;
//				countery_vert = 0;
//				label_in_work_vert = false;
//			}
//
//			counterx_vert++;
//
//			if (new_map_vert.at<unsigned char>(y,x) != 0)
//			{
//				if (label_in_work_vert)
//				{
//					// pixel links
//					if (new_map_vert.at<unsigned char>(y,x-1) == 0 || (new_map_vert.at<unsignvoid addElementNotInVec(std::vector<int> &reallabelcount, int label)
//					{
//						new_map_vert.at<unsigned char>(y,x) = 250;
//					}
////					else if (new_map.at<unsigned char>(y-1,x) == 0 || (new_map.at<unsigned char>(y-1,x) == 250))
////					{
////						new_map.at<unsigned char>(y,x) = 250;
////					}
//					else
//					{
//						new_map_vert.at<unsigned char>(y,x) = label_vert;
//					}
//				}
//				else
//				{
//					new_map_vert.at<unsigned char>(y,x) = label_vert;
//					label_in_work_vert = true;
//				}
//
//				if (std::find(list_of_labels_vert.begin(), list_of_labels_vert.end(), label_vert) != list_of_labels_vert.end())
//				{
//
//				}
//				else
//				{
//					list_of_labels_vert.push_back(label_vert);
//				}
//			}
//		}
//		countery_vert++;
//	}
//
//
//
////	cv::imshow("testoutput_map", new_map);
//
//	//display
//	cv::Mat colour_map_vert = new_map_vert.clone();
//	cv::cvtColor(colour_map_vert, colour_map_vert, CV_GRAY2BGR);
//	ROS_INFO_STREAM("list_of_labels.size() "<< list_of_labels_vert.size());
//	for (int l = 0; l < list_of_labels_vert.size(); l++)
//	{
//		//choose random color for each room
//		int blue = (rand() % 250) + 1;
//		int green = (rand() % 250) + 1;
//		int red = (rand() % 250) + 1;
//		for (int y = 0; y < new_map_vert.rows; y++)
//		{
//			for (int x = 0; x < new_map_vert.cols; x++)
//			{
//				if (new_map_vert.at<unsigned char>(y,x) ==  list_of_labels_vert.at(l))
//				{
//					colour_map_vert.at<cv::Vec3b>(y,x) = cv::Vec3b(blue,green,red);
//				}
//				else if (new_map_vert.at<unsigned char>(y,x) == 250)
//				{
//					colour_map_vert.at<cv::Vec3b>(y,x) = cv::Vec3b(255,255,255);
//				}
//			}
//		}
//	}
//	cv::imshow("testoutput_color_vert", colour_map_vert);
	cv::waitKey();
	//exit
	return 0;
}




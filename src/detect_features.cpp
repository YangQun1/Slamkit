/*
 * 提取图像特征点
 * Author: YangQun
 * Date: 2018/5/9
 * Last Update: 2018/5/9
 */

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "frame.h"
#include "detect_features.h"
#include "param_reader.h"

using namespace std;

DetectFeatures::DetectFeatures(ParameterReader::Ptr param_reader)
{
	detector_name_ = param_reader->getParam<string>("detector_name");
	is_show_ = param_reader->getParam<bool>("is_show_feature_detection");
	feature_num_ = param_reader->getParam<int>("feature_num");
	
	if (detector_name_ == "ORB") {
		feature_detector_ = cv::ORB::create(feature_num_);
	}
	else {
		cout << "invalid detector_name" << endl;
	}
}

DetectFeatures::~DetectFeatures()
{
}

void DetectFeatures::detectFeaturePoints(Frame::Ptr frame)
{
	cv::Mat &rgb = frame->rgb_img_;
	cv::Mat &depth = frame->depth_img_;
	vector<cv::KeyPoint> &keypoints = frame->keypoints_;
	cv::Mat &descrip = frame->descrip_;
	
	// 提取特征点,包括关键点和描述子
	feature_detector_->detectAndCompute(rgb, cv::Mat(), keypoints, descrip);
	cout << "detect total " << keypoints.size() << " KeyPoints" << endl;
	
	// 显示关键点
	if (is_show_) {
		cv::Mat imgShow;
		cv::drawKeypoints(rgb, keypoints, imgShow);
		cv::imshow("keypoints", imgShow);
		cv::waitKey(0);
	}
	
	return;
}





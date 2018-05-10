/*
 * 
 */

#ifndef ALIGN_IMAGE_H
#define ALIGN_IMAGE_H

#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sophus/se3.h>

#include "pinhole_camera.h"
#include "detect_features.h"
#include "param_reader.h"

using namespace std;

class AlignImage 
{
private:
	PinHoleCamera *cam_;
	
	vector<cv::DMatch> matches_;
	vector<cv::DMatch> good_matches_;
	
	cv::Ptr<cv::DescriptorMatcher> matcher_;
	
	string matcher_name_;
	double good_match_threshold_;
	bool is_show_;
	
	Eigen::Matrix3d R_;	// 两帧之间的旋转矩阵
	Eigen::Vector3d t_;		// 两帧之间的平移向量
	Eigen::Isometry3d T_;	// 两帧之间的变换关系
	
	Sophus::SE3 T_f_r_;		// 两帧之间的变换关系,李群表示
	
public:
	AlignImage(PinHoleCamera *cam, ParameterReader *param_reader);
	~AlignImage();
	
	void alignImage(Frame& frame1, Frame& frame2);
	
};

#endif
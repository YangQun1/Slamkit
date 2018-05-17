/*
 * 
 */

#ifndef ALIGN_IMAGE_H
#define ALIGN_IMAGE_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sophus/se3.h>

#include "pinhole_camera.h"
#include "detect_features.h"
#include "param_reader.h"
#include "map.h"

using namespace std;

class AlignImage 
{
public:
	typedef boost::shared_ptr<AlignImage> Ptr;
	
private:
	PinHoleCamera::Ptr cam_;
	
	vector<cv::DMatch> matches_;
	vector<cv::DMatch> good_matches_;
	
	cv::Ptr<cv::DescriptorMatcher> matcher_;
	
	string matcher_name_;
	double good_match_threshold_;
	bool is_show_;
	
// 	Eigen::Matrix3d R_;	// 两帧之间的旋转矩阵
// 	Eigen::Vector3d t_;		// 两帧之间的平移向量
// 	Eigen::Isometry3d T_;	// 两帧之间的变换关系
	
	Sophus::SE3 T_r2c_;		/* 匹配两帧时,表示ref_frame到curr_frame的变换关系,
							匹配帧和地图时,表示world到curr_frame的变换关系*/
	
	// 与配准质量有关的变量
	int min_good_matches_;		// 最少优良匹配匹配点数量
	int min_inliers_;				// 最小符合pnp约束的内点数量
	double max_norm_;			// 两帧之间允许的最大运动量
	
	bool is_good_align_;			// 指示是否匹配良好
	
	double normOfTransform(cv::Mat rvec, cv::Mat tvec);
	void optimizePoseOfPnp(vector<cv::Point3f>& points_obj,  
									vector<cv::Point2f>& points_img, 
									cv::Mat inliers,  
									cv::Mat& rvec,  
									cv::Mat& tvec);
public:
	AlignImage(PinHoleCamera::Ptr cam, ParameterReader::Ptr param_reader);
	~AlignImage();
	
	vector<int> match_2dkp_index_;
	int inliers_num_;
	
	void alignTwoFrames(Frame::Ptr ref_frame, Frame::Ptr curr_frame);
	bool checkAlignQuality(){ return is_good_align_; }
	void alignMapFrame(Map::Ptr local_map, Frame::Ptr curr_frame, Frame::Ptr ref_frame);
};

#endif
/*
 * 
 */
#include <stdlib.h>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>	// opencv自带的与eigen类型转换api
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include "pinhole_camera.h"
#include "frame.h"
#include "detect_features.h"
#include "align_image.h"
#include "param_reader.h"

using namespace std;

AlignImage::AlignImage(PinHoleCamera *cam, ParameterReader *param_reader)
{
	cam_ = cam;
	matcher_name_ = param_reader->getParam<string>("matcher_name");
	good_match_threshold_ = param_reader->getParam<double>("good_match_threshold");
	is_show_ = param_reader->getParam<bool>("is_show_feature_match");
	
	T_ = Eigen::Isometry3d::Identity();
	
	// 根据名字动态构建相应的匹配器
	if (matcher_name_ == "FLANN"){
		matcher_ = cv::FlannBasedMatcher::create();
	}
	else if(matcher_name_ == "BF") {
		matcher_ = cv::BFMatcher::create();
	}
	else {
		cout << "invalid matcher name" << endl;
	}
}

AlignImage::~AlignImage()
{
}


// 求解PnP,求解出两帧图像之间的位姿变换关系
void AlignImage:: alignImage(Frame& frame1, Frame& frame2)
{
	// 匹配两帧的描述子
	matcher_->match(frame1.descrip, frame2.descrip, matches_);
	cout << "find total " << matches_.size() << " matches" << endl;
	
	// 根据距离,筛选好的匹配
	// 寻找最小距离
	double min_dist = 9999;
	for (cv::DMatch  m : matches_) {
		if (m.distance < min_dist)
			min_dist = m.distance;
	}
	// 筛选
	for (cv::DMatch  m : matches_) {
		if (m.distance < good_match_threshold_ * min_dist)
			good_matches_.push_back(m);
	}
	cout << "good matches: " << good_matches_.size() << endl;
	if (is_show_) {
		cv::Mat imgMatches;
		cv::drawMatches(frame1.rgbImg_, frame1.keypoints, frame2.rgbImg_ , frame2.keypoints, good_matches_, imgMatches);
		cv::imshow("good matches", imgMatches);
		cv::waitKey(0);
	}
	
	// 第一帧中的三维点
	vector<cv::Point3f> points_obj;
	// 第二帧中的图像点
	vector<cv::Point2f> points_img;
	
	// 获得对应的三维点和像素点坐标
	for (cv::DMatch m : good_matches_) {
		// 获取第一帧图像中点的像素坐标和深度
		cv::Point2f p = frame1.keypoints[m.queryIdx].pt;
		ushort d = frame1.depthImg_.ptr<ushort>(int(p.y))[int(p.x)];
		if (d == 0)
			continue;
		
		// 得到第一帧图像中空间点坐标
		cv::Point3f pt_2d_with_depth(p.x, p.y, d);
		cv::Point3f pt_obj = cam_->point2dTo3d(pt_2d_with_depth);
		points_obj.push_back(pt_obj);
		
		// 得到与之对应的第二帧图像中像素点坐标
		cv::Point2f pt_img = frame2.keypoints[m.trainIdx].pt;
		points_img.push_back(pt_img);
	}
	
	// 求解PnP问题,同时使用RANSAC去除outlier
	cv::Mat intrisic_matrix = cv::Mat_<double>::ones(3, 3);
	cv::Mat rvec, tvec, inliers;
	cv::eigen2cv(cam_->K(), intrisic_matrix);
	cv::solvePnPRansac(points_obj, points_img, intrisic_matrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers);
	cout<<"inliers: "<<inliers.rows<<endl;
	cout<<"rvec="<<rvec<<endl;
	cout<<"tvec="<<tvec<<endl;
	if (is_show_) {
		// 画出inliers匹配 
		vector< cv::DMatch > matchesShow;
		cv::Mat imgMatches;
		for (size_t i=0; i<inliers.rows; i++) {
			matchesShow.push_back( good_matches_[inliers.ptr<int>(i)[0]] );    
		}
		cv::drawMatches(frame1.rgbImg_, frame1.keypoints, frame2.rgbImg_, frame2.keypoints, matchesShow, imgMatches);
		cv::imshow( "inlier matches", imgMatches );
		cv::waitKey( 0 );
	}
	
	// 将rvec和tvec转成Eigen的数据类型
	cv::Mat R;
	cv::Rodrigues(rvec, R);
	cout << "OpenCV R:"<<endl << R << endl;
	cv::cv2eigen(R, R_);
	cout << "Eigen R:" << endl << R_.matrix() << endl;
	t_(0) = tvec.at<double>(0,0);
	t_(1) = tvec.at<double>(0,1);
	t_(2) = tvec.at<double>(0,2);
	// 构造Eigen变换关系
	Eigen::AngleAxisd angle(R_);
	T_.rotate ( angle ); 
	T_.pretranslate (t_ );
	cout << "Eigen T:" << endl << T_.matrix() << endl;
	// 构造Sophus变换关系
	Sophus::SE3 T_f_r(R_, t_);
	T_f_r_ = T_f_r;
	cout << "Sophus T:" << endl << T_f_r_ << endl;
	
	// 计算当前帧与世界坐标之间的变换
	frame2.T_f_w_ = T_f_r_ * frame1.T_f_w_;
	
	
	return;
}







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
	min_good_matches_ = param_reader->getParam<int>("min_good_matches");
	min_inliers_ = param_reader->getParam<int>("min_inliers");
	max_norm_ = param_reader->getParam<double>("max_norm");
	
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
void AlignImage:: alignImage(FramePtr ref_frame, FramePtr curr_frame)
{
	is_good_align_ = true;
	
	// 清空之前的匹配数据,防止不同帧对之间的数据累积
	matches_.clear();
	good_matches_.clear();
	
	// 匹配两帧的描述子
	matcher_->match(ref_frame->descrip_, curr_frame->descrip_, matches_);
	cout << "find total " << matches_.size() << " matches" << endl;
	
	// 根据距离,筛选好的匹配
	// 寻找最小距离
	double min_dist = 9999;
	for (cv::DMatch  m : matches_) {
		if (m.distance < min_dist)
			min_dist = m.distance;
	}
	cout << "min_dist: " << min_dist << endl;
	
	// 筛选
	if (min_dist < 10 ) min_dist = 10;		// 防止因为min_dist等于0而找不到good_match的情况
	for (cv::DMatch  m : matches_) {
		if (m.distance < good_match_threshold_ * min_dist)
			good_matches_.push_back(m);
	}
	cout << "good matches: " << good_matches_.size() << endl;
	if (good_matches_.size() < min_good_matches_) {
		is_good_align_ = false;
		return;
	}
	
	if (is_show_) {
		cv::Mat imgMatches;
		cv::drawMatches(ref_frame->rgb_img_, ref_frame->keypoints_, curr_frame->rgb_img_ , curr_frame->keypoints_, good_matches_, imgMatches);
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
		cv::Point2f p = ref_frame->keypoints_[m.queryIdx].pt;
		ushort d = ref_frame->depth_img_.ptr<ushort>(int(p.y))[int(p.x)];
		if (d == 0) continue;
		
		// 得到第一帧图像中空间点坐标
		cv::Point3f pt_2d_with_depth(p.x, p.y, d);
		cv::Point3f pt_obj = cam_->point2dTo3d(pt_2d_with_depth);
		points_obj.push_back(pt_obj);
		
		// 得到与之对应的第二帧图像中像素点坐标
		cv::Point2f pt_img = curr_frame->keypoints_[m.trainIdx].pt;
		points_img.push_back(pt_img);
	}
	
	// 检查有效的目标点的数量,小于4则会引发opencv异常,需要放弃该帧
	if (points_obj.size() < 4) {
		is_good_align_ = false;
		return;
	}
	
	// 求解PnP问题,同时使用RANSAC去除outlier
	cv::Mat intrisic_matrix = cv::Mat_<double>::ones(3, 3);
	cv::Mat rvec, tvec, inliers;
	cv::eigen2cv(cam_->K(), intrisic_matrix);
	// solvePnPRansac函数输出的是三维点坐标系(模型坐标系)到二维点坐标系(相机坐标系)的变换关系
	// 在这里,就是fram1坐标系中的点左乘得到的变换关系,可以变换到curr_frame坐标系中
	cv::solvePnPRansac(points_obj, points_img, intrisic_matrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers);
	// cv::solvePnPRansac(points_obj, points_img, intrisic_matrix, cv::Mat(), rvec, tvec, false, 100, 5.0, 0.90, inliers);
	cout<<"inliers: "<<inliers.rows<<endl;
	cout<<"rvec="<<rvec<<endl;
	cout<<"tvec="<<tvec<<endl;
	if (inliers.rows < min_inliers_ || normOfTransform(rvec, tvec) > max_norm_){
		is_good_align_ = false;
		return;
	}
	
	if (is_show_) {
		// 画出inliers匹配 
		vector< cv::DMatch > matchesShow;
		cv::Mat imgMatches;
		for (size_t i=0; i<inliers.rows; i++) {
			matchesShow.push_back( good_matches_[inliers.ptr<int>(i)[0]] );    
		}
		cv::drawMatches(ref_frame->rgb_img_, ref_frame->keypoints_, curr_frame->rgb_img_, curr_frame->keypoints_, matchesShow, imgMatches);
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
	Sophus::SE3 T_r2c(R_, t_);
	T_r2c_ = T_r2c;
	cout << "Sophus T:" << endl << T_r2c_ << endl;
	
	// 计算当前帧到世界坐标的变换
	/******************************************************************
	 * 注意,这个地方之所以是右乘 T_r2c_ 的逆,是因为从当前帧坐标系变到世界坐标系的
	 * 过程等价于先变到前一帧的坐标系,再变到前前帧的坐标系,以此类推,用公式表示
	 * 就是Pw=Tw1^-1 * T12^-1 * T23^-1 * ...* Tn-1n^-1 * Pn,因此相应的变换矩阵就是
	 * 从第一帧到世界坐标系的变换开始不断的右乘参考帧到当前帧变换的逆
	 ******************************************************************/
	curr_frame->T_c2w_ = ref_frame->T_c2w_ * T_r2c_.inverse();
	
	return;
}

double AlignImage::normOfTransform(cv::Mat rvec, cv::Mat tvec)
{
	return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}





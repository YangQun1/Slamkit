/*
 * 
 */
#include <stdlib.h>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <sophus/se3.h>
#include <sophus/so3.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>	// opencv自带的与eigen类型转换api
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/block_solver.h>
// #include <g2o/core/robust_kernel.h>
// #include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include "pinhole_camera.h"
#include "frame.h"
#include "detect_features.h"
#include "align_image.h"
#include "param_reader.h"
#include "g2o_types_costom.h"
#include "map_point.h"
#include "map.h"

using namespace std;

AlignImage::AlignImage(PinHoleCamera::Ptr cam, ParameterReader:: Ptr param_reader)
{
	cam_ = cam;
	matcher_name_ = param_reader->getParam<string>("matcher_name");
	good_match_threshold_ = param_reader->getParam<double>("good_match_threshold");
	is_show_ = param_reader->getParam<bool>("is_show_feature_match");
	min_good_matches_ = param_reader->getParam<int>("min_good_matches");
	min_inliers_ = param_reader->getParam<int>("min_inliers");
	max_norm_ = param_reader->getParam<double>("max_norm");
	
// 	T_ = Eigen::Isometry3d::Identity();
	
	// 根据名字动态构建相应的匹配器
	if (matcher_name_ == "FLANN"){
		matcher_ = cv::makePtr<cv::FlannBasedMatcher>(new cv::flann::LshIndexParams ( 5,10,2 ));
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
void AlignImage:: alignTwoFrames(Frame::Ptr ref_frame, Frame::Ptr curr_frame)
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
		Eigen::Vector3d pt_temp = cam_->pixel2camera(Eigen::Vector2d(p.x, p.y), (double)d);
		cv::Point3f pt_obj( pt_temp(0,0), pt_temp(1,0), pt_temp(2,0) );
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
	
	optimizePoseOfPnp(points_obj, points_img, inliers, rvec, tvec);
	
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


/**
 * @brief ...
 * 
 * @param local_map ...
 * @param curr_frame ...
 * @param ref_frame 只用于显示,调试时看图
 * @return void
 */
void AlignImage::alignMapFrame(Map::Ptr local_map, Frame::Ptr curr_frame, Frame::Ptr ref_frame)
{
	is_good_align_ = true;
	
	// 清空之前的匹配数据,防止数据累积
	matches_.clear();
	good_matches_.clear();
	match_2dkp_index_.clear();
	
	// 根据假设的当前帧位姿,对地图点进行投影,筛选可能会出现在当前帧中的地图点作为匹配候选点
	vector<MapPoint::Ptr> candidate_map_points;
	cv::Mat map_descriptor;
	for ( auto point_pair : local_map->map_points_ ) {
		MapPoint::Ptr& point = point_pair.second;
		if (curr_frame->isInFrame(point->pose_)) {
			// 如果地图点可能出现在当前帧中,则将其作为候选
			candidate_map_points.push_back(point);
			map_descriptor.push_back(point->descriptor_);
		}
	}
	cout << "candidate_map_points: " << candidate_map_points.size() << endl;
	
	// 匹配候选的地图点和当前帧的特征点
	matcher_->match(map_descriptor, curr_frame->descrip_, matches_);
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
	if (min_dist < 10.0 ) min_dist = 10.0;		// 防止因为min_dist等于0而找不到good_match的情况
	for (cv::DMatch  m : matches_) {
		if (m.distance < good_match_threshold_ * min_dist) {
			good_matches_.push_back(m);
			match_2dkp_index_.push_back( m.trainIdx );
		}
	}
	cout << "good matches: " << good_matches_.size() << endl;
	if (good_matches_.size() < min_good_matches_) {
		is_good_align_ = false;
		return;
	}
	
	if (is_show_) {
		cv::Mat img_show = ref_frame->rgb_img_.clone();
		for ( auto& pt : candidate_map_points ) {
			Eigen::Vector2d pixel = ref_frame->cam_->world2pixel ( pt->pose_, ref_frame->T_c2w_.inverse() );
			cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
		}
		cv::imshow("candidate", img_show);
		cv::waitKey(0);
	}
	
	vector<cv::Point3f> points_world;
	vector<cv::Point2f> points_img;
	
	// 获得对应的三维点和像素点坐标
	for (cv::DMatch m : good_matches_) {
		// 3D点
		cv::Point3f pt_world = (candidate_map_points[m.queryIdx])->getPositionCV();
		points_world.push_back(pt_world);
		// 得到与之对应的第二帧图像中像素点坐标
		cv::Point2f pt_img = curr_frame->keypoints_[m.trainIdx].pt;
		points_img.push_back(pt_img);
	}
	
	// 检查有效的目标点的数量,小于4则会引发opencv异常,需要放弃该帧
	if (points_world.size() < 4) {
		is_good_align_ = false;
		return;
	}
	
	// 求解PnP问题,同时使用RANSAC去除outlier
	cv::Mat intrisic_matrix = cv::Mat_<double>::ones(3, 3);
	cv::Mat rvec, tvec, inliers;
	cv::eigen2cv(cam_->K(), intrisic_matrix);
	// solvePnPRansac函数输出的是三维点坐标系(模型坐标系)到二维点坐标系(相机坐标系)的变换关系
	// 在这里,就是世界坐标系中的点左乘得到的变换关系,可以变换到curr_frame坐标系中
	cv::solvePnPRansac(points_world, points_img, intrisic_matrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers);
	cout<<"inliers: "<<inliers.rows<<endl;
	// cout<<"rvec="<<rvec<<endl;
	// cout<<"tvec="<<tvec<<endl;
	inliers_num_ = inliers.rows;
	if (inliers.rows < min_inliers_){
		is_good_align_ = false;
		return;
	}
	
	if (is_show_) {
		// TODO
	}
	
	optimizePoseOfPnp(points_world, points_img, inliers, rvec, tvec);
	
	// 计算当前帧到世界坐标的变换
	/*
	 * 因为此时T_r2c_表示的是世界到帧的变换
	 */
	curr_frame->T_c2w_ = T_r2c_.inverse();
	
	// 根据求得的当前帧的准确位姿,对地图点的相关变量进行更新
	for ( auto point_pair : local_map->map_points_ ) {
		MapPoint::Ptr& point = point_pair.second;
		if (curr_frame->isInFrame(point->pose_)) {
			point->observed_times_++;
		}
	}
	for (int i = 0; i < inliers.rows; i++) {
		int index = inliers.at<int>(i,0);	
		// 符合几何关系的地图点,其被匹配的次数加1
		candidate_map_points[index]->matched_times_++;
	}
	
	return;
}

double AlignImage::normOfTransform(cv::Mat rvec, cv::Mat tvec)
{
	return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

void AlignImage::optimizePoseOfPnp(vector<cv::Point3f>& points_obj,  vector<cv::Point2f>& points_img, cv::Mat inliers,  cv::Mat& rvec,  cv::Mat& tvec)
{
	// 使用g2o对求解出来的变换关系进行优化
	typedef g2o::BlockSolver<g2o::BlockSolverTraits< Eigen::Dynamic, Eigen::Dynamic >> MyBlockSolver;
	typedef g2o::LinearSolverDense<MyBlockSolver::PoseMatrixType> MyLinerSolver;
	g2o::OptimizationAlgorithmLevenberg *opt_alg = new g2o::OptimizationAlgorithmLevenberg(
													g2o::make_unique<MyBlockSolver>(
														g2o::make_unique<MyLinerSolver>() ) );
	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm(opt_alg);
	
	// 使用PnP输出构建待优化的SE3变量
	Sophus::SE3 T_r2c_no_opt(
		Sophus::SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)),
		Eigen::Vector3d(tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0)) );
	// cout << "优化前的T_r2c_:" << endl <<  T_r2c_no_opt.matrix() << endl;
	
	// 添加待优化节点
	g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap;
	pose->setId(0);
	pose->setEstimate( g2o::SE3Quat( T_r2c_no_opt.rotation_matrix(), T_r2c_no_opt.translation() ) );
	optimizer.addVertex( pose );
	// 添加边,每一条边表示一次三维点到图像点的投影测量,能计算到一个误差
	for (int i = 0; i < inliers.rows; i++) {
		int index = inliers.at<int>(i,0);	// 第i个内点在原来的点列表中的索引
		EdgeProjectXYZ2UVUPoseOnly *edge = new EdgeProjectXYZ2UVUPoseOnly;
		edge->setId(i);
		edge->setVertex(0, pose);
		edge->cam_ = cam_;
		edge->point_ = Eigen::Vector3d(points_obj[index].x, points_obj[index].y, points_obj[index].z);
		edge->setMeasurement(Eigen::Vector2d(points_img[index].x, points_img[index].y));
		edge->setInformation(Eigen::Matrix2d::Identity());	// 信息矩阵是协方差矩阵的逆,表示的是对误差向量中不同分量的重视程度(权重),最简单的即设置为单位阵
		optimizer.addEdge( edge );
	}
	// 启动优化
	optimizer.initializeOptimization();
	optimizer.optimize(10);
	
	// 转存优化后的变量
	T_r2c_ = Sophus::SE3( pose->estimate().rotation(),  pose->estimate().translation() );
// 	R_ = T_r2c_.rotation_matrix();
// 	t_ = T_r2c_.translation();
// 	T_.rotate ( Eigen::AngleAxisd(R_) ); 		// 构造Eigen变换关系
// 	T_.pretranslate (t_ );
	cout << "优化后的T_r2c_:" << endl <<  T_r2c_.matrix() << endl;
	
	return;
}





#include <string>
#include <boost/shared_ptr.hpp>

#include <sophus/se3.h>
#include <Eigen/Eigen>
#include <opencv2/highgui/highgui.hpp>

#include "param_reader.h"
#include "pinhole_camera.h"
#include "detect_features.h"
#include "align_image.h"
#include "generate_point_cloud.h"
#include "map_point.h"
#include "map.h"
#include "image_reader.h"
#include "visual_odometry.h"

VisualOdometry::VisualOdometry(ParameterReader::Ptr param_reader):
state_(INITIALIZING),
local_map_(new Map),
cam_(new PinHoleCamera(param_reader)),
detector_(new DetectFeatures(param_reader)),
align_image_(new AlignImage(cam_, param_reader)),
point_cloud_generator_(new PointCloudGenerator(cam_, param_reader)),
min_keyframe_rotate_(param_reader->getParam<double>("min_keyframe_rotate")),
min_keyframe_translate_(param_reader->getParam<double>("min_keyframe_translate")),
map_point_erase_ratio_(param_reader->getParam<double>("map_point_erase_ratio")),
max_num_lost_(param_reader->getParam<double>("max_num_lost"))
{
	
}

VisualOdometry::~VisualOdometry()
{
	// 因为使用智能指针,所以析构函数中不需要做内存释放工作
}

bool VisualOdometry::addImage(cv::Mat& rgb_img, cv::Mat& depth_img)
{
	switch (state_)
	{
		case INITIALIZING: {
			curr_frame_ = Frame::createFrame(cam_, rgb_img.clone(), depth_img.clone());
			curr_frame_->T_c2w_ = Sophus::SE3( Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero() );
			ref_frame_ = ref_keyframe_  = curr_frame_;
			detector_->detectFeaturePoints(curr_frame_);		// 计算关键点和描述子
			local_map_->insertKeyFrame(curr_frame_);		// 第一帧必然是关键帧
			addMapPoint();								// 向地图中添加3D点
			point_cloud_generator_->joinPointCloud(curr_frame_);
			state_ = TRACKING;
			break;
		}
		case TRACKING: {
			curr_frame_ = Frame::createFrame(cam_, rgb_img.clone(), depth_img.clone());
			curr_frame_->T_c2w_ = ref_frame_->T_c2w_;	// 用相邻帧的位姿来近似当前帧位姿
			detector_->detectFeaturePoints(curr_frame_);
			align_image_->alignMapFrame(local_map_, curr_frame_, ref_frame_);
			if (true == checkEstimatedPose()) {
				
				 num_lost_ = 0;
				 
				 optimizeMap();
				 
				if (true == checkKeyFrame()) {
					local_map_->insertKeyFrame(curr_frame_);
					ref_keyframe_ = curr_frame_;
				}
				point_cloud_generator_->joinPointCloud(curr_frame_);
				ref_frame_ = curr_frame_;
			}
			else {
				 num_lost_++;
				 if ( num_lost_ > max_num_lost_ ) {
					state_ = LOST;
				}
				
				curr_frame_->T_c2w_ = ref_frame_->T_c2w_;	// 若估计结果出错,则不更新当前帧的位姿
			}
			break;
		}
		case LOST: {
			cout<<"vo has lost."<<endl;
			break;
		}
	}
}

void VisualOdometry::addMapPoint()
{
	int add_num = 0;
	
	if (INITIALIZING == state_) {
		// 初始化阶段,将第一帧的所有特征点都插入到地图中
		for (size_t i=0; i<curr_frame_->keypoints_.size(); i++ ) {
			double d = curr_frame_->findDepth(curr_frame_->keypoints_[i]);
			if(d < 0) 
				continue;
			Eigen::Vector2d pt_img(curr_frame_->keypoints_[i].pt.x, curr_frame_->keypoints_[i].pt.y);
			Eigen::Vector3d pt_world = curr_frame_->cam_->pixel2world(pt_img, curr_frame_->T_c2w_.inverse(), d);
			Eigen::Vector3d n = pt_world - ref_frame_->getCameraCenter();
			n.normalize();
			MapPoint::Ptr map_point = MapPoint::createMapPoint(pt_world, n, curr_frame_->descrip_.row(i).clone());
			local_map_->insertMapPoint(map_point);
			add_num++;
		}
	}
	else if (TRACKING == state_) {
		// 正常的tracking阶段,仅将没有匹配的点添加到地图中
		vector<bool> matched(curr_frame_->keypoints_.size(), false); 
		for ( int index : align_image_->match_2dkp_index_ )
			matched[index] = true;
		for ( int i=0; i<curr_frame_->keypoints_.size(); i++ )
		{
			// 如果在匹配过程中,存在良好的匹配,则认为已经存在于地图中,不在重复添加
			if ( matched[i] == true )   
				continue;
			double d = curr_frame_->findDepth ( curr_frame_->keypoints_[i]);
			if ( d<0 )  
				continue;
			Eigen::Vector2d pt_img(curr_frame_->keypoints_[i].pt.x, curr_frame_->keypoints_[i].pt.y);
			Eigen::Vector3d pt_world = curr_frame_->cam_->pixel2world(pt_img, curr_frame_->T_c2w_.inverse(), d);
			
			Eigen::Vector3d n = pt_world - curr_frame_->getCameraCenter();
			n.normalize();
			MapPoint::Ptr map_point = MapPoint::createMapPoint(pt_world, n, curr_frame_->descrip_.row(i).clone());
			local_map_->insertMapPoint(map_point);
			add_num++;
		}
	}
	else {
		// TODO 
	}
	
	cout << "add " << add_num << " points to map" << endl;
	return;
}

// 检查当前帧与上一关键帧之间的运动量来确定是否将其作为关键帧
bool VisualOdometry::checkKeyFrame()
{
	Sophus::SE3 T_c2r = ref_keyframe_->T_c2w_.inverse() * curr_frame_->T_c2w_;
	
	Sophus::Vector6d d = T_c2r.log();
	Eigen::Vector3d trans = d.head<3>();
	Eigen::Vector3d rot = d.tail<3>();
	if ( rot.norm() >min_keyframe_rotate_ || trans.norm() >min_keyframe_translate_ )
		return true;
	
	return false;
}

bool VisualOdometry::checkEstimatedPose()
{
    // 检查运动估计过程是否出错
    if (false == align_image_->checkAlignQuality()) {
        cout<<"reject because bad align: "<<endl;
        return false;
    }
    
    // 检查估计结果是否正常,太大则认为出错
    Sophus::SE3 T_c2r = ref_frame_->T_c2w_.inverse() * curr_frame_->T_c2w_;
    Sophus::Vector6d d = T_c2r.log();
    if ( d.norm() > 5.0 ) {
        cout<<"reject because motion is too large: "<<d.norm() <<endl;
        return false;
    }
    
    return true;
}

void VisualOdometry::optimizeMap()
{
	int i(0), j(0), k(0);
	// 遍历所有地图点,去掉不合格的点
	for(auto iter = local_map_->map_points_.begin(); iter != local_map_->map_points_.end(); ) {
		// 删除未出现在当前帧中的地图点
		if (!curr_frame_->isInFrame(iter->second->pose_)){
			iter = local_map_->map_points_.erase(iter);
			i++;
			continue;
		}
		// 删除匹配率过低的点
		float match_ratio = float(iter->second->matched_times_) / iter->second->observed_times_;
		if ( match_ratio < map_point_erase_ratio_ )
		{
			iter = local_map_->map_points_.erase(iter);
			j++;
			continue;	
		}
		// 删除当前观测角度与初始观测角度变化太大的地图点
		// (因为视角变化太大会造成特征点外观变化明显,不利于匹配?)
		double angle = getViewAngle( curr_frame_, iter->second );
		if ( angle > M_PI/6. )
		{
			iter = local_map_->map_points_.erase(iter);
			k++;
			continue;
		}
// 		if ( iter->second->good_ == false )
// 		{
// 			// TODO try triangulate this map point 
// 		}
		iter++;
	}
	cout << "i: " << i << "j: " << j <<" k: " << k <<endl;
	
	// 地图点与当前帧匹配过少时,将当前帧中未匹配的点加到地图中
	if (align_image_->match_2dkp_index_.size() < 30 || align_image_->inliers_num_ < 20) {
		addMapPoint();
	}
	
	// 地图点过多时,降低筛选门限,去除更多的点
	if (local_map_->map_points_.size() > 800) {
		map_point_erase_ratio_ += 0.05;
	}
	else {
		map_point_erase_ratio_ = 0.1;
	}
	cout << "map_point_erase_ratio: " << map_point_erase_ratio_ << endl;
	cout << "map size: " << local_map_->map_points_.size() << endl;
	
	return;
}

double VisualOdometry::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr map_point )
{
	Eigen::Vector3d n = map_point->pose_ - frame->getCameraCenter();
	n.normalize();
	return acos( n.transpose() * map_point->norm_observe_direct_ );
}


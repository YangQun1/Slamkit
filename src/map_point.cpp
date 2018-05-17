

#include <boost/shared_ptr.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>

#include "map_point.h"

unsigned long MapPoint::factory_id_ = 0;	// 类内的静态成员变量要单独初始化

MapPoint::MapPoint(long unsigned int id, const Eigen::Vector3d& pose, const Eigen::Vector3d& norm_observe_direct, const cv::Mat& descriptor):
id_(id), pose_(pose), norm_observe_direct_(norm_observe_direct), descriptor_(descriptor), observed_times_(1), matched_times_(1)
{

}

MapPoint::~MapPoint()
{

}

MapPoint::Ptr MapPoint::createMapPoint(const Eigen::Vector3d& pose, const Eigen::Vector3d& norm_observe_direct, const  cv::Mat& descriptor)
{
	return MapPoint::Ptr( new MapPoint(factory_id_++, pose, norm_observe_direct, descriptor) );
}


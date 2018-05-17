#ifndef MAP_POINT_H
#define MAP_POINT_H

#include <boost/shared_ptr.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>


class MapPoint 
{
public:
	MapPoint(unsigned long id, const Eigen::Vector3d& pose, const Eigen::Vector3d& norm_observe_direct, const cv::Mat& descriptor);
	~MapPoint();
	
	typedef boost::shared_ptr<MapPoint> Ptr;
	
	static unsigned long factory_id_;			// 静态成员变量,使用工厂设计模式,用作接口,避免人为显式地指定id_
	unsigned long id_;						// 地图点的id号
	
	Eigen::Vector3d pose_;					// 在世界坐标系中的3D坐标
	Eigen::Vector3d norm_observe_direct_;	// 该点与观察到它的相机光心之间连线的方向向量(归一化的)
	cv::Mat descriptor_;					// 描述子
	
	int matched_times_;					// 被匹配成功的次数
	int observed_times_;					// 被观测到的次数

public:
	// 工厂生产函数,MapPoint类对外的接口函数
	// 该函数必须声明为static,以访问static类型的成员变量factory_id_
	static MapPoint::Ptr createMapPoint(const Eigen::Vector3d& pose, const Eigen::Vector3d& norm_observe_direct, const  cv::Mat& descriptor);
	
	cv::Point3f getPositionCV() const { return cv::Point3f( pose_(0,0), pose_(1,0), pose_(2,0) );
    }
};

#endif
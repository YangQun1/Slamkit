/*
 */

#ifndef generatePointCloud_H
#define generatePointCloud_H

// C++标准库

// 第三方库头文件
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 工程头文件
#include "pinhole_camera.h"
#include "frame.h"
#include "param_reader.h"

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 


class PointCloudGenerator
{
private:
	PinHoleCamera *cam_;
	
	bool is_show_;
	bool is_save_;
	
public:
	PointCloudGenerator(PinHoleCamera *cam, ParameterReader *param_reader);
	~PointCloudGenerator();
	
	PointCloud::Ptr generatePointCloud(Frame& frame);
	int joinPointCloud(Frame& frame1, Frame& frame2);
};

#endif
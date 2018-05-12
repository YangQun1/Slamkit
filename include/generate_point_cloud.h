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
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

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
	PointCloud::Ptr cloud_out_;
	pcl::VoxelGrid<PointT> voxel_;
	pcl::visualization::CloudViewer *viewer_;
	
	double grid_size_;	// 点云体素滤波的栅格大小
	
	bool is_show_;
	bool is_save_;
	
	bool is_filter_;
	
public:
	PointCloudGenerator(PinHoleCamera *cam, ParameterReader *param_reader);
	~PointCloudGenerator();
	
	PointCloud::Ptr generatePointCloud(FramePtr frame);
	// 将当前帧坐标系下的点云和并到世界坐标系下的点云中去
	int joinPointCloud(FramePtr frame);
	int savePointCloud();
	
};

#endif
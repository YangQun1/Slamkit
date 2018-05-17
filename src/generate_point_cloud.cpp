/*
 * 根据2D图像和深度图生成点云
 * Author: YangQun
 * Date: 2018/5/9
 * Last Update:2018/5/9
 */
// C++ 标准库
#include <iostream>
#include <string>
#include <vector>

// 第三方库头文件
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

// 工程头文件
#include "generate_point_cloud.h"
#include "param_reader.h"

using namespace std;


PointCloudGenerator::PointCloudGenerator(PinHoleCamera::Ptr cam, ParameterReader::Ptr param_reader)
// viewer_("viewer")	// 初始化点云显示窗口的名字
{
	cam_ = cam;
	is_show_ = param_reader->getParam<bool>("is_show_point_cloud");
	is_save_ = param_reader->getParam<bool>("is_save_point_cloud");
	grid_size_ = param_reader->getParam<double>("voxel_grid");
	is_filter_ = param_reader->getParam<bool>("is_filter");
	
	if (is_filter_) {
		voxel_.setLeafSize(grid_size_, grid_size_, grid_size_);
	}
	
	cloud_out_.reset(new PointCloud);
	
	if (is_show_) {
		viewer_ = new pcl::visualization::CloudViewer("viewer");
	}
	else {
		viewer_ = nullptr;
	}
}

PointCloudGenerator::~PointCloudGenerator()
{
	if (nullptr != viewer_) 
		delete viewer_;
}


PointCloud::Ptr PointCloudGenerator::generatePointCloud(Frame::Ptr frame)
{
    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloud::Ptr cloud ( new PointCloud );
    // 遍历深度图
    for (int m = 0; m <  frame->depth_img_.rows; m++)
        for (int n=0; n < frame->depth_img_.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = frame->depth_img_.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
		continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / cam_->factor();
            p.x = (n - cam_->cx()) * p.z / cam_->fx();
            p.y = (m - cam_->cy()) * p.z / cam_->fy();
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = frame->rgb_img_.ptr<uchar>(m)[n*3];
            p.g = frame->rgb_img_.ptr<uchar>(m)[n*3+1];
            p.r = frame->rgb_img_.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
        
	// 设置点云
	cloud->height = 1;
	cloud->width = cloud->points.size();
	// cout<<"point cloud size = "<<cloud->points.size()<<endl;
	cloud->is_dense = false;
	
// 	if (is_save_) {
// 		pcl::io::savePCDFile( "../data/pointcloud.pcd", *cloud );
// 		cout<<"Point cloud saved."<<endl;
// 	}
	
// 	if (is_show_) {
// 		viewer_->showCloud(cloud);
// 		// while ( ! viewer_->wasStopped()) { }
// 	}
	
	// 清除数据并退出
	// cloud->points.clear();
	
	return cloud;
}

int PointCloudGenerator::joinPointCloud(Frame::Ptr frame)
{
	// 转换成点云
	PointCloud::Ptr curr_cloud = generatePointCloud(frame);
	
	// 合并点云
	PointCloud::Ptr cloud_temp(new PointCloud);
	Eigen::Matrix3d rotation_matrix = frame->T_c2w_.rotation_matrix();
	Eigen::Vector3d translation_vector = frame->T_c2w_.translation();
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	T.rotate(rotation_matrix);
	T.pretranslate(translation_vector);
	pcl::transformPointCloud(*curr_cloud, *cloud_temp, T.matrix());		// 将当前点云变换到世界坐标系下
	*cloud_temp += *cloud_out_;									// 在世界坐标系下将已有点云和当前帧点云进行拼接
	cloud_out_->clear();											// 清空旧的世界点云数据
	
	if (is_filter_) {
		// 拼接之后的点云进行体素滤波降采样
		voxel_.setInputCloud(cloud_temp);
		voxel_.filter(*cloud_out_);
	}
	else {
		cloud_out_ = cloud_temp;
	}
	
	
	// 可视化
	if (is_show_) {
		viewer_->showCloud(cloud_out_);
		// while ( ! viewer.wasStopped()) { }
	}
	
	return 0;
}

int PointCloudGenerator::savePointCloud()
{
	// 保存成pcd文件
	if (is_save_) {
		pcl::io::savePCDFile("../data/joinPointCloud.pcd", *cloud_out_);
	}
	
	return 0;
}







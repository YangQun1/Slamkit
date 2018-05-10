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


PointCloudGenerator::PointCloudGenerator(PinHoleCamera* cam, ParameterReader *param_reader)
{
	cam_ = cam;
	is_show_ = param_reader->getParam<bool>("is_show_point_cloud");
	is_save_ = param_reader->getParam<bool>("is_save_point_cloud");
}

PointCloudGenerator::~PointCloudGenerator()
{

}


PointCloud::Ptr PointCloudGenerator::generatePointCloud(Frame& frame)
{
    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloud::Ptr cloud ( new PointCloud );
    // 遍历深度图
    for (int m = 0; m <  frame.depthImg_.rows; m++)
        for (int n=0; n < frame.depthImg_.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = frame.depthImg_.ptr<ushort>(m)[n];
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
            p.b = frame.rgbImg_.ptr<uchar>(m)[n*3];
            p.g = frame.rgbImg_.ptr<uchar>(m)[n*3+1];
            p.r = frame.rgbImg_.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
        
	// 设置点云
	cloud->height = 1;
	cloud->width = cloud->points.size();
	// cout<<"point cloud size = "<<cloud->points.size()<<endl;
	cloud->is_dense = false;
	
// 	if (is_save) {
// 		pcl::io::savePCDFile( "../data/pointcloud.pcd", *cloud );
// 		cout<<"Point cloud saved."<<endl;
// 	}
	
	// 清除数据并退出
	// cloud->points.clear();
	
	return cloud;
}

int PointCloudGenerator::joinPointCloud(Frame& frame1, Frame& frame2)
{
	// 转换成点云
	PointCloud::Ptr cloud1 = generatePointCloud(frame1);
	PointCloud::Ptr cloud2 = generatePointCloud(frame2);
	
	// 合并点云
	PointCloud::Ptr cloud_out(new PointCloud);
	Sophus::SE3 T_w_f = frame2.T_f_w_.inverse();			// 帧坐标系到世界坐标系的变换关系
	Eigen::Matrix3d rotation_matrix = T_w_f.rotation_matrix();
	Eigen::Vector3d translation_vector = T_w_f.translation();
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	T.rotate(rotation_matrix);
	T.pretranslate(translation_vector);
	pcl::transformPointCloud(*cloud2, *cloud_out, T.matrix());
	*cloud_out += *cloud1;
	
	// 保存成pcd文件
	if (is_save_) {
		pcl::io::savePCDFile("../data/joinPointCloud.pcd", *cloud_out);
	}
	
	// 可视化
	if (is_show_) {
		pcl::visualization::CloudViewer viewer("viewer");
		viewer.showCloud(cloud_out);
		while ( ! viewer.wasStopped()) { }
	}
	
	return 0;
}








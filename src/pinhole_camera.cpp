/*
 */

#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <sophus/se3.h>

#include "pinhole_camera.h"
#include "param_reader.h"

using namespace std;

PinHoleCamera::PinHoleCamera(ParameterReader::Ptr param_reader) :
fx_(param_reader->getParam<double>("camera.fx")),
fy_(param_reader->getParam<double>("camera.fy")),
cx_ (param_reader->getParam<double>("camera.cx")),
cy_ (param_reader->getParam<double>("camera.cx")), 
factor_ (param_reader->getParam<double>("camera.factor")),
skew_ (param_reader->getParam<double>("camera.skew"))
{
	// 初始化内参矩阵
	K_ << fx_, skew_, cx_,   0.0, fy_, cy_,   0.0, 0.0, 1.0;
	K_inv_ = K_.inverse();
}

PinHoleCamera::~PinHoleCamera()
{
	
}

// // 将根据像素点的横纵坐标和深度值恢复其在相机坐标系中的三维坐标
// cv::Point3f PinHoleCamera::point2dTo3d(cv::Point3f& pixel_point_with_depth)
// {
// 	int x = pixel_point_with_depth.x;
// 	int y = pixel_point_with_depth.y;
// 	double d = pixel_point_with_depth.z;
// 	
// 	cv::Point3f p;
// 	p.z = d / factor_;
// 	p.x = (x - cx_) * p.z / fx_;
// 	p.y = (y - cy_) * p.z / fy_;
// 	
// 	return p;
// }


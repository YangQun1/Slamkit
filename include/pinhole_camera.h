/*
 */
#ifndef PINHOLE_CAMERA_H_
#define PINHOLE_CAMERA_H_

#include <iostream>

#include <Eigen/Eigen>

#include <opencv2/core/core.hpp>

#include "param_reader.h"

class PinHoleCamera
{
private:
	// 相机内参
	const double fx_, fy_;
	const double cx_,  cy_;
	const double skew_;
	const double factor_;	// 深度图的缩放因子,例如对于Kinect相机,通常1000个像素代表1m,因此这个值设置为1000
	// 内参矩阵
	Eigen::Matrix3d K_;
	Eigen::Matrix3d K_inv_;
	
public:
	PinHoleCamera(ParameterReader *param_reader);
	~PinHoleCamera();
	
	cv::Point3f point2dTo3d(cv::Point3f& pixel_point_with_depth);

	// 私有成员变量访问接口
	inline double fx() const { return fx_; }
	inline double fy()  const { return fy_; }
	inline double cx() const { return cx_; }
	inline double cy() const { return cy_; }
	inline double skew() const  { return skew_; }
	inline double factor() const { return factor_; }
	inline Eigen::Matrix3d& K() { return K_; }
	inline Eigen::Matrix3d& K_inv() { return K_inv_; }
};

#endif
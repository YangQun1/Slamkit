/*
 */
#ifndef PINHOLE_CAMERA_H_
#define PINHOLE_CAMERA_H_

#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <sophus/se3.h>

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
	
// 	cv::Point3f point2dTo3d(cv::Point3f& pixel_point_with_depth);

	// 私有成员变量访问接口
	double fx() const { return fx_; }
	double fy()  const { return fy_; }
	double cx() const { return cx_; }
	double cy() const { return cy_; }
	double skew() const  { return skew_; }
	double factor() const { return factor_; }
	Eigen::Matrix3d& K() { return K_; }
	Eigen::Matrix3d& K_inv() { return K_inv_; }
	
	Eigen::Vector3d world2camera( const Eigen::Vector3d &p_w, const Sophus::SE3 &T_w2c );
	Eigen::Vector3d camera2world( const Eigen::Vector3d &p_c, const Sophus::SE3 &T_w2c );
	Eigen::Vector2d camera2pixel  ( const Eigen::Vector3d &p_c );
	Eigen::Vector3d pixel2camera  ( const Eigen::Vector2d &p_p, double depth=1 ); 
	Eigen::Vector3d pixel2world     ( const Eigen::Vector2d &p_p, const Sophus::SE3 &T_w2c, double depth=1 );
	Eigen::Vector2d world2pixel     ( const Eigen::Vector3d &p_w, const Sophus::SE3 &T_w2c);
	
};

/* Note:
 * 1. inline关键字是"用于实现的关键字",因此必须放在函数定义前面
 * 2. 类成员函数如果需要被定义成inline,则其实现应该在.h文件中,否则在链接时会出现undefined的错误
 */

 inline Eigen::Vector3d PinHoleCamera::world2camera( const Eigen::Vector3d& p_w, const Sophus::SE3& T_w2c )
{
	return T_w2c * p_w;
}

 inline Eigen::Vector3d PinHoleCamera::camera2world( const Eigen::Vector3d& p_c, const Sophus::SE3& T_w2c )
{
	return T_w2c.inverse() * p_c;
}

 inline Eigen::Vector2d PinHoleCamera::camera2pixel( const Eigen::Vector3d& p_c )
{
	return Eigen::Vector2d(
		fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
		fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
	);
}

// 此处的depth是深度图的值,需要根据factor_转换成标准距离单位
inline Eigen:: Vector3d PinHoleCamera::pixel2camera( const Eigen::Vector2d& p_p, double depth)
{
	double d = depth / factor_;
	return Eigen::Vector3d (
		( p_p ( 0,0 )-cx_ ) *d/fx_,
		( p_p ( 1,0 )-cy_ ) *d/fy_,
		d
	);
}

inline Eigen:: Vector3d PinHoleCamera::pixel2world( const Eigen::Vector2d& p_p, const Sophus::SE3& T_w2c, double depth)
{
	return camera2world ( pixel2camera ( p_p, depth ), T_w2c );
}


 inline Eigen::Vector2d PinHoleCamera::world2pixel( const Eigen::Vector3d& p_w, const Sophus::SE3& T_w2c)
{
	return camera2pixel( world2camera ( p_w, T_w2c ) );
}


#endif
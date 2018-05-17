/*
 */

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sophus/so3.h>
#include <sophus/se3.h>

#include "frame.h"

unsigned long Frame::factory_id_ = 0;

Frame::Frame(unsigned long id, PinHoleCamera::Ptr cam, const cv::Mat& rgb_img, const cv::Mat& depth_img)
{
	cam_ = cam;
	rgb_img_ = rgb_img;
	depth_img_ = depth_img;
}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame(PinHoleCamera::Ptr cam, const cv::Mat& rgb_img, const cv::Mat& depth_img)
{
	return Frame::Ptr( new Frame(factory_id_++, cam, rgb_img, depth_img) );
}

double Frame::findDepth(const cv::KeyPoint& kp)
{
	int x = cvRound(kp.pt.x);
	int y = cvRound(kp.pt.y);
	
	ushort d = depth_img_.ptr<ushort>(y)[x];
	
	if ( d != 0 ) {
		// return double(d) / cam_->factor();
		return double(d);
	}
	else  {
		// check the nearby points 
		int dx[4] = {-1,0,1,0};
		int dy[4] = {0,-1,0,1};
		for ( int i = 0; i < 4; i++ ) {
			d = depth_img_.ptr<ushort>( y+dy[i] )[x+dx[i]];
			if ( d != 0 ) {
				// return double(d) / cam_->factor();
				return double(d);
			}
		}
	}
	return -1.0;
}

Eigen::Vector3d Frame::getCameraCenter() const
{
	// 即相机坐标系的(0,0,0)点变换到世界坐标系下的坐标
	return T_c2w_.translation();
}

bool Frame::isInFrame(const Eigen::Vector3d& pt_world)
{
	Eigen::Vector3d pt_cam = cam_->world2camera(pt_world, T_c2w_.inverse());
	
	// 在像平面后面,不能成像
	if (pt_cam(2,0) < 0) {
		return false;
	}
	
	Eigen::Vector2d pt_img = cam_->camera2pixel(pt_cam);
	
	// 是否超出传感器范围
	if ( (pt_img(0,0) < 0) && (pt_img(0,0) > rgb_img_.cols) &&
		(pt_img(1,0) < 0) && (pt_img(1,0) > rgb_img_.rows) ) {
		return false;
	}
	
	return true;
}


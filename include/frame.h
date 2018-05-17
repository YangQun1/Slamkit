/*
 */

#ifndef FRAME_H
#define FRAME_H

#include <string>
#include <vector>
#include <memory>

#include <boost/shared_ptr.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Eigen>
#include <sophus/so3.h>
#include <sophus/se3.h>

#include "pinhole_camera.h"

using namespace std;


class Frame
{
public:
	typedef boost::shared_ptr<Frame> Ptr;
	
	Frame(unsigned long id,  PinHoleCamera::Ptr cam, const cv::Mat& rgb_img, const cv::Mat& depth_img);
	~Frame();
	
	static unsigned long factory_id_;		// 静态成员变量,使用工厂设计模式,用作接口,避免人为显式地指定id_
	unsigned long id_;					// 帧的id号
	
	PinHoleCamera::Ptr cam_;
	
	cv::Mat rgb_img_, depth_img_;		// 彩图与深度图
	
	vector<cv::KeyPoint> keypoints_;		// 关键点
	cv::Mat descrip_;					// 描述子
// 	vector<int> matched_kp_index_;		// 在地图中找到正确匹配点(满足几何关系)的特征点的索引
	
	Sophus::SE3 T_c2w_;				// 当前帧坐标系到世界坐标系的变换
	
	static Frame::Ptr createFrame(PinHoleCamera::Ptr cam, const cv::Mat& rgb_img, const cv::Mat& depth_img);	// static声明,可以不实例化对象,直接调用类成员函数,否则会报错
	
	double findDepth(const cv::KeyPoint& kp);
	Eigen::Vector3d getCameraCenter() const;
	bool isInFrame(const Eigen::Vector3d& pt_world);
};

#endif
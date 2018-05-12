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

// Frame结构体,用于存储一帧图像的数据
// typedef struct _Frame
// {
// 	cv::Mat rgbImg_, depthImg_;			// 彩图与深度图
// 	vector<cv::KeyPoint> keypoints;		// 关键点
// 	cv::Mat descrip;					// 描述子
// 	
// 	Sophus::SE3 T_f_w_;				// 从世界坐标系到该帧下的相机坐标系之间的变换矩阵
// } Frame;


 class Frame
{
public:
	
	cv::Mat rgb_img_, depth_img_;			// 彩图与深度图
	
	vector<cv::KeyPoint> keypoints_;		// 关键点
	cv::Mat descrip_;					// 描述子
	
	Sophus::SE3 T_c2w_;				// 当前帧坐标系到世界坐标系的变换
	
	Frame(PinHoleCamera *cam, const cv::Mat& rgb_img, const cv::Mat& depth_img);
	~Frame();
	
private:
	PinHoleCamera *cam_;
};

typedef boost::shared_ptr<Frame> FramePtr;
#endif
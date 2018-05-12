/*
 */

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sophus/so3.h>
#include <sophus/se3.h>

#include "frame.h"

Frame::Frame(PinHoleCamera* cam, const cv::Mat& rgb_img, const cv::Mat& depth_img)
{
	cam_ = cam;
	rgb_img_ = rgb_img;
	depth_img_ = depth_img;
}

Frame::~Frame()
{

}


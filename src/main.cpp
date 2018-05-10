#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Eigen>
#include <sophus/se3.h>

// user
#include "pinhole_camera.h"
#include "generate_point_cloud.h"
#include "frame.h"
#include "detect_features.h"
#include "align_image.h"
#include "param_reader.h"

using namespace std;

int main(int argc, char**argv)
{
	int status;
	Frame frame1, frame2;
	
	// 实例化
	ParameterReader param_reader("./param.txt");
	
	PinHoleCamera cam( &param_reader );
	DetectFeatures detector(&param_reader);
	AlignImage align_image(&cam,&param_reader);
	PointCloudGenerator point_cloud_generator(&cam, &param_reader);
	// 读取图像
	frame1.rgbImg_ = cv::imread( "../data/rgb1.png" );
	frame1.depthImg_ = cv::imread( "../data/depth1.png", -1 );
	frame2.rgbImg_ = cv::imread( "../data/rgb2.png" );
	frame2.depthImg_ = cv::imread( "../data/depth2.png", -1 );
	
	// 设置第一帧的坐标系为世界坐标系
	frame1.T_f_w_ = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
	
	// 检测特征
	detector.detectFeaturePoints(frame1);
	detector.detectFeaturePoints(frame2);
	
	// 匹配,计算变换关系
	align_image.alignImage(frame1, frame2);
	
	// 合并点云并显示
 	point_cloud_generator.joinPointCloud(frame1, frame2);
	
	return 0;
}
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
#include "image_reader.h"

using namespace std;

int main(int argc, char**argv)
{
	int start_index, end_index;
	
	ParameterReader param_reader("./param.txt");
	ImageReader image_reader(&param_reader);
	PinHoleCamera cam( &param_reader );
	DetectFeatures detector(&param_reader);
	AlignImage align_image(&cam,&param_reader);
	PointCloudGenerator point_cloud_generator(&cam, &param_reader);
	
	start_index = param_reader.getParam<int>("start_index");
	end_index = param_reader.getParam<int>("end_index");
	
	// 构造第一帧
	FramePtr ref_frame, curr_frame;
	image_reader.readImage(start_index);
	ref_frame.reset( new Frame( &cam, image_reader.getRGBImg().clone(), image_reader.getDepthImg().clone() ) );
	ref_frame->T_c2w_ = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());	// 设置第一帧的坐标系到世界坐标系的变换为单位阵
	
	// 提取第一帧的特征
	detector.detectFeaturePoints(ref_frame);
	// 生成第一帧的点云
	point_cloud_generator.joinPointCloud(ref_frame);
	
	// 循环处理其他帧
	for(int index = start_index + 1; index <= end_index; index++) {
		// 构造当前帧
		image_reader.readImage(index);
		curr_frame.reset( new Frame( &cam, image_reader.getRGBImg().clone(), image_reader.getDepthImg().clone() ) ); 
		
		// 提取当前帧的特征
		detector.detectFeaturePoints(curr_frame);
		
		// 匹配,计算变换关系
		align_image.alignImage(ref_frame, curr_frame);
		
		if (true == align_image.checkAlignQuality()) {
			// 合并点云并显示
			point_cloud_generator.joinPointCloud(curr_frame);
			ref_frame = curr_frame;
		}
		curr_frame.reset();
	}
	
	// 保存最终的点云文件
	point_cloud_generator.savePointCloud();
	
	return 0;
}
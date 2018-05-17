
#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include <string>
#include <boost/shared_ptr.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "param_reader.h"
#include "pinhole_camera.h"
#include "detect_features.h"
#include "align_image.h"
#include "generate_point_cloud.h"
#include "map_point.h"
#include "map.h"
#include "image_reader.h"

class VisualOdometry 
{
public:
	typedef boost::shared_ptr<VisualOdometry> Ptr;
	enum VO_STATE {					// VO的状态变量,有待继续完善
		INITIALIZING = 1,
		TRACKING = 2,
		LOST = 3
	};
	
	VisualOdometry(ParameterReader::Ptr param_reader);
	~VisualOdometry();
	
	VO_STATE state_;
	
	Map::Ptr local_map_;				// 局部地图,用于tracking
	
	Frame::Ptr curr_frame_;			// 当前帧
	Frame::Ptr ref_frame_;				// 相邻帧
	Frame::Ptr ref_keyframe_;			// 上一关键帧
	
	// bool addFrame(Frame::Ptr frame);	
	bool addImage(cv::Mat& rgb_img, cv::Mat& depth_img);
	
private:
// 	ParameterReader::Ptr param_reader_;
// 	ImageReader::Ptr image_reader_;
	PinHoleCamera::Ptr cam_;
	DetectFeatures::Ptr detector_;
	AlignImage::Ptr align_image_;
	PointCloudGenerator::Ptr point_cloud_generator_;
	
	double min_keyframe_rotate_;			// 若将当前帧相对上一帧的旋转大于该值,则将其判为关键帧
	double min_keyframe_translate_;		// 若将当前帧相对上一帧的平移大于该值,则将其判为关键帧
	double map_point_erase_ratio_;
	int max_num_lost_;     					 // max number of continuous lost times
	
	int num_lost_;           // number of lost times
	 
	// void addKeyFrame();		// 将curr_frame_添加到map去
	void addMapPoint();		// 将curr_frame_中的特征点添加变换成世界坐标,并添加到map中
	bool checkKeyFrame();		// 判断当前帧能否作为一个关键帧
	bool checkEstimatedPose();
	void optimizeMap();
	double getViewAngle ( Frame::Ptr frame, MapPoint::Ptr map_point );
	
};	


#endif
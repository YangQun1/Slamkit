/*
 * 
 */

#ifndef DETECT_FEATURES_H
#define DETECT_FEATURES_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "frame.h"
#include "param_reader.h"

using namespace std;

// TODO:暂时先实现ORB特征的提取,以后增加更多的特征
// TODO:使用栅格,使得提取到的特征点分布更加均匀
class DetectFeatures 
{
public:
	typedef boost::shared_ptr<DetectFeatures> Ptr;
	
private:
// 	cv::Ptr<cv::FeatureDetector> keypoint_detector_;
// 	cv::Ptr<cv::DescriptorExtractor> descrip_extractor_;
	cv::Ptr<cv::Feature2D> feature_detector_;
	string detector_name_;
	int feature_num_;
	bool is_show_;
	
public:
	DetectFeatures(ParameterReader::Ptr param_reader);
	~DetectFeatures();
	
	void detectFeaturePoints(Frame::Ptr frame);
};

#endif
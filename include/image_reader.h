/*
 * 
 */

#ifndef IMAGE_READER_H
#define IMAGE_READER_H

#include <string>
#include <boost/shared_ptr.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "param_reader.h"

using namespace std;

class ImageReader 
{
public:
	typedef boost::shared_ptr<ImageReader> Ptr;
	
	ImageReader(ParameterReader::Ptr param_reader);
	~ImageReader();
	
	bool readImage(int index);	// 根据图片的序号读取文件
	
	// 类内定义的,默认为inline函数
	 cv::Mat& getRGBImg() { return rgb_img_; }
	 cv::Mat& getDepthImg() { return depth_img_; }
	
private:
	string rgb_dir_ , depth_dir_ ;	// 文件路径
	string extension_;	// 文件拓展名
	
	cv::Mat rgb_img_ , depth_img_ ;	
};

#endif

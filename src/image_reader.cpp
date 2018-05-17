/*
 * 
 */


#include <string>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "param_reader.h"
#include "image_reader.h"

using namespace std;

ImageReader::ImageReader(ParameterReader::Ptr param_reader)
{
	rgb_dir_ = param_reader->getParam<string>("rgb_dir");
	depth_dir_ = param_reader->getParam<string>("depth_dir");
	extension_ = param_reader->getParam<string>("extension");
}

ImageReader::~ImageReader()
{

}


bool ImageReader::readImage(int index)
{
	string rgb_filename;
	stringstream rgb_ss;
	rgb_ss<<rgb_dir_<<index<<extension_;
	rgb_ss>>rgb_filename;
	
	string depth_filename;
	stringstream depth_ss;
	depth_ss<<depth_dir_<<index<<extension_;
	depth_ss>>depth_filename;
	
	rgb_img_ = cv::imread( rgb_filename );
	depth_img_ = cv::imread(depth_filename, cv::IMREAD_UNCHANGED);	// 注意:这个地方一定要指定读取原始数据的参数
	
	return true;
}










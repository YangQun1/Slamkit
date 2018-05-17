
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
#include "visual_odometry.h"

using namespace std;

int main(int argc, char**argv)
{
	int start_index, end_index;
	
	ParameterReader::Ptr param_reader( new ParameterReader("./param.txt") );
	ImageReader image_reader(param_reader);
	VisualOdometry visual_odometry(param_reader);
	
	start_index = param_reader->getParam<int>("start_index");
	end_index = param_reader->getParam<int>("end_index");
	
	
	// 循环处理其他帧
	for(int index = start_index; index <= end_index; index++) {
		cout << "***********loop " << index-start_index +1<< " ***********index " << index << " *************" << endl;
		image_reader.readImage(index);
		visual_odometry.addImage(image_reader.getRGBImg(), image_reader.getDepthImg());
	}
	
	// 保存最终的点云文件
// 	point_cloud_generator.savePointCloud();
	
	return 0;
}
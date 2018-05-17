#ifndef MAP_H
#define MAP_H

#include <boost/shared_ptr.hpp>
#include <unordered_map>
#include <map>	// stl map

#include "map_point.h"
#include "frame.h"

class Map
{
public:
	Map(){};
	~Map(){};
	
	typedef boost::shared_ptr<Map> Ptr;
	
	std::unordered_map<unsigned long, MapPoint::Ptr> map_points_;		// 地图点的集合
	std::unordered_map<unsigned long, Frame::Ptr> keyframes_;			// 关键帧的集合
	
	void insertMapPoint(MapPoint::Ptr map_point);
	void insertKeyFrame(Frame::Ptr keyframe);
};

#endif


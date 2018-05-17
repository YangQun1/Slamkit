#include <boost/shared_ptr.hpp>
#include <map>	// stl map

#include "map_point.h"
#include "frame.h"
#include "map.h"

void Map::insertMapPoint(MapPoint::Ptr map_point)
{
	if (map_points_.find(map_point->id_) == map_points_.end()) {
		map_points_.insert( make_pair(map_point->id_, map_point));
	}
	else {
		map_points_[map_point->id_] = map_point;
	}
	
	return;
}

void Map::insertKeyFrame(Frame::Ptr keyframe)
{
	if (keyframes_.find(keyframe->id_) == keyframes_.end()) {
		keyframes_.insert( make_pair(keyframe->id_, keyframe));
	}
	else {
		keyframes_[keyframe->id_] = keyframe;
	}
	
	return;
}



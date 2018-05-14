/*
 * 自定义的g2o的节点和边类型
 */

#include "g2o_types_costom.h"


void EdgeProjectXYZ2UVUPoseOnly::computeError()
{
	const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
	
	_error = _measurement - cam_->camera2pixel(pose->estimate().map(point_));
}

/*
 * 自定义的g2o的节点和边类型
 */

#ifndef G2O_TYPES_COSTOM_H
#define G2O_TYPES_COSTOM_H


#include <Eigen/Eigen>

// #include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

// #include <g2o/core/block_solver.h>
// // #include <g2o/core/robust_kernel.h>
// // #include <g2o/core/robust_kernel_impl.h>
// #include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/solvers/dense/linear_solver_dense.h>

#include "pinhole_camera.h"

/*
 * D = 2,代表误差的维度是2维的
 * E = Vector2d,代表误差数据类型
 * Vertex = VertexSE3Expmap,代表的是边连接的节点的类型
 */
class  EdgeProjectXYZ2UVUPoseOnly : public  g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
public:
	// 这两个函数可以不进行重写,只声明一下就可以
	virtual bool read(std::istream& is){};
	virtual bool write(std::ostream& os) const{};
	
	// 误差计算函数
	virtual void computeError();

	//  virtual void linearizeOplus();
	
	Eigen::Vector3d point_;			// 目标点, 用于计算在当前位姿下的投影误差

	PinHoleCamera::Ptr  cam_;		// 相机模型,用于计算投影
};

#endif



#include "common_includes.hpp"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"


g2o::SE3Quat eigenToSE3quat(const Eigen::Matrix4d &T)
{
   /* Convertion from Homogenous Transformation Matrix in Eigen to g2o Quaterion, because g2o is stupid */
   Eigen::Matrix3d R; // Rotation matrix 
   Eigen::Vector3d t; //translation vector 
   R = T.block<3,3>(0,0); 
   t = T.col(3); 
   //not sure if it works //!add tests 
   return g2o::SE3Quat(R, t); //http://docs.ros.org/en/fuerte/api/re_vision/html/classg2o_1_1SE3Quat.html 
}

class Pose3DVertex : public g2o::BaseVertex<6, Eigen::Matrix4d>
{
public: 
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

};

class VectorVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public: 
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}; 

class PoseProjectionEdge : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, Pose3DVertex>
{
public: 
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

   PoseProjectionEdge(const Eigen::Vector3d &position, Eigen::Matrix3d camera_matrix) 
   : position_in_3D(position), K(camera_matrix)
   {}

   virtual void computeError() override
   {
      const Pose3DVertex* vertex = static_cast<Pose3DVertex*>(_vertices[0]); //_verticies - g2o variable
      Eigen::Matrix4d T = vertex->estimate(); 
      Eigen::Vector3d homogenous_pixel_pos = K*(T*position_in_3D); // do point projection to img 
      homogenous_pixel_pos = homogenous_pixel_pos/homogenous_pixel_pos[2]; // convert to non homogenous by dividing by w/scale factor
      Eigen::Vector2d pixel_position = homogenous_pixel_pos.head<2>(); 
      _error = _measurement - homogenous_pixel_pos
   }

private: 
   Eigen::Vector3d position_in_3D; 
   Eigen::Matrix3d K; 

}; 
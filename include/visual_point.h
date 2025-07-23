/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_POINT_H_
#define LIVO_POINT_H_

#include <boost/noncopyable.hpp>
#include "common_lib.h"
#include "frame.h"

class Feature;

/// A visual map point on the surface of the scene.
class VisualPoint : boost::noncopyable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Vector3d pos_;                //!< 3d pos of the point in the world coordinate frame.
  // Vector3d normal_;             //!< Surface normal at point.
  // Matrix3d normal_information_; //!< Inverse covariance matrix of normal estimation.
  // Vector3d previous_normal_;    //!< Last updated normal vector.
  // list<Feature *> obs_;         //!< Reference patches which observe the point.
  // Eigen::Matrix3d covariance_;  //!< Covariance of the point.
  // bool is_converged_;           //!< True if the point is converged.
  // bool is_normal_initialized_;  //!< True if the normal is initialized.
  // bool has_ref_patch_;          //!< True if the point has a reference patch.
  // Feature *ref_patch;           //!< Reference patch of the point.

  Vector3d pos_;                //!< 点在世界坐标系中的三维位置
  Vector3d normal_;             //!< 点的表面法向量
  Matrix3d normal_information_; //!< 法向估计的逆协方差矩阵（信息矩阵）
  Vector3d previous_normal_;    //!< 上一次更新的法向量
  list<Feature *> obs_;         //!< 观测到该点的参考图像特征列表
  Eigen::Matrix3d covariance_;  //!< 该点的协方差矩阵
  bool is_converged_;           //!< 若该点已收敛，则为真
  bool is_normal_initialized_;  //!< 若法向已初始化，则为真
  bool has_ref_patch_;          //!< 若该点已有参考图像patch，则为真
  Feature *ref_patch;           //!< 该点的参考图像patch指针


  VisualPoint(const Vector3d &pos);
  ~VisualPoint();
  void findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const;
  void deleteNonRefPatchFeatures();
  void deleteFeatureRef(Feature *ftr);
  void addFrameRef(Feature *ftr);
  bool getCloseViewObs(const Vector3d &pos, Feature *&obs, const Vector2d &cur_px) const;
};

#endif // LIVO_POINT_H_

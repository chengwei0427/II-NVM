#pragma once
// c++
#include <iostream>

// eigen
#include <Eigen/Core>

// ceres
#include <ceres/ceres.h>

// utility
#include "lio/lio_utils.h"

namespace zjloc
{
     struct PointToPlaneFunctor
     {
          static constexpr int NumResiduals() { return 1; }

          PointToPlaneFunctor(const Eigen::Vector3d &reference,
                              const Eigen::Vector3d &target,
                              const Eigen::Vector3d &reference_normal,
                              double weight = 1.0) : reference_(reference),
                                                     target_(target),
                                                     reference_normal_(reference_normal),
                                                     weight_(weight) {}

          template <typename T>
          bool operator()(const T *const trans_params, const T *const rot_params, T *residual) const
          {
               Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T *>(rot_params));
               Eigen::Matrix<T, 3, 1> target_temp(T(target_(0, 0)), T(target_(1, 0)), T(target_(2, 0)));
               Eigen::Matrix<T, 3, 1> transformed = quat * target_temp;
               transformed(0, 0) += trans_params[0];
               transformed(1, 0) += trans_params[1];
               transformed(2, 0) += trans_params[2];

               Eigen::Matrix<T, 3, 1> reference_temp(T(reference_(0, 0)), T(reference_(1, 0)), T(reference_(2, 0)));
               Eigen::Matrix<T, 3, 1> reference_normal_temp(T(reference_normal_(0, 0)), T(reference_normal_(1, 0)), T(reference_normal_(2, 0)));

               residual[0] = T(weight_) * (transformed - reference_temp).transpose() * reference_normal_temp;
               return true;
          }

          static ceres::CostFunction *Create(const Eigen::Vector3d &point_world_,
                                             const Eigen::Vector3d &point_body_,
                                             const Eigen::Vector3d &norm_vector_,
                                             double weight_ = 1.0)
          {
               return (new ceres::AutoDiffCostFunction<PointToPlaneFunctor, 1, 3, 4>(
                   new PointToPlaneFunctor(point_world_, point_body_, norm_vector_, weight_)));
          }

          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          Eigen::Vector3d reference_;
          Eigen::Vector3d target_;
          Eigen::Vector3d reference_normal_;
          double weight_ = 1.0;
     };

}
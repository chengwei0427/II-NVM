#ifndef LOAM_LIO_UTILS_H_
#define LOAM_LIO_UTILS_H_

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/math_utils.h"
#include "common/nav_state.h"

#include "common/cloudMap.hpp"

namespace zjloc
{
     struct Neighborhood
     {
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          Eigen::Vector3d center = Eigen::Vector3d::Zero();
          Eigen::Vector3d normal = Eigen::Vector3d::Zero();
          Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
          double a2D = 1.0; // Planarity coefficient
     };

     struct MeasureGroup
     {
          double lidar_begin_time_ = 0; // 雷达包的起始时间
          double lidar_end_time_ = 0;   // 雷达的终止时间
          std::vector<point3D> lidar_;  // 雷达点云
          std::deque<IMUPtr> imu_;      // 上一时时刻到现在的IMU读数
     };

     class state
     {
     public:
          Eigen::Quaterniond rotation;
          Eigen::Vector3d translation;
          Eigen::Vector3d velocity;
          Eigen::Vector3d ba;
          Eigen::Vector3d bg;

          state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_,
                const Eigen::Vector3d &velocity_, const Eigen::Vector3d &ba_,
                const Eigen::Vector3d &bg_);

          state(const state *state_temp, bool copy = false);

          void release();
     };

     class cloudFrame
     {
     public:
          double time_frame_begin; //  current frame front stamp
          double time_frame_end;   //    next frame front stamp

          int frame_id;

          state *p_state;

          std::vector<point3D> point_surf; //  global frame
          std::vector<point3D> const_surf; //  lidar frame

          std::vector<point3D> surf_keypoints;

          bool success;

          cloudFrame(std::vector<point3D> &point_surf_, std::vector<point3D> &const_surf_,
                     state *p_state_);

          cloudFrame(cloudFrame *p_cloud_frame);

          void release();
     };
}
#endif // LIO_UTILS_H_
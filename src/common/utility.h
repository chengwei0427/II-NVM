#ifndef UTILITY_H_
#define UTILITY_H_
// c++
#include <iostream>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tr1/unordered_map>

#include "cloudMap.hpp"

#include "sophus/so3.hpp"

namespace zjloc::global
{
     extern bool FLAG_EXIT; // 退出程序标志
}

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b);

void subSampleFrame2(std::vector<point3D> &frame, double size_voxel);

void subSampleFrame(std::vector<point3D> &frame, double size_voxel);

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling);

void transformPoint(point3D &point_temp, Eigen::Quaterniond &q_end, Eigen::Vector3d &t_end, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

Eigen::Matrix3d g2R(const Eigen::Vector3d &g);

/// @brief 点云数据下采样，采用自定义方式
/// @param frame input
/// @param keypoints    output
/// @param size_voxel_subsampling   downsample resolution
void gridSampling0(const pcl::PointCloud<pcl::PointXYZI>::Ptr &frame, pcl::PointCloud<pcl::PointXYZI>::Ptr &keypoints, double size_voxel_subsampling);

namespace std
{
     template <typename T, typename... Args>
     std::unique_ptr<T> make_unique(Args &&...args)
     {
          return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
     }
}
#endif
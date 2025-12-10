/*
 * @Author: chengwei zhao
 * @LastEditors: cc
 * @Data:
 */
#ifndef UTILITY_H_
#define UTILITY_H_
// c++
#include <iostream>
#include <string>
#include <tr1/unordered_map>

#include "cloudMap.hpp"

#include "sophus/so3.hpp"

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b);

void subSampleFrame(std::vector<point3D> &frame, double size_voxel);

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling);

Eigen::Matrix3d g2R(const Eigen::Vector3d &g);

namespace std
{
     template <typename T, typename... Args>
     std::unique_ptr<T> make_unique(Args &&...args)
     {
          return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
     }
}
#endif
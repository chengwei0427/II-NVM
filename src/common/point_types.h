//
// Created by xiang on 2021/8/18.
//

#ifndef SLAM_IN_AUTO_DRIVING_POINT_TYPES_H
#define SLAM_IN_AUTO_DRIVING_POINT_TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>

#include "common/eigen_types.h"

namespace zjloc
{

    // 定义系统中用到的点和点云类型
    using PointType = pcl::PointXYZI;
    using PointCloudType = pcl::PointCloud<PointType>;
    using CloudPtr = PointCloudType::Ptr;
    using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using IndexVec = std::vector<int>;

    // 点云到Eigen的常用的转换函数
    inline Vec3f ToVec3f(const PointType &pt) { return pt.getVector3fMap(); }
    inline Vec3d ToVec3d(const PointType &pt) { return pt.getVector3fMap().cast<double>(); }

    // 模板类型转换函数
    template <typename T, int dim>
    inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType &pt);

    template <>
    inline Eigen::Matrix<float, 2, 1> ToEigen<float, 2>(const PointType &pt)
    {
        return Vec2f(pt.x, pt.y);
    }

    template <>
    inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3>(const PointType &pt)
    {
        return Vec3f(pt.x, pt.y, pt.z);
    }

    template <typename S>
    inline PointType ToPointType(const Eigen::Matrix<S, 3, 1> &pt)
    {
        PointType p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
        return p;
    }

} // namespace zjloc

namespace velodyne_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        float time;
        std::uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace velodyne_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                      (float, time, time)(std::uint16_t, ring, ring))


namespace robosense_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        // float intensity;
        uint8_t intensity;
        uint16_t ring = 0;
        double timestamp = 0;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (uint8_t, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

namespace ouster_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t ambient;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                      (std::uint32_t, t, t)
                                      (std::uint16_t, reflectivity, reflectivity)
                                      (std::uint8_t, ring, ring)
                                      (std::uint16_t, ambient, ambient)
                                      (std::uint32_t, range, range)
)
// clang-format on

namespace livox_ros
{
    struct Point
    {
        PCL_ADD_POINT4D;
        PCL_ADD_INTENSITY; // 官方将 intensity 注释为 reflectivity
        std::uint8_t tag;
        std::uint8_t line;
        double timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint8_t, tag, tag)(std::uint8_t, line, line)(double, timestamp, timestamp))
#endif // SLAM_IN_AUTO_DRIVING_POINT_TYPES_H

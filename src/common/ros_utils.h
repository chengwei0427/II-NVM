//
// Created by xiang on 25-3-24.
//

#ifndef LIGHTNING_ROS_UTILS_H
#define LIGHTNING_ROS_UTILS_H

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

namespace zjloc
{

    inline double ToSec(const builtin_interfaces::msg::Time &time) { return double(time.sec) + 1e-9 * time.nanosec; }
    inline uint64_t ToNanoSec(const builtin_interfaces::msg::Time &time) { return time.sec * 1e9 + time.nanosec; }

    inline double ros_time_sec(const builtin_interfaces::msg::Time &time)
    {
        return rclcpp::Time(time).seconds();
    }

    inline rclcpp::Time get_ros_time(double timestamp)
    {
        int32_t sec = std::floor(timestamp);
        auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
        uint32_t nanosec = nanosec_d;
        return rclcpp::Time(sec, nanosec);
    }
} // namespace lightning

#endif // LIGHTNING_ROS_UTILS_H

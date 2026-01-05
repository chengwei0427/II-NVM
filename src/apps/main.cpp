/*
 * @Author: chengwei zhao
 * @LastEditors: cc
 * @Data:
 */

// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>

//  ros2 lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <random>

#include "common/utility.h"
#include "common/cloud_convert/cloud_convert.h"
#include "lio/lidarodom.h"

#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/" + name))

class II_NVM_Node : public rclcpp::Node
{
public:
    II_NVM_Node() : Node("II_NVM_node"),
                    tf_broadcaster_(this)
    {
        this->declare_parameter<std::string>("config_file_path", "./config/");
        std::string config_file_path = this->get_parameter("config_file_path").as_string();
        RCLCPP_INFO(this->get_logger(), "config file path: %s", config_file_path.c_str());

        config_file = config_file_path /* + "mapping.yaml"*/;
        std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;

        lio = new zjloc::lidarodom();
        if (!lio->init(config_file))
        {
            RCLCPP_INFO(this->get_logger(), "Init Failed.");
            return;
        }
        std::cout << ANSI_COLOR_GREEN_BOLD << "LIO init successful" << ANSI_COLOR_RESET << std::endl;

        convert = new zjloc::CloudConvert;
        convert->LoadFromYAML(config_file);
        std::cout << ANSI_COLOR_GREEN_BOLD << "CloudConvert init successful" << ANSI_COLOR_RESET << std::endl;

        initROS();

        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(30));
        // timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&zjloc::lidarodom::loop, lio));
        run_thread = std::thread(&zjloc::lidarodom::run, lio);

        std::cout << ANSI_COLOR_GREEN_BOLD << "init successful" << ANSI_COLOR_RESET << std::endl;
    }

private:
    ~II_NVM_Node()
    {
        run_thread.join();
    }

    void initROS()
    {
        pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan", 10);
        laser_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/laser_odom", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/odometry_path", 10);

        auto cloud_pub_func = std::function<bool(std::string & topic_name, zjloc::CloudPtr & cloud, double time)>(
            [&](std::string &topic_name, zjloc::CloudPtr &cloud, double time)
            {
                sensor_msgs::msg::PointCloud2 cloud_msg;
                pcl::toROSMsg(*cloud, cloud_msg);

                cloud_msg.header.stamp = zjloc::get_ros_time(time);
                cloud_msg.header.frame_id = "map";
                if (topic_name == "laser")
                    pub_scan_->publish(cloud_msg);

                return true;
            });

        auto pose_pub_func = std::function<bool(std::string & topic_name, SE3 & pose, double stamp)>(
            [&](std::string &topic_name, SE3 &pose, double stamp)
            {
                geometry_msgs::msg::TransformStamped transform;
                transform.header.stamp = zjloc::get_ros_time(stamp);

                Eigen::Quaterniond q_current(pose.so3().matrix());
                transform.transform.translation.x = pose.translation().x();
                transform.transform.translation.y = pose.translation().y();
                transform.transform.translation.z = pose.translation().z();
                transform.transform.rotation.x = q_current.x();
                transform.transform.rotation.y = q_current.y();
                transform.transform.rotation.z = q_current.z();
                transform.transform.rotation.w = q_current.w();

                if (topic_name == "laser")
                {
                    transform.header.frame_id = "map";
                    transform.child_frame_id = "base_link";
                    tf_broadcaster_.sendTransform(transform);

                    // 发布里程计
                    nav_msgs::msg::Odometry laser_odometry;

                    laser_odometry.header.frame_id = "map";
                    laser_odometry.child_frame_id = "base_link";
                    laser_odometry.header.stamp = zjloc::get_ros_time(stamp);

                    laser_odometry.pose.pose.orientation.x = q_current.x();
                    laser_odometry.pose.pose.orientation.y = q_current.y();
                    laser_odometry.pose.pose.orientation.z = q_current.z();
                    laser_odometry.pose.pose.orientation.w = q_current.w();
                    laser_odometry.pose.pose.position.x = pose.translation().x();
                    laser_odometry.pose.pose.position.y = pose.translation().y();
                    laser_odometry.pose.pose.position.z = pose.translation().z();

                    laser_odom_pub_->publish(std::move(laser_odometry));

                    // 发布路径
                    geometry_msgs::msg::PoseStamped laser_pose;
                    laser_pose.header = laser_odometry.header;
                    laser_pose.pose = laser_odometry.pose.pose;

                    laser_odo_path_.header.stamp = laser_odometry.header.stamp;
                    laser_odo_path_.header.frame_id = "map";
                    laser_odo_path_.poses.push_back(laser_pose);

                    path_pub_->publish(laser_odo_path_);
                }
                else if (topic_name == "world")
                {
                    transform.header.frame_id = "world";
                    transform.child_frame_id = "map";
                    tf_broadcaster_.sendTransform(transform);
                }

                return true;
            });

        lio->setFunc(cloud_pub_func);
        lio->setFunc(pose_pub_func);

        auto yaml = YAML::LoadFile(config_file);
        std::string laser_topic = yaml["common"]["lid_topic"].as<std::string>();
        std::string imu_topic = yaml["common"]["imu_topic"].as<std::string>();
        g_gravity = yaml["common"]["gravity"].as<double>();

        rclcpp::QoS qos(100);
        if (convert->lidar_type_ == zjloc::LidarType::AVIA)
            livox_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                laser_topic, qos,
                std::bind(&II_NVM_Node::livox_pcl_cbk, this, std::placeholders::_1));
        else
            standard_pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                laser_topic, qos,
                std::bind(&II_NVM_Node::standard_pcl_cbk, this, std::placeholders::_1));

        rclcpp::QoS qos_imu(500);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, qos_imu,
            std::bind(&II_NVM_Node::imuHandler, this, std::placeholders::_1));
    }

    void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {

        std::vector<point3D> cloud_out;
        zjloc::common::Timer::Evaluate([&]()
                                       { convert->Process(msg, cloud_out); },
                                       "laser convert");

        zjloc::common::Timer::Evaluate([&]()
                                       {  
        double sample_size = lio->getIndex() < 20 ? 0.01 : 0.02;
        std::mt19937_64 g;
        std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        subSampleFrame(cloud_out, sample_size);
        std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                       "laser ds");

        lio->pushData(cloud_out, std::make_pair(zjloc::ToSec(msg->header.stamp), convert->getTimeSpan()));
    }

    void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::vector<point3D> cloud_out;
        zjloc::common::Timer::Evaluate([&]()
                                       { convert->Process(msg, cloud_out); },
                                       "laser convert");

        double sample_size = lio->getIndex() < 30 ? 0.02 : 0.1;

        zjloc::common::Timer::Evaluate([&]()
                                       { 
            std::mt19937_64 g;
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            subSampleFrame(cloud_out, sample_size);
            std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                       "laser ds");

        lio->pushData(cloud_out, std::make_pair(zjloc::ToSec(msg->header.stamp), convert->getTimeSpan()));
    }

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        Vec3d acc = Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        acc = acc * g_gravity / acc.norm();
        IMUPtr imu = std::make_shared<zjloc::IMU>(
            zjloc::ToSec(msg->header.stamp),
            Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), acc);
        lio->pushData(imu);
    }

private:
    zjloc::CloudConvert *convert;
    zjloc::lidarodom *lio;
    std::string config_file;

    double g_gravity = 9.805;

    nav_msgs::msg::Path laser_odo_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr laser_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr standard_pcl_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread run_thread;
};

int main(int argc, char **argv)
{
    // google::InitGoogleLogging(argv[0]);
    // FLAGS_stderrthreshold = google::INFO;
    // FLAGS_colorlogtostderr = true;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<II_NVM_Node>();

    rclcpp::spin(node);

    zjloc::common::Timer::PrintAll();
    zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));
    rclcpp::shutdown();

    std::cout << ANSI_COLOR_GREEN_BOLD << " out done. " << ANSI_COLOR_RESET << std::endl;

    sleep(3);
    return 0;
}

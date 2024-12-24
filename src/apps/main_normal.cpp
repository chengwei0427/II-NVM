// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>

// ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <random>

#include "common/utility.h"
#include "preprocess/cloud_convert2.h"

#include "lio/lidarodom.h"
zjloc::loam_tbbic_normal::lidarodom *lio;

nav_msgs::Path laserOdoPath;

zjloc::CloudConvert2 *convert;

double g_gravity = 9.805;

DEFINE_string(config_directory, "/home/cc/catkin_context/src/II_NVM/config", "配置文件目录");
#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "log/" + name))

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{

    std::vector<point3D> cloud_out;
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_out); },
                                   "laser convert");

    zjloc::common::Timer::Evaluate([&]()
                                   {  
        double sample_size = lio->getIndex() < 20 ? 0.01 : 0.02;
        // double sample_size = 0.01;
        std::mt19937_64 g;
        std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        subSampleFrame2(cloud_out, sample_size);
        std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                   "laser ds");

    lio->pushData(cloud_out, std::make_pair(msg->header.stamp.toSec(), convert->getTimeSpan()));
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // std::string laser_file = std::string(ROOT_DIR) + "laser.txt";
    // static std::ofstream fout(laser_file, std::ios::out | std::ios::app);
    // fout << std::setprecision(18) << (msg->header.stamp.toSec() - ros::Time::now().toSec()) * 1000 << " ms" << std::endl;

    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2(*msg));
    static int c = 0;
    // if (c % 2 == 0 && use_velodyne)
    {
        std::vector<point3D> cloud_out;
        zjloc::common::Timer::Evaluate([&]()
                                       { convert->Process(msg, cloud_out); },
                                       "laser convert");

        double sample_size = lio->getIndex() < 30 ? 0.02 : 0.1;

        zjloc::common::Timer::Evaluate([&]()
                                       { 
            // double sample_size = 0.01;
            std::mt19937_64 g;
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            subSampleFrame2(cloud_out, sample_size);
            std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                       "laser ds");

        lio->pushData(cloud_out, std::make_pair(msg->header.stamp.toSec(), convert->getTimeSpan()));
    }
    c++;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
{
    // std::string imu_file = std::string(ROOT_DIR) + "imu.txt";
    // static std::ofstream fout2(imu_file, std::ios::out | std::ios::app);
    // fout2 << std::setprecision(18) << (msg->header.stamp.toSec() - ros::Time::now().toSec()) * 1000 << " ms" << std::endl;

    Vec3d acc = Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    acc = acc * g_gravity / acc.norm();
    IMUPtr imu = std::make_shared<zjloc::IMU>(
        msg->header.stamp.toSec(),
        Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), acc);
    lio->pushData(imu);
}

void updateStatus(const std_msgs::Int32::ConstPtr &msg)
{
    int type = msg->data;
    if (type == 1)
    {
    }
    else if (type == 2)
    {
    }
    else if (type == 3)
        ;
    else if (type == 4)
        ;
    else
        ;
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    std::cout << "config file:" << FLAGS_config_directory << std::endl;

    std::string config_file = std::string(FLAGS_config_directory) + "/mapping.yaml";
    std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;

    lio = new zjloc::loam_tbbic_normal::lidarodom();

    if (!lio->init(config_file))
    {
        return -1;
    }

    ros::Publisher pub_scan = nh.advertise<sensor_msgs::PointCloud2>("scan", 10);
    auto cloud_pub_func = std::function<bool(std::string & topic_name, zjloc::CloudPtr & cloud, double time)>(
        [&](std::string &topic_name, zjloc::CloudPtr &cloud, double time)
        {
            sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*cloud, *cloud_ptr_output);

            cloud_ptr_output->header.stamp = ros::Time().fromSec(time);
            cloud_ptr_output->header.frame_id = "map";
            if (topic_name == "laser")
                pub_scan.publish(*cloud_ptr_output);
            else
                ; // publisher_.publish(*cloud_ptr_output);
            return true;
        }

    );

    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    ros::Publisher pubLaserOdometryPath = nh.advertise<nav_msgs::Path>("/odometry_path", 5);

    auto pose_pub_func = std::function<bool(std::string & topic_name, SE3 & pose, double stamp)>(
        [&](std::string &topic_name, SE3 &pose, double stamp)
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            Eigen::Quaterniond q_current(pose.so3().matrix());
            transform.setOrigin(tf::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
            tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
            transform.setRotation(q);
            if (topic_name == "laser")
            {
                br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "map", "base_link"));

                // publish odometry
                nav_msgs::Odometry laserOdometry;
                laserOdometry.header.frame_id = "map";
                laserOdometry.child_frame_id = "base_link";
                laserOdometry.header.stamp = ros::Time().fromSec(stamp);

                laserOdometry.pose.pose.orientation.x = q_current.x();
                laserOdometry.pose.pose.orientation.y = q_current.y();
                laserOdometry.pose.pose.orientation.z = q_current.z();
                laserOdometry.pose.pose.orientation.w = q_current.w();
                laserOdometry.pose.pose.position.x = pose.translation().x();
                laserOdometry.pose.pose.position.y = pose.translation().y();
                laserOdometry.pose.pose.position.z = pose.translation().z();
                pubLaserOdometry.publish(laserOdometry);

                { //  FIXME:
                    std::ofstream fout(std::string(FLAGS_config_directory) + "/traj_pose.txt", std::ios::app);
                    fout << std::setprecision(18) << stamp << " " << std::setprecision(7) << pose.translation()[0] << " " << pose.translation()[1]
                         << " " << pose.translation()[2] << " " << q_current.x() << " "
                         << q_current.y() << " " << q_current.z() << " " << q_current.w() << std::endl;
                }

                //  publish path
                geometry_msgs::PoseStamped laserPose;
                laserPose.header = laserOdometry.header;
                laserPose.pose = laserOdometry.pose.pose;
                laserOdoPath.header.stamp = laserOdometry.header.stamp;
                laserOdoPath.poses.push_back(laserPose);
                laserOdoPath.header.frame_id = "/map";
                pubLaserOdometryPath.publish(laserOdoPath);
            }
            else if (topic_name == "world")
            {
                br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "world", "map"));
            }

            return true;
        }

    );

    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("/velocity", 1);
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float32>("/move_dist", 1);

    auto data_pub_func = std::function<bool(std::string & topic_name, double time1, double time2)>(
        [&](std::string &topic_name, double time1, double time2)
        {
            std_msgs::Float32 time_rviz;

            time_rviz.data = time1;
            if (topic_name == "velocity")
                vel_pub.publish(time_rviz);
            else
                dist_pub.publish(time_rviz);

            return true;
        }

    );

    lio->setFunc(cloud_pub_func);
    lio->setFunc(pose_pub_func);
    lio->setFunc(data_pub_func);

    convert = new zjloc::CloudConvert2;
    convert->LoadFromYAML(config_file);
    std::cout << ANSI_COLOR_GREEN_BOLD << "init successful" << ANSI_COLOR_RESET << std::endl;

    auto yaml = YAML::LoadFile(config_file);
    std::string laser_topic = yaml["common"]["lid_topic"].as<std::string>();
    std::string imu_topic = yaml["common"]["imu_topic"].as<std::string>();
    g_gravity = yaml["common"]["gravity"].as<double>();

    ros::Subscriber subLaserCloud = convert->lidar_type_ == zjloc::LidarType::AVIA
                                        ? nh.subscribe(laser_topic, 100, livox_pcl_cbk)
                                        : nh.subscribe<sensor_msgs::PointCloud2>(laser_topic, 100, standard_pcl_cbk);

    ros::Subscriber sub_imu_ori = nh.subscribe<sensor_msgs::Imu>(imu_topic, 500, imuHandler);

    ros::Subscriber sub_type = nh.subscribe<std_msgs::Int32>("/change_status", 2, updateStatus);

    std::thread measurement_process(&zjloc::loam_tbbic_normal::lidarodom::run, lio);
    std::cout << ANSI_COLOR_GREEN_BOLD << " lidarodom use tbb normal. " << ANSI_COLOR_RESET << std::endl;

    ros::spin();

    zjloc::common::Timer::PrintAll();
    // zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));
    zjloc::common::Timer::DumpIntoFile(std::string(FLAGS_config_directory) + "/log_time.txt");

    std::cout << ANSI_COLOR_GREEN_BOLD << " out done. " << ANSI_COLOR_RESET << std::endl;

    sleep(3);
    return 0;
}

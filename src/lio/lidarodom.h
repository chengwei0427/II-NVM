/*
 * @Author: chengwei zhao
 * @LastEditors: cc
 * @Data:
 */
#ifndef LOAM_LIDAR_ODOM_NORMAL_H__
#define LOAM_LIDAR_ODOM_NORMAL_H__
#include "common/timer/timer.h"
#include "lio/lidarFactor.hpp"

#include "common/algo/eskf.hpp"
#include "common/algo/static_imu_init.h"
#include "lio/lio_utils.h"

#include <condition_variable>

#include "common/timer/tool_color_printf.hpp"
#include "common/timer/timer.h"
#include "common/utility.h"

#include "common/point_types.h"

#include <sys/times.h>

#include <pcl/io/pcd_io.h>

namespace zjloc
{

     class lidarodom
     {
     public:
          struct liwOptions
          {
               double surf_res;
               int max_iteration;
               bool log_print;
               //   ct_icp
               double size_voxel_map;
               double min_distance_points;
               int max_num_points_in_voxel;
               double max_distance;
               double weight_alpha;
               double weight_neighborhood;
               double max_dist_to_plane_icp;
               int init_num_frames;
               int voxel_neighborhood;
               int max_number_neighbors;
               int threshold_voxel_occupancy;
               bool estimate_normal_from_neighborhood;
               int min_number_neighbors;
               double power_planarity;
               int num_closest_neighbors;

               double sampling_rate;
               int max_num_residuals;
               int min_num_residuals;

               double thres_orientation_norm;
               double thres_translation_norm;
          };

          lidarodom(/* args */);
          ~lidarodom();

          lidarodom(const lidarodom &) = delete;
          lidarodom &operator=(const lidarodom &) = delete;

          bool init(const std::string &config_yaml);

          void pushData(std::vector<point3D>, std::pair<double, double> data);
          void pushData(IMUPtr imu);

          void run();

          int getIndex();

          void setFunc(std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> &fun);
          void setFunc(std::function<bool(std::string &topic_name, SE3 &pose, double time)> &fun);

     private:
          void loadOptions();
          /// 使用IMU初始化
          void TryInitIMU();

          /// 利用IMU预测状态信息
          /// 这段时间的预测数据会放入imu_states_里
          void Predict();

          /// 对measures_中的点云去畸变
          void Undistort(std::vector<point3D> &points);

          std::vector<MeasureGroup> getMeasureMents();

          /// 处理同步之后的IMU和雷达数据
          void ProcessMeasurements(MeasureGroup &meas);

          void stateInitialization();

          cloudFrame *buildFrame(std::vector<point3D> &const_surf, state *cur_state,
                                 double timestamp_begin, double timestamp_end);

          void poseEstimation(cloudFrame *p_frame);

          void optimize(cloudFrame *p_frame);

          void lasermap_fov_segment();

          void map_incremental(cloudFrame *p_frame, int min_num_points = 0);

          void addPointToMap(voxelHashMap2 &map, const point3D &point, double voxel_size,
                             int max_num_points_in_voxel, double min_distance_points,
                             int min_num_points);

          void addSurfCostFactor(std::vector<ceres::CostFunction *> &surf,
                                 std::vector<point3D> &keypoints, const cloudFrame *p_frame);

          void addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points,
                             const Eigen::Vector3d &point, const double &intensity);

          // search neighbors
          Neighborhood computeNeighborhoodDistribution(
              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points);

          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
          searchNeighbors(const voxelHashMap2 &map, const Eigen::Vector3d &point, const Eigen::Vector3d &normal,
                          int nb_voxels_visited, double size_voxel_map, int max_num_neighbors,
                          int threshold_voxel_capacity = 1, std::vector<voxel> *voxels = nullptr);

     private:
          /// @brief 数据
          std::string config_yaml_;
          StaticIMUInit imu_init_;    // IMU静止初始化
          SO3 RIG_;                   //   imu 转换到world
          SE3 TIL_;                   //   lidar 转换到 imu
          MeasureGroup measures_;     // sync IMU and lidar scan
          bool imu_need_init_ = true; // 是否需要估计IMU初始零偏
          int index_frame = 1;
          liwOptions options_;

          bool b_out_channel = false;

          voxelHashMap2 voxel_map;
          Eigen::Matrix3d R_imu_lidar = Eigen::Matrix3d::Identity(); //   lidar 转换到 imu坐标系下
          Eigen::Vector3d t_imu_lidar = Eigen::Vector3d::Zero();     //   need init

          /// @brief 滤波器
          ESKFD eskf_;
          std::vector<NavStated> imu_states_; // ESKF预测期间的状态
          IMUPtr last_imu_ = nullptr;
          double adapt_sample_res;

          double time_curr;
          Vec3d g_{0, 0, -9.8};

          /// @brief 数据管理及同步
          std::deque<std::vector<point3D>> lidar_buffer_;
          std::deque<IMUPtr> imu_buffer_;    // imu数据缓冲
          double last_timestamp_imu_ = -1.0; // 最近imu时间
          double last_timestamp_lidar_ = 0;  // 最近lidar时间
          std::deque<std::pair<double, double>> time_buffer_;

          /// @brief mutex
          std::mutex mtx_buf;
          std::mutex mtx_state;
          std::condition_variable cond;

          state *current_state;
          std::vector<state *> all_state_frame;

          std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> pub_cloud_to_ros;
          std::function<bool(std::string &topic_name, SE3 &pose, double time)> pub_pose_to_ros;
          pcl::PointCloud<pcl::PointXYZI>::Ptr points_world;
     };
}

#endif //   LOAM_LIDAR_ODOM_NORMAL_H__

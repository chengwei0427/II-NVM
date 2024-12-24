#ifndef LOAM_LIDAR_ODOM_TBBIC_H__
#define LOAM_LIDAR_ODOM__TBBIC_H__
#include "common/timer/timer.h"
#include "lio/lidarFactor.hpp"

#include "common/algo/eskf.hpp"
#include "common/algo/static_imu_init.h"
#include "lio/lio_utils.h"

#include <condition_variable>

#include "tools/tool_color_printf.hpp"
#include "common/timer/timer.h"
#include "common/utility.h"
// #include "common/utils.h"
#include "tools/point_types.h"

#include <sys/times.h>
#include <sys/vtimes.h>

#include "lio/lidarodom_interface.hpp"

#include <pcl/io/pcd_io.h>

namespace zjloc::loam_tbbic_normal
{

     class lidarodom : public lidarodomInterface
     {
     public:
          struct liwOptions
          {
               double surf_res;
               double delay_time;
               int max_iteration;
               bool log_print;
               size_t capacity_;
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

          bool init(const std::string &config_yaml) override;

          void pushData(std::vector<point3D>, std::pair<double, double> data) override;
          void pushData(IMUPtr imu) override;

          void run() override;

          int getIndex() override;

          void setFunc(std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> &fun) override;
          void setFunc(std::function<bool(std::string &topic_name, SE3 &pose, double time)> &fun) override;
          void setFunc(std::function<bool(std::string &topic_name, double time1, double time2)> &fun) override;

     private:
          void loadOptions();
          /// 使用IMU初始化
          void TryInitIMU();

          /// 利用IMU预测状态信息
          /// 这段时间的预测数据会放入imu_states_里
          void Predict();

          /// 对measures_中的点云去畸变
          void Undistort(std::vector<point3D> &points, bool only_R = false);

          std::vector<loam::MeasureGroup> getMeasureMents();

          /// 处理同步之后的IMU和雷达数据
          void ProcessMeasurements(loam::MeasureGroup &meas);

          void stateInitialization();

          loam::cloudFrame *buildFrame(std::vector<point3D> &const_surf, loam::state *cur_state,
                                       double timestamp_begin, double timestamp_end);

          void poseEstimation(loam::cloudFrame *p_frame);

          void optimize(loam::cloudFrame *p_frame);

          void map_incremental(loam::cloudFrame *p_frame, int min_num_points = 0);

          void addPointToMap(voxelHashMap3 &map, const point3D &point, double voxel_size,
                             int max_num_points_in_voxel, double min_distance_points,
                             int min_num_points);

          void addSurfCostFactor(std::vector<ceres::CostFunction *> &surf,
                                 std::vector<point3D> &keypoints, const loam::cloudFrame *p_frame);

          void addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points,
                             const Eigen::Vector3d &point, const double &intensity);

          // search neighbors
          loam::Neighborhood computeNeighborhoodDistribution(
              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points);

          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
          searchNeighbors(const voxelHashMap3 &map, const Eigen::Vector3d &point, const Eigen::Vector3d &normal,
                          int nb_voxels_visited, double size_voxel_map, int max_num_neighbors,
                          int threshold_voxel_capacity = 1, std::vector<voxel> *voxels = nullptr);

     private:
          /// @brief 数据
          std::string config_yaml_;
          StaticIMUInit imu_init_;      // IMU静止初始化
          SO3 RIG_;                     //   imu 转换到world
          SE3 TIL_;                     //   lidar 转换到 imu
          loam::MeasureGroup measures_; // sync IMU and lidar scan
          bool imu_need_init_ = true;   // 是否需要估计IMU初始零偏
          int index_frame = 1;
          liwOptions options_;

          voxelHashMap3 voxel_map;
          // std::unordered_map<voxel, typename std::list<std::pair<voxel, voxelBlock>>::iterator> grids_map_;
          std::list<std::pair<voxel, std::shared_ptr<voxelBlock3>>> grids_cache_;

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

          loam::state *current_state;
          std::vector<loam::cloudFrame *> all_cloud_frame; //  cache all frame
          std::vector<loam::state *> all_state_frame;      //   多保留一份state，这样可以不用去缓存all_cloud_frame

          std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> pub_cloud_to_ros;
          std::function<bool(std::string &topic_name, SE3 &pose, double time)> pub_pose_to_ros;
          std::function<bool(std::string &topic_name, double time1, double time2)> pub_data_to_ros;
          pcl::PointCloud<pcl::PointXYZI>::Ptr points_world;

          SE3 last_pose_;
     };
}

#endif //   LOAM_LIDAR_ODOM_H__
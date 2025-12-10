#include "cloud_convert.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "common/nano_flann/nanoflann_pcl.hpp"
#include "common/timer/timer.h"
#include "tbb/tbb.h"
#include <mutex>

#define USE_TBB_PARAL 1

namespace zjloc
{

    void CloudConvert::Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, std::vector<point3D> &pcl_out)
    {
        AviaHandler(msg);
        pcl_out = cloud_out_;
    }

    void CloudConvert::Process(const sensor_msgs::PointCloud2::ConstPtr &msg,
                               std::vector<point3D> &pcl_out)
    {
        switch (param_.lidar_type)
        {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        case LidarType::ROBOSENSE16:
            RobosenseHandler(msg);
            break;

        case LidarType::AVIA_PC:
            AVIAPCHandler(msg);
            break;

        default:
            LOG(ERROR) << "Error LiDAR Type: " << int(lidar_type_);
            break;
        }
        pcl_out = cloud_out_;
    }

    void CloudConvert::AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();
        int plsize = msg->point_num;
        cloud_out_.reserve(plsize);

        static double tm_scale = 1e9;

        double headertime = msg->header.stamp.toSec();
        timespan_ = msg->points.back().offset_time / tm_scale;

        // std::cout << "span:" << timespan_ << ",0: " << msg->points[0].offset_time / tm_scale
        //           << " , 100: " << msg->points[100].offset_time / tm_scale << std::endl;

        zjloc::common::TicToc tc;
        tc.tic();
        CloudPtr cloud(new PointCloudType);
        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(msg->points[i].x) &&
                  std::isfinite(msg->points[i].y) &&
                  std::isfinite(msg->points[i].z)))
                continue;
            PointType pt;
            pt.x = msg->points[i].x;
            pt.y = msg->points[i].y;
            pt.z = msg->points[i].z;
            pt.intensity = msg->points[i].reflectivity;
            cloud->push_back(pt);
        }
        double t1 = tc.toc();
        tc.tic();
        nanoflann::KdTreeFLANN<PointType> nano_kdtree;
        nano_kdtree.setInputCloud(cloud);
        double t2 = tc.toc();
        tc.tic();

#ifdef USE_TBB_PARAL
        std::mutex valid_points_mutex;
        tbb::parallel_for(tbb::blocked_range<size_t>(0, plsize),
                          [&](const tbb::blocked_range<size_t> &r)
                          {
                              std::vector<point3D> local_points;
                              for (size_t i = r.begin(); i != r.end(); ++i)
                              {
                                  if (!(std::isfinite(msg->points[i].x) &&
                                        std::isfinite(msg->points[i].y) &&
                                        std::isfinite(msg->points[i].z)))
                                      continue;
                                  if (i % param_.point_filter_num != 0)
                                      continue;
                                  double range = msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y +
                                                 msg->points[i].z * msg->points[i].z;
                                  if (range > 250 * 250 || range < param_.blind * param_.blind)
                                      continue;

                                  auto adaptive_r = [&](pcl::PointXYZI &pt)
                                  {
                                      double max_dist = 30;
                                      double min_dist = 5;
                                      double max_r = 4;
                                      double min_r = 0.2;
                                      double dist = pt.getVector3fMap().norm();
                                      if (dist > max_dist)
                                          return max_r;
                                      if (dist < min_dist)
                                          return min_r;
                                      return (dist - min_dist) / (max_dist - min_dist) * (max_r - min_r);
                                  };

                                  if (/*(msg->points[i].line < N_SCANS) &&*/ ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
                                  {
                                      PointType pt;
                                      pt.x = msg->points[i].x, pt.y = msg->points[i].y, pt.z = msg->points[i].z;
                                      pt.intensity = msg->points[i].reflectivity;
                                      double radius = adaptive_r(pt);

                                      std::vector<int> pointIdxRSearch;
                                      std::vector<float> pointRSquaredDistance;
                                      if (nano_kdtree.radiusSearch(pt, radius, pointIdxRSearch, pointRSquaredDistance) > 5)
                                      {
                                          std::vector<Eigen::Vector3f> neighbors;
                                          for (size_t j = 0; j < pointIdxRSearch.size(); ++j)
                                          {
                                              neighbors.push_back(cloud->points[pointIdxRSearch[j]].getVector3fMap());
                                          }

                                          Eigen::Vector3f normal = computeNormal(neighbors);

                                          point3D point_temp;
                                          point_temp.raw_point = Eigen::Vector3d(msg->points[i].x, msg->points[i].y, msg->points[i].z);
                                          point_temp.point = point_temp.raw_point;
                                          point_temp.raw_normal = normal.cast<double>();
                                          point_temp.normal = point_temp.raw_normal;
                                          point_temp.relative_time = msg->points[i].offset_time / tm_scale; // curvature unit: ms
                                          point_temp.intensity = msg->points[i].reflectivity;

                                          point_temp.timestamp = headertime + point_temp.relative_time;
                                          point_temp.alpha_time = point_temp.relative_time / timespan_;
                                          point_temp.timespan = timespan_;
                                          point_temp.ring = msg->points[i].line;
                                          point_temp.lid = 1;

                                          local_points.push_back(point_temp);
                                      }
                                  }
                              }
                              std::lock_guard<std::mutex> lock(valid_points_mutex);
                              cloud_out_.insert(cloud_out_.end(), local_points.begin(), local_points.end());
                          });

#else
        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(msg->points[i].x) &&
                  std::isfinite(msg->points[i].y) &&
                  std::isfinite(msg->points[i].z)))
                continue;

            if (msg->points[i].offset_time / tm_scale > timespan_)
                std::cout << "------" << __FUNCTION__ << ", " << __LINE__ << ", error timespan:" << timespan_ << " < " << msg->points[i].offset_time / tm_scale << std::endl;

            if (i % param_.point_filter_num != 0)
                continue;

            // if (msg->points[i].reflectivity < 5)
            //     continue;

            double range = msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y +
                           msg->points[i].z * msg->points[i].z;
            if (range > 250 * 250 || range < param_.blind * param_.blind)
                continue;

            auto adaptive_r = [&](pcl::PointXYZI &pt)
            {
                double max_dist = 30;
                double min_dist = 5;
                double max_r = 4;
                double min_r = 0.2;
                double dist = pt.getVector3fMap().norm();
                if (dist > max_dist)
                    return max_r;
                if (dist < min_dist)
                    return min_r;
                return (dist - min_dist) / (max_dist - min_dist) * (max_r - min_r);
            };

            if (/*(msg->points[i].line < N_SCANS) &&*/ ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
            {
                PointType pt;
                pt.x = msg->points[i].x, pt.y = msg->points[i].y, pt.z = msg->points[i].z;
                pt.intensity = msg->points[i].reflectivity;
                double radius = adaptive_r(pt);

                std::vector<int> pointIdxRSearch;
                std::vector<float> pointRSquaredDistance;
                if (nano_kdtree.radiusSearch(pt, radius, pointIdxRSearch, pointRSquaredDistance) > 5)
                {
                    std::vector<Eigen::Vector3f> neighbors;
                    for (size_t j = 0; j < pointIdxRSearch.size(); ++j)
                    {
                        neighbors.push_back(cloud->points[pointIdxRSearch[j]].getVector3fMap());
                    }

                    Eigen::Vector3f normal = computeNormal(neighbors);

                    point3D point_temp;
                    point_temp.raw_point = Eigen::Vector3d(msg->points[i].x, msg->points[i].y, msg->points[i].z);
                    point_temp.point = point_temp.raw_point;
                    point_temp.raw_normal = normal.cast<double>();
                    point_temp.normal = point_temp.raw_normal;
                    point_temp.relative_time = msg->points[i].offset_time / tm_scale; // curvature unit: ms
                    point_temp.intensity = msg->points[i].reflectivity;

                    point_temp.timestamp = headertime + point_temp.relative_time;
                    point_temp.alpha_time = point_temp.relative_time / timespan_;
                    point_temp.timespan = timespan_;
                    point_temp.ring = msg->points[i].line;
                    point_temp.lid = 1;

                    cloud_out_.push_back(point_temp);
                }
            }
        }
#endif
        double t3 = tc.toc();
        std::cout << "takes: " << t1 << ", " << t2 << ", " << t3 << std::endl;
    }

    void CloudConvert::Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();
        pcl::PointCloud<ouster_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();
        cloud_out_.reserve(plsize);

        static double tm_scale = 1e9;

        double headertime = msg->header.stamp.toSec();
        // timespan_ = pl_orig.points.back().t / tm_scale;
        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].t / tm_scale
        //           << " , 100: " << pl_orig.points[100].t / tm_scale << " , 1000: " << pl_orig.points[1000].t / tm_scale
        //           << " ," << pl_orig.points.back().t / tm_scale << std::endl;

        zjloc::common::TicToc tc;
        tc.tic();
        CloudPtr cloud(new PointCloudType);
        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;
            PointType pt;
            pt.x = pl_orig.points[i].x;
            pt.y = pl_orig.points[i].y;
            pt.z = pl_orig.points[i].z;
            pt.intensity = pl_orig.points[i].intensity;
            cloud->push_back(pt);
        }
        double t1 = tc.toc();
        tc.tic();
        nanoflann::KdTreeFLANN<PointType> nano_kdtree;
        nano_kdtree.setInputCloud(cloud);
        double t2 = tc.toc();
        tc.tic();

#ifdef USE_TBB_PARAL
        std::mutex valid_points_mutex;
        tbb::parallel_for(tbb::blocked_range<size_t>(0, plsize),
                          [&](const tbb::blocked_range<size_t> &r)
                          {
                              std::vector<point3D> local_points;
                              for (size_t i = r.begin(); i != r.end(); ++i)
                              {
                                  if (!(std::isfinite(pl_orig.points[i].x) &&
                                        std::isfinite(pl_orig.points[i].y) &&
                                        std::isfinite(pl_orig.points[i].z)))
                                      continue;
                                  if (i % param_.point_filter_num != 0)
                                      continue;
                                  double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                                                 pl_orig.points[i].z * pl_orig.points[i].z;
                                  if (range > 150 * 150 || range < param_.blind * param_.blind)
                                      continue;

                                  auto adaptive_r = [&](pcl::PointXYZI &pt)
                                  {
                                      double max_dist = 30;
                                      double min_dist = 5;
                                      double max_r = 4;
                                      double min_r = 0.2;
                                      double dist = pt.getVector3fMap().norm();
                                      if (dist > max_dist)
                                          return max_r;
                                      if (dist < min_dist)
                                          return min_r;
                                      return (dist - min_dist) / (max_dist - min_dist) * (max_r - min_r);
                                  };

                                  {
                                      PointType pt;
                                      pt.x = pl_orig.points[i].x, pt.y = pl_orig.points[i].y, pt.z = pl_orig.points[i].z;
                                      pt.intensity = pl_orig.points[i].intensity;
                                      double radius = adaptive_r(pt);

                                      std::vector<int> pointIdxRSearch;
                                      std::vector<float> pointRSquaredDistance;
                                      if (nano_kdtree.radiusSearch(pt, radius, pointIdxRSearch, pointRSquaredDistance) > 5)
                                      {
                                          std::vector<Eigen::Vector3f> neighbors;
                                          for (size_t j = 0; j < pointIdxRSearch.size(); ++j)
                                          {
                                              neighbors.push_back(cloud->points[pointIdxRSearch[j]].getVector3fMap());
                                          }

                                          Eigen::Vector3f normal = computeNormal(neighbors);

                                          point3D point_temp;
                                          point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
                                          point_temp.point = point_temp.raw_point;
                                          point_temp.raw_normal = normal.cast<double>();
                                          point_temp.normal = point_temp.raw_normal;
                                          point_temp.relative_time = pl_orig.points[i].t / tm_scale; // curvature unit: ms
                                          point_temp.intensity = pl_orig.points[i].intensity;

                                          point_temp.timestamp = headertime + point_temp.relative_time;
                                          point_temp.alpha_time = point_temp.relative_time / timespan_;
                                          point_temp.timespan = timespan_;
                                          point_temp.ring = pl_orig.points[i].ring;

                                          local_points.push_back(point_temp);
                                      }
                                  }
                              }
                              std::lock_guard<std::mutex> lock(valid_points_mutex);
                              cloud_out_.insert(cloud_out_.end(), local_points.begin(), local_points.end());
                          });
#else
        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % param_.point_filter_num != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < param_.blind * param_.blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].t / tm_scale; // curvature unit: ms
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
#endif
        double t3 = tc.toc();
        std::cout << "takes: " << t1 << ", " << t2 << ", " << t3 << std::endl;
    }

    void CloudConvert::RobosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();
        pcl::PointCloud<robosense_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.toSec();
        //  FIXME:  时间戳大于0.1
        auto time_list_robosense = [&](robosense_ros::Point &point_1, robosense_ros::Point &point_2)
        {
            return (point_1.timestamp < point_2.timestamp);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_robosense);
        while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }

        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

        // std::cout << timespan_ << std::endl;

        // std::cout << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points.back().timestamp << std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            // if (i % param_.point_filter_num != 0)
            //     continue;
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < param_.blind * param_.blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            // point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.timestamp = pl_orig.points[i].timestamp;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;
            point_temp.lid = 4;
            if (point_temp.alpha_time > 1 || point_temp.alpha_time < 0)
                std::cout << point_temp.alpha_time << ", this may error." << std::endl;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();

        pcl::PointCloud<velodyne_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.toSec();

        static double tm_scale = 1; //   1e6 - nclt kaist or 1

        //  FIXME:  nclt 及kaist时间戳大于0.1
        auto time_list_velodyne = [&](velodyne_ros::Point &point_1, velodyne_ros::Point &point_2)
        {
            return (point_1.time < point_2.time);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_velodyne);
        // std::cout << "cloud sizeZ:" << plsize << ",last t:" << pl_orig.points[plsize - 1].time << std::endl;
        while (pl_orig.points[plsize - 1].time / tm_scale >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }
        // timespan_ = -pl_orig.points[0].time / tm_scale;
        timespan_ = pl_orig.points.back().time / tm_scale;
        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].time / tm_scale << " , 800: " << pl_orig.points[100].time / tm_scale << std::endl;

        zjloc::common::TicToc tc;
        tc.tic();
        CloudPtr cloud(new PointCloudType);
        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;
            PointType pt;
            pt.x = pl_orig.points[i].x;
            pt.y = pl_orig.points[i].y;
            pt.z = pl_orig.points[i].z;
            pt.intensity = pl_orig.points[i].intensity;
            cloud->push_back(pt);
        }
        double t1 = tc.toc();
        tc.tic();
        nanoflann::KdTreeFLANN<PointType> nano_kdtree;
        nano_kdtree.setInputCloud(cloud);
        double t2 = tc.toc();
        tc.tic();
#ifdef USE_TBB_PARAL
        std::mutex valid_points_mutex;
        tbb::parallel_for(tbb::blocked_range<size_t>(0, plsize),
                          [&](const tbb::blocked_range<size_t> &r)
                          {
                              std::vector<point3D> local_points;
                              for (size_t i = r.begin(); i != r.end(); ++i)
                              {
                                  if (!(std::isfinite(pl_orig.points[i].x) &&
                                        std::isfinite(pl_orig.points[i].y) &&
                                        std::isfinite(pl_orig.points[i].z)))
                                      continue;
                                  if (i % param_.point_filter_num != 0)
                                      continue;
                                  double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                                                 pl_orig.points[i].z * pl_orig.points[i].z;
                                  if (range > 150 * 150 || range < param_.blind * param_.blind)
                                      continue;

                                  auto adaptive_r = [&](pcl::PointXYZI &pt)
                                  {
                                      double max_dist = 30;
                                      double min_dist = 5;
                                      double max_r = 4;
                                      double min_r = 0.2;
                                      double dist = pt.getVector3fMap().norm();
                                      if (dist > max_dist)
                                          return max_r;
                                      if (dist < min_dist)
                                          return min_r;
                                      return (dist - min_dist) / (max_dist - min_dist) * (max_r - min_r);
                                  };

                                  {
                                      PointType pt;
                                      pt.x = pl_orig.points[i].x, pt.y = pl_orig.points[i].y, pt.z = pl_orig.points[i].z;
                                      pt.intensity = pl_orig.points[i].intensity;
                                      double radius = adaptive_r(pt);

                                      std::vector<int> pointIdxRSearch;
                                      std::vector<float> pointRSquaredDistance;
                                      if (nano_kdtree.radiusSearch(pt, radius, pointIdxRSearch, pointRSquaredDistance) > 5)
                                      {
                                          std::vector<Eigen::Vector3f> neighbors;
                                          for (size_t j = 0; j < pointIdxRSearch.size(); ++j)
                                          {
                                              neighbors.push_back(cloud->points[pointIdxRSearch[j]].getVector3fMap());
                                          }

                                          Eigen::Vector3f normal = computeNormal(neighbors);

                                          point3D point_temp;
                                          point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
                                          point_temp.point = point_temp.raw_point;
                                          point_temp.raw_normal = normal.cast<double>();
                                          point_temp.normal = point_temp.raw_normal;
                                          point_temp.relative_time = pl_orig.points[i].time / tm_scale; // curvature unit: ms
                                          point_temp.intensity = pl_orig.points[i].intensity;

                                          point_temp.timestamp = headertime + point_temp.relative_time;
                                          point_temp.alpha_time = point_temp.relative_time / timespan_;
                                          point_temp.timespan = timespan_;
                                          point_temp.ring = pl_orig.points[i].ring;
                                          point_temp.lid = 2;

                                          local_points.push_back(point_temp);
                                      }
                                  }
                              }
                              std::lock_guard<std::mutex> lock(valid_points_mutex);
                              cloud_out_.insert(cloud_out_.end(), local_points.begin(), local_points.end());
                          });
#else
        //  TODO:  need modified
        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % param_.point_filter_num != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < param_.blind * param_.blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].time / tm_scale; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;
            point_temp.lid = 2;

            cloud_out_.push_back(point_temp);
        }
#endif
        double t3 = tc.toc();
        std::cout << "takes: " << t1 << ", " << t2 << ", " << t3 << std::endl;
    }

    void CloudConvert::AVIAPCHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();

        pcl::PointCloud<livox_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.toSec();

        static double tm_scale = 1e9;
        auto time_list_livox = [&](livox_ros::Point &point_1, livox_ros::Point &point_2)
        {
            return (point_1.timestamp < point_2.timestamp);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_livox);
        // while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
        // {
        //     plsize--;
        //     pl_orig.points.pop_back();
        // }
        timespan_ = (pl_orig.points.back().timestamp - pl_orig.points[0].timestamp) / tm_scale;

        zjloc::common::TicToc tc;
        tc.tic();
        CloudPtr cloud(new PointCloudType);
        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;
            PointType pt;
            pt.x = pl_orig.points[i].x;
            pt.y = pl_orig.points[i].y;
            pt.z = pl_orig.points[i].z;
            pt.intensity = pl_orig.points[i].intensity;
            cloud->push_back(pt);
        }
        double t1 = tc.toc();
        tc.tic();
        nanoflann::KdTreeFLANN<PointType> nano_kdtree;
        nano_kdtree.setInputCloud(cloud);
        double t2 = tc.toc();
        tc.tic();
#ifdef USE_TBB_PARAL
        std::mutex valid_points_mutex;
        tbb::parallel_for(tbb::blocked_range<size_t>(0, plsize),
                          [&](const tbb::blocked_range<size_t> &r)
                          {
                              std::vector<point3D> local_points;
                              for (size_t i = r.begin(); i != r.end(); ++i)
                              {
                                  if (!(std::isfinite(pl_orig.points[i].x) &&
                                        std::isfinite(pl_orig.points[i].y) &&
                                        std::isfinite(pl_orig.points[i].z)))
                                      continue;
                                  if (i % param_.point_filter_num != 0)
                                      continue;
                                  double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                                                 pl_orig.points[i].z * pl_orig.points[i].z;
                                  if (range > 150 * 150 || range < param_.blind * param_.blind)
                                      continue;

                                  auto adaptive_r = [&](pcl::PointXYZI &pt)
                                  {
                                      double max_dist = 30;
                                      double min_dist = 5;
                                      double max_r = 4;
                                      double min_r = 0.2;
                                      double dist = pt.getVector3fMap().norm();
                                      if (dist > max_dist)
                                          return max_r;
                                      if (dist < min_dist)
                                          return min_r;
                                      return (dist - min_dist) / (max_dist - min_dist) * (max_r - min_r);
                                  };

                                  {
                                      PointType pt;
                                      pt.x = pl_orig.points[i].x, pt.y = pl_orig.points[i].y, pt.z = pl_orig.points[i].z;
                                      pt.intensity = pl_orig.points[i].intensity;
                                      double radius = adaptive_r(pt);

                                      std::vector<int> pointIdxRSearch;
                                      std::vector<float> pointRSquaredDistance;
                                      if (nano_kdtree.radiusSearch(pt, radius, pointIdxRSearch, pointRSquaredDistance) > 5)
                                      {
                                          std::vector<Eigen::Vector3f> neighbors;
                                          for (size_t j = 0; j < pointIdxRSearch.size(); ++j)
                                          {
                                              neighbors.push_back(cloud->points[pointIdxRSearch[j]].getVector3fMap());
                                          }

                                          Eigen::Vector3f normal = computeNormal(neighbors);

                                          point3D point_temp;
                                          point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
                                          point_temp.point = point_temp.raw_point;
                                          point_temp.raw_normal = normal.cast<double>();
                                          point_temp.normal = point_temp.raw_normal;
                                          point_temp.relative_time = (pl_orig.points[i].timestamp - pl_orig.points[0].timestamp) / tm_scale;
                                          point_temp.intensity = pl_orig.points[i].intensity;

                                          point_temp.timestamp = headertime + point_temp.relative_time;
                                          point_temp.alpha_time = point_temp.relative_time / timespan_;
                                          point_temp.timespan = timespan_;
                                          point_temp.ring = pl_orig.points[i].line;
                                          point_temp.lid = 1;

                                          local_points.push_back(point_temp);
                                      }
                                  }
                              }
                              std::lock_guard<std::mutex> lock(valid_points_mutex);
                              cloud_out_.insert(cloud_out_.end(), local_points.begin(), local_points.end());
                          });
#else
        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) && std::isfinite(pl_orig.points[i].y) && std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % param_.point_filter_num != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x +
                           pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;

            if (range > 120 * 120 || range < blind * blind)
                continue;

            point3D tPoint;
            tPoint.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            tPoint.point = tPoint.raw_point;
            tPoint.relative_time = (pl_orig.points[i].timestamp - pl_orig.points[0].timestamp) / tm_scale;
            tPoint.intensity = pl_orig.points[i].intensity;
            tPoint.timestamp = headertime + tPoint.relative_time;
            tPoint.alpha_time = tPoint.relative_time / timespan_;
            tPoint.timespan = timespan_;
            tPoint.ring = pl_orig.points[i].line;

            cloud_out_.push_back(tPoint);
        }
#endif
        // std::cout << "cloud size out: " << cloud_out_.size() << std::endl;
    }

    //  计算协方差矩阵的最小特征值对应的特征向量，即法向量
    Eigen::Vector3f CloudConvert::computeNormal(const std::vector<Eigen::Vector3f> &neighbors)
    {
        //  计算质心  计算协方差
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();

        for (const auto &point : neighbors)
        {
            centroid += point;
            covariance += point * point.transpose();
        }
        centroid /= (float)neighbors.size();
        covariance /= (float)neighbors.size();
        covariance -= centroid * centroid.transpose();

        //  计算协方差矩阵的特征值和特征向量
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        Eigen::Vector3f normal(solver.eigenvectors().col(0).normalized()); //  最小特征值对应的特征向量
        if (normal.dot(-centroid) < 0)
            normal *= -1.0;
        return normal;
    }

    void CloudConvert::initFromConfig(const CVTParam &param)
    {
        param_.point_filter_num = param.point_filter_num;
        param_.blind = param.blind;
        param_.lidar_type = param.lidar_type;
        lidar_type_ = param.lidar_type;
        if (param_.lidar_type == LidarType::AVIA)
            LOG(INFO) << "Using AVIA Lidar";
        else if (param_.lidar_type == LidarType::VELO32)
            LOG(INFO) << "Using Velodyne 32 Lidar";
        else if (param_.lidar_type == LidarType::OUST64)
            LOG(INFO) << "Using OUST 64 Lidar";
        else if (param_.lidar_type == LidarType::ROBOSENSE16)
            LOG(INFO) << "Using Robosense 16 LIdar";
        else if (param_.lidar_type == LidarType::AVIA_PC)
            LOG(INFO) << "Using AVIA PC LIdar";
        else
            LOG(WARNING) << "unknown lidar_type";
    }

    void CloudConvert::LoadFromYAML(const std::string &yaml_file)
    {
        auto yaml = YAML::LoadFile(yaml_file);
        int lidar_type = yaml["preprocess"]["lidar_type"].as<int>();

        param_.point_filter_num = yaml["preprocess"]["point_filter_num"].as<int>();
        param_.blind = yaml["preprocess"]["blind"].as<double>();

        if (lidar_type == 1)
        {
            lidar_type_ = LidarType::AVIA;
            LOG(INFO) << "Using AVIA Lidar";
        }
        else if (lidar_type == 2)
        {
            lidar_type_ = LidarType::VELO32;
            LOG(INFO) << "Using Velodyne 32 Lidar";
        }
        else if (lidar_type == 3)
        {
            lidar_type_ = LidarType::OUST64;
            LOG(INFO) << "Using OUST 64 Lidar";
        }
        else if (lidar_type == 4)
        {
            lidar_type_ = LidarType::ROBOSENSE16;
            LOG(INFO) << "Using Robosense 16 LIdar";
        }
        else if (lidar_type == 5)
        {
            lidar_type_ = LidarType::AVIA_PC;
            LOG(INFO) << "Using AVIA PC LIdar";
        }
        else
        {
            LOG(WARNING) << "unknown lidar_type";
        }
        param_.lidar_type = lidar_type_;
    }

} // namespace zjloc

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

#include <tbb/tbb.h>

#include "common/timer/timer.h"
#include "common/nano_flann/nanoflann_pcl.hpp"

//  计算协方差矩阵的最小特征值对应的特征向量，即法向量
Eigen::Vector3f computeNormal(const std::vector<Eigen::Vector3f> &neighbors)
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

int main(int argc, char **argv)
{
    std::string file_dir = "/home/cc/catkin_context/src/normal_lio/log/kf_data/";
    std::string file_name = file_dir + "0.pcd";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_name, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", file_name);
        return -1;
    }
    std::cout << "cloud size:" << cloud->size() << std::endl;

    zjloc::common::TicToc tc;
    tc.tic();
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_tbb;
    kdtree_tbb.setInputCloud(cloud);
    for (int i = 0; i < 100; i++)
    {

        int k = 80;
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        cloud_normals->points.resize(cloud->points.size());

        tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud->points.size()),
                          [&](const tbb::blocked_range<size_t> &r)
                          {
                              for (size_t i = r.begin(); i != r.end(); ++i)
                              {
                                  pcl::PointXYZI searchPoint = cloud->points[i];
                                  std::vector<int> pointIdxRSearch;
                                  std::vector<float> pointRSquaredDistance;
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

                                  double radius = adaptive_r(searchPoint);
                                  //   if (kdtree_tbb.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                                  if (kdtree_tbb.radiusSearch(searchPoint, radius, pointIdxRSearch, pointRSquaredDistance) > 5)
                                  {
                                      std::vector<Eigen::Vector3f> neighbors;
                                      for (size_t j = 0; j < pointIdxRSearch.size(); ++j)
                                      {
                                          neighbors.push_back(cloud->points[pointIdxRSearch[j]].getVector3fMap());
                                      }

                                      Eigen::Vector3f normal = computeNormal(neighbors);
                                      cloud_normals->points[i].normal_x = normal.x();
                                      cloud_normals->points[i].normal_y = normal.y();
                                      cloud_normals->points[i].normal_z = normal.z();
                                  }
                              }
                          });
    }
    std::cout << "pcl-tbb cost time: " << tc.toc() << std::endl;

    tc.tic();
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);
    for (int i = 0; i < 100; i++)
    {

        int k = 80;
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        cloud_normals->points.resize(cloud->points.size());

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            pcl::PointXYZI searchPoint = cloud->points[i];
            std::vector<int> pointIdxRSearch;
            std::vector<float> pointRSquaredDistance;
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

            double radius = adaptive_r(searchPoint);
            //   if (kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            if (kdtree.radiusSearch(searchPoint, radius, pointIdxRSearch, pointRSquaredDistance) > 5)
            {
                std::vector<Eigen::Vector3f> neighbors;
                for (size_t j = 0; j < pointIdxRSearch.size(); ++j)
                {
                    neighbors.push_back(cloud->points[pointIdxRSearch[j]].getVector3fMap());
                }

                Eigen::Vector3f normal = computeNormal(neighbors);
                cloud_normals->points[i].normal_x = normal.x();
                cloud_normals->points[i].normal_y = normal.y();
                cloud_normals->points[i].normal_z = normal.z();
            }
        }
    }
    std::cout << "pcl cost time: " << tc.toc() << std::endl;

    tc.tic();
    nanoflann::KdTreeFLANN<pcl::PointXYZI> nano_kdtree_tbb;
    nano_kdtree_tbb.setInputCloud(cloud);
    for (int i = 0; i < 100; i++)
    {

        int k = 80;
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        cloud_normals->points.resize(cloud->points.size());

        tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud->points.size()),
                          [&](const tbb::blocked_range<size_t> &r)
                          {
                              for (size_t i = r.begin(); i != r.end(); ++i)
                              {
                                  pcl::PointXYZI searchPoint = cloud->points[i];
                                  std::vector<int> pointIdxRSearch;
                                  std::vector<float> pointRSquaredDistance;
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

                                  double radius = adaptive_r(searchPoint);
                                  //   if (nano_kdtree_tbb.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                                  if (nano_kdtree_tbb.radiusSearch(searchPoint, radius, pointIdxRSearch, pointRSquaredDistance) > 5)
                                  {
                                      std::vector<Eigen::Vector3f> neighbors;
                                      for (size_t j = 0; j < pointIdxRSearch.size(); ++j)
                                      {
                                          neighbors.push_back(cloud->points[pointIdxRSearch[j]].getVector3fMap());
                                      }

                                      Eigen::Vector3f normal = computeNormal(neighbors);
                                      cloud_normals->points[i].normal_x = normal.x();
                                      cloud_normals->points[i].normal_y = normal.y();
                                      cloud_normals->points[i].normal_z = normal.z();
                                  }
                              }
                          });
    }
    std::cout << "nano-tbb cost time: " << tc.toc() << std::endl;

    tc.tic();
    nanoflann::KdTreeFLANN<pcl::PointXYZI> nano_kdtree;
    nano_kdtree.setInputCloud(cloud);
    for (int i = 0; i < 100; i++)
    {

        int k = 80;
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        cloud_normals->points.resize(cloud->points.size());

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            pcl::PointXYZI searchPoint = cloud->points[i];
            std::vector<int> pointIdxRSearch;
            std::vector<float> pointRSquaredDistance;
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

            double radius = adaptive_r(searchPoint);
            //   if (nano_kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            if (nano_kdtree.radiusSearch(searchPoint, radius, pointIdxRSearch, pointRSquaredDistance) > 5)
            {
                std::vector<Eigen::Vector3f> neighbors;
                for (size_t j = 0; j < pointIdxRSearch.size(); ++j)
                {
                    neighbors.push_back(cloud->points[pointIdxRSearch[j]].getVector3fMap());
                }

                Eigen::Vector3f normal = computeNormal(neighbors);
                cloud_normals->points[i].normal_x = normal.x();
                cloud_normals->points[i].normal_y = normal.y();
                cloud_normals->points[i].normal_z = normal.z();
            }
        }
    }
    std::cout << "nano cost time: " << tc.toc() << std::endl;

    // for (size_t i = 0; i < cloud_normals->points.size(); ++i)
    // {
    //     std::cout << "Normal of point " << i << ": ("
    //               << cloud_normals->points[i].normal_x << ", "
    //               << cloud_normals->points[i].normal_y << ", "
    //               << cloud_normals->points[i].normal_z << ")" << std::endl;
    // }

    // 可视化点云和法向量
    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->setBackgroundColor(0, 0, 0);
    // viewer->addPointCloud<pcl::PointXYZI>(cloud, "sample cloud");
    // viewer->addPointCloudNormals<pcl::PointXYZI, pcl::Normal>(cloud, cloud_normals, 1, 0.05, "normals");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    // viewer->addCoordinateSystem(1.0);
    // viewer->initCameraParameters();

    // // 循环直到可视化窗口关闭
    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce(100);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    return 0;
}

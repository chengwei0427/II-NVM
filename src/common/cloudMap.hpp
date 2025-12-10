/*
 * @Author: chengwei zhao
 * @LastEditors: cc
 * @Data:
 */
#ifndef CLOUD_MAP_HPP_
#define CLOUD_MAP_HPP_
// c++
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>
#include <vector>
#include <queue>
#include <list>

// eigen
#include <Eigen/Core>

// robin_map
#include <tsl/robin_map.h>

struct point3D
{
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

     Eigen::Vector3d raw_point;  //  raw point
     Eigen::Vector3d point;      //  global frame
     Eigen::Vector3d raw_normal; // raw normal
     Eigen::Vector3d normal;     // normal in global frame

     double intensity;           //   intensity
     double alpha_time = 0.0;    //  reference to last point of current frame [0,1]
     double relative_time = 0.0; //  feference to current frame
     double timespan;            //   total time of current frame
     double timestamp = 0.0;     //   global timestamp
     int ring;                   //   ring
     int lid = 0;                //  for different lidar

     point3D() = default;
};

class normalPoint //  Reference  sr-livo
{
public:
     normalPoint(const Eigen::Vector3d &position_,
                 const Eigen::Vector3d &normal_ =
                     Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::quiet_NaN()));

     normalPoint(const point3D &pt_);

     void setPosition(const Eigen::Vector3d &position_);

     Eigen::Vector3d getPosition() const;

     void setNormal(const Eigen::Vector3d &position_);

     Eigen::Vector3d getNormal() const;

     void setTimeStamp(const double &t_);

     double getTimeStamp();

     void reset();

private:
     Eigen::Vector3f position;
     Eigen::Vector3f normal;
     double last_observe_time;
};

struct voxel
{

     voxel() = default;

     voxel(short x, short y, short z) : x(x), y(y), z(z) {}

     bool operator==(const voxel &vox) const { return x == vox.x && y == vox.y && z == vox.z; }

     inline bool operator<(const voxel &vox) const
     {
          return x < vox.x || (x == vox.x && y < vox.y) || (x == vox.x && y == vox.y && z < vox.z);
     }

     inline static voxel coordinates(const Eigen::Vector3d &point, double voxel_size)
     {
          return {short(point.x() / voxel_size),
                  short(point.y() / voxel_size),
                  short(point.z() / voxel_size)};
     }

     inline static voxel coordinates(normalPoint &point, double voxel_size)
     {
          return {short(point.getPosition().x() / voxel_size),
                  short(point.getPosition().y() / voxel_size),
                  short(point.getPosition().z() / voxel_size)};
     }

     short x;
     short y;
     short z;
};

struct voxelBlock3
{

     explicit voxelBlock3(int num_points_ = 20) : num_points(num_points_)
     {
          points.reserve(num_points_);
          other_points.reserve(num_points_);
     }

     // std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;
     std::vector<normalPoint> points;
     std::vector<normalPoint> other_points;

     double last_observe_time = -1;
     double last_other_observe_time = -1;

     bool is_plane_check = false;
     bool is_otherplane_check = false;

     Eigen::Vector3d normal = Eigen::Vector3d(0, 0, 0);
     Eigen::Vector3d other_normal = Eigen::Vector3d(0, 0, 0);

     bool IsFull() const { return num_points == points.size(); }
     bool isOtherFull() const { return num_points == other_points.size(); }

     void AddPoint(const normalPoint &point)
     {
          assert(num_points > points.size());
          normalPoint pt = point;
          normal = (normal * points.size() + pt.getNormal().normalized()) / (points.size() + 1);
          normal.normalize();
          points.push_back(point);
     }

     void AddOtherPoint(const normalPoint &point)
     {
          assert(num_points > other_points.size());
          normalPoint pt = point;
          other_normal = (other_normal * points.size() + pt.getNormal().normalized()) / (points.size() + 1);
          other_normal.normalize();
          other_points.push_back(point);
     }

     inline int NumPoints() const { return points.size(); }
     inline int NumOtherPoints() const { return other_points.size(); }

     inline int Capacity() { return num_points; }

private:
     int num_points;
};

typedef tsl::robin_map<voxel, voxelBlock3> voxelHashMap2;

namespace std
{

     template <>
     struct hash<voxel>
     {
          std::size_t operator()(const voxel &vox) const
          {
               const size_t kP1 = 73856093;
               const size_t kP2 = 19349669;
               const size_t kP3 = 83492791;
               return vox.x * kP1 + vox.y * kP2 + vox.z * kP3;
          }
     };
}

#endif
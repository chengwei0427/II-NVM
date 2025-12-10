#include "utility.h"

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b)
{
     Eigen::Matrix3d rota = q_a.toRotationMatrix();
     Eigen::Matrix3d rotb = q_b.toRotationMatrix();

     double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
     norm = std::acos(norm) * 180 / M_PI;
     return norm;
}

void subSampleFrame(std::vector<point3D> &frame, double size_voxel)
{
     std::tr1::unordered_map<voxel, std::vector<point3D>, std::hash<voxel>> grid;
     for (int i = 0; i < (int)frame.size(); i++)
     {
          auto kx = static_cast<short>(frame[i].point[0] / size_voxel);
          auto ky = static_cast<short>(frame[i].point[1] / size_voxel);
          auto kz = static_cast<short>(frame[i].point[2] / size_voxel);
          grid[voxel(kx, ky, kz)].push_back(frame[i]);
     }
     frame.resize(0);
     int step = 0;
     for (const auto &n : grid)
     {
          if (n.second.size() > 0)
          {
               frame.push_back(n.second[0]);
               step++;
          }
     }
}

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling)
{
     keypoints.resize(0);
     std::vector<point3D> frame_sub;
     frame_sub.resize(frame.size());
     for (int i = 0; i < (int)frame_sub.size(); i++)
     {
          frame_sub[i] = frame[i];
     }
     subSampleFrame(frame_sub, size_voxel_subsampling);
     keypoints.reserve(frame_sub.size());
     for (int i = 0; i < (int)frame_sub.size(); i++)
     {
          keypoints.push_back(frame_sub[i]);
     }
}

static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
     Eigen::Vector3d n = R.col(0);
     Eigen::Vector3d o = R.col(1);
     Eigen::Vector3d a = R.col(2);

     Eigen::Vector3d ypr(3);
     double y = atan2(n(1), n(0));
     double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
     double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
     ypr(0) = y;
     ypr(1) = p;
     ypr(2) = r;

     return ypr / M_PI * 180.0;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
{
     typedef typename Derived::Scalar Scalar_t;

     Scalar_t y = ypr(0) / 180.0 * M_PI;
     Scalar_t p = ypr(1) / 180.0 * M_PI;
     Scalar_t r = ypr(2) / 180.0 * M_PI;

     Eigen::Matrix<Scalar_t, 3, 3> Rz;
     Rz << cos(y), -sin(y), 0,
         sin(y), cos(y), 0,
         0, 0, 1;

     Eigen::Matrix<Scalar_t, 3, 3> Ry;
     Ry << cos(p), 0., sin(p),
         0., 1., 0.,
         -sin(p), 0., cos(p);

     Eigen::Matrix<Scalar_t, 3, 3> Rx;
     Rx << 1., 0., 0.,
         0., cos(r), -sin(r),
         0., sin(r), cos(r);

     return Rz * Ry * Rx;
}

Eigen::Matrix3d g2R(const Eigen::Vector3d &g)
{
     Eigen::Matrix3d R0;
     Eigen::Vector3d ng1 = g.normalized();
     Eigen::Vector3d ng2{0, 0, 1.0};
     R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
     double yaw = R2ypr(R0).x();
     R0 = ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
     // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
     return R0;
}
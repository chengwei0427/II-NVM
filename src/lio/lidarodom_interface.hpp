#ifndef LIDAR_ODOM_INTERFACE_HPP__
#define LIDAR_ODOM_INTERFACE_HPP__

#include "tools/imu.h"
#include <pcl/io/pcd_io.h>

namespace zjloc
{

     class lidarodomInterface
     {
     public:
          lidarodomInterface() {}
          virtual ~lidarodomInterface() {}

          lidarodomInterface(const lidarodomInterface &) = delete;
          lidarodomInterface &operator=(const lidarodomInterface &) = delete;

          virtual bool init(const std::string &config_yaml) = 0;

          virtual void pushData(std::vector<point3D>, std::pair<double, double> data) = 0;
          virtual void pushData(IMUPtr imu) = 0;

          virtual void run() = 0;

          virtual int getIndex() = 0;

          virtual void setFunc(std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> &fun) = 0;
          virtual void setFunc(std::function<bool(std::string &topic_name, SE3 &pose, double time)> &fun) = 0;
          virtual void setFunc(std::function<bool(std::string &topic_name, double time1, double time2)> &fun) = 0;

     private:
     };
}

#endif
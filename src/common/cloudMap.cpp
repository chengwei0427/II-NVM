/*
 * @Author: chengwei zhao
 * @LastEditors: cc
 * @Data:
 */
#include "cloudMap.hpp"

normalPoint::normalPoint(const Eigen::Vector3d &position_, const Eigen::Vector3d &normal_)
{
    position = position_.cast<float>();
    normal = normal_.cast<float>();
    reset();
}

normalPoint::normalPoint(const point3D &pt_)
{
    position = pt_.point.cast<float>();
    normal = pt_.normal.cast<float>();
    last_observe_time = pt_.timestamp;
}

void normalPoint::reset()
{
    last_observe_time = 0;
}

void normalPoint::setPosition(const Eigen::Vector3d &position_)
{
    position = position_.cast<float>();
}

Eigen::Vector3d normalPoint::getPosition() const
{
    return position.cast<double>();
}

void ::normalPoint::setNormal(const Eigen::Vector3d &normal_)
{
    normal = normal_.cast<float>();
}

Eigen::Vector3d normalPoint::getNormal() const
{
    return normal.cast<double>();
}

void normalPoint::setTimeStamp(const double &t_) { last_observe_time = t_; }

double normalPoint::getTimeStamp() { return last_observe_time; }

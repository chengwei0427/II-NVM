//
// Created by xiang on 2021/7/19.
//

#ifndef MAPPING_MATH_UTILS_H
#define MAPPING_MATH_UTILS_H

#include <glog/logging.h>
#include <boost/array.hpp>
#include <boost/math/tools/precision.hpp>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
// #include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include "eigen_types.h"

/// 常用的数学函数
namespace zjloc::math
{
    // using Eigen::MatrixXf;
    using namespace Eigen;
    using namespace Eigen::internal;
    using namespace Eigen::Architecture;

    // 常量定义
    constexpr double kDEG2RAD = M_PI / 180.0; // deg->rad
    constexpr double kRAD2DEG = 180.0 / M_PI; // rad -> deg
    constexpr double G_m_s2 = 9.81;           // 重力大小

    // 非法定义
    constexpr size_t kINVALID_ID = std::numeric_limits<size_t>::max();

    /**
     * 计算一个容器内数据的均值与对角形式协方差
     * @tparam C    容器类型
     * @tparam D    结果类型
     * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个D类型
     */
    template <typename C, typename D, typename Getter>
    void ComputeMeanAndCovDiag(const C &data, D &mean, D &cov_diag, Getter &&getter)
    {
        size_t len = data.size();
        assert(len > 1);
        // clang-format off
    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);
        // clang-format on
    }

    /// PCD点转Eigen
    template <typename PointT, typename S = double, int N>
    inline Eigen::Matrix<S, N, 1> ToEigen(const PointT &pt)
    {
        Eigen::Matrix<S, N, 1> v;
        if (N == 2)
        {
            v << pt.x, pt.y;
        }
        else if (N == 3)
        {
            v << pt.x, pt.y, pt.z;
        }

        return v;
    }

    template <typename S>
    inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<S> &v)
    {
        return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
    }

    template <typename S>
    inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<S> &v)
    {
        Eigen::Matrix<S, 3, 3> m;
        m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
        return m;
    }

    /**
     * pose 插值算法
     * @tparam T    数据类型
     * @tparam C 数据容器类型
     * @tparam FT 获取时间函数
     * @tparam FP 获取pose函数
     * @param query_time 查找时间
     * @param data  数据容器
     * @param take_pose_func 从数据中取pose的谓词，接受一个数据，返回一个SE3
     * @param result 查询结果
     * @param best_match_iter 查找到的最近匹配
     *
     * NOTE 要求query_time必须在data最大时间和最小时间之间(容许0.5s内误差)
     * data的map按时间排序
     * @return
     */
    template <typename T, typename C, typename FT, typename FP>
    inline bool PoseInterp(double query_time, C &&data, FT &&take_time_func, FP &&take_pose_func, SE3 &result,
                           T &best_match, float time_th = 0.5)
    {
        if (data.empty())
        {
            LOG(INFO) << "cannot interp because data is empty. ";
            return false;
        }

        double last_time = take_time_func(*data.rbegin());
        if (query_time > last_time)
        {
            if (query_time < (last_time + time_th))
            {
                // 尚可接受
                result = take_pose_func(*data.rbegin());
                best_match = *data.rbegin();
                return true;
            }
            return false;
        }

        auto match_iter = data.begin();
        for (auto iter = data.begin(); iter != data.end(); ++iter)
        {
            auto next_iter = iter;
            next_iter++;

            if (take_time_func(*iter) < query_time && take_time_func(*next_iter) >= query_time)
            {
                match_iter = iter;
                break;
            }
        }

        auto match_iter_n = match_iter;
        match_iter_n++;

        double dt = take_time_func(*match_iter_n) - take_time_func(*match_iter);
        double s = (query_time - take_time_func(*match_iter)) / dt; // s=0 时为第一帧，s=1时为next
        // 出现了 dt为0的bug
        if (fabs(dt) < 1e-6)
        {
            best_match = *match_iter;
            result = take_pose_func(*match_iter);
            return true;
        }

        SE3 pose_first = take_pose_func(*match_iter);
        SE3 pose_next = take_pose_func(*match_iter_n);
        result = {pose_first.unit_quaternion().slerp(s, pose_next.unit_quaternion()),
                  pose_first.translation() * (1 - s) + pose_next.translation() * s};
        best_match = s < 0.5 ? *match_iter : *match_iter_n;
        return true;
    }

    /**
     * Calculate cosine and sinc of sqrt(x2).
     * @param x2 the squared angle must be non-negative
     * @return a pair containing cos and sinc of sqrt(x2)
     */
    template <class scalar>
    inline std::pair<scalar, scalar> cos_sinc_sqrt(const scalar &x2)
    {
        using std::cos;
        using std::sin;
        using std::sqrt;
        static scalar const taylor_0_bound = boost::math::tools::epsilon<scalar>();
        static scalar const taylor_2_bound = sqrt(taylor_0_bound);
        static scalar const taylor_n_bound = sqrt(taylor_2_bound);

        assert(x2 >= 0 && "argument must be non-negative");

        // FIXME check if bigger bounds are possible
        if (x2 >= taylor_n_bound)
        {
            // slow fall-back solution
            scalar x = sqrt(x2);
            return std::make_pair(cos(x), sin(x) / x); // x is greater than 0.
        }

        // FIXME Replace by Horner-Scheme (4 instead of 5 FLOP/term, numerically more stable, theoretically cos and sinc can
        // be calculated in parallel using SSE2 mulpd/addpd)
        // TODO Find optimal coefficients using Remez algorithm
        static scalar const inv[] = {1 / 3., 1 / 4., 1 / 5., 1 / 6., 1 / 7., 1 / 8., 1 / 9.};
        scalar cosi = 1., sinc = 1;
        scalar term = -1 / 2. * x2;
        for (int i = 0; i < 3; ++i)
        {
            cosi += term;
            term *= inv[2 * i];
            sinc += term;
            term *= -inv[2 * i + 1] * x2;
        }

        return std::make_pair(cosi, sinc);
    }

    inline double exp(Vec3d &result, const Vec3d &vec, const double &scale = 1)
    {
        double norm2 = vec.squaredNorm();
        std::pair<double, double> cos_sinc = cos_sinc_sqrt(scale * scale * norm2);
        double mult = cos_sinc.second * scale;
        result = mult * vec;
        return cos_sinc.first;
    }

    inline SO3 exp(const Vec3d &vec, const double &scale = 1)
    {
        double norm2 = vec.squaredNorm();
        std::pair<double, double> cos_sinc = cos_sinc_sqrt(scale * scale * norm2);
        double mult = cos_sinc.second * scale;
        Vec3d result = mult * vec;
        return SO3(Quatd(cos_sinc.first, result[0], result[1], result[2]));
    }

    /**
     * pose 插值算法
     * @tparam T    数据类型
     * @param query_time 查找时间
     * @param data  数据
     * @param take_pose_func 从数据中取pose的谓词
     * @param result 查询结果
     * @param best_match_iter 查找到的最近匹配
     *
     * NOTE 要求query_time必须在data最大时间和最小时间之间，不会外推
     * data的map按时间排序
     * @return 插值是否成功
     */
    template <typename T>
    bool PoseInterp(double query_time, const std::map<double, T> &data, const std::function<SE3(const T &)> &take_pose_func,
                    SE3 &result, T &best_match)
    {
        if (data.empty())
        {
            LOG(INFO) << "data is empty";
            return false;
        }

        if (query_time > data.rbegin()->first)
        {
            LOG(INFO) << "query time is later than last, " << std::setprecision(18) << ", query: " << query_time
                      << ", end time: " << data.rbegin()->first;

            return false;
        }

        auto match_iter = data.begin();
        for (auto iter = data.begin(); iter != data.end(); ++iter)
        {
            auto next_iter = iter;
            next_iter++;

            if (iter->first < query_time && next_iter->first >= query_time)
            {
                match_iter = iter;
                break;
            }
        }

        auto match_iter_n = match_iter;
        match_iter_n++;
        assert(match_iter_n != data.end());

        double dt = match_iter_n->first - match_iter->first;
        double s = (query_time - match_iter->first) / dt; // s=0 时为第一帧，s=1时为next
        if (dt > 0.5)                                     // false when time large
            return false;
        SE3 pose_first = take_pose_func(match_iter->second);
        SE3 pose_next = take_pose_func(match_iter_n->second);
        result = {pose_first.unit_quaternion().slerp(s, pose_next.unit_quaternion()),
                  pose_first.translation() * (1 - s) + pose_next.translation() * s};
        best_match = s < 0.5 ? match_iter->second : match_iter_n->second;
        return true;
    }

} // namespace zjloc::math

#endif // MAPPING_MATH_UTILS_H


#ifndef LLS_SLAM_MODELS_NANO_FLANN_NANO_FLANN_PCL_HPP_
#define LLS_SLAM_MODELS_NANO_FLANN_NANO_FLANN_PCL_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include "nanoflann.hpp"

namespace nanoflann
{
  // Adapter class to give to nanoflann the same "look and fell" of pcl::KdTreeFLANN.
  // limited to squared distance between 3D points
  template <typename PointT>
  class KdTreeFLANN
  {
  public:
    typedef boost::shared_ptr<KdTreeFLANN<PointT>> Ptr;
    typedef boost::shared_ptr<const KdTreeFLANN<PointT>> ConstPtr;

    typedef typename pcl::PointCloud<PointT> PointCloud;
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

    typedef boost::shared_ptr<std::vector<int>> IndicesPtr;
    typedef boost::shared_ptr<const std::vector<int>> IndicesConstPtr;

    KdTreeFLANN(bool sorted = true);

    KdTreeFLANN(const KdTreeFLANN<PointT> &k);

    void setEpsilon(float eps);

    void setSortedResults(bool sorted);

    inline Ptr makeShared()
    {
      return Ptr(new KdTreeFLANN<PointT>(*this));
    }

    void setInputCloud(const PointCloudPtr &cloud, const IndicesConstPtr &indices = IndicesConstPtr());

    int nearestKSearch(const PointT &point, int k, std::vector<int> &k_indices,
                       std::vector<float> &k_sqr_distances) const;

    int radiusSearch(const PointT &point, double radius, std::vector<int> &k_indices,
                     std::vector<float> &k_sqr_distances) const;

  private:
    nanoflann::SearchParams _params;

    struct PointCloud_Adaptor
    {
      inline size_t kdtree_get_point_count() const;
      inline float kdtree_get_pt(const size_t idx, int dim) const;
      template <class BBOX>
      bool kdtree_get_bbox(BBOX &) const
      {
        return false;
      }
      PointCloudConstPtr pcl;
      IndicesConstPtr indices;
    };

    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::SO3_Adaptor<float, PointCloud_Adaptor>, PointCloud_Adaptor, 3,
                                                int>
        KDTreeFlann_PCL_SO3;

    PointCloud_Adaptor _adaptor;

    KDTreeFlann_PCL_SO3 _kdtree;
  };

  //---------- Definitions ---------------------

  template <typename PointT>
  inline KdTreeFLANN<PointT>::KdTreeFLANN(bool sorted) : _kdtree(3, _adaptor)
  {
    _params.sorted = sorted;
  }

  template <typename PointT>
  inline void KdTreeFLANN<PointT>::setEpsilon(float eps)
  {
    _params.eps = eps;
  }

  template <typename PointT>
  inline void KdTreeFLANN<PointT>::setSortedResults(bool sorted)
  {
    _params.sorted = sorted;
  }

  template <typename PointT>
  inline void KdTreeFLANN<PointT>::setInputCloud(const KdTreeFLANN::PointCloudPtr &cloud, const IndicesConstPtr &indices)
  {
    _adaptor.pcl = cloud;
    _adaptor.indices = indices;
    _kdtree.buildIndex();
  }

  template <typename PointT>
  inline int KdTreeFLANN<PointT>::nearestKSearch(const PointT &point, int num_closest, std::vector<int> &k_indices,
                                                 std::vector<float> &k_sqr_distances) const
  {
    k_indices.resize(num_closest);
    k_sqr_distances.resize(num_closest);

    nanoflann::KNNResultSet<float, int> resultSet(num_closest);
    resultSet.init(k_indices.data(), k_sqr_distances.data());
    _kdtree.findNeighbors(resultSet, point.data, nanoflann::SearchParams());
    return resultSet.size();
  }

  template <typename PointT>
  inline int KdTreeFLANN<PointT>::radiusSearch(const PointT &point, double radius, std::vector<int> &k_indices,
                                               std::vector<float> &k_sqr_distances) const
  {
    std::vector<std::pair<int, float>> indices_dist;
    indices_dist.reserve(128);
    RadiusResultSet<float, int> resultSet(static_cast<float>(radius * radius), indices_dist);
    _kdtree.findNeighbors(resultSet, point.data, _params);
    const size_t nFound = resultSet.size();
    if (_params.sorted)
    {
      std::sort(indices_dist.begin(), indices_dist.end(), IndexDist_Sorter());
    }
    k_indices.resize(nFound);
    k_sqr_distances.resize(nFound);
    for (int i = 0; i < nFound; i++)
    {
      k_indices[i] = indices_dist[i].first;
      k_sqr_distances[i] = indices_dist[i].second;
    }
    return nFound;
  }

  template <typename PointT>
  inline size_t KdTreeFLANN<PointT>::PointCloud_Adaptor::kdtree_get_point_count() const
  {
    if (indices)
      return indices->size();
    if (pcl)
      return pcl->points.size();
    return 0;
  }

  template <typename PointT>
  inline float KdTreeFLANN<PointT>::PointCloud_Adaptor::kdtree_get_pt(const size_t idx, int dim) const
  {
    const PointT &p = (indices) ? pcl->points[(*indices)[idx]] : pcl->points[idx];
    if (dim == 0)
      return p.x;
    else if (dim == 1)
      return p.y;
    else if (dim == 2)
      return p.z;
    else
      return 0.0;
  }

} // namespace nanoflann

#endif

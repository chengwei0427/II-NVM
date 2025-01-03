#include <yaml-cpp/yaml.h>
#include "lio/lidarodom.h"

#include <tbb/tbb.h>

#define P_USE_TBB_ 1 //   是否使用tbb加速

namespace zjloc::loam_tbbic_normal
{

     lidarodom::lidarodom(/* args */)
     {
          current_state = new loam::state(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

          index_frame = 1;
          points_world.reset(new pcl::PointCloud<pcl::PointXYZI>());
     }

     lidarodom::~lidarodom() {}

     void lidarodom::loadOptions()
     {
          auto yaml = YAML::LoadFile(config_yaml_);
          options_.surf_res = yaml["odometry"]["surf_res"].as<double>();
          options_.delay_time = yaml["odometry"]["delay_time"].as<double>();
          options_.max_iteration = yaml["odometry"]["max_iteration"].as<int>();
          options_.log_print = yaml["odometry"]["log_print"].as<bool>();
          options_.capacity_ = 400000; //   ref to ig_lio

          options_.size_voxel_map = yaml["odometry"]["size_voxel_map"].as<double>();
          options_.min_distance_points = yaml["odometry"]["min_distance_points"].as<double>();
          options_.max_num_points_in_voxel = yaml["odometry"]["max_num_points_in_voxel"].as<int>();
          options_.max_distance = yaml["odometry"]["max_distance"].as<double>();
          options_.weight_alpha = yaml["odometry"]["weight_alpha"].as<double>();
          options_.weight_neighborhood = yaml["odometry"]["weight_neighborhood"].as<double>();
          options_.max_dist_to_plane_icp = yaml["odometry"]["max_dist_to_plane_icp"].as<double>();
          options_.init_num_frames = yaml["odometry"]["init_num_frames"].as<int>();
          options_.voxel_neighborhood = yaml["odometry"]["voxel_neighborhood"].as<int>();
          options_.max_number_neighbors = yaml["odometry"]["max_number_neighbors"].as<int>();
          options_.threshold_voxel_occupancy = yaml["odometry"]["threshold_voxel_occupancy"].as<int>();
          options_.estimate_normal_from_neighborhood = yaml["odometry"]["estimate_normal_from_neighborhood"].as<bool>();
          options_.min_number_neighbors = yaml["odometry"]["min_number_neighbors"].as<int>();
          options_.power_planarity = yaml["odometry"]["power_planarity"].as<double>();
          options_.num_closest_neighbors = yaml["odometry"]["num_closest_neighbors"].as<int>();

          options_.sampling_rate = yaml["odometry"]["sampling_rate"].as<double>();
          options_.max_num_residuals = yaml["odometry"]["max_num_residuals"].as<int>();
          options_.min_num_residuals = yaml["odometry"]["min_num_residuals"].as<int>();

          options_.thres_orientation_norm = yaml["odometry"]["thres_orientation_norm"].as<double>();
          options_.thres_translation_norm = yaml["odometry"]["thres_translation_norm"].as<double>();
          adapt_sample_res = options_.sampling_rate;
     }

     bool lidarodom::init(const std::string &config_yaml)
     {
          config_yaml_ = config_yaml;
          StaticIMUInit::Options imu_init_options;
          imu_init_options.use_speed_for_static_checking_ = false; // 本节数据不需要轮速计
          imu_init_ = StaticIMUInit(imu_init_options);

          auto yaml = YAML::LoadFile(config_yaml_);

          // lidar和IMU外参
          std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
          std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
          Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
          Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
          std::cout << yaml["mapping"]["extrinsic_R"] << std::endl;
          Eigen::Quaterniond q_IL(lidar_R_wrt_IMU);
          q_IL.normalized();
          lidar_R_wrt_IMU = q_IL;
          // init TIL
          TIL_ = SE3(q_IL, lidar_T_wrt_IMU);
          R_imu_lidar = lidar_R_wrt_IMU;
          t_imu_lidar = lidar_T_wrt_IMU;
          std::cout << "RIL:\n"
                    << R_imu_lidar << std::endl;
          std::cout << "tIL:" << t_imu_lidar.transpose() << std::endl;

          loadOptions();

          return true;
     }

     void lidarodom::pushData(std::vector<point3D> msg, std::pair<double, double> data)
     {
          if (data.first < last_timestamp_lidar_)
          {
               LOG(ERROR) << "lidar loop back, clear buffer";
               lidar_buffer_.clear();
               time_buffer_.clear();
          }

          mtx_buf.lock();
          lidar_buffer_.push_back(msg);
          time_buffer_.push_back(data);
          last_timestamp_lidar_ = data.first;
          mtx_buf.unlock();
          cond.notify_one();
     }

     void lidarodom::pushData(IMUPtr imu)
     {
          double timestamp = imu->timestamp_;
          if (timestamp < last_timestamp_imu_)
          {
               LOG(WARNING) << "imu loop back, clear buffer";
               imu_buffer_.clear();
          }

          last_timestamp_imu_ = timestamp;

          mtx_buf.lock();
          imu_buffer_.emplace_back(imu);
          mtx_buf.unlock();
          cond.notify_one();
     }

     void lidarodom::run()
     {
          while (true)
          {
               std::vector<loam::MeasureGroup> measurements;
               std::unique_lock<std::mutex> lk(mtx_buf);
               cond.wait(lk, [&]
                         { return (measurements = getMeasureMents()).size() != 0; });
               lk.unlock();

               for (auto &m : measurements)
               {
                    zjloc::common::Timer::Evaluate([&]()
                                                   { ProcessMeasurements(m); },
                                                   "processMeasurement");

                    {
                         auto real_time = std::chrono::high_resolution_clock::now();
                         static std::chrono::system_clock::time_point prev_real_time = real_time;

                         if (real_time - prev_real_time > std::chrono::seconds(5))
                         {
                              auto data_time = m.lidar_end_time_;
                              static double prev_data_time = data_time;
                              auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time - prev_real_time).count() * 0.001;
                              auto delta_sim = data_time - prev_data_time;
                              printf("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);

                              prev_data_time = data_time;
                              prev_real_time = real_time;
                         }
                    }
               }
          }
     }

     void lidarodom::ProcessMeasurements(loam::MeasureGroup &meas)
     {
          measures_ = meas;

          if (imu_need_init_)
          {
               TryInitIMU();
               return;
          }

          // std::cout << ANSI_DELETE_LAST_LINE << "============== process frame: " << index_frame << std::endl;
          // ANSI_SCREEN_FLUSH("============== process frame: %d", index_frame);
          // std::cout << ANSI_DELETE_LAST_LINE;
          std::cout << ANSI_COLOR_GREEN << "============== process frame: "
                    << index_frame << ANSI_COLOR_RESET << std::endl;
          // ANSI_SCREEN_FLUSH("");
          imu_states_.clear(); //   need clear here

          // 利用IMU数据进行状态预测
          zjloc::common::Timer::Evaluate([&]()
                                         { Predict(); },
                                         "predict");
          //
          zjloc::common::Timer::Evaluate([&]()
                                         { stateInitialization(); },
                                         "state init");

          std::vector<point3D> const_surf;
          const_surf.insert(const_surf.end(), meas.lidar_.begin(), meas.lidar_.end());
          // const_surf.assign(meas.lidar_.begin(), meas.lidar_.end());

          loam::cloudFrame *p_frame;
          zjloc::common::Timer::Evaluate([&]()
                                         { p_frame = buildFrame(const_surf, current_state,
                                                                meas.lidar_begin_time_,
                                                                meas.lidar_end_time_); },
                                         "build frame");

          //   lio
          zjloc::common::Timer::Evaluate([&]()
                                         { poseEstimation(p_frame); },
                                         "poseEstimate");

          //   观测
          SE3 pose_of_lo_ = SE3(current_state->rotation, current_state->translation);

          zjloc::common::Timer::Evaluate([&]()
                                         { eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2); },
                                         "eskf_obs");

          zjloc::common::Timer::Evaluate([&]()
                                         {
               std::string laser_topic = "laser";
               pub_pose_to_ros(laser_topic, pose_of_lo_, meas.lidar_end_time_);

               laser_topic = "world";
               SE3 pose_of_world = SE3(RIG_,Eigen::Vector3d(0,0,0));
               pub_pose_to_ros(laser_topic, pose_of_world, meas.lidar_end_time_);

               laser_topic = "velocity";
               SE3 pred_pose = eskf_.GetNominalSE3();
               Eigen::Vector3d vel_world = eskf_.GetNominalVel();
               Eigen::Vector3d vel_base = pred_pose.rotationMatrix().inverse()*vel_world;
               pub_data_to_ros(laser_topic, vel_base.x(), 0);
               if(index_frame%8==0)
               {
                    laser_topic = "dist";
                    static Eigen::Vector3d last_t = Eigen::Vector3d::Zero();
                    Eigen::Vector3d t = pred_pose.translation();
                    static double dist = 0;
                    dist += (t - last_t).norm();
                    last_t = t;
                    pub_data_to_ros(laser_topic, dist, 0);
               } },
                                         "pub cloud");

          p_frame->p_state = new loam::state(current_state, true);
          // all_cloud_frame.push_back(p_frame); //        保存这个，特别费内存
          loam::state *tmp_state = new loam::state(current_state, true);
          all_state_frame.push_back(tmp_state);
          current_state = new loam::state(current_state, false);

          index_frame++;
          p_frame->release();
          std::vector<point3D>().swap(meas.lidar_);
          std::vector<point3D>().swap(const_surf);
     }

     void lidarodom::poseEstimation(loam::cloudFrame *p_frame)
     {
          //   TODO: check current_state data
          if (index_frame > 1)
          {
               zjloc::common::Timer::Evaluate([&]()
                                              { optimize(p_frame); },
                                              "optimize");
          }

          bool add_points = true;
          SE3 curr_pose_ = SE3(p_frame->p_state->rotation, p_frame->p_state->translation);
          // Eigen::Matrix4d delta_p = (last_pose_.inverse() * curr_pose_).matrix();
          // if (index_frame < 30 || delta_p.block<3, 1>(0, 3).norm() > 0.5 ||
          //     Sophus::SO3d(delta_p.block<3, 3>(0, 0)).log().norm() > 0.1) //   5.7度
          //      add_points = true;

          if (add_points)
          { //   update map here
               last_pose_ = curr_pose_;
               zjloc::common::Timer::Evaluate([&]()
                                              { map_incremental(p_frame); },
                                              "map update");
          }
     }

     void lidarodom::optimize(loam::cloudFrame *p_frame)
     {
          loam::state *curr_state = p_frame->p_state;
          Eigen::Quaterniond end_quat = curr_state->rotation;
          Eigen::Vector3d end_t = curr_state->translation;

          // std::vector<point3D> surf_keypoints;
          //   FIXME: 需要增加一个自适应滤波 res_new = res_old*(N_scan/N_last);
          // gridSampling(p_frame->point_surf, surf_keypoints, options_.sampling_rate * options_.surf_res);
          gridSampling(p_frame->point_surf, p_frame->surf_keypoints, adapt_sample_res * options_.surf_res);
          {
               std::mt19937_64 g;
               std::shuffle(p_frame->surf_keypoints.begin(), p_frame->surf_keypoints.end(), g);
          }
          double tt = p_frame->surf_keypoints.size() / options_.max_num_residuals;
          if (tt < 1)
          {
               adapt_sample_res = 0.7 * adapt_sample_res; //   0.5~options_.sample_rate
               if (adapt_sample_res < 0.5)
                    adapt_sample_res = 0.5;
               // gridSampling(p_frame->point_surf, surf_keypoints, adapt_sample_res * options_.surf_res);
          }
          else
               adapt_sample_res = options_.sampling_rate;
          // std::cout << "cloud size: " << p_frame->point_surf.size() << ", after ds:" << surf_keypoints.size() << " res:" << adapt_sample_res << std::endl;

          size_t num_size = p_frame->point_surf.size();
          auto transformKeypoints = [&](std::vector<point3D> &point_frame)
          {
               Eigen::Matrix3d R = end_quat.normalized().toRotationMatrix();
               Eigen::Vector3d t = end_t;
#ifdef P_USE_TBB_
               tbb::parallel_for(size_t(0), point_frame.size(), [&](size_t id)
                                 { point_frame[id].point = R * (TIL_ * point_frame[id].raw_point) + t; 
                                   point_frame[id].normal = R * TIL_.rotationMatrix() * point_frame[id].raw_normal; });
#else
               for (auto &keypoint : point_frame)
               {
                    keypoint.point = R * (TIL_ * keypoint.raw_point) + t;
                    keypoint.normal = R * TIL_.rotationMatrix() * keypoint.raw_normal;
               }
#endif
          };
          for (int iter(0); iter < options_.max_iteration; iter++)
          {
               // ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1 / (1.5e-3));
               ceres::LossFunction *loss_function = new ceres::HuberLoss(0.5);
               ceres::Problem::Options problem_options;
               ceres::Problem problem(problem_options);

               auto *parameterization = new ceres::EigenQuaternionParameterization();

               problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
               problem.AddParameterBlock(&end_t.x(), 3);

               std::vector<ceres::CostFunction *> surfFactor;
               addSurfCostFactor(surfFactor, p_frame->surf_keypoints, p_frame);

               int surf_num = 0;
               if (options_.log_print)
                    std::cout << "get factor: " << surfFactor.size() << std::endl;
               for (auto &e : surfFactor)
               {
                    surf_num++;
                    problem.AddResidualBlock(e, loss_function, &end_t.x(), &end_quat.x());
               }
               //   release
               std::vector<ceres::CostFunction *>().swap(surfFactor);

               if (surf_num < options_.min_num_residuals)
               {
                    std::stringstream ss_out;
                    ss_out << "[Optimization] Error : not enough keypoints selected in ct-icp !" << std::endl;
                    ss_out << "[Optimization] number_of_residuals : " << surf_num << std::endl;
                    std::cout << "ERROR: " << ss_out.str();
                    return;
               }

               ceres::Solver::Options options;
               options.max_num_iterations = 5;
               options.num_threads = 6;
               options.minimizer_progress_to_stdout = false;
               options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

               // ceres::Solver::Options options;
               // options.linear_solver_type = ceres::DENSE_SCHUR;
               // options.trust_region_strategy_type = ceres::DOGLEG;
               // options.max_num_iterations = 10;
               // options.minimizer_progress_to_stdout = false;
               // options.num_threads = 6;

               ceres::Solver::Summary summary;

               ceres::Solve(options, &problem, &summary);

               double diff_trans = 0, diff_rot = 0;
               diff_trans += (current_state->translation - end_t).norm();
               diff_rot += AngularDistance(current_state->rotation, end_quat);

               p_frame->p_state->translation = end_t;
               p_frame->p_state->rotation = end_quat;

               current_state->translation = end_t;
               current_state->rotation = end_quat;

               if (diff_rot < options_.thres_orientation_norm &&
                   diff_trans < options_.thres_translation_norm)
               {
                    if (options_.log_print)
                         std::cout << "Optimization: Finished with N=" << iter << " ICP iterations" << std::endl;
                    break;
               }
          }

          // transformKeypoints(p_frame->surf_keypoints);
          // transformKeypoints(p_frame->point_surf);
     }

     loam::Neighborhood lidarodom::computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points)
     {
          loam::Neighborhood neighborhood;
          // Compute the normals
          Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
          for (auto &point : points)
          {
               barycenter += point;
          }

          barycenter /= (double)points.size();
          neighborhood.center = barycenter;

          Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
          for (auto &point : points)
          {
               for (int k = 0; k < 3; ++k)
                    for (int l = k; l < 3; ++l)
                         covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                                    (point(l) - barycenter(l));
          }
          covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
          covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
          covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
          neighborhood.covariance = covariance_Matrix;
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
          Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
          neighborhood.normal = normal;

          double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));
          double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
          double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
          neighborhood.a2D = (sigma_2 - sigma_3) / sigma_1;

          if (neighborhood.a2D != neighborhood.a2D)
          {
               throw std::runtime_error("error");
          }

          return neighborhood;
     }

     void lidarodom::addSurfCostFactor(std::vector<ceres::CostFunction *> &surf,
                                       std::vector<point3D> &keypoints, const loam::cloudFrame *p_frame)
     {

          auto estimatePointNeighborhood = [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
                                               Eigen::Vector3d &location, double &planarity_weight)
          {
               auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
               planarity_weight = std::pow(neighborhood.a2D, options_.power_planarity);

               if (neighborhood.normal.dot(p_frame->p_state->translation - location) < 0)
               {
                    neighborhood.normal = -1.0 * neighborhood.normal;
               }
               return neighborhood;
          };

          double lambda_weight = std::abs(options_.weight_alpha);
          double lambda_neighborhood = std::abs(options_.weight_neighborhood);
          const double kMaxPointToPlane = options_.max_dist_to_plane_icp;
          const double sum = lambda_weight + lambda_neighborhood;

          lambda_weight /= sum;
          lambda_neighborhood /= sum;

          const short nb_voxels_visited = p_frame->frame_id < options_.init_num_frames
                                              ? 2
                                              : options_.voxel_neighborhood;

          const int kThresholdCapacity = p_frame->frame_id < options_.init_num_frames
                                             ? 1
                                             : options_.threshold_voxel_occupancy;

          size_t num = keypoints.size();
          int num_residuals = 0;

          for (int k = 0; k < num; k++)
          {
               auto &keypoint = keypoints[k];
               auto &raw_point = keypoint.raw_point;
               keypoint.point = p_frame->p_state->rotation * (TIL_ * raw_point) + p_frame->p_state->translation;
               keypoint.normal = p_frame->p_state->rotation * TIL_.unit_quaternion() * keypoint.raw_normal;

               std::vector<voxel> voxels;
               auto vector_neighbors = searchNeighbors(voxel_map, keypoint.point,
                                                       keypoint.normal, nb_voxels_visited,
                                                       options_.size_voxel_map,
                                                       options_.max_number_neighbors,
                                                       kThresholdCapacity,
                                                       options_.estimate_normal_from_neighborhood
                                                           ? nullptr
                                                           : &voxels);

               if (vector_neighbors.size() < options_.min_number_neighbors)
                    continue;

               double weight;

               Eigen::Vector3d location = TIL_ * raw_point;

               auto neighborhood = estimatePointNeighborhood(vector_neighbors, location /*raw_point*/, weight);

               weight = lambda_weight * weight + lambda_neighborhood *
                                                     std::exp(-(vector_neighbors[0] -
                                                                keypoint.point)
                                                                   .norm() /
                                                              (kMaxPointToPlane *
                                                               options_.min_number_neighbors));

               double point_to_plane_dist;
               std::set<voxel> neighbor_voxels;
               for (int i(0); i < options_.num_closest_neighbors; ++i)
               {
                    point_to_plane_dist = std::abs((keypoint.point - vector_neighbors[i]).transpose() * neighborhood.normal);

                    if (point_to_plane_dist < options_.max_dist_to_plane_icp)
                    {

                         num_residuals++;

                         Eigen::Vector3d norm_vector = neighborhood.normal;
                         norm_vector.normalize();

                         double norm_offset = -norm_vector.dot(vector_neighbors[i]);

                         Eigen::Vector3d point_end = p_frame->p_state->rotation.inverse() * keypoints[k].point -
                                                     p_frame->p_state->rotation.inverse() * p_frame->p_state->translation;

                         auto *cost_function = zjloc::loam::PointToPlaneFunctor::Create(vector_neighbors[0],
                                                                                        point_end, norm_vector, weight);

                         surf.push_back(cost_function);
                    }
               }

               if (num_residuals >= options_.max_num_residuals)
                    break;
          }
     }

     ///  ===================  for search neighbor  ===================================================
     using pair_distance_t = std::tuple<double, Eigen::Vector3d, voxel>;

     struct comparator
     {
          bool operator()(const pair_distance_t &left, const pair_distance_t &right) const
          {
               return std::get<0>(left) < std::get<0>(right);
          }
     };

     using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;

     std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
     lidarodom::searchNeighbors(const voxelHashMap3 &map, const Eigen::Vector3d &point,
                                const Eigen::Vector3d &normal, int nb_voxels_visited,
                                double size_voxel_map, int max_num_neighbors,
                                int threshold_voxel_capacity, std::vector<voxel> *voxels)
     {

          if (voxels != nullptr)
               voxels->reserve(max_num_neighbors);

          short kx = static_cast<short>(point[0] / size_voxel_map);
          short ky = static_cast<short>(point[1] / size_voxel_map);
          short kz = static_cast<short>(point[2] / size_voxel_map);

          auto calculateAngle = [&](const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
          {
               double dot_product = v1.dot(v2);
               return std::acos(dot_product) * 180.0 / M_PI; //   degree
          };

          priority_queue_t priority_queue;

          voxel voxel_temp(kx, ky, kz);
          for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx)
          {
               for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy)
               {
                    for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz)
                    {
                         voxel_temp.x = kxx;
                         voxel_temp.y = kyy;
                         voxel_temp.z = kzz;

                         auto search = map.find(voxel_temp);
                         if (search != map.end())
                         {
                              const auto &voxel_block = search.value();
                              if (voxel_block->second->NumPoints() < threshold_voxel_capacity)
                                   continue;
                              for (int i(0); i < voxel_block->second->NumPoints(); ++i)
                              {
                                   auto &neighbor = voxel_block->second->points[i];
                                   double angle = calculateAngle(neighbor.getNormal(), normal);
                                   if (angle > 100.0)
                                        continue;

                                   Eigen::Vector3d neighbor_point = neighbor.getPosition();
                                   double distance = (neighbor_point - point).norm();
                                   if (priority_queue.size() == max_num_neighbors)
                                   {
                                        if (distance < std::get<0>(priority_queue.top()))
                                        {
                                             priority_queue.pop();
                                             priority_queue.emplace(distance, neighbor_point, voxel_temp);
                                        }
                                   }
                                   else
                                        priority_queue.emplace(distance, neighbor_point, voxel_temp);
                              }
                         }
                    }
               }
          }

          auto size = priority_queue.size();
          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> closest_neighbors(size);
          if (voxels != nullptr)
          {
               voxels->resize(size);
          }
          for (auto i = 0; i < size; ++i)
          {
               closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());
               if (voxels != nullptr)
                    (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
               priority_queue.pop();
          }

          return closest_neighbors;
     }

     void lidarodom::addPointToMap(voxelHashMap3 &map, const point3D &pt, double voxel_size,
                                   int max_num_points_in_voxel, double min_distance_points,
                                   int min_num_points)
     {
          short kx = static_cast<short>(pt.point[0] / voxel_size);
          short ky = static_cast<short>(pt.point[1] / voxel_size);
          short kz = static_cast<short>(pt.point[2] / voxel_size);

          addPointToPcl(points_world, pt.point, pt.intensity);

          voxelHashMap3::iterator search = map.find(voxel(kx, ky, kz));

          auto calculateAngle = [&](const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
          {
               double dot_product = v1.dot(v2);
               return std::acos(dot_product) * 180.0 / M_PI; //   degree
          };

          if (search != map.end())
          {
               auto &voxel_block = (search.value());

               double angle = calculateAngle(pt.normal, voxel_block->second->normal); //   和平均主方向夹角
               if (angle > 100.0)
               {
                    std::shared_ptr<voxelBlock3> block = std::make_shared<voxelBlock3>(max_num_points_in_voxel);
                    voxel_block->second.reset(new voxelBlock3(max_num_points_in_voxel));
                    block->AddPoint(normalPoint(pt));
               }
               else if (!voxel_block->second->IsFull())
               {
                    double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
                    for (int i(0); i < voxel_block->second->NumPoints(); ++i)
                    {
                         auto &_point = voxel_block->second->points[i];
                         double sq_dist = (_point.getPosition() - pt.point).squaredNorm();
                         if (sq_dist < sq_dist_min_to_points)
                         {
                              sq_dist_min_to_points = sq_dist;
                         }
                    }
                    if (sq_dist_min_to_points > (min_distance_points * min_distance_points))
                    {
                         if (min_num_points <= 0 || voxel_block->second->NumPoints() >= min_num_points)
                         {
                              voxel_block->second->AddPoint(normalPoint(pt));
                              // addPointToPcl(points_world, point, intensity);
                         }
                    }
               }
               grids_cache_.splice(grids_cache_.begin(), grids_cache_, search->second);
               map[voxel(kx, ky, kz)] = grids_cache_.begin();
          }
          else
          {
               if (min_num_points <= 0)
               {
                    std::shared_ptr<voxelBlock3> block = std::make_shared<voxelBlock3>(max_num_points_in_voxel);
                    block->AddPoint(normalPoint(pt));

                    //   LRU cache
                    grids_cache_.push_front({voxel(kx, ky, kz), std::move(block)});
                    map.insert({voxel(kx, ky, kz), grids_cache_.begin()});
                    // map[voxel(kx, ky, kz)] = std::move(block);
                    if (map.size() > options_.capacity_)
                    {
                         map.erase(grids_cache_.back().first);
                         grids_cache_.pop_back();
                    }
               }
          }
     }

     void lidarodom::addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, const Eigen::Vector3d &point, const double &intensity)
     {
          pcl::PointXYZI cloudTemp;

          cloudTemp.x = point.x();
          cloudTemp.y = point.y();
          cloudTemp.z = point.z();
          cloudTemp.intensity = intensity;
          // cloudTemp.intensity = 50 * (point.z() - p_frame->p_state->translation.z());
          pcl_points->points.push_back(cloudTemp);
     }

     void lidarodom::map_incremental(loam::cloudFrame *p_frame, int min_num_points)
     {
          std::vector<point3D> point_map;
          if (index_frame < 20)
               point_map = p_frame->point_surf;
          else
               point_map = p_frame->surf_keypoints;

          for (auto &point : p_frame->point_surf)
          // for (auto &point : point_map)
          {
               point.point = p_frame->p_state->rotation * (TIL_ * point.raw_point) + p_frame->p_state->translation;
               point.normal = p_frame->p_state->rotation * TIL_.unit_quaternion() * point.raw_normal;
               point.normal.normalize();

               addPointToMap(voxel_map, point,
                             options_.size_voxel_map, options_.max_num_points_in_voxel,
                             options_.min_distance_points, min_num_points);
          }

          {
               std::string laser_topic = "laser";
               pub_cloud_to_ros(laser_topic, points_world, p_frame->time_frame_end);
          }
          points_world->clear();
          std::cout << "map size: " << voxel_map.size() << std::endl;
     }

     loam::cloudFrame *lidarodom::buildFrame(std::vector<point3D> &const_surf, loam::state *cur_state,
                                             double timestamp_begin, double timestamp_end)
     {
          std::vector<point3D> frame_surf(const_surf);
          if (index_frame >= 2)
          {
               //   FIXME: 低速场景，未同步传感器，畸变矫正会有副作用 -_-||  可以考虑，只做旋转去畸变
               zjloc::common::Timer::Evaluate([&]()
                                              { Undistort(frame_surf, false); },
                                              "cloudFrame");
          }
          //     else
          if (0)
          {
               for (auto &point_temp : frame_surf)
               {
                    transformPoint(point_temp, cur_state->rotation, cur_state->translation, R_imu_lidar, t_imu_lidar);
               }
          }
          loam::cloudFrame *p_frame = new loam::cloudFrame(frame_surf, const_surf, cur_state);

          p_frame->time_frame_begin = timestamp_begin;
          p_frame->time_frame_end = timestamp_end;

          p_frame->frame_id = index_frame;

          return p_frame;
     }

     void lidarodom::stateInitialization()
     {
          if (index_frame < 2) //   only first frame
          {
               current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
               current_state->translation = imu_states_.back().p_;
               last_pose_ = SE3(current_state->rotation, current_state->translation); //   init here
          }
          else
          {
               //   use imu predict
               current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
               current_state->translation = imu_states_.back().p_;
          }
     }

     std::vector<loam::MeasureGroup> lidarodom::getMeasureMents()
     {
          std::vector<loam::MeasureGroup> measurements;
          while (true)
          {
               if (imu_buffer_.empty())
                    return measurements;

               if (lidar_buffer_.empty())
                    return measurements;

               if (imu_buffer_.back()->timestamp_ - time_curr < options_.delay_time)
                    return measurements;

               loam::MeasureGroup meas;

               meas.lidar_ = lidar_buffer_.front();
               meas.lidar_begin_time_ = time_buffer_.front().first;
               meas.lidar_end_time_ = meas.lidar_begin_time_ + time_buffer_.front().second;
               lidar_buffer_.pop_front();
               time_buffer_.pop_front();

               time_curr = meas.lidar_end_time_;

               double imu_time = imu_buffer_.front()->timestamp_;
               meas.imu_.clear();
               while ((!imu_buffer_.empty()) && (imu_time < meas.lidar_end_time_))
               {
                    imu_time = imu_buffer_.front()->timestamp_;
                    if (imu_time > meas.lidar_end_time_)
                    {
                         break;
                    }
                    meas.imu_.push_back(imu_buffer_.front());
                    imu_buffer_.pop_front();
               }

               if (meas.imu_.empty())
               {
                    std::cout << "no imu before current lidar frame." << std::endl;
                    return measurements;
               }

               if (!imu_buffer_.empty())
                    meas.imu_.push_back(imu_buffer_.front()); //   added for Interp

               measurements.push_back(meas);
          }
     }

     void lidarodom::Predict()
     {
          imu_states_.emplace_back(eskf_.GetNominalState());

          /// 对IMU状态进行预测
          double time_current = measures_.lidar_end_time_;
          Vec3d last_gyr, last_acc;
          for (auto &imu : measures_.imu_)
          {
               double time_imu = imu->timestamp_;
               if (imu->timestamp_ <= time_current)
               {
                    if (last_imu_ == nullptr)
                         last_imu_ = imu;
                    eskf_.Predict(*imu);
                    imu_states_.emplace_back(eskf_.GetNominalState());
                    last_imu_ = imu;
               }
               else
               {
                    double dt_1 = time_imu - time_current;
                    double dt_2 = time_current - last_imu_->timestamp_;
                    double w1 = dt_1 / (dt_1 + dt_2);
                    double w2 = dt_2 / (dt_1 + dt_2);
                    Eigen::Vector3d acc_temp = w1 * last_imu_->acce_ + w2 * imu->acce_;
                    Eigen::Vector3d gyr_temp = w1 * last_imu_->gyro_ + w2 * imu->gyro_;
                    IMUPtr imu_temp = std::make_shared<zjloc::IMU>(time_current, gyr_temp, acc_temp);
                    eskf_.Predict(*imu_temp);
                    imu_states_.emplace_back(eskf_.GetNominalState());
                    last_imu_ = imu_temp;
               }
          }
          // std::cout << imu_states_.size() << ", " << last_imu_->timestamp_ << std::endl;
     }

     void lidarodom::Undistort(std::vector<point3D> &points, bool only_R)
     {
          // auto &cloud = measures_.lidar_;
          auto imu_state = eskf_.GetNominalState(); // 最后时刻的状态
          SE3 T_end = SE3(imu_state.R_, imu_state.p_);

#ifdef P_USE_TBB_
          tbb::parallel_for(size_t(0), points.size(), [&](size_t id)
                            {
                                   SE3 Ti = T_end;
                                   NavStated match;
                                   point3D& pt = points[id];
                                   // 根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
                                   math::PoseInterp<NavStated>(
                                   pt.timestamp, imu_states_, [](const NavStated &s)
                                   { return s.timestamp_; },
                                   [](const NavStated &s)
                                   { return s.GetSE3(); },
                                   Ti, match);
                                   if (only_R)
                                   {
                                        //    转到imu坐标系
                                        pt.raw_point = TIL_ * pt.raw_point;
                                        //   完成去畸变
                                        pt.raw_point = imu_state.R_.inverse() * Ti.so3() * pt.raw_point; //  rm distort first,and then transform to end
                                        //   转回激光坐标系
                                        pt.raw_point = TIL_.inverse() * pt.raw_point;
                                   }
                                   else
                                        pt.raw_point = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pt.raw_point; });
#else
          /// 将所有点转到最后时刻状态上
          for (auto &pt : points)
          {
               SE3 Ti = T_end;
               NavStated match;

               // 根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
               math::PoseInterp<NavStated>(
                   pt.timestamp, imu_states_, [](const NavStated &s)
                   { return s.timestamp_; },
                   [](const NavStated &s)
                   { return s.GetSE3(); },
                   Ti, match);
               if (only_R)
               {
                    //    转到imu坐标系
                    pt.raw_point = TIL_ * pt.raw_point;
                    //   完成去畸变
                    pt.raw_point = imu_state.R_.inverse() * Ti.so3() * pt.raw_point; //  rm distort first,and then transform to end
                    //   转回激光坐标系
                    pt.raw_point = TIL_.inverse() * pt.raw_point;
               }
               else
                    pt.raw_point = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pt.raw_point;
          }
#endif
     }

     void lidarodom::TryInitIMU()
     {
          for (auto imu : measures_.imu_)
          {
               imu_init_.AddIMU(*imu);
          }

          if (imu_init_.InitSuccess())
          {
               // 读取初始零偏，设置ESKF
               zjloc::ESKFD::Options options;
               // 噪声由初始化器估计
               // options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
               // options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
               options.update_bias_acce_ = true;
               options.update_bias_gyro_ = true;
               eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
               imu_need_init_ = false;
               RIG_ = SO3(g2R(imu_init_.GetMeanAcc()));

               std::cout << ANSI_COLOR_GREEN_BOLD << "IMU初始化成功" << ANSI_COLOR_RESET << std::endl;
          }
     }

     int lidarodom::getIndex() { return index_frame; }
     void lidarodom::setFunc(std::function<bool(std::string &topic_name,
                                                CloudPtr &cloud, double time)> &fun)
     {
          pub_cloud_to_ros = fun;
     }
     void lidarodom::setFunc(std::function<bool(std::string &topic_name,
                                                SE3 &pose, double time)> &fun)
     {
          pub_pose_to_ros = fun;
     }
     void lidarodom::setFunc(std::function<bool(std::string &topic_name,
                                                double time1, double time2)> &fun)
     {
          pub_data_to_ros = fun;
     }

}

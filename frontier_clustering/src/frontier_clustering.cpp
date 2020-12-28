//
// Created by op on 11/25/20.
//

#include "frontier_clustering/frontier_clustering.h"

namespace frontiers {

FrontierClustering::FrontierClustering(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      id_counter_(0),
      manager_(nh_, nh_private_, &params_) {
  frontiers_pub_ = nh_.advertise<visualization_msgs::Marker>("frontiers", 100);

  frontier_voxels_sub_ =
      nh_.subscribe("/frontierClustering/frontier_voxels", 10,
                    &FrontierClustering::insertFrontierVoxels, this);

  params_.setParametersFromRos(nh_private_);
  setUpCamera();

  viewpoints_.setShape(params_.radius_cylinder_, params_.h_cylinder_);

  ROS_INFO("All set! Spinning...");
}

void FrontierClustering::setUpCamera() {
  // Set camera FOV
  camera_.setIntrinsicsFromFoV(params_.camera_hfov_, params_.camera_vfov_,
                               params_.sensor_min_range_,
                               params_.sensor_max_range_);
  // Set Boundaries of Exploration
  camera_.setBoundingBox(params_.bbx_min_, params_.bbx_max_);
  // Set camera frame transformation
  static tf::TransformListener listener;
  tf::StampedTransform stamped_transform;

  try {
    listener.waitForTransform(params_.camera_frame_, "base_link",
                              ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform(params_.camera_frame_, "base_link", ros::Time(0),
                             stamped_transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN("%s", ex.what());
  }
  Transformation T_C_B;
  tf::transformTFToKindr(stamped_transform, &T_C_B);
  camera_.setExtrinsics(T_C_B);
}

void FrontierClustering::insertFrontierVoxels(
    const voxblox_msgs::FrontierVoxels::Ptr& frontier_voxels) {
  voxblox::LongIndexHashMapType<size_t>::type voxel_map;
  voxblox::LongIndexSet remove_voxel;
  // Deserialize the ros msg
  deserializeFrontierVoxelsMsg(frontier_voxels, voxel_map, remove_voxel);

  // Remove the outdated frontier voxels and add the rest influenced to the map
  removeOldFrontierVoxels(voxel_map, remove_voxel);

  voxblox::AlignedList<Frontier*> local_frontiers;
  // Cluster new frontiers inside the updated area
  clusterNewFrontiers(local_frontiers, voxel_map);
}

void FrontierClustering::removeOldFrontierVoxels(
    voxblox::LongIndexHashMapType<size_t>::type& voxel_map,
    voxblox::LongIndexSet& remove_voxel) {
  // Select frontiers inside AABB of the msg
  auto iter = frontiers_.begin();
  while (iter != frontiers_.end()) {
    if (isInsideAabb(*iter)) {
      for (size_t i = 0; i < iter->frontier_voxels().rows(); ++i) {
        auto row = iter->frontier_voxels().row(i);
        voxblox::GlobalIndex tmp_idx{row.x(), row.y(), row.z()};
        if (remove_voxel.find(tmp_idx) == remove_voxel.end()) {
          voxel_map[tmp_idx] = -1;
        }
      }
      visualization_msgs::Marker frontier_msg;
      frontier_msg.header.frame_id = "map";
      frontier_msg.header.stamp = ros::Time();
      frontier_msg.ns = "frontiers";
      frontier_msg.id = iter->id();
      frontier_msg.type = visualization_msgs::Marker::CUBE_LIST;
      frontier_msg.action = visualization_msgs::Marker::DELETE;
      frontiers_pub_.publish(frontier_msg);
      iter = frontiers_.erase(iter);
      continue;
    }
    ++iter;
  }
}

void FrontierClustering::clusterNewFrontiers(
    voxblox::AlignedList<Frontier*>& local_frontiers,
    voxblox::LongIndexHashMapType<size_t>::type& voxel_map) {
  if (not voxel_map.empty()) {
    voxblox::AlignedVector<Frontier> frontiers_tmp;
    for (auto it = voxel_map.begin(); it != voxel_map.end(); ++it) {
      clusterNewFrontiersRec(voxel_map, frontiers_tmp, frontiers_tmp.size(),
                             it);
    }
    for (auto& f_tmp : frontiers_tmp) {
      if (f_tmp.frontier_voxels().rows() > params_.min_num_voxels_) {
        insertNewFrontiersRec(f_tmp, local_frontiers);
      }
    }
  }
}

void FrontierClustering::clusterNewFrontiersRec(
    voxblox::LongIndexHashMapType<size_t>::type& voxel_map,
    voxblox::AlignedVector<Frontier>& frontiers_tmp, size_t cluster,
    voxblox::LongIndexHashMapType<size_t>::type::iterator it) {
  if (it == voxel_map.end()) {
    return;
  } else if (it->second == -1) {
    it->second = cluster;
    if (cluster == frontiers_tmp.size()) {
      frontiers_tmp.emplace_back(Frontier(0));
    }
    frontiers_tmp[cluster].addVoxel(it->first);
    for (const auto& adj : adjacent) {
      clusterNewFrontiersRec(voxel_map, frontiers_tmp, cluster,
                             voxel_map.find(it->first + adj));
    }
  }
}

void FrontierClustering::insertNewFrontiersRec(
    Frontier& frontier, voxblox::AlignedList<Frontier*>& local_frontiers) {
  frontier.setMean();
  MatrixX3d centered = frontier.frontier_voxels().cast<double>().rowwise() -
                       frontier.mean().transpose();
  MatrixX3d cov = centered.adjoint() * centered;

  Eigen::SelfAdjointEigenSolver<MatrixX3d> eig(cov);
  double eigen_val = eig.eigenvalues()[2];

  if (eigen_val > params_.size_threshold_) {
    Eigen::Vector3d eigen_vec = eig.eigenvectors().col(2);
    Eigen::Vector3d normal = eigen_vec + frontier.mean();
    normal /= normal.norm();
    double distance = normal.dot(frontier.mean());
    Frontier frontier1(0);
    Frontier frontier2(0);
    for (size_t i = 0; i < frontier.frontier_voxels().rows(); ++i) {
      Eigen::Matrix<int64_t, 3, 1> row = frontier.frontier_voxels().row(i);
      if (normal.dot(row.cast<double>()) >= distance) {
        frontier1.addVoxel(row);
      } else {
        frontier2.addVoxel(row);
      }
    }
    /*std::thread t1(&FrontierClustering::insertNewFrontiersRec, this,
                   std::ref(frontier1), std::ref(local_frontiers));
    std::thread t2(&FrontierClustering::insertNewFrontiersRec, this,
                   std::ref(frontier2), std::ref(local_frontiers));
    t1.join();
    t2.join();*/
    insertNewFrontiersRec(frontier1, local_frontiers);
    insertNewFrontiersRec(frontier2, local_frontiers);
  } else {
    frontier.setId(id_counter_);
    ++id_counter_;

    sampleViewpoints(frontier);

    frontiers_.emplace_back(frontier);
    local_frontiers.emplace_back(&frontiers_.back());
    serializeFrontierClustersMsg(frontier);
  }
}

void FrontierClustering::sampleViewpoints(Frontier& frontier) {
  AlignedVector<Point> samples;
  viewpoints_.generateSamples(params_.num_viewpoints_, params_.voxel_size_,
                              frontier.mean(), &manager_, samples);
  VLOG(5) << "Sample size: " << samples.size();

  std::unique_ptr<voxblox::ThreadSafeIndex> index_getter(
      voxblox::ThreadSafeIndexFactory::get(samples.size()));

  voxblox::LongIndexSet voxels;
  for (size_t i = 0; i < frontier.frontier_voxels().size(); ++i) {
    voxels.insert(voxblox::GlobalIndex{frontier.frontier_voxels()(i, 0),
                                       frontier.frontier_voxels()(i, 1),
                                       frontier.frontier_voxels()(i, 2)});
  }

  std::list<std::thread> integration_threads;
  AlignedVector<std::pair<double, size_t>> gains;
  gains.reserve(samples.size());
  for (size_t i = 0; i < params_.max_num_threads_; ++i) {
    integration_threads.emplace_back(
        &Frontier::assignYawAndFilter, frontier, camera_, std::ref(samples),
        std::ref(gains), std::ref(voxels), std::ref(manager_), index_getter.get());
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }
}

void FrontierClustering::serializeFrontierClustersMsg(Frontier& frontier) {
  visualization_msgs::Marker frontier_msg;
  frontier_msg.header.frame_id = "map";
  frontier_msg.header.stamp = ros::Time();
  frontier_msg.ns = "frontiers";
  frontier_msg.id = frontier.id();
  frontier_msg.type = visualization_msgs::Marker::CUBE_LIST;
  frontier_msg.action = visualization_msgs::Marker::ADD;
  frontier_msg.pose.orientation.x = 0.0;
  frontier_msg.pose.orientation.y = 0.0;
  frontier_msg.pose.orientation.z = 0.0;
  frontier_msg.pose.orientation.w = 1.0;
  frontier_msg.color = frontier.color();
  frontier_msg.scale.x = params_.voxel_size_;
  frontier_msg.scale.y = params_.voxel_size_;
  frontier_msg.scale.z = params_.voxel_size_;
  frontier_msg.points.reserve(frontier.frontier_voxels().rows());
  geometry_msgs::Point p;
  for (size_t i = 0; i < frontier.frontier_voxels().rows(); ++i) {
    auto voxel = frontier.frontier_voxels().row(i);
    p.x = (static_cast<voxblox::FloatingPoint>(voxel.x()) + 0.5) *
          params_.voxel_size_;
    p.y = (static_cast<voxblox::FloatingPoint>(voxel.y()) + 0.5) *
          params_.voxel_size_;
    p.z = (static_cast<voxblox::FloatingPoint>(voxel.z()) + 0.5) *
          params_.voxel_size_;
    frontier_msg.points.emplace_back(p);
  }
  frontiers_pub_.publish(frontier_msg);
}

void FrontierClustering::deserializeFrontierVoxelsMsg(
    const voxblox_msgs::FrontierVoxels::Ptr& frontier_voxels,
    voxblox::LongIndexHashMapType<size_t>::type& voxel_map,
    voxblox::LongIndexSet& remove_voxel) {
  aabb_min_ = {frontier_voxels->aabb_min.x, frontier_voxels->aabb_min.y,
               frontier_voxels->aabb_min.z};
  aabb_max_ = {frontier_voxels->aabb_max.x, frontier_voxels->aabb_max.y,
               frontier_voxels->aabb_max.z};

  voxblox::GlobalIndex global_index;
  for (const auto& fvoxel : frontier_voxels->voxels) {
    if (not isInsideBbx(fvoxel.index)) {
      continue;
    }
    global_index = {fvoxel.index.x, fvoxel.index.y, fvoxel.index.z};
    if (fvoxel.action == 0) {
      remove_voxel.insert(global_index);
    } else {
      voxel_map[global_index] = -1;
    }
  }
}

bool FrontierClustering::isInsideAabb(const Frontier& frontier) const {
  voxblox::GlobalIndex f_aabb_min;
  voxblox::GlobalIndex f_aabb_max;
  frontier.getAabb(&f_aabb_min, &f_aabb_max);
  for (size_t i = 0; i < 3; ++i) {
    if (not((aabb_min_[i] <= f_aabb_min[i] and f_aabb_min[i] <= aabb_max_[i]) or
            (f_aabb_min[i] <= aabb_min_[i] and
             aabb_min_[i] <= f_aabb_max[i]))) {
      return false;
    }
  }
  return true;
}

}  // namespace frontiers

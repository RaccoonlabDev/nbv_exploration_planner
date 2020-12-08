//
// Created by op on 11/25/20.
//

#include "frontier_clustering/frontier_clustering.h"

namespace frontiers {

FrontierClustering::FrontierClustering(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), id_counter_(0) {
  frontiers_pub_ = nh_.advertise<visualization_msgs::Marker>("frontiers", 100);
  frontiers_aabb_pub_ =
      nh_.advertise<visualization_msgs::Marker>("frontiers_aabb", 100);
  frontier_voxels_sub_ =
      nh_.subscribe("/iris/nbvePlanner/frontier_voxels", 10,
                    &FrontierClustering::insertFrontierVoxels, this);

  nh_private_.param("size_threshold", size_threshold_, 1000.0);

  ROS_INFO("All set! Spinning...");
}

void FrontierClustering::insertFrontierVoxels(
    const voxblox_msgs::FrontierVoxels::Ptr& frontier_voxels) {
  voxblox::LongIndexHashMapType<size_t>::type voxel_map;
  voxblox::GlobalIndexVector remove_voxel;
  deserializeFrontierVoxelsMsg(frontier_voxels, voxel_map, remove_voxel);
  voxblox::AlignedList<Frontier*> local_frontiers;
  removeOldFrontierVoxels(local_frontiers, remove_voxel);
  clusterNewFrontiers(local_frontiers, voxel_map);
  serializeFrontierClustersMsg(local_frontiers);
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

void FrontierClustering::removeOldFrontierVoxels(
    voxblox::AlignedList<Frontier*>& local_frontiers,
    voxblox::GlobalIndexVector& remove_voxel) {
  // Select frontiers inside AABB of the msg
  auto iter = frontiers_.begin();
  while (iter != frontiers_.end()) {
    if (iter->size() < 10) {
      iter = frontiers_.erase(iter);
      continue;
    }
    if (isInsideAabb(*iter)) {
      local_frontiers.emplace_back(&(*iter));
    }
    ++iter;
  }
  // VLOG(5) << "Local Frontiers size: " << local_frontiers.size();

  for (const auto& voxel : remove_voxel) {
    for (auto it = local_frontiers.begin(); it != local_frontiers.end(); ++it) {
      if ((*it)->hasVoxel(voxel)) {
        if ((*it)->removeVoxel(voxel) < 10) {
          visualization_msgs::Marker frontier_msg;
          frontier_msg.header.frame_id = "map";
          frontier_msg.header.stamp = ros::Time();
          frontier_msg.ns = "frontiers";
          frontier_msg.id = (*it)->id();
          frontier_msg.type = visualization_msgs::Marker::CUBE_LIST;
          frontier_msg.action = visualization_msgs::Marker::DELETE;
          frontiers_pub_.publish(frontier_msg);

          local_frontiers.erase(it);
        }
        break;
      }
    }
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
    // Merge the new frontiers with the existing ones
    /*bool merged;
    for (auto& f_tmp : frontiers_tmp) {
      merged = false;
      for (auto& frontier : local_frontiers) {
        if (f_tmp.checkIntersectionAabb(*frontier)) {
          frontier->addFrontier(f_tmp.frontier_voxels());
          merged = true;
          break;
        }
      }
      if (not merged) {
        f_tmp.setId(id_counter_);
        ++id_counter_;
        frontiers_.emplace_back(f_tmp);
        local_frontiers.emplace_back(&frontiers_.back());
      }
    }*/
    for (auto& f_tmp : frontiers_tmp) {
      if (f_tmp.size() > 10) {
        // call the recursive insertion
        f_tmp.setId(id_counter_);
        ++id_counter_;
        frontiers_.emplace_back(f_tmp);
        local_frontiers.emplace_back(&frontiers_.back());
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

void FrontierClustering::insertNewFrontiersRec(Frontier& frontier) {
  MatrixX3d centered =
      frontier.mat().rowwise() - frontier.mat().colwise().mean();
  MatrixX3d cov = centered.adjoint() * centered;
  Eigen::SelfAdjointEigenSolver<MatrixX3d> eig(cov);
  double eigen_val = eig.eigenvalues()[2];

  voxblox::GlobalIndex f_aabb_min;
  voxblox::GlobalIndex f_aabb_max;
  const static voxblox::GlobalIndex ones{1, 1, 1};
  frontier.getAabb(&f_aabb_min, &f_aabb_max);
  VLOG(5) << "Eigen Value for "
          << ((f_aabb_max - f_aabb_min) + ones).transpose() << ": "
          << eigen_val;
  if (eigen_val > size_threshold_) {
    // somehow divide the dataset into two equal sides along the main axis
    // auto eigen_vec = eig.eigenvectors().col(2);
    //insertNewFrontiersRec();
  }
}

void FrontierClustering::serializeFrontierClustersMsg(
    voxblox::AlignedList<Frontier*>& local_frontiers) {
  for (const auto& frontier : local_frontiers) {
    visualization_msgs::Marker frontier_msg;
    frontier_msg.header.frame_id = "map";
    frontier_msg.header.stamp = ros::Time();
    frontier_msg.ns = "frontiers";
    frontier_msg.id = frontier->id();
    frontier_msg.type = visualization_msgs::Marker::CUBE_LIST;
    frontier_msg.action = visualization_msgs::Marker::ADD;
    frontier_msg.pose.orientation.x = 0.0;
    frontier_msg.pose.orientation.y = 0.0;
    frontier_msg.pose.orientation.z = 0.0;
    frontier_msg.pose.orientation.w = 1.0;
    frontier_msg.color = frontier->color();
    frontier_msg.scale.x = voxel_size_;
    frontier_msg.scale.y = voxel_size_;
    frontier_msg.scale.z = voxel_size_;
    frontier_msg.points.reserve(frontier->size());
    geometry_msgs::Point p;
    for (const auto& voxel : frontier->frontier_voxels()) {
      p.x =
          (static_cast<voxblox::FloatingPoint>(voxel.x()) + 0.5) * voxel_size_;
      p.y =
          (static_cast<voxblox::FloatingPoint>(voxel.y()) + 0.5) * voxel_size_;
      p.z =
          (static_cast<voxblox::FloatingPoint>(voxel.z()) + 0.5) * voxel_size_;
      frontier_msg.points.emplace_back(p);
    }
    frontiers_pub_.publish(frontier_msg);
  }
}

void FrontierClustering::deserializeFrontierVoxelsMsg(
    const voxblox_msgs::FrontierVoxels::Ptr& frontier_voxels,
    voxblox::LongIndexHashMapType<size_t>::type& voxel_map,
    voxblox::GlobalIndexVector& remove_voxel) {
  voxel_size_ = frontier_voxels->voxel_size;
  voxels_per_side_ = frontier_voxels->voxels_per_side;

  aabb_min_ = {frontier_voxels->aabb_min.x, frontier_voxels->aabb_min.y,
               frontier_voxels->aabb_min.z};
  aabb_max_ = {frontier_voxels->aabb_max.x, frontier_voxels->aabb_max.y,
               frontier_voxels->aabb_max.z};

  voxblox::GlobalIndex global_index;
  for (auto fvoxel : frontier_voxels->voxels) {
    global_index = {fvoxel.index.x, fvoxel.index.y, fvoxel.index.z};
    if (fvoxel.action == 0) {
      remove_voxel.emplace_back(global_index);
    } else {
      voxel_map[global_index] = -1;
    }
  }
}

}  // namespace frontiers

//
// Created by op on 11/25/20.
//

#include "frontier_clustering/frontier_clustering.h"

namespace frontiers {

FrontierClustering::FrontierClustering(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  frontiers_pub_ = nh_.advertise<visualization_msgs::Marker>("frontiers", 1);
  frontier_voxels_sub_ =
      nh_.subscribe("/iris/nbvePlanner/frontier_voxels", 10,
                    &FrontierClustering::insertFrontierVoxels, this);

  ROS_INFO("All set! Spinning...");
}

void FrontierClustering::insertFrontierVoxels(
    const voxblox_msgs::FrontierVoxels::Ptr& frontier_voxels) {
  voxblox::LongIndexHashMapType<size_t>::type voxel_map;
  voxblox::GlobalIndexVector remove_voxel;
  deserializeFrontierVoxelsMsg(frontier_voxels, &voxel_map, &remove_voxel);
  
  removeOldFrontierVoxels(remove_voxel);
  clusterNewFrontiers(voxel_map);
  serializeFrontierClustersMsg();
}

void FrontierClustering::removeOldFrontierVoxels(
    voxblox::GlobalIndexVector& remove_voxel) {
  visualization_msgs::Marker m;
  for (const auto& voxel : remove_voxel) {
    for (auto f : frontiers_) {
      if (f.hasVoxel(voxel)) {
        f.removeVoxel(voxel);
        break;
      }
    }
  }
}

void FrontierClustering::clusterNewFrontiers(
    voxblox::LongIndexHashMapType<size_t>::type& voxel_map) {
  if (not voxel_map.empty()) {
    voxblox::AlignedVector<Frontier> frontiers_tmp;
    for (auto it = voxel_map.begin(); it != voxel_map.end(); ++it) {
      clusterNewFrontiersRec(voxel_map, frontiers_tmp, frontiers_tmp.size(),
                             it);
    }
    // Merge the new frontiers with the existing ones
    bool merged;
    for (const auto& f_tmp : frontiers_tmp) {
      merged = false;
      for (auto& frontier : frontiers_) {
        if (f_tmp.checkIntersectionAabb(frontier)) {
          frontier.addFrontier(f_tmp.frontier_voxels());
          merged = true;
          break;
        }
      }
      if (not merged) {
        frontiers_.emplace_back(f_tmp);
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
      frontiers_tmp.emplace_back(Frontier());
    }
    frontiers_tmp[cluster].addVoxel(it->first);

    for (const auto& adj : adjacent) {
      clusterNewFrontiersRec(voxel_map, frontiers_tmp, cluster,
                             voxel_map.find(it->first + adj));
    }
  }
}

void FrontierClustering::serializeFrontierClustersMsg() {
  frontiers_msg_.action = visualization_msgs::Marker::DELETE;
  frontiers_pub_.publish(frontiers_msg_);
  frontiers_msg_.header.frame_id = "map";
  frontiers_msg_.header.stamp = ros::Time::now();
  frontiers_msg_.ns = "frontiers";
  frontiers_msg_.id = 0;
  frontiers_msg_.type = visualization_msgs::Marker::CUBE_LIST;
  frontiers_msg_.action = visualization_msgs::Marker::ADD;
  frontiers_msg_.pose.orientation.x = 0.0;
  frontiers_msg_.pose.orientation.y = 0.0;
  frontiers_msg_.pose.orientation.z = 0.0;
  frontiers_msg_.pose.orientation.w = 1.0;
  frontiers_msg_.scale.x = voxel_size_;
  frontiers_msg_.scale.y = voxel_size_;
  frontiers_msg_.scale.z = voxel_size_;
  frontiers_msg_.points.clear();
  frontiers_msg_.colors.clear();
  geometry_msgs::Point p;
  for (const auto& frontier : frontiers_) {
    for (const auto& voxel : frontier.frontier_voxels()) {
      frontiers_msg_.colors.emplace_back(frontier.color());
      p.x =
          (static_cast<voxblox::FloatingPoint>(voxel.x()) + 0.5) * voxel_size_;
      p.y =
          (static_cast<voxblox::FloatingPoint>(voxel.y()) + 0.5) * voxel_size_;
      p.z =
          (static_cast<voxblox::FloatingPoint>(voxel.z()) + 0.5) * voxel_size_;
      frontiers_msg_.points.emplace_back(p);
    }
  }
  frontiers_pub_.publish(frontiers_msg_);
}

void FrontierClustering::deserializeFrontierVoxelsMsg(
    const voxblox_msgs::FrontierVoxels::Ptr& frontier_voxels,
    voxblox::LongIndexHashMapType<size_t>::type* voxel_map,
    voxblox::GlobalIndexVector* remove_voxel) {
  CHECK_NOTNULL(voxel_map);
  CHECK_NOTNULL(remove_voxel);
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
      remove_voxel->emplace_back(global_index);
    } else {
      (*voxel_map)[global_index] = -1;
    }
  }
}

}  // namespace frontiers

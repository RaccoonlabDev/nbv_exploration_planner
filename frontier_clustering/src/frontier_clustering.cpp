//
// Created by op on 11/25/20.
//

#include "frontier_clustering/frontier_clustering.h"

namespace frontiers {

FrontierClustering::FrontierClustering(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  frontier_vis_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 1);
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
  frontier_vis_pub_.publish(frontiers_marker_);
  frontiers_marker_.markers.clear();
}

void FrontierClustering::removeOldFrontierVoxels(
    voxblox::GlobalIndexVector& remove_voxel) {
  visualization_msgs::Marker m;
  for (const auto& voxel : remove_voxel) {
    for (auto f : frontiers_) {
      if (f.hasVoxel(voxel)) {
        f.removeVoxel(voxel);

        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "frontiers";
        m.action = visualization_msgs::Marker::DELETE;
        m.id = voxblox::LongIndexHash()(voxel);
        frontiers_marker_.markers.emplace_back(m);
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
    // Try to merge the new frontiers with the existing ones
    for (const auto& f_tmp : frontiers_tmp) {
      voxblox::AlignedQueue<size_t> candidate_frontiers_;
      for (auto frontier : frontiers_) {
      }
    }
  }
}

void FrontierClustering::clusterNewFrontiersRec(
    voxblox::LongIndexHashMapType<size_t>::type& voxel_map,
    voxblox::AlignedVector<Frontier>& frontiers_tmp, size_t cluster,
    voxblox::LongIndexHashMapType<size_t>::type::iterator it) {
  static const voxblox::GlobalIndexVector adjacent{
      voxblox::GlobalIndex{1, 0, 0}, voxblox::GlobalIndex{-1, 0, 0},
      voxblox::GlobalIndex{0, 1, 0}, voxblox::GlobalIndex{0, -1, 0},
      voxblox::GlobalIndex{0, 0, 1}, voxblox::GlobalIndex{0, 0, -1}};

  if (it == voxel_map.end()) {
    return;
  } else if (it->second == -1) {
    it->second = cluster;
    if (cluster == frontiers_tmp.size()) {
      frontiers_tmp.emplace_back(Frontier());
    }
    frontiers_tmp[cluster].addVoxel(it->first);
    visualization_msgs::Marker m;
    /*serializeFrontierClustersMsg(it, cluster, &m);
    frontiers_marker_.markers.emplace_back(m);*/
    for (const auto& adj : adjacent) {
      clusterNewFrontiersRec(voxel_map, frontiers_tmp, cluster,
                             voxel_map.find(it->first + adj));
    }
  }
}

void FrontierClustering::serializeFrontierClustersMsg(
    const voxblox::LongIndexHashMapType<size_t>::type::iterator& it,
    const size_t cluster, visualization_msgs::Marker* m) {
  m->header.frame_id = "map";
  m->header.stamp = ros::Time::now();
  m->ns = "frontiers";
  m->id = voxblox::LongIndexHash()(it->first);
  m->type = visualization_msgs::Marker::CUBE;
  m->color = frontiers_[cluster].color();
  m->action = visualization_msgs::Marker::ADD;
  m->pose.position.x =
      (static_cast<voxblox::FloatingPoint>(it->first.x()) + 0.5) * voxel_size_;
  m->pose.position.y =
      (static_cast<voxblox::FloatingPoint>(it->first.y()) + 0.5) * voxel_size_;
  m->pose.position.z =
      (static_cast<voxblox::FloatingPoint>(it->first.z()) + 0.5) * voxel_size_;
  m->pose.orientation.x = 0.0;
  m->pose.orientation.y = 0.0;
  m->pose.orientation.z = 0.0;
  m->pose.orientation.w = 1.0;
  m->scale.x = voxel_size_;
  m->scale.y = voxel_size_;
  m->scale.z = voxel_size_;
}

void FrontierClustering::deserializeFrontierVoxelsMsg(
    const voxblox_msgs::FrontierVoxels::Ptr& frontier_voxels,
    voxblox::LongIndexHashMapType<size_t>::type* voxel_map,
    voxblox::GlobalIndexVector* remove_voxel) {
  CHECK_NOTNULL(voxel_map);
  CHECK_NOTNULL(remove_voxel);
  voxel_size_ = frontier_voxels->voxel_size;
  voxels_per_side_ = frontier_voxels->voxels_per_side;

  const size_t size = frontier_voxels->voxels.size();
  voxel_map->reserve(size);
  remove_voxel->reserve(size);
  voxblox::GlobalIndex global_index;
  for (auto fvoxel : frontier_voxels->voxels) {
    global_index = {fvoxel.x_index, fvoxel.y_index, fvoxel.z_index};
    if (fvoxel.action == 0) {
      remove_voxel->emplace_back(global_index);
    } else {
      (*voxel_map)[global_index] = -1;
    }
  }
}

}  // namespace frontiers

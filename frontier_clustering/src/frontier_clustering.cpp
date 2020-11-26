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
  frontier_voxels_sub_ = nh_.subscribe(
      "frontier_voxels", 10, &FrontierClustering::insertFrontierVoxels, this);
}

void FrontierClustering::insertFrontierVoxels(
    const voxblox_msgs::FrontierVoxels::Ptr& frontier_voxels) {
  voxblox::LongIndexHashMapType<int8_t>::type voxel_map;
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
        m.action = visualization_msgs::Marker::DELETE;
        m.id = voxblox::LongIndexHash()(voxel);
        frontiers_marker_.markers.emplace_back(m);
        break;
      }
    }
  }
}

void FrontierClustering::clusterNewFrontiers(
    voxblox::LongIndexHashMapType<int8_t>::type& voxel_map) {
  if (not voxel_map.empty()) {
    clusterNewFrontiersRec(voxel_map, frontiers_.size(), voxel_map.begin());
  }
}

void FrontierClustering::clusterNewFrontiersRec(
    voxblox::LongIndexHashMapType<int8_t>::type& voxel_map, int8_t cluster,
    voxblox::LongIndexHashMapType<int8_t>::type::iterator it) {
  static const voxblox::GlobalIndexVector adjacent{
      voxblox::GlobalIndex{1, 0, 0}, voxblox::GlobalIndex{-1, 0, 0},
      voxblox::GlobalIndex{0, 1, 0}, voxblox::GlobalIndex{0, -1, 0},
      voxblox::GlobalIndex{0, 0, 1}, voxblox::GlobalIndex{0, 0, -1}};

  if (it == voxel_map.end() or it->second != -1) {
    return;
  }
  it->second = cluster;
  if (cluster == frontiers_.size()) {
    frontiers_.emplace_back(Frontier());
  }
  frontiers_[cluster].addVoxel(it->first);
  
  for (const auto& adj : adjacent) {
    clusterNewFrontiersRec(voxel_map, cluster, voxel_map.find(it->first + adj));
  }
  clusterNewFrontiersRec(voxel_map, cluster + 1, ++it);
}

void FrontierClustering::deserializeFrontierVoxelsMsg(
    const voxblox_msgs::FrontierVoxels::Ptr& frontier_voxels,
    voxblox::LongIndexHashMapType<int8_t>::type* voxel_map,
    voxblox::GlobalIndexVector* remove_voxel) {
  CHECK_NOTNULL(voxel_map);
  CHECK_NOTNULL(remove_voxel);

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

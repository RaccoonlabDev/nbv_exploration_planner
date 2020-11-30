//
// Created by op on 11/25/20.
//

#ifndef SRC_FRONTIER_CLUSTERING_H
#define SRC_FRONTIER_CLUSTERING_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox_msgs/FrontierVoxels.h>
#include <boost/dynamic_bitset.hpp>

#include "frontier_clustering/frontier.h"

namespace frontiers {

class FrontierClustering {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrontierClustering(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private);

  virtual ~FrontierClustering() = default;

  void insertFrontierVoxels(
      const voxblox_msgs::FrontierVoxels::Ptr& frontier_voxels);

 private:
  void removeOldFrontierVoxels(voxblox::GlobalIndexVector& remove_voxel);

  void clusterNewFrontiers(
      voxblox::LongIndexHashMapType<size_t>::type& voxel_map);

  void clusterNewFrontiersRec(
      voxblox::LongIndexHashMapType<size_t>::type& voxel_map,
      voxblox::AlignedVector<Frontier>& frontiers_tmp, size_t cluster,
      voxblox::LongIndexHashMapType<size_t>::type::iterator it);

  void deserializeFrontierVoxelsMsg(
      const voxblox_msgs::FrontierVoxels::Ptr& frontier_voxels,
      voxblox::LongIndexHashMapType<size_t>::type* voxel_map,
      voxblox::GlobalIndexVector* remove_voxel);

  void serializeFrontierClustersMsg();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  double voxel_size_;
  double voxels_per_side_;

  visualization_msgs::Marker frontiers_msg_;

  ros::Publisher frontiers_pub_;

  ros::Subscriber frontier_voxels_sub_;

  voxblox::AlignedVector<Frontier> frontiers_;
};

}  // namespace frontiers

#endif  // SRC_FRONTIER_CLUSTERING_H

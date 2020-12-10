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

typedef voxblox::AlignedList<Frontier> Frontiers;

class FrontierClustering {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrontierClustering(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private);

  virtual ~FrontierClustering() = default;

  void insertFrontierVoxels(
      const voxblox_msgs::FrontierVoxels::Ptr& frontier_voxels);

 private:
  bool isInsideAabb(const Frontier& frontier) const;

  void removeOldFrontierVoxels(voxblox::AlignedList<Frontier*>& local_frontiers,
                               voxblox::GlobalIndexVector& remove_voxel);

  void clusterNewFrontiers(
      voxblox::AlignedList<Frontier*>& local_frontiers,
      voxblox::LongIndexHashMapType<size_t>::type& voxel_map);

  void clusterNewFrontiersRec(
      voxblox::LongIndexHashMapType<size_t>::type& voxel_map,
      voxblox::AlignedVector<Frontier>& frontiers_tmp, size_t cluster,
      voxblox::LongIndexHashMapType<size_t>::type::iterator it);

  void insertNewFrontiersRec(Frontier& frontier,
                             voxblox::AlignedList<Frontier*>& local_frontiers);

  void deserializeFrontierVoxelsMsg(
      const voxblox_msgs::FrontierVoxels::Ptr& frontier_voxels,
      voxblox::LongIndexHashMapType<size_t>::type& voxel_map,
      voxblox::GlobalIndexVector& remove_voxel);

  inline bool isInsideBbx(const voxblox_msgs::Index & voxel) const {
    static Eigen::Vector3d bbx_min = bbx_min_/voxel_size_;
    static Eigen::Vector3d bbx_max = bbx_max_/voxel_size_;

    if (voxel.x < bbx_min.x() or voxel.x > bbx_max.x() or
        voxel.y < bbx_min.y() or voxel.y > bbx_max.y() or
        voxel.z < bbx_min.z() or voxel.z > bbx_max.z()) {
      return false;
    }
    return true;
  }

  void serializeFrontierClustersMsg(
      voxblox::AlignedList<Frontier*>& local_frontiers);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  double voxel_size_;
  double voxels_per_side_;
  double size_threshold_;

  Eigen::Vector3d bbx_min_;
  Eigen::Vector3d bbx_max_;

  voxblox::GlobalIndex aabb_min_;
  voxblox::GlobalIndex aabb_max_;

  unsigned int id_counter_;

  ros::Publisher frontiers_pub_;
  ros::Publisher frontiers_aabb_pub_;

  ros::Subscriber frontier_voxels_sub_;

  Frontiers frontiers_;
};

}  // namespace frontiers

#endif  // SRC_FRONTIER_CLUSTERING_H

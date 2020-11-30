//
// Created by op on 11/25/20.
//

#ifndef SRC_FRONTIER_H
#define SRC_FRONTIER_H

#include <std_msgs/ColorRGBA.h>
#include <voxblox/core/block_hash.h>
#include <Eigen/Core>
#include <chrono>
#include <random>

namespace frontiers {

class Frontier {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frontier();

  virtual ~Frontier() = default;

  size_t size() { return frontier_voxels_.size(); }

  bool hasVoxel(const voxblox::GlobalIndex& global_index);

  void addVoxel(const voxblox::GlobalIndex& global_index);

  void addVoxelVector(voxblox::GlobalIndexVector::const_iterator it_begin,
                      voxblox::GlobalIndexVector::const_iterator it_end);

  void removeVoxel(const voxblox::GlobalIndex& global_index);

  const std_msgs::ColorRGBA& color() { return color_; }

 private:
  voxblox::LongIndexSet frontier_voxels_;
  std_msgs::ColorRGBA color_;

  Eigen::Vector3i aabb_min_;
  Eigen::Vector3i aabb_max_;
};

}  // namespace frontiers

#endif  // SRC_FRONTIER_H

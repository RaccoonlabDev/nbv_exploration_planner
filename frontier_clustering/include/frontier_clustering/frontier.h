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

static const voxblox::GlobalIndexVector adjacent{
    voxblox::GlobalIndex{1, 0, 0}, voxblox::GlobalIndex{-1, 0, 0},
    voxblox::GlobalIndex{0, 1, 0}, voxblox::GlobalIndex{0, -1, 0},
    voxblox::GlobalIndex{0, 0, 1}, voxblox::GlobalIndex{0, 0, -1}};

class Frontier {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frontier();

  virtual ~Frontier() = default;

  size_t size() const { return frontier_voxels_.size(); }

  const voxblox::LongIndexSet& frontier_voxels() const {
    return frontier_voxels_;
  }

  bool hasVoxel(const voxblox::GlobalIndex& global_index) const;

  void addVoxel(const voxblox::GlobalIndex& global_index);

  void addFrontier(const voxblox::LongIndexSet& frontier_voxels);

  void removeVoxel(const voxblox::GlobalIndex& global_index);

  const std_msgs::ColorRGBA& color() const { return color_; }

  void getAabb(voxblox::GlobalIndex* aabb_min, voxblox::GlobalIndex* aabb_max) const;

  bool checkIntersectionAabb(const Frontier& frontier) const;

 private:
  voxblox::LongIndexSet frontier_voxels_;
  std_msgs::ColorRGBA color_;

  voxblox::GlobalIndex aabb_min_;
  voxblox::GlobalIndex aabb_max_;
};

}  // namespace frontiers

#endif  // SRC_FRONTIER_H

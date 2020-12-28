//
// Created by op on 11/25/20.
//

#ifndef SRC_FRONTIER_H
#define SRC_FRONTIER_H

#include <frontier_clustering/camera_model.h>
#include <frontier_clustering/common.h>
#include <frontier_clustering/voxblox_manager.h>
#include <std_msgs/ColorRGBA.h>
#include <voxblox/core/block_hash.h>
#include <voxblox/integrator/integrator_utils.h>

#include <chrono>
#include <random>
#include <thread>

namespace frontiers {

static const voxblox::GlobalIndexVector adjacent{
    voxblox::GlobalIndex{1, 0, 0},  voxblox::GlobalIndex{-1, 0, 0},
    voxblox::GlobalIndex{0, 1, 0},  voxblox::GlobalIndex{0, -1, 0},
    voxblox::GlobalIndex{0, 0, 1},  voxblox::GlobalIndex{0, 0, -1},
    voxblox::GlobalIndex{1, 1, 0},  voxblox::GlobalIndex{-1, 1, 0},
    voxblox::GlobalIndex{1, -1, 0}, voxblox::GlobalIndex{-1, -1, 0},
    voxblox::GlobalIndex{1, 0, 1},  voxblox::GlobalIndex{-1, 0, 1},
    voxblox::GlobalIndex{1, 0, -1}, voxblox::GlobalIndex{-1, 0, -1},
    voxblox::GlobalIndex{0, 1, 1},  voxblox::GlobalIndex{0, -1, 1},
    voxblox::GlobalIndex{0, 1, -1}, voxblox::GlobalIndex{0, -1, -1}};

class Frontier {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frontier(unsigned int id);

  virtual ~Frontier() = default;

  const MatrixX3li& frontier_voxels() const { return frontier_voxels_; }

  const unsigned int& id() const { return id_; }

  void setId(unsigned int id) { id_ = id; }

  void setMean();

  const Eigen::Vector3d& mean() const { return mean_; }

  void setColor(const std_msgs::ColorRGBA& color) { color_ = color; }

  void addVoxel(const voxblox::GlobalIndex& global_index);

  const std_msgs::ColorRGBA& color() const { return color_; }

  void getAabb(voxblox::GlobalIndex* aabb_min,
               voxblox::GlobalIndex* aabb_max) const;

  void assignYawAndFilter(CameraModel camera,
                          const AlignedVector<Point>& samples,
                          AlignedVector<std::pair<double, size_t>>& gains,
                          const voxblox::LongIndexSet& voxels,
                          const HighResManager& manager,
                          voxblox::ThreadSafeIndex* index_getter);

 private:
  MatrixX3li frontier_voxels_;

  std_msgs::ColorRGBA color_;

  Eigen::Vector3d mean_;
  AlignedVector<Pose> viewpoints_;

  voxblox::GlobalIndex aabb_min_;
  voxblox::GlobalIndex aabb_max_;

  unsigned int id_;
};

}  // namespace frontiers

#endif  // SRC_FRONTIER_H

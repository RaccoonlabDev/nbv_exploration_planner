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

typedef Eigen::Matrix<double, Eigen::Dynamic, 3> MatrixX3d;
typedef Eigen::Matrix<int64_t, Eigen::Dynamic, 3> MatrixX3li;

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

  void addVoxel(const Eigen::Matrix<int64_t, 1, 3>& global_index);

  const std_msgs::ColorRGBA& color() const { return color_; }

  void getAabb(voxblox::GlobalIndex* aabb_min,
               voxblox::GlobalIndex* aabb_max) const;

 private:
  MatrixX3li frontier_voxels_;

  std_msgs::ColorRGBA color_;

  Eigen::Vector3d mean_;

  voxblox::GlobalIndex aabb_min_;
  voxblox::GlobalIndex aabb_max_;

  unsigned int id_;
};

}  // namespace frontiers

#endif  // SRC_FRONTIER_H

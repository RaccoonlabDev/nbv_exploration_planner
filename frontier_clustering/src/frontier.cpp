//
// Created by op on 11/25/20.
//

#include "frontier_clustering/frontier.h"

namespace frontiers {

Frontier::Frontier(unsigned int id)
    : aabb_min_(INT64_MAX, INT64_MAX, INT64_MAX),
      aabb_max_(INT64_MIN, INT64_MIN, INT64_MIN),
      id_(id),
      frontier_voxels_(0, 3) {
  uint64_t timeSeed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32u)};
  std::mt19937_64 rng(ss);
  std::uniform_real_distribution<double> uniform(0, 1);

  color_.r = uniform(rng);
  color_.g = uniform(rng);
  color_.b = uniform(rng);
  color_.a = 0.5;
}

void Frontier::addVoxel(const voxblox::GlobalIndex &global_index) {
  frontier_voxels_[global_index] = mat_.rows();
  // frontier_voxels_.insert(global_index);
  mat_.conservativeResize(mat_.rows() + 1, Eigen::NoChange_t());
  mat_.row(mat_.rows() - 1) << static_cast<double>(global_index.x()),
      static_cast<double>(global_index.y()),
      static_cast<double>(global_index.z());

  for (size_t i = 0; i < 3; ++i) {
    if (global_index[i] < aabb_min_[i]) {
      aabb_min_[i] = global_index[i];
    }
    if (global_index[i] > aabb_max_[i]) {
      aabb_max_[i] = global_index[i];
    }
  }
}

void Frontier::getAabb(voxblox::GlobalIndex *aabb_min,
                       voxblox::GlobalIndex *aabb_max) const {
  *aabb_min = aabb_min_;
  *aabb_max = aabb_max_;
}

}  // namespace frontiers
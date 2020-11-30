//
// Created by op on 11/25/20.
//

#include "frontier_clustering/frontier.h"

namespace frontiers {

Frontier::Frontier()
    : aabb_min_(INT32_MAX, INT32_MAX, INT32_MAX),
      aabb_max_(INT32_MIN, INT32_MIN, INT32_MIN) {
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

bool Frontier::hasVoxel(const voxblox::GlobalIndex &global_index) const {
  if (frontier_voxels_.find(global_index) != frontier_voxels_.end()) {
    return true;
  }
  return false;
}

void Frontier::addVoxel(const voxblox::GlobalIndex &global_index) {
  frontier_voxels_.insert(global_index);

  for (size_t i = 0; i < 3; ++i) {
    if (global_index[i] < aabb_min_[i]) {
      aabb_min_[i] = global_index[i];
    }
    if (global_index[i] > aabb_max_[i]) {
      aabb_max_[i] = global_index[i];
    }
  }
}

void Frontier::addFrontier(const voxblox::LongIndexSet& frontier_voxels) {
  frontier_voxels_.insert(frontier_voxels.begin(), frontier_voxels.end());
}

// TODO: Check if cluster is still connected
void Frontier::removeVoxel(const voxblox::GlobalIndex &global_index) {
  frontier_voxels_.erase(global_index);

  for (const auto &voxel : frontier_voxels_) {
    for (size_t i = 0; i < 3; ++i) {
      if (voxel[i] == aabb_min_[i]) {
        aabb_min_[i] = voxel[i];
      }
      if (voxel[i] == aabb_max_[i]) {
        aabb_max_[i] = voxel[i];
      }
    }
  }
}

bool Frontier::checkIntersectionAabb(const Frontier &frontier) const {
  for (size_t i = 0; i < 3; ++i) {
    if (aabb_min_[i] - 1 < frontier.aabb_min_[i] or
        aabb_max_[i] + 1 > frontier.aabb_max_[i]) {
      return false;
    }
  }
  for (const auto &voxel : frontier_voxels_) {
    for (const auto &adj : adjacent) {
      if (frontier.hasVoxel(voxel + adj)) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace frontiers
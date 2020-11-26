//
// Created by op on 11/25/20.
//

#include "frontier_clustering/frontier.h"

namespace frontiers {

Frontier::Frontier() {
  std::mt19937_64 rng;
  uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed>>32)};
  rng.seed(ss);
  std::uniform_real_distribution<double> unif(0, 1);

  color_.r = unif(rng);
  color_.g = unif(rng);
  color_.b = unif(rng);
  color_.a = 1.0;
}

bool Frontier::hasVoxel(const voxblox::GlobalIndex &global_index) {
  if (frontier_voxels_.find(global_index) != frontier_voxels_.end()) {
    return true;
  }
  return false;
}

void Frontier::addVoxel(const voxblox::GlobalIndex &global_index) {
  frontier_voxels_.insert(global_index);
}

void Frontier::addVoxelVector(
    voxblox::GlobalIndexVector::const_iterator it_begin,
    voxblox::GlobalIndexVector::const_iterator it_end) {
  frontier_voxels_.insert(it_begin, it_end);
}

void Frontier::removeVoxel(const voxblox::GlobalIndex &global_index) {
  frontier_voxels_.erase(global_index);
}


}  // namespace frontiers
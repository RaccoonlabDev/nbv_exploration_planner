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
  std::mt19937_64 rng(
      std::chrono::steady_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> uniform(0, 1);

  color_.r = uniform(rng);
  color_.g = uniform(rng);
  color_.b = uniform(rng);
  color_.a = 0.5;

  // TODO: Add it as a param in the roslaunch
  viewpoints_.reserve(15);
}

void Frontier::addVoxel(const voxblox::GlobalIndex &global_index) {
  frontier_voxels_.conservativeResize(frontier_voxels_.rows() + 1,
                                      Eigen::NoChange_t());

  frontier_voxels_.row(frontier_voxels_.rows() - 1) << global_index.x(),
      global_index.y(), global_index.z();

  for (size_t i = 0; i < 3; ++i) {
    if (global_index[i] < aabb_min_[i]) {
      aabb_min_[i] = global_index[i];
    }
    if (global_index[i] > aabb_max_[i]) {
      aabb_max_[i] = global_index[i];
    }
  }
}

void Frontier::setMean() {
  auto mean = frontier_voxels_.cast<double>().colwise().mean();
  mean_ = {mean.x(), mean.y(), mean.z()};
}

void Frontier::getAabb(voxblox::GlobalIndex *aabb_min,
                       voxblox::GlobalIndex *aabb_max) const {
  *aabb_min = aabb_min_;
  *aabb_max = aabb_max_;
}

void Frontier::generateViewpoints(const CameraModel &camera,
                                  const size_t &max_num_threads) {
  /*std::unique_ptr<ThreadSafeIndex> index_getter(
      ThreadSafeIndexFactory::get(config_.integration_order_mode, points_C));

  std::list<std::thread> integration_threads;
  for (size_t i = 0; i <max_num_threads; ++i) {
    integration_threads.emplace_back(&SimpleTsdfIntegrator::integrateFunction,
                                     this, T_G_C, points_C, colors,
                                     freespace_points, index_getter.get());
  }

  for (std::thread &thread : integration_threads) {
    thread.join();
  }*/
}

}  // namespace frontiers
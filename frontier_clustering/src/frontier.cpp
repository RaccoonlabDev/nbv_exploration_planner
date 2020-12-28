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

void Frontier::assignYawAndFilter(
    CameraModel camera, const AlignedVector<Point> &samples,
    AlignedVector<std::pair<double, size_t>> &gains,
    const voxblox::LongIndexSet &voxels, const HighResManager &manager,
    voxblox::ThreadSafeIndex *index_getter) {
  CHECK_NOTNULL(index_getter);

  static const double voxel_size = manager.getResolution();
  static const double voxel_size_inv = 1.0 / voxel_size;
  static const int voxels_per_side = manager.getVoxelsPerSide();
  static const double voxels_per_side_inv = 1.0 / voxels_per_side;
  static const double cubic_voxel_size = voxel_size * voxel_size * voxel_size;
  static const int half_hfov = std::floor(69.0 / 2.0);

  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx)) {
    const Point &sample = samples[point_idx];

    // TODO: Make a class for the gain
    // TODO: Sort incrementally the gains
    voxblox::HierarchicalIndexSet checked_voxels_set;
    Eigen::Quaterniond orientation;
    Transformation T_G_B;
    double yaw;
    AlignedVector<Point> plane_points;
    Point u_distance, u_slope, v_center;
    int u_max;
    AlignedVector<double> vertical_gain;
    vertical_gain.reserve(360);
    for (int i = -180; i < 180; ++i) {
      yaw = M_PI * i / 180.0;
      orientation = Eigen::AngleAxisd(yaw, Point::UnitZ());
      T_G_B = Transformation(sample, orientation);
      camera.setBodyPose(T_G_B);
      const Point camera_position = camera.getCameraPose().getPosition();

      // Get the three points defining the back plane of the camera frustum.
      camera.getFarPlanePoints(0, &plane_points);

      // We map the plane into u and v coordinates, which are the plane's
      // coordinate system, with the origin at plane_points[1] and outer bounds
      // at plane_points[0] and plane_points[2].
      u_distance = plane_points[0] - plane_points[1];
      u_slope = u_distance.normalized();
      u_max = static_cast<int>(std::ceil(u_distance.norm() * voxel_size_inv));
      v_center = (plane_points[2] - plane_points[1]) / 2.0;

      Point pos;
      double gain = 0.0;
      voxblox::GlobalIndex global_voxel_idx;
      voxblox::BlockIndex block_index;
      voxblox::VoxelIndex voxel_index;
      // We then iterate over all the voxels in the coordinate space of the
      // horizontal center back bounding plane of the frustum.
      for (int u = 0; u < u_max; ++u) {
        // Get the 'real' coordinates back from the plane coordinate space.
        pos = plane_points[1] + u * u_slope * voxel_size + v_center;

        global_voxel_idx =
            (voxel_size_inv * pos).cast<voxblox::LongIndexElement>();
        block_index = voxblox::getBlockIndexFromGlobalVoxelIndex(
            global_voxel_idx, voxels_per_side_inv);
        voxel_index = voxblox::getLocalFromGlobalVoxelIndex(global_voxel_idx,
                                                            voxels_per_side);
        if (checked_voxels_set[block_index].count(voxel_index) > 0) {
          continue;
        }

        const voxblox::Point start_scaled =
            (camera_position * voxel_size_inv).cast<voxblox::FloatingPoint>();
        const voxblox::Point end_scaled =
            (pos * voxel_size_inv).cast<voxblox::FloatingPoint>();

        voxblox::LongIndexVector global_voxel_indices;
        voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

        voxblox::BlockIndex block_index_ray;
        voxblox::VoxelIndex voxel_index_ray;

        for (const voxblox::GlobalIndex &global_index_ray :
             global_voxel_indices) {
          block_index_ray = voxblox::getBlockIndexFromGlobalVoxelIndex(
              global_index_ray, voxels_per_side_inv);
          voxel_index_ray = voxblox::getLocalFromGlobalVoxelIndex(
              global_index_ray, voxels_per_side);

          bool voxel_checked = false;
          if (checked_voxels_set[block_index_ray].count(voxel_index_ray) > 0) {
            voxel_checked = true;
          }

          Point recovered_pos = global_index_ray.cast<double>() * voxel_size;
          if (not voxel_checked and camera.isPointInView(recovered_pos)) {
            if (voxels.find(global_index_ray) != voxels.end()) {
              gain += 1.0;
            } else {
              VoxelStatus status = manager.getVoxelStatusByIndex(
                  block_index_ray, voxel_index_ray);
              if (status == kUnknown or status == kOccupied) {
                break;
              }
            }
          }
        }
      }
      vertical_gain[i + 180] = gain;
    }
    double max_gain = 0.0;
    int max_gain_yaw = 0;
    double current_gain;
    for (int i = -180; i < 180; i += 5) {
      current_gain = 0.0;
      int left_idx = (i + 180) - half_hfov;
      if (left_idx < 0) {
        for (int j = 360 + left_idx; j < 360; ++j) {
          current_gain += vertical_gain[j];
        }
        left_idx = 0;
      }
      int right_idx = (i + 180) + half_hfov;
      if (right_idx >= 360) {
        for (int j = 0; j < right_idx - 359; ++j) {
          current_gain += vertical_gain[j];
        }
        right_idx = 359;
      }
      for (int j = left_idx; j <= right_idx; ++j) {
        current_gain += vertical_gain[j];
      }
      if (current_gain > max_gain) {
        max_gain = current_gain;
        max_gain_yaw = i;
      }
    }
    // state[3] = max_gain_yaw * M_PI / 180.0;
    // return max_gain * cubic_voxel_size;
  }
}

}  // namespace frontiers
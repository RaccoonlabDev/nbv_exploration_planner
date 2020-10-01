//
// Created by victor on 4/15/20.
//

#include "nbveplanner/voxblox_manager.h"

namespace nbveplanner {

VoxbloxManager::VoxbloxManager(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private,
                               const std::string &ns)
    : nh_(nh), nh_private_(nh_private), esdf_server_(nh, nh_private, ns) {
  tsdf_layer_ = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr();
  esdf_layer_ = esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr();
  CHECK_NOTNULL(tsdf_layer_);
  CHECK_NOTNULL(esdf_layer_);
  voxel_size_ = tsdf_layer_->voxel_size();
}

VoxbloxManager::VoxelStatus VoxbloxManager::getVoxelStatus(
    const voxblox::BlockIndex &block_idx,
    const voxblox::VoxelIndex &voxel_idx) const {
  const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
      tsdf_layer_->getBlockPtrByIndex(block_idx);
  if (block_ptr) {
    // If this block exists, get the voxel.
    const voxblox::TsdfVoxel &voxel =
        block_ptr->getVoxelByVoxelIndex(voxel_idx);
    if (voxel.weight <= 1e-1) {
      return VoxelStatus::kUnknown;
    } else if (voxel.distance <= 0.0) {
      return VoxelStatus::kOccupied;
    } else {
      return VoxelStatus::kFree;
    }
  } else {
    return VoxelStatus::kUnknown;
  }
}

VoxbloxManager::VoxelStatus VoxbloxManager::getVoxelStatus(
    const Point &position) const {
  double distance = 0.0;
  if (esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
                                                          &distance)) {
    // This means the voxel is observed
    if (distance < voxel_size_) {
      return VoxelStatus::kOccupied;
    } else {
      return VoxelStatus::kFree;
    }
  } else {
    return VoxelStatus::kUnknown;
  }
}

bool VoxbloxManager::checkCollisionWithRobotAtVoxel(
    const voxblox::GlobalIndex &global_index) const {
  voxblox::EsdfVoxel *voxel =
      esdf_layer_->getVoxelPtrByGlobalIndex(global_index);
  if (voxel == nullptr) {
    return true;
  }
  return robot_radius_ >= voxel->distance;
}

bool VoxbloxManager::checkMotion(const Point &start, const Point &end) {
  voxblox::Point start_scaled, end_scaled;
  voxblox::AlignedVector<voxblox::GlobalIndex> indices;

  start_scaled =
      start.cast<voxblox::FloatingPoint>() / tsdf_layer_->voxel_size();
  end_scaled = end.cast<voxblox::FloatingPoint>() / tsdf_layer_->voxel_size();

  voxblox::castRay(start_scaled, end_scaled, &indices);
  for (const auto &global_index : indices) {
    bool collision = checkCollisionWithRobotAtVoxel(global_index);
    if (collision) {
      return false;
    }
  }
  return true;
}

bool VoxbloxManager::checkMotion(const Pose &start4d, const Pose &end4d) {
  Point start = {start4d.x(), start4d.y(), start4d.z()};
  Point end = {end4d.x(), end4d.y(), end4d.z()};
  voxblox::Point start_scaled, end_scaled;
  voxblox::AlignedVector<voxblox::GlobalIndex> indices;

  start_scaled =
      start.cast<voxblox::FloatingPoint>() / tsdf_layer_->voxel_size();
  end_scaled = end.cast<voxblox::FloatingPoint>() / tsdf_layer_->voxel_size();

  voxblox::castRay(start_scaled, end_scaled, &indices);
  for (const auto &global_index : indices) {
    bool collision = checkCollisionWithRobotAtVoxel(global_index);
    if (collision) {
      return false;
    }
  }
  return true;
}

double VoxbloxManager::getDistanceAtPosition(const Point &pos) {
  double distance = 0.0;
  if (not esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(pos, &distance)) {
    return 0.0;
  }
  return distance;
}

bool VoxbloxManager::getDistanceAtPosition(const Point &pos, double *distance) {
  return esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(pos, distance);
}

bool VoxbloxManager::getDistanceAndGradientAtPosition(const Point &pos,
                                                      double *distance,
                                                      Point *grad) {
  return esdf_server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(
      pos, distance, grad);
}

VoxbloxManager::VoxelStatus VoxbloxManager::getVisibility(
    const Point &view_point, const Point &voxel_to_test,
    bool stop_at_unknown_voxel) const {
  // This involves doing a raycast from view point to voxel to test.
  // Let's get the global voxel coordinates of both.
  const static double voxel_size = tsdf_layer_->voxel_size();
  const static double voxel_size_inv = 1.0 / voxel_size;

  const voxblox::Point start_scaled =
      view_point.cast<voxblox::FloatingPoint>() * voxel_size_inv;
  const voxblox::Point end_scaled =
      voxel_to_test.cast<voxblox::FloatingPoint>() * voxel_size_inv;

  voxblox::LongIndexVector global_voxel_indices;
  voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
  // Iterate over the ray.
  for (const voxblox::GlobalIndex &global_index : global_voxel_indices) {
    voxblox::TsdfVoxel *voxel =
        tsdf_layer_->getVoxelPtrByGlobalIndex(global_index);
    if (voxel == nullptr || voxel->weight < 1e-6) {
      if (stop_at_unknown_voxel) {
        return VoxelStatus::kUnknown;
      }
    } else if (voxel->distance <= 0.0) {
      return VoxelStatus::kOccupied;
    }
  }
  return VoxelStatus::kFree;
}

void VoxbloxManager::clear() {
  esdf_server_.clear();
  tsdf_layer_ = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr();
  esdf_layer_ = esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr();
}

/*
bool VoxbloxManager::isLineInCollision(const Point &start,
                                       const Point &end) const {
  CHECK_NOTNULL(esdf_layer_);
  CHECK_GT(voxel_size_, 0.0);

  Point direction = (end - start);
  double distance = direction.norm();
  direction.normalize();

  // Don't check anything smaller than 1 voxel.
  if (distance < voxel_size_) {
    return false;
  }

  // Start at the start, keep going by distance increments...
  Point current_position = start;
  double distance_so_far = 0.0;

  while (distance_so_far <= distance) {
    voxblox::EsdfVoxel *esdf_voxel = esdf_layer_->getVoxelPtrByCoordinates(
        current_position.cast<voxblox::FloatingPoint>());
    if (esdf_voxel == nullptr) {
      return true;
    }
    if (esdf_voxel->distance < robot_radius_) {
      return true;
    }

    double step_size =
        std::max(voxel_size_, esdf_voxel->distance - robot_radius_);

    current_position += direction * step_size;
    distance_so_far += step_size;
  }
  return false;
}

bool VoxbloxManager::isLineInCollision(const Pose &start4d,
                                       const Pose &end4d) const {
  CHECK_NOTNULL(esdf_layer_);
  CHECK_GT(voxel_size_, 0.0);

  Point start = {start4d.x(), start4d.y(), start4d.z()};
  Point end = {end4d.x(), end4d.y(), end4d.z()};

  Point direction = (end - start);
  double distance = direction.norm();
  direction.normalize();

  // Don't check anything smaller than 1 voxel.
  if (distance < voxel_size_) {
    return false;
  }

  // Start at the start, keep going by distance increments...
  Point current_position = start;
  double distance_so_far = 0.0;

  while (distance_so_far <= distance) {
    voxblox::EsdfVoxel *esdf_voxel = esdf_layer_->getVoxelPtrByCoordinates(
        current_position.cast<voxblox::FloatingPoint>());
    if (esdf_voxel == nullptr) {
      return true;
    }
    if (esdf_voxel->distance < robot_radius_) {
      return true;
    }

    double step_size =
        std::max(voxel_size_, esdf_voxel->distance - robot_radius_);

    current_position += direction * step_size;
    distance_so_far += step_size;
  }
  return false;
}
 */
}  // namespace nbveplanner
//
// Created by victor on 4/15/20.
//

#include "nbveplanner/voxblox_manager.h"

namespace nbveplanner {

VoxbloxManager::VoxbloxManager(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private,
                               Params *params)
    : nh_(nh), nh_private_(nh_private), params_(CHECK_NOTNULL(params)) {}

VoxelStatus VoxbloxManager::getVoxelStatus(
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

VoxelStatus VoxbloxManager::getVisibility(const Point &view_point,
                                          const Point &voxel_to_test,
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

LowResManager::LowResManager(const ros::NodeHandle &nh,
                             const ros::NodeHandle &nh_private, Params *params,
                             const std::string &ns)
    : VoxbloxManager(nh, nh_private, params), tsdf_server_(nh, nh_private, ns) {
  tsdf_layer_ = tsdf_server_.getTsdfMapPtr()->getTsdfLayerPtr();
  voxel_size_ = tsdf_layer_->voxel_size();
}

void LowResManager::clear() {
  tsdf_server_.clear();
  tsdf_layer_ = tsdf_server_.getTsdfMapPtr()->getTsdfLayerPtr();
}

HighResManager::HighResManager(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private,
                               Params *params)
    : VoxbloxManager(nh, nh_private, params), esdf_server_(nh, nh_private, "") {
  tsdf_layer_ = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr();
  esdf_layer_ = esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr();
  voxel_size_ = tsdf_layer_->voxel_size();
}

VoxelStatus HighResManager::getVoxelStatus(const Point &position) const {
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

bool HighResManager::checkCollisionWithRobotAtVoxel(
    const voxblox::GlobalIndex &global_index) const {
  voxblox::EsdfVoxel *voxel =
      esdf_layer_->getVoxelPtrByGlobalIndex(global_index);
  if (voxel == nullptr) {
    return true;
  }
  return params_->robot_radius_ >= voxel->distance;
}

template <typename Type>
bool HighResManager::checkMotion(const StateVec &start_, const StateVec &end_) {
  Point start = {start_.x(), start_.y(), start_.z()};
  Point end = {end_.x(), end_.y(), end_.z()};
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

double HighResManager::getDistanceAtPosition(const Point &pos) {
  double distance = 0.0;
  if (not esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(pos, &distance)) {
    return 0.0;
  }
  return distance;
}

bool HighResManager::getDistanceAtPosition(const Point &pos, double *distance) {
  return esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(pos, distance);
}

bool HighResManager::getDistanceAndGradientAtPosition(const Point &pos,
                                                      double *distance,
                                                      Point *grad) {
  return esdf_server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(
      pos, distance, grad);
}

void HighResManager::clear() {
  esdf_server_.clear();
  tsdf_layer_ = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr();
  esdf_layer_ = esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr();
}

}  // namespace nbveplanner
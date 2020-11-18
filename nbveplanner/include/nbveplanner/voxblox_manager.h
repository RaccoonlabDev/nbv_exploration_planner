//
// Created by victor on 4/15/20.
//

#ifndef NBVPLANNER_VOXBLOXMANAGER_H_
#define NBVPLANNER_VOXBLOXMANAGER_H_

#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "nbveplanner/common.h"
#include "nbveplanner/params.h"

namespace nbveplanner {

enum VoxelStatus { kUnknown, kOccupied, kFree };

class VoxbloxManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VoxbloxManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                 Params *params);

  VoxelStatus getVisibility(const Point &view_point, const Point &voxel_to_test,
                            bool stop_at_unknown_voxel) const;

  VoxelStatus getVoxelStatusByIndex(const voxblox::BlockIndex &block_idx,
                                    const voxblox::VoxelIndex &voxel_idx) const;

  virtual VoxelStatus getVoxelStatus(const Point &position) const = 0;

  double getResolution() const { return tsdf_layer_->voxel_size(); }

  int getVoxelsPerSide() const { return tsdf_layer_->voxels_per_side(); }

  virtual void clear() = 0;

  bool isTsdfEmpty() const {
    return tsdf_layer_->getNumberOfAllocatedBlocks() == 0;
  }

  double getNumberMappedVoxels() {
    static const double voxels_per_side = tsdf_layer_->voxels_per_side();
    static const double volume_per_block = voxels_per_side * voxels_per_side *
                                           voxels_per_side * voxel_size_ *
                                           voxel_size_ * voxel_size_;
    return tsdf_layer_->getNumberOfAllocatedBlocks() * volume_per_block;
  }

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  Params *params_;
  voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer_;
  double voxel_size_;
};

class LowResManager : public VoxbloxManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LowResManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                Params *params, const std::string &ns);

  VoxelStatus getVoxelStatus(const Point &position) const override;

  void clear() override;

 private:
  voxblox::TsdfServer tsdf_server_;
};

class HighResManager : public VoxbloxManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HighResManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                 Params *params);

  VoxelStatus getVoxelStatus(const Point &position) const override;

  bool checkCollisionWithRobotAtVoxel(
      const voxblox::GlobalIndex &global_index) const;

  template <typename T>
  bool checkMotion(const T &start_, const T &end_) {
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

  double getDistanceAtPosition(const Point &pos);

  bool getDistanceAtPosition(const Point &pos, double *distance);

  bool getDistanceAndGradientAtPosition(const Point &pos, double *distance,
                                        Point *grad);

  bool isEsdfEmpty() const {
    return esdf_layer_->getNumberOfAllocatedBlocks() == 0;
  }

  void clear() override;

 private:
  voxblox::EsdfServer esdf_server_;
  voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer_;
};

}  // namespace nbveplanner

#endif  // NBVPLANNER_VOXBLOXMANAGER_H_

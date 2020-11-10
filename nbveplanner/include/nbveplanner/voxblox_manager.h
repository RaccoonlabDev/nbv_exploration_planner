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

class VoxbloxManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum VoxelStatus { kUnknown, kOccupied, kFree };

  VoxbloxManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                 Params *params, const std::string &ns);

  VoxelStatus getVisibility(const Point &view_point, const Point &voxel_to_test,
                            bool stop_at_unknown_voxel) const;

  VoxelStatus getVoxelStatus(const voxblox::BlockIndex &block_idx,
                             const voxblox::VoxelIndex &voxel_idx) const;

  VoxelStatus getVoxelStatus(const Point &position) const;

  bool checkCollisionWithRobotAtVoxel(
      const voxblox::GlobalIndex &global_index) const;

  bool checkMotion(const Point &start, const Point &end);

  bool checkMotion(const Pose &start4d, const Pose &end4d);

  double getResolution() const { return tsdf_layer_->voxel_size(); }

  int getVoxelsPerSide() const { return tsdf_layer_->voxels_per_side(); }

  bool isTsdfEmpty() const {
    return tsdf_layer_->getNumberOfAllocatedBlocks() == 0;
  }

  bool isEsdfEmpty() const {
    return esdf_layer_->getNumberOfAllocatedBlocks() == 0;
  }

  double getDistanceAtPosition(const Point &pos);

  bool getDistanceAtPosition(const Point &pos, double *distance);

  bool getDistanceAndGradientAtPosition(const Point &pos, double *distance,
                                        Point *grad);

  void clear();

  double getNumberMappedVoxels() {
    static const double voxels_per_side = tsdf_layer_->voxels_per_side();
    static const double volume_per_block = voxels_per_side * voxels_per_side *
                                           voxels_per_side * voxel_size_ *
                                           voxel_size_ * voxel_size_;
    return tsdf_layer_->getNumberOfAllocatedBlocks() * volume_per_block;
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  Params *params_;
  voxblox::EsdfServer esdf_server_;

  // Cached:
  voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer_;
  voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer_;

  double voxel_size_;
};

}  // namespace nbveplanner

#endif  // NBVPLANNER_VOXBLOXMANAGER_H_

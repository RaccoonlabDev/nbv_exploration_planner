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
                 std::shared_ptr<Params> params, const std::string &ns);

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

  double getNumberOfMappedVoxels() {
    // static const double volume = pow(esdf_layer_->block_size(), 3);
    double nvoxels =
        esdf_layer_->getNumberOfAllocatedBlocks() * esdf_layer_->block_size();
    std::cout << "Number of mapped space: " << nvoxels << std::endl;
    return nvoxels;
  }

  void clear();

  /*
  bool isLineInCollision(const Point &start,
                         const Point &end) const;

  bool isLineInCollision(const Pose &start4d,
                         const Pose &end4d) const;
                         */

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::shared_ptr<Params> params_;
  voxblox::EsdfServer esdf_server_;

  // Cached:
  voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer_;
  voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer_;

  double voxel_size_;

};

}  // namespace nbveplanner

#endif  // NBVPLANNER_VOXBLOXMANAGER_H_

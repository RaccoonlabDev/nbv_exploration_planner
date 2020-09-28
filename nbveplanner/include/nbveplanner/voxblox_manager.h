//
// Created by victor on 4/15/20.
//

#ifndef NBVPLANNER_VOXBLOXMANAGER_H_
#define NBVPLANNER_VOXBLOXMANAGER_H_

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>

class VoxbloxManager {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum VoxelStatus { kUnknown, kOccupied, kFree };

  VoxbloxManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                 const std::string &ns);

  VoxelStatus getVisibility(const Eigen::Vector3d &view_point,
                            const Eigen::Vector3d &voxel_to_test,
                            bool stop_at_unknown_voxel) const;



  VoxelStatus getVoxelStatus(const Eigen::Vector3d &position) const;

  bool checkCollisionWithRobotAtVoxel(
      const voxblox::GlobalIndex &global_index) const;

  bool checkMotion(const Eigen::Vector3d &start, const Eigen::Vector3d &end);

  bool checkMotion(const Eigen::Vector4d &start4d,
                   const Eigen::Vector4d &end4d);

  double getResolution() const { return tsdf_layer_->voxel_size(); }

  bool isTsdfEmpty() const {
    return tsdf_layer_->getNumberOfAllocatedBlocks() == 0;
  }

  bool isEsdfEmpty() const {
    return esdf_layer_->getNumberOfAllocatedBlocks() == 0;
  }

  double getDistanceAtPosition(const Eigen::Vector3d &pos);

  bool getDistanceAtPosition(const Eigen::Vector3d &pos, double *distance);

  bool getDistanceAndGradientAtPosition(const Eigen::Vector3d &pos,
                                        double *distance,
                                        Eigen::Vector3d *grad);

  void setRobotRadius(const double &robot_radius) {
    robot_radius_ = robot_radius;
  }

  double getNumberOfMappedVoxels() {
    // static const double volume = pow(esdf_layer_->block_size(), 3);
    double nvoxels =
        esdf_layer_->getNumberOfAllocatedBlocks() * esdf_layer_->block_size();
    std::cout << "Number of mapped space: " << nvoxels << std::endl;
    return nvoxels;
  }

  void clear();

  /*
  bool isLineInCollision(const Eigen::Vector3d &start,
                         const Eigen::Vector3d &end) const;

  bool isLineInCollision(const Eigen::Vector4d &start4d,
                         const Eigen::Vector4d &end4d) const;
                         */

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  voxblox::EsdfServer esdf_server_;

  // Cached:
  voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer_;
  voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer_;

  double robot_radius_;
  double voxel_size_;
};

#endif // NBVPLANNER_VOXBLOXMANAGER_H_

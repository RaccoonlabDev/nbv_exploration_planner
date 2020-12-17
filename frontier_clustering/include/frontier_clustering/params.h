//
// Created by victor on 9/25/20.
//

#ifndef SRC_PARAMS_H_
#define SRC_PARAMS_H_

#include <ros/package.h>
#include <ros/ros.h>
#include <thread>
#include "frontier_clustering/camera_model.h"

namespace frontiers {

struct Params {
 public:
  Params()
      : v_max_(1.0),
        dyaw_max_(0.5),
        camera_pitch_(0.0),
        camera_hfov_(90.0),
        camera_vfov_(60),
        frame_id_("world"),
        camera_frame_("camera"),
        sensor_min_range_(0.1),
        sensor_max_range_(5.0),
        robot_radius_(0.5),
        size_threshold_(50000),
        min_num_voxels_(10),
        voxel_size_(0.1),
        voxels_per_side_(16) {}

  void setParametersFromRos(const ros::NodeHandle& nh) {
    nh.param("system/v_max", v_max_, v_max_);
    VLOG(5) << "v_max: " << v_max_;
    nh.param("system/dyaw_max", dyaw_max_, dyaw_max_);
    VLOG(5) << "dyaw_max: " << dyaw_max_;
    nh.param("system/camera/pitch", camera_pitch_, camera_pitch_);
    VLOG(5) << "camera_pitch: " << camera_pitch_;
    nh.param("system/camera/hfov", camera_hfov_, camera_hfov_);
    VLOG(5) << "camera_hfov: " << camera_hfov_;
    nh.param("system/camera/vfov", camera_vfov_, camera_vfov_);
    VLOG(5) << "camera_vfov: " << camera_vfov_;

    nh.param("bbx/minX", bbx_min_.x(), bbx_min_.x());
    VLOG(5) << "min_x: " << bbx_min_.x();
    nh.param("bbx/minY", bbx_min_.y(), bbx_min_.y());
    VLOG(5) << "min_y: " << bbx_min_.y();
    nh.param("bbx/minZ", bbx_min_.z(), bbx_min_.z());
    VLOG(5) << "min_z: " << bbx_min_.z();
    nh.param("bbx/maxX", bbx_max_.x(), bbx_max_.x());
    VLOG(5) << "max_x: " << bbx_max_.x();
    nh.param("bbx/maxY", bbx_max_.y(), bbx_max_.y());
    VLOG(5) << "max_y: " << bbx_max_.y();
    nh.param("bbx/maxZ", bbx_max_.z(), bbx_max_.z());
    VLOG(5) << "max_z: " << bbx_max_.z();

    nh.param("tf_frame", frame_id_, frame_id_);
    VLOG(5) << "frame_id: " << frame_id_;
    nh.param("camera_frame", camera_frame_, camera_frame_);
    VLOG(5) << "camera_frame: " << camera_frame_;
    nh.param("sensor_min_range", sensor_min_range_, sensor_min_range_);
    VLOG(5) << "sensor_min_range: " << sensor_min_range_;
    nh.param("sensor_max_range", sensor_max_range_, sensor_max_range_);
    VLOG(5) << "sensor_max_range: " << sensor_max_range_;
    nh.param("system/robot_radius", robot_radius_, robot_radius_);
    VLOG(5) << "robot_radius: " << robot_radius_;

    nh.param("size_threshold", size_threshold_, size_threshold_);
    VLOG(5) << "size_threshold: " << size_threshold_;
    nh.param("min_num_voxels", min_num_voxels_, min_num_voxels_);
    VLOG(5) << "min_num_voxels: " << min_num_voxels_;

    nh.param("voxel_size", voxel_size_, voxel_size_);
    VLOG(5) << "voxel_size: " << voxel_size_;
    nh.param("voxels_per_side", voxels_per_side_, voxels_per_side_);
    VLOG(5) << "voxels_per_side: " << voxels_per_side_;
  }

  double robot_radius_;
  double camera_pitch_;
  double camera_hfov_;
  double camera_vfov_;

  double sensor_min_range_;
  double sensor_max_range_;

  double v_max_;
  double dyaw_max_;
  Point bbx_min_;
  Point bbx_max_;

  std::string frame_id_;
  std::string camera_frame_;
  double size_threshold_;
  int min_num_voxels_;

  double voxel_size_;
  double voxels_per_side_;

  size_t max_num_threads_ = std::thread::hardware_concurrency();
};
}  // namespace frontiers

#endif  // SRC_PARAMS_H_

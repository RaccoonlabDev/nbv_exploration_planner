//
// Created by victor on 9/25/20.
//

#ifndef NBVEPLANNER_PARAMS_H_
#define NBVEPLANNER_PARAMS_H_

#include <nbveplanner/camera_model.h>
#include <ros/ros.h>

struct Params {
 public:
  Params()
      : v_max_(1.0),
        dyaw_max_(0.5),
        camera_pitch_(0.0),
        camera_hfov_(90.0),
        camera_vfov_(60),
        gain_free_(0.0),
        gain_occupied_(0.0),
        gain_unmapped_(1.0),
        degressive_coeff_(0.25),
        init_iterations_(15),
  extension_range_(1.0),
  vicinity_range_(5.0),
  dt_(0.1),
  gain_range_(5.0),
  {}

  void setParametersFromRos(const ros::NodeHandle& nh) {
    nh.param("/system/v_max", v_max_, v_max_);
    nh.param("/system/dyaw_max", dyaw_max_, dyaw_max_);
    nh.param("/system/camera/pitch", camera_pitch_, camera_pitch_);
    nh.param("/system/camera/hfov", camera_hfov_, camera_hfov_);
    nh.param("/system/camera/vfov", camera_vfov_, camera_vfov_);
    nh.param("/nbvep/gain/free", gain_free_, gain_free_);
    nh.param("/nbvep/gain/occupied", gain_occupied_, gain_occupied_);
    nh.param("/nbvep/gain/unmapped", gain_unmapped_, gain_unmapped_);
    nh.param("/nbvep/gain/degressive_coeff", degressive_coeff_,
             degressive_coeff_);
    nh.param("/nbvep/tree/init_iterations", init_iterations_, init_iterations_);
    nh.param("/nbvep/tree/extension_range", extension_range_, extension_range_);
    nh.param("/nbvep/tree/vicinity_range", vicinity_range_, vicinity_range_);
    nh.param("/nbvep/dt", dt_,dt_);
    nh.param("/nbvep/gain/range", gain_range_, gain_range_);
    nh.param()

    if (!ros::param::get(ns + "/bbx/minX", params_.minX_)) {
      ROS_WARN("No x-min value specified. Looking for %s",
               (ns + "/bbx/minX").c_str());
      ret = false;
    }
    if (!ros::param::get(ns + "/bbx/minY", params_.minY_)) {
      ROS_WARN("No y-min value specified. Looking for %s",
               (ns + "/bbx/minY").c_str());
      ret = false;
    }
    if (!ros::param::get(ns + "/bbx/minZ", params_.minZ_)) {
      ROS_WARN("No z-min value specified. Looking for %s",
               (ns + "/bbx/minZ").c_str());
      ret = false;
    }
    if (!ros::param::get(ns + "/bbx/maxX", params_.maxX_)) {
      ROS_WARN("No x-max value specified. Looking for %s",
               (ns + "/bbx/maxX").c_str());
      ret = false;
    }
    if (!ros::param::get(ns + "/bbx/maxY", params_.maxY_)) {
      ROS_WARN("No y-max value specified. Looking for %s",
               (ns + "/bbx/maxY").c_str());
      ret = false;
    }
    if (!ros::param::get(ns + "/bbx/maxZ", params_.maxZ_)) {
      ROS_WARN("No z-max value specified. Looking for %s",
               (ns + "/bbx/maxZ").c_str());
      ret = false;
    }
    params_.boundingBox_[0] = 0.5;
    if (!ros::param::get(ns + "/system/bbx/x", params_.boundingBox_[0])) {
      ROS_WARN("No x size value specified. Looking for %s. Default is 0.5m.",
               (ns + "/system/bbx/x").c_str());
    }
    params_.boundingBox_[1] = 0.5;
    if (!ros::param::get(ns + "/system/bbx/y", params_.boundingBox_[1])) {
      ROS_WARN("No y size value specified. Looking for %s. Default is 0.5m.",
               (ns + "/system/bbx/y").c_str());
    }
    params_.boundingBox_[2] = 0.3;
    if (!ros::param::get(ns + "/system/bbx/z", params_.boundingBox_[2])) {
      ROS_WARN("No z size value specified. Looking for %s. Default is 0.3m.",
               (ns + "/system/bbx/z").c_str());
    }
    params_.cuttoffIterations_ = 200;
    if (!ros::param::get(ns + "/nbvep/tree/cuttoff_iterations",
                         params_.cuttoffIterations_)) {
      ROS_WARN(
          "No cuttoff iterations value specified. Looking for %s. Default "
          "is 200.",
          (ns + "/nbvep/tree/cuttoff_iterations").c_str());
    }
    params_.zero_gain_ = 0.0;
    if (!ros::param::get(ns + "/nbvep/gain/zero", params_.zero_gain_)) {
      ROS_WARN("No zero gain value specified. Looking for %s. Default is 0.0.",
               (ns + "/nbvep/gain/zero").c_str());
    }
    params_.dOvershoot_ = 0.5;
    if (!ros::param::get(ns + "/system/bbx/overshoot", params_.dOvershoot_)) {
      ROS_WARN(
          "No estimated overshoot value for collision avoidance specified. "
          "Looking for %s. Default is 0.5m.",
          (ns + "/system/bbx/overshoot").c_str());
    }
    params_.navigationFrame_ = "world";
    if (!ros::param::get(ns + "/tf_frame", params_.navigationFrame_)) {
      ROS_WARN(
          "No navigation frame specified. Looking for %s. Default is 'world'.",
          (ns + "/tf_frame").c_str());
    }
    params_.voxelSize_ = 0.2;
    if (!ros::param::get(ns + "/tsdf_voxel_size", params_.voxelSize_)) {
      ROS_WARN(
          "No option for voxel size specified. Looking for %s. "
          "Default is 0.2.",
          (ns + "/tsdf_voxel_size").c_str());
    }

    nh_private_.param("sensor_min_range", params_.sensor_min_range_, 0.1);
    nh_private_.param("sensor_max_range", params_.sensor_max_range_, 5.0);
    params_.robot_radius_ = params_.boundingBox_.norm() / 2.0;
    manager_->setRobotRadius(params_.robot_radius_);
    return ret;
  }

  CameraModel camera_model_;
  double camera_pitch_;
  double camera_hfov_;
  double camera_vfov_;

  double sensor_min_range_;
  double sensor_max_range_;

  double gain_free_;
  double gain_occupied_;
  double gain_unmapped_;
  double gain_range_;
  double degressive_coeff_;
  double zero_gain_;

  double v_max_;
  double dyaw_max_;
  double dist_overshoot_;
  double extension_range_;
  double vicinity_range_;
  int init_iterations_;
  int cuttoff_iterations_;
  double dt_;

  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;

  double robot_radius_;
  ros::Publisher inspection_path_;
  ros::Publisher exploration_tree_;
  std::string frame_id_;

  double voxel_size_;
};

#endif  // NBVEPLANNER_PARAMS_H_

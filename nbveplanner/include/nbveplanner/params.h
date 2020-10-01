//
// Created by victor on 9/25/20.
//

#ifndef NBVEPLANNER_PARAMS_H_
#define NBVEPLANNER_PARAMS_H_

#include <ros/ros.h>
#include "nbveplanner/camera_model.h"

namespace nbveplanner {

struct Params {
 public:
  Params()
      : camera_model_(),
        v_max_(1.0),
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
        bbx_min_(0.0, 0.0, 0.0),
        bbx_max_(0.0, 0.0, 0.0),
        cutoff_iterations_(200),
        zero_gain_(0.0),
        dist_overshoot_(0.25),
        frame_id_("world"),
        sensor_min_range_(0.1),
        sensor_max_range_(5.0),
        robot_radius_(0.5) {}

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
    nh.param("nbvep/gain/free", gain_free_, gain_free_);
    VLOG(5) << "gain_free: " << gain_free_;
    nh.param("nbvep/gain/occupied", gain_occupied_, gain_occupied_);
    VLOG(5) << "gain_occupied: " << gain_occupied_;
    nh.param("nbvep/gain/unmapped", gain_unmapped_, gain_unmapped_);
    VLOG(5) << "gain_unmapped: " << gain_unmapped_;
    nh.param("nbvep/gain/degressive_coeff", degressive_coeff_,
             degressive_coeff_);
    VLOG(5) << "degressive_coeff: " << degressive_coeff_;
    nh.param("nbvep/tree/init_iterations", init_iterations_, init_iterations_);
    VLOG(5) << "init_iterations: " << init_iterations_;
    nh.param("nbvep/tree/extension_range", extension_range_, extension_range_);
    VLOG(5) << "extension_range: " << extension_range_;
    nh.param("nbvep/tree/vicinity_range", vicinity_range_, vicinity_range_);
    VLOG(5) << "vicinity_range: " << vicinity_range_;
    nh.param("nbvep/dt", dt_, dt_);
    VLOG(5) << "dt: " << dt_;
    nh.param("nbvep/gain/range", gain_range_, gain_range_);
    VLOG(5) << "gain_range: " << gain_range_;
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
    nh.param("nbvep/tree/cutoff_iterations", cutoff_iterations_,
             cutoff_iterations_);
    VLOG(5) << "cutoff_iterations: " << cutoff_iterations_;
    nh.param("nbvep/gain/zero", zero_gain_, zero_gain_);
    VLOG(5) << "zero_gain: " << zero_gain_;
    nh.param("system/bbx/overshoot", dist_overshoot_, dist_overshoot_);
    VLOG(5) << "overshoot: " << dist_overshoot_;
    nh.param("tf_frame", frame_id_, frame_id_);
    VLOG(5) << "frame_id: " << frame_id_;
    nh.param("sensor_min_range", sensor_min_range_, sensor_min_range_);
    VLOG(5) << "sensor_min_range: " << sensor_min_range_;
    nh.param("sensor_max_range", sensor_max_range_, sensor_max_range_);
    VLOG(5) << "sensor_max_range: " << sensor_max_range_;
    nh.param("system/robot_radius", robot_radius_, robot_radius_);
    VLOG(5) << "robot_radius: " << robot_radius_;
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
  int cutoff_iterations_;
  double dt_;

  Point bbx_min_;
  Point bbx_max_;

  double robot_radius_;
  ros::Publisher inspection_path_;
  ros::Publisher exploration_tree_;
  std::string frame_id_;
};
}  // namespace nbveplanner

#endif  // NBVEPLANNER_PARAMS_H_

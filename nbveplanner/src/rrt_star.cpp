//
// Created by op on 05.10.2020.
//

#include "nbveplanner/rrt_star.h"

namespace nbveplanner {

RRTStar::RRTStar(VoxbloxManager *manager, VoxbloxManager *manager_lowres,
                 Params *params)
    : TreeBase(manager, manager_lowres, params) {}

RRTStar::~RRTStar() {
  delete rootNode_;
  kd_free(kdTree_);
}

void RRTStar::setStateFromPoseCovMsg(
    const geometry_msgs::PoseWithCovarianceStamped &pose) {
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_->frame_id_, pose.header.frame_id,
                             pose.header.stamp, transform);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  tf::Vector3 position = poseTF.getOrigin();
  position = transform * position;
  tf::Quaternion quat = poseTF.getRotation();
  quat = transform * quat;
  root_[0] = position.x();
  root_[1] = position.y();
  root_[2] = position.z();
  root_[3] = tf::getYaw(quat);
}

void RRTStar::setStateFromOdometryMsg(const nav_msgs::Odometry &pose) {
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_->frame_id_, pose.header.frame_id,
                             pose.header.stamp, transform);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  tf::Vector3 position = poseTF.getOrigin();
  position = transform * position;
  tf::Quaternion quat = poseTF.getRotation();
  quat = transform * quat;
  root_[0] = position.x();
  root_[1] = position.y();
  root_[2] = position.z();
  root_[3] = tf::getYaw(quat);
}


}  // namespace nbveplanner
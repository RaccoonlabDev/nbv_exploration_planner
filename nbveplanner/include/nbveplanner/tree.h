#ifndef NBVEPLANNER_TREE_H_
#define NBVEPLANNER_TREE_H_

#include "nbveplanner/camera_model.h"
#include "nbveplanner/common.h"
#include "nbveplanner/params.h"
#include "nbveplanner/voxblox_manager.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace nbveplanner {

class Node {
 public:
  Node();

  ~Node();

  Pose state_;
  Node *parent_;
  std::vector<Node *> children_;
  double gain_;
  double distance_;
  int id_;
};

class TreeBase {
 protected:
  int counter_;
  double bestGain_;
  Node *bestNode_;
  Node *rootNode_;

  Pose root_;
  Pose hist_root_;
  Pose exact_root_;

  Params *params_;
  VoxbloxManager *manager_;
  VoxbloxManager *manager_lowres_;

 public:
  TreeBase(VoxbloxManager *manager, VoxbloxManager *manager_lowres,
           Params *params);

  ~TreeBase();

  virtual void setStateFromPoseCovMsg(
      const geometry_msgs::PoseWithCovarianceStamped &pose) = 0;

  virtual void setStateFromOdometryMsg(const nav_msgs::Odometry &pose) = 0;

  virtual void initialize(bool seedHistory) = 0;

  virtual void iterate() = 0;

  virtual void getBestBranch(std::vector<geometry_msgs::Pose> &path,
                             std::vector<geometry_msgs::Pose> &trajectory) = 0;

  virtual void clear() = 0;

  virtual void reset() = 0;

  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(
      std::string targetFrame) = 0;

  virtual void setRootVicinity(double rootVicinity) = 0;

  void setHistRoot(const Pose &root);

  int getCounter() const;

  bool gainFound();
};

}  // namespace nbveplanner
#endif

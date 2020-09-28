/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TREE_H_
#define TREE_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nbveplanner/camera_model.h>
#include <nbveplanner/voxblox_manager.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Dense>
#include <vector>

template <typename stateVec>
class Node {
 public:
  Node();

  ~Node();

  stateVec state_;
  Node *parent_;
  std::vector<Node *> children_;
  double gain_;
  double distance_;
};

template <typename stateVec>
class TreeBase {
 protected:
  Params *params_;
  int counter_;
  double bestGain_;
  Node<stateVec> *bestNode_;
  Node<stateVec> *rootNode_;
  VoxbloxManager *manager_;
  VoxbloxManager *manager_lowres_;
  stateVec root_;
  stateVec hist_root_;
  stateVec exact_root_;

 public:
  typedef Eigen::Vector4d StateVec;

  TreeBase();

  TreeBase(VoxbloxManager *manager, VoxbloxManager *manager_lowres, Params *params);

  ~TreeBase();

  virtual void setStateFromPoseCovMsg(
      const geometry_msgs::PoseWithCovarianceStamped &pose) = 0;

  virtual void setStateFromOdometryMsg(const nav_msgs::Odometry &pose) = 0;

  virtual void iterate() = 0;

  virtual void initialize(bool seedHistory) = 0;

  virtual std::vector<geometry_msgs::Pose> getBestEdge(
      std::string targetFrame) = 0;

  virtual void getBestBranch(std::vector<geometry_msgs::Pose> &path,
                             std::vector<geometry_msgs::Pose> &trajectory) = 0;

  virtual void clear() = 0;

  virtual void reset() = 0;

  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(
      std::string targetFrame) = 0;

  virtual void setRootVicinity(double rootVicinity) = 0;

  void setParams(Params *params);

  void setHistRoot(const Eigen::Vector4d &root);

  int getCounter();

  bool gainFound();
};

#endif

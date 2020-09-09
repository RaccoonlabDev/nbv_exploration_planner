#ifndef RRTTREE_HPP_
#define RRTTREE_HPP_

#include <cstdlib>
#include <nbveplanner/rrt.h>
#include <nbveplanner/tree.hpp>

RrtTree::RrtTree() : TreeBase<StateVec>::TreeBase() {
  kdTree_ = kd_create(3);
  iterationCount_ = 0;
  // If logging is required, set up files here
  bool ifLog = false;
  const std::string &ns = ros::this_node::getName();
  ros::param::get(ns + "/nbvep/log/on", ifLog);
  if (ifLog) {
    time_t rawtime;
    struct tm *ptm;
    time(&rawtime);
    ptm = gmtime(&rawtime);
    logFilePath_ =
        ros::package::getPath("nbveplanner") + "/data/" +
        std::to_string(ptm->tm_year + 1900) + "_" +
        std::to_string(ptm->tm_mon + 1) + "_" + std::to_string(ptm->tm_mday) +
        "_" + std::to_string(ptm->tm_hour) + "_" + std::to_string(ptm->tm_min) +
        "_" + std::to_string(ptm->tm_sec);
    system(("mkdir -p " + logFilePath_).c_str());
    logFilePath_ += "/";
    fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
    filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
  }
}

RrtTree::RrtTree(VoxbloxManager *manager) {
  manager_ = manager;
  kdTree_ = kd_create(3);
  iterationCount_ = 0;

  // If logging is required, set up files here
  bool ifLog = false;
  std::string ns = ros::this_node::getName();
  ros::param::get(ns + "/nbvep/log/on", ifLog);
  if (ifLog) {
    time_t rawtime;
    struct tm *ptm;
    time(&rawtime);
    ptm = gmtime(&rawtime);
    logFilePath_ =
        ros::package::getPath("nbveplanner") + "/data/" +
        std::to_string(ptm->tm_year + 1900) + "_" +
        std::to_string(ptm->tm_mon + 1) + "_" + std::to_string(ptm->tm_mday) +
        "_" + std::to_string(ptm->tm_hour) + "_" + std::to_string(ptm->tm_min) +
        "_" + std::to_string(ptm->tm_sec);
    system(("mkdir -p " + logFilePath_).c_str());
    logFilePath_ += "/";
    fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
    filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
  }
}

RrtTree::~RrtTree() {
  delete rootNode_;
  kd_free(kdTree_);
  if (fileResponse_.is_open()) {
    fileResponse_.close();
  }
  if (fileTree_.is_open()) {
    fileTree_.close();
  }
  if (filePath_.is_open()) {
    filePath_.close();
  }
}

void RrtTree::setStateFromPoseCovMsg(
    const geometry_msgs::PoseWithCovarianceStamped &pose) {
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id,
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

  // Log the vehicle response in the planning frame
  static double logThrottleTime = ros::Time::now().toSec();
  if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_) {
    logThrottleTime += params_.log_throttle_;
    if (params_.log_) {
      for (int i = 0; i < root_.size() - 1; i++) {
        fileResponse_ << root_[i] << ",";
      }
      fileResponse_ << root_[root_.size() - 1] << "\n";
    }
  }
}

void RrtTree::setStateFromOdometryMsg(const nav_msgs::Odometry &pose) {
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id,
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

  // Log the vehicle response in the planning frame
  static double logThrottleTime = ros::Time::now().toSec();
  if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_) {
    logThrottleTime += params_.log_throttle_;
    if (params_.log_) {
      for (int i = 0; i < root_.size() - 1; i++) {
        fileResponse_ << root_[i] << ",";
      }
      fileResponse_ << root_[root_.size() - 1] << "\n";
    }
  }
}

void RrtTree::iterate() {
  // In this function a new configuration is sampled and added to the tree.
  StateVec newState;

  Node<StateVec> *newParent;
  Eigen::Vector3d origin;
  Eigen::Vector3d direction;
  bool solutionFound = false;

  std::mt19937 generator(
      std::chrono::steady_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  while (!solutionFound) {
    for (int i = 0; i < 3; i++) {
      newState[i] = 2.0 * rootVicinity_ * (distribution(generator) - 0.5);
    }
    if (SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]) >
        pow(rootVicinity_, 2.0))
      continue;
    // Offset new state by root
    newState += rootNode_->state_;

    if (newState.x() < params_.minX_ + 0.5 * params_.boundingBox_.x() or
        newState.y() < params_.minY_ + 0.5 * params_.boundingBox_.y() or
        newState.z() < params_.minZ_ + 0.5 * params_.boundingBox_.z() or
        newState.x() > params_.maxX_ - 0.5 * params_.boundingBox_.x() or
        newState.y() > params_.maxY_ - 0.5 * params_.boundingBox_.y() or
        newState.z() > params_.maxZ_ - 0.5 * params_.boundingBox_.z()) {
      continue;
    }
    solutionFound = true;

    // Find nearest neighbour
    kdres *nearest =
        kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
    if (kd_res_size(nearest) <= 0) {
      kd_res_free(nearest);
      return;
    }
    newParent = (Node<StateVec> *)kd_res_item_data(nearest);
    kd_res_free(nearest);

    // Check for collision of new connection plus some overshoot distance.
    origin = Eigen::Vector3d(newParent->state_[0], newParent->state_[1],
                             newParent->state_[2]);
    direction =
        Eigen::Vector3d(newState[0] - origin[0], newState[1] - origin[1],
                        newState[2] - origin[2]);
    if (direction.norm() > params_.extensionRange_) {
      direction = params_.extensionRange_ * direction.normalized();
    }
    newState[0] = origin[0] + direction[0];
    newState[1] = origin[1] + direction[1];
    newState[2] = origin[2] + direction[2];

    if (manager_->checkMotion(origin, direction + origin +
                                          direction.normalized() *
                                              params_.dOvershoot_)) {
      double maxGainFound = -DBL_MAX;
      double orientationFound;
      newState[3] = 2.0 * M_PI * (distribution(generator) - 0.5);
      std::thread t1(&RrtTree::gain, this, newState, std::ref(maxGainFound),
                     std::ref(orientationFound));
      newState[3] = 2.0 * M_PI * (distribution(generator) - 0.5);
      std::thread t2(&RrtTree::gain, this, newState, std::ref(maxGainFound),
                     std::ref(orientationFound));
      newState[3] = 2.0 * M_PI * (distribution(generator) - 0.5);
      std::thread t3(&RrtTree::gain, this, newState, std::ref(maxGainFound),
                     std::ref(orientationFound));
      newState[3] = 2.0 * M_PI * (distribution(generator) - 0.5);
      std::thread t4(&RrtTree::gain, this, newState, std::ref(maxGainFound),
                     std::ref(orientationFound));

      t1.join();
      t2.join();
      t3.join();
      t4.join();
      newState[3] = orientationFound;
      // Create new node and insert into tree
      auto *newNode = new Node<StateVec>;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);
      newNode->gain_ =
          maxGainFound * exp(-params_.degressiveCoeff_ * newNode->distance_);
      //std::cout << "Gain: " << newNode->gain_ << std::endl;
      kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

      // Display new node
      publishNode(newNode);

      // Update best IG and node if applicable
      if (newNode->gain_ > bestGain_) {
        bestGain_ = newNode->gain_;
        bestNode_ = newNode;
      }
      counter_++;
    }
  }
}

void RrtTree::setRootVicinity(double rootVicinity) {
  rootVicinity_ = rootVicinity;
}

void RrtTree::initialize(const bool seedHistory) {
  // This function is to initialize the tree, including insertion of remainder
  // of previous best branch.
  g_ID_ = 0;

  // Initialize kd-tree with root node and prepare log file
  kdTree_ = kd_create(3);

  if (params_.log_) {
    if (fileTree_.is_open()) {
      fileTree_.close();
    }
    fileTree_.open(
        (logFilePath_ + "tree" + std::to_string(iterationCount_) + ".txt")
            .c_str(),
        std::ios::out);
  }

  rootNode_ = new Node<StateVec>;
  rootNode_->distance_ = 0.0;
  rootNode_->gain_ = params_.zero_gain_;
  rootNode_->parent_ = nullptr;
  if (seedHistory) {
    rootNode_->state_ = hist_root_;
  } else {
    rootNode_->state_ = root_;
  }

  kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(),
             rootNode_->state_.z(), rootNode_);
  iterationCount_++;

  // Publish visualization of total exploration area
  // TODO: Publish once in the main code
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = 0;
  p.header.frame_id = params_.navigationFrame_;
  p.id = 0;
  p.ns = "workspace";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = 0.5 * (params_.minX_ + params_.maxX_);
  p.pose.position.y = 0.5 * (params_.minY_ + params_.maxY_);
  p.pose.position.z = 0.5 * (params_.minZ_ + params_.maxZ_);
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, 0.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = params_.maxX_ - params_.minX_;
  p.scale.y = params_.maxY_ - params_.minY_;
  p.scale.z = params_.maxZ_ - params_.minZ_;
  p.color.r = 200.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 0.1;
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

void RrtTree::getBestBranch(std::vector<geometry_msgs::Pose> &path,
                            std::vector<geometry_msgs::Pose> &trajectory) {
  // This function returns the best branch
  Node<StateVec> *current = bestNode_->parent_;
  std::vector<Node<StateVec> *> pathNodes;
  pathNodes.emplace_back(bestNode_);
  while (current->parent_ != nullptr) {
    if (not manager_->checkMotion(pathNodes.back()->state_,
                                  current->parent_->state_)) {
      pathNodes.emplace_back(current);
    }
    current = current->parent_;
  }
  pathNodes.emplace_back(current);
  sampleBranch(pathNodes, path, trajectory);
  exact_root_ = current->state_;
}

std::vector<geometry_msgs::Pose> RrtTree::getBestEdge(std::string targetFrame) {
  // This function returns the first edge of the best branch
  std::vector<geometry_msgs::Pose> ret;
  Node<StateVec> *current = bestNode_;
  if (current->parent_ != nullptr) {
    while (current->parent_ != rootNode_ && current->parent_ != nullptr) {
      current = current->parent_;
    }
    ret = samplePath(current->parent_->state_, current->state_, targetFrame);
    history_.push(current->parent_->state_);
    exact_root_ = current->state_;
  }
  return ret;
}

void RrtTree::gain(StateVec state, double &maxGainFound,
                      double &orientationFound) {

  double gain = 0.0;
  int checked_voxels = 0;

  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec;
  double rangeSq = pow(params_.gainRange_, 2.0);
  // Iterate over all nodes within the allowed distance
  for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
       vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_);
       vec[0] += params_.voxelSize_) {
    for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
         vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_);
         vec[1] += params_.voxelSize_) {
      for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
           vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_);
           vec[2] += params_.voxelSize_) {
        Eigen::Vector3d dir = vec - origin;
        // Skip if distance is too large
        if (dir.transpose().dot(dir) > rangeSq) {
          continue;
        }
        bool insideAFieldOfView = false;
        // Check that voxel center is inside one of the fields of view.
        for (auto &camBoundNormal : params_.camBoundNormals_) {
          bool inThisFieldOfView = true;
          for (auto &itSingleCBN : camBoundNormal) {
            Eigen::Vector3d normal =
                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                itSingleCBN;
            double val = dir.dot(normal.normalized());
            if (val < SQRT2 * params_.voxelSize_) {
              inThisFieldOfView = false;
              break;
            }
          }
          if (inThisFieldOfView) {
            insideAFieldOfView = true;
            break;
          }
        }
        if (!insideAFieldOfView) {
          continue;
        }
        ++checked_voxels;

        if (VoxbloxManager::kOccupied !=
            this->manager_->getVisibility(origin, vec, false)) {
          VoxbloxManager::VoxelStatus node = manager_->getVoxelStatus(vec);
          if (node == VoxbloxManager::kUnknown) {
            gain += params_.igUnmapped_;
          } else if (node == VoxbloxManager::kOccupied) {
            gain += params_.igOccupied_;
          } else {
            gain += params_.igFree_;
          }
        }
      }
    }
  }
  gain *= pow(params_.voxelSize_, 3.0);
  // gain /= checked_voxels;
  compareGain(state, gain, maxGainFound, orientationFound);
}

void RrtTree::compareGain(StateVec &state, double gain, double &maxGainFound,
                          double &orientationFound) {
  std::lock_guard<std::mutex> guard(myMutex);
  if (gain > maxGainFound) {
    maxGainFound = gain;
    orientationFound = state[3];
  }
}

std::vector<geometry_msgs::Pose>
RrtTree::getPathBackToPrevious(std::string targetFrame) {
  std::vector<geometry_msgs::Pose> ret;
  if (history_.empty()) {
    return ret;
  }
  ret = samplePath(root_, history_.top(), targetFrame);
  history_.pop();
  return ret;
}

void RrtTree::clear() {
  delete rootNode_;
  rootNode_ = nullptr;
  tree_msg_.markers.clear();
  visualization_msgs::Marker p;
  p.action = visualization_msgs::Marker::DELETEALL;
  tree_msg_.markers.emplace_back(p);
  params_.explorationTree_.publish(tree_msg_);
  tree_msg_.markers.clear();
  counter_ = 0;
  bestGain_ = params_.zero_gain_;
  bestNode_ = nullptr;

  kd_free(kdTree_);
}

void RrtTree::reset() {
  tree_msg_.markers.clear();
  visualization_msgs::Marker p;
  p.action = visualization_msgs::Marker::DELETEALL;
  tree_msg_.markers.emplace_back(p);
  params_.explorationTree_.publish(tree_msg_);
  tree_msg_.markers.clear();
  std::stack<StateVec>().swap(history_);
}

void RrtTree::publishNode(Node<StateVec> *node) {
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID_;
  p.header.frame_id = params_.navigationFrame_;
  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_tree";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->state_[0];
  p.pose.position.y = node->state_[1];
  p.pose.position.z = node->state_[2];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, node->state_[3]);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = std::max(node->gain_ - params_.zero_gain_, 0.05);
  p.scale.y = 0.1;
  p.scale.z = 0.1;
  p.color.r = 167.0 / 255.0;
  p.color.g = 167.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 1.0;
  // p.lifetime = ros::Duration(20.0);
  tree_msg_.markers.emplace_back(p);
  // params_.inspectionPath_.publish(p);

  if (!node->parent_) {
    params_.explorationTree_.publish(tree_msg_);
    return;
  }
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID_;
  p.header.frame_id = params_.navigationFrame_;
  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_branches";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->parent_->state_[0];
  p.pose.position.y = node->parent_->state_[1];
  p.pose.position.z = node->parent_->state_[2];
  Eigen::Quaternion<float> q;
  Eigen::Vector3f init(1.0, 0.0, 0.0);
  Eigen::Vector3f dir(node->state_[0] - node->parent_->state_[0],
                      node->state_[1] - node->parent_->state_[1],
                      node->state_[2] - node->parent_->state_[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  p.scale.x = dir.norm();
  p.scale.y = 0.03;
  p.scale.z = 0.03;
  p.color.r = 100.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.7;
  p.color.a = 1.0;
  tree_msg_.markers.emplace_back(p);
  params_.explorationTree_.publish(tree_msg_);

  if (params_.log_) {
    for (int i = 0; i < node->state_.size(); i++) {
      fileTree_ << node->state_[i] << ",";
    }
    fileTree_ << node->gain_ << ",";
    for (int i = 0; i < node->parent_->state_.size(); i++) {
      fileTree_ << node->parent_->state_[i] << ",";
    }
    fileTree_ << node->parent_->gain_ << "\n";
  }
}

std::vector<geometry_msgs::Pose>
RrtTree::samplePath(StateVec start, StateVec end,
                    const std::string &targetFrame) {
  std::vector<geometry_msgs::Pose> ret;
  static tf::TransformListener listener;
  /*tf::StampedTransform transform;
  try {
      listener.lookupTransform(targetFrame, params_.navigationFrame_,
  ros::Time(0), transform); } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      return ret;
  }*/
  Eigen::Vector3d distance(end[0] - start[0], end[1] - start[1],
                           end[2] - start[2]);
  double yaw_direction = end[3] - start[3];
  if (yaw_direction > M_PI) {
    yaw_direction -= 2.0 * M_PI;
  }
  if (yaw_direction < -M_PI) {
    yaw_direction += 2.0 * M_PI;
  }
  double disc = std::min(params_.dt_ * params_.v_max_ / distance.norm(),
                         params_.dt_ * params_.dyaw_max_ / abs(yaw_direction));
  assert(disc > 0.0);
  for (double it = 0.0; it <= 1.0; it += disc) {
    tf::Vector3 origin((1.0 - it) * start[0] + it * end[0],
                       (1.0 - it) * start[1] + it * end[1],
                       (1.0 - it) * start[2] + it * end[2]);
    double yaw = start[3] + yaw_direction * it;
    if (yaw > M_PI)
      yaw -= 2.0 * M_PI;
    if (yaw < -M_PI)
      yaw += 2.0 * M_PI;
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, yaw);
    // origin = transform * origin;
    // quat = transform * quat;
    tf::Pose poseTF(quat, origin);
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(poseTF, pose);
    ret.push_back(pose);
    if (params_.log_) {
      filePath_ << poseTF.getOrigin().x() << ",";
      filePath_ << poseTF.getOrigin().y() << ",";
      filePath_ << poseTF.getOrigin().z() << ",";
      filePath_ << tf::getYaw(poseTF.getRotation()) << "\n";
    }
  }
  return ret;
}

void RrtTree::sampleBranch(const std::vector<Node<StateVec> *> &pathNodes,
                           std::vector<geometry_msgs::Pose> &result,
                           std::vector<geometry_msgs::Pose> &trajectory) {

  if (not trajectory.empty()) {
    (*pathNodes.rbegin())->state_.w() =
        tf::getYaw(trajectory.back().orientation);
  }
  for (auto iter = pathNodes.rbegin(); (iter + 1) != pathNodes.rend(); ++iter) {
    StateVec start = (*iter)->state_;
    StateVec end = (*(iter + 1))->state_;
    history_.push(start);

    Eigen::Vector3d distance(end[0] - start[0], end[1] - start[1],
                             end[2] - start[2]);
    double yaw_direction = end[3] - start[3];
    if (yaw_direction > M_PI) {
      yaw_direction -= 2.0 * M_PI;
    }
    if (yaw_direction < -M_PI) {
      yaw_direction += 2.0 * M_PI;
    }
    double disc =
        std::min(params_.dt_ * params_.v_max_ / distance.norm(),
                 params_.dt_ * params_.dyaw_max_ / abs(yaw_direction));
    assert(disc > 0.0);
    for (double it = 0.0; it <= 1.0; it += disc) {
      tf::Vector3 origin((1.0 - it) * start[0] + it * end[0],
                         (1.0 - it) * start[1] + it * end[1],
                         (1.0 - it) * start[2] + it * end[2]);
      double yaw = start[3] + yaw_direction * it;
      if (yaw > M_PI)
        yaw -= 2.0 * M_PI;
      if (yaw < -M_PI)
        yaw += 2.0 * M_PI;
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      tf::Pose poseTF(quat, origin);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(poseTF, pose);
      result.emplace_back(pose);
    }
  }
}

#endif

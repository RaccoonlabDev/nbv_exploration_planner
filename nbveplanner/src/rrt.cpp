#include "nbveplanner/rrt.h"

#include <minkindr_conversions/kindr_tf.h>
#include <cstdlib>
#include "nbveplanner/tree.h"

namespace nbveplanner {

RrtTree::RrtTree(VoxbloxManager *manager, VoxbloxManager *manager_lowres,
                 Params *params)
    : TreeBase(manager, manager_lowres, params) {
  g_ID_ = 0;
  root_vicinity_ = 0;
  kdTree_ = kd_create(3);
  iteration_count_ = 0;

  // Set camera FOV
  params_->camera_model_.setIntrinsicsFromFoV(
      params_->camera_hfov_, params_->camera_vfov_, params_->sensor_min_range_,
      params_->gain_range_);
  // Set Boundaries of Exploration
  params_->camera_model_.setBoundingBox(params_->bbx_min_, params_->bbx_max_);

  if (params_->log_) {
    file_response_.open((params_->log_path_ + "response.txt").c_str(),
                        std::ios::out);
    file_path_.open((params_->log_path_ + "path.txt").c_str(), std::ios::out);
  }
}

void RrtTree::visualizeFrustum() {
  static tf::TransformListener listener;
  tf::StampedTransform stamped_transform;
  ros::Rate rate(1);

  while (ros::ok()) {
    try {
      listener.waitForTransform("camera", "base_link", ros::Time::now(),
                                ros::Duration(3.0));
      listener.lookupTransform("camera", "base_link", ros::Time(0),
                               stamped_transform);
    } catch (tf::TransformException &ex) {
      ROS_WARN("%s", ex.what());
    }
    Transformation T_C_B;
    tf::transformTFToKindr(stamped_transform, &T_C_B);
    params_->camera_model_.setExtrinsics(T_C_B);

    try {
      listener.waitForTransform("map", "base_link", ros::Time::now(),
                                ros::Duration(3.0));
      listener.lookupTransform("map", "base_link", ros::Time(0),
                               stamped_transform);
    } catch (tf::TransformException &ex) {
      ROS_WARN("%s", ex.what());
    }
    Transformation T_G_C;
    tf::transformTFToKindr(stamped_transform, &T_G_C);
    params_->camera_model_.setBodyPose(T_G_C);
    AlignedVector<Point> lines;
    params_->camera_model_.getBoundingLines(&lines);

    visualization_msgs::Marker p;
    p.header.stamp = ros::Time::now();
    p.header.seq = g_ID_;
    p.header.frame_id = params_->frame_id_;
    p.ns = "frustum";
    p.type = visualization_msgs::Marker::LINE_LIST;
    p.action = visualization_msgs::Marker::ADD;
    for (const auto &line : lines) {
      geometry_msgs::Point point;
      tf::pointEigenToMsg(line, point);
      p.points.emplace_back(point);
    }
    p.scale.x = 0.01;
    p.color.r = 167.0 / 255.0;
    p.color.g = 167.0 / 255.0;
    p.color.b = 0.0;
    p.color.a = 1.0;
    params_->inspection_path_.publish(p);
    rate.sleep();
  }
}

RrtTree::~RrtTree() {
  delete rootNode_;
  kd_free(kdTree_);
  if (file_response_.is_open()) {
    file_response_.close();
  }
  if (file_tree_.is_open()) {
    file_tree_.close();
  }
  if (file_path_.is_open()) {
    file_path_.close();
  }
}

void RrtTree::setStateFromPoseCovMsg(
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

  // Logging Current position of the MAV
  static double log_throttle_time = ros::Time::now().toSec();
  if (ros::Time::now().toSec() - log_throttle_time > params_->dt_) {
    log_throttle_time += params_->dt_;
    if (params_->log_) {
      for (size_t i = 0; i < root_.size() - 1; i++) {
        file_response_ << root_[i] << ";";
      }
      file_response_ << root_[root_.size() - 1] << "\n";
    }
  }
}

void RrtTree::setStateFromOdometryMsg(const nav_msgs::Odometry &pose) {
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

  // Logging Current position of the MAV
  static double log_throttle_time = ros::Time::now().toSec();
  if (ros::Time::now().toSec() - log_throttle_time > params_->dt_) {
    log_throttle_time += params_->dt_;
    if (params_->log_) {
      for (size_t i = 0; i < root_.size() - 1; i++) {
        file_response_ << root_[i] << ";";
      }
      file_response_ << root_[root_.size() - 1] << "\n";
    }
  }
}

void RrtTree::iterate() {
  // In this function a new configuration is sampled and added to the tree.
  Pose newState;

  Node *newParent;
  Eigen::Vector3d origin;
  Eigen::Vector3d direction;
  bool solutionFound = false;

  std::mt19937 generator(
      std::chrono::steady_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  double radius_sqrt = SQ(root_vicinity_);
  while (!solutionFound) {
    for (int i = 0; i < 3; i++) {
      newState[i] = 2.0 * root_vicinity_ * (distribution(generator) - 0.5);
    }
    if (SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]) > radius_sqrt)
      continue;
    // Offset new state by root
    newState += rootNode_->state_;
    if (not isInsideBounds(params_->bbx_min_, params_->bbx_max_, newState)) {
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
    newParent = (Node *)kd_res_item_data(nearest);
    kd_res_free(nearest);

    // Check for collision of new connection plus some overshoot distance.
    origin = Eigen::Vector3d(newParent->state_[0], newParent->state_[1],
                             newParent->state_[2]);
    direction =
        Eigen::Vector3d(newState[0] - origin[0], newState[1] - origin[1],
                        newState[2] - origin[2]);
    if (direction.norm() > params_->extension_range_) {
      direction = params_->extension_range_ * direction.normalized();
    }
    newState[0] = origin[0] + direction[0];
    newState[1] = origin[1] + direction[1];
    newState[2] = origin[2] + direction[2];

    if (manager_->checkMotion(
            origin,
            direction + origin +
                direction.normalized() * params_->dist_overshoot_,
            true)) {
      // Create new node and insert into tree
      double num_unmapped = gain2(newState);
      auto *newNode = new Node;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + direction.norm();
      newNode->num_unmapped_ = num_unmapped;
      double yaw_direction = newState[3] - newParent->state_[3];
      if (yaw_direction > M_PI) {
        yaw_direction -= 2.0 * M_PI;
      }
      if (yaw_direction < -M_PI) {
        yaw_direction += 2.0 * M_PI;
      }
      newNode->time_to_reach_ = std::max(yaw_direction / params_->dyaw_max_,
                                         newNode->distance_ / params_->v_max_);
      newNode->gain_ = newNode->num_unmapped_ / newNode->time_to_reach_;
      newNode->id_ = g_ID_;
      newParent->children_.emplace_back(newNode);
      kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

      // Display new node
      publishNode(newNode);

      // Update best IG and node if applicable
      if (newNode->gain_ > bestGain_) {
        bestGain_ = newNode->gain_;
        bestNode_ = newNode;
      }
      counter_++;

      if (params_->log_) {
        for (size_t i = 0; i < newNode->state_.size(); i++) {
          file_tree_ << newNode->state_[i] << ";";
        }
        file_tree_ << newNode->num_unmapped_ << ";" << newNode->time_to_reach_
                   << ";" << newNode->gain_ << ";";
        for (size_t i = 0; i < newNode->parent_->state_.size() - 1; i++) {
          file_tree_ << newNode->parent_->state_[i] << ";";
        }
        file_tree_
            << newNode->parent_->state_[newNode->parent_->state_.size() - 1]
            << "\n";
      }
    }
  }
}

void RrtTree::setRootVicinity(double rootVicinity) {
  root_vicinity_ = rootVicinity;
}

void RrtTree::initialize(bool seedHistory) {
  g_ID_ = 0;

  // Initialize kd-tree with root node
  kdTree_ = kd_create(3);

  rootNode_ = new Node;
  rootNode_->distance_ = 0.0;
  rootNode_->gain_ = params_->zero_gain_;
  rootNode_->parent_ = nullptr;
  if (seedHistory) {
    rootNode_->state_ = hist_root_;
  } else {
    rootNode_->state_ = root_;
  }

  kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(),
             rootNode_->state_.z(), rootNode_);
  iteration_count_++;

  if (params_->log_) {
    if (file_tree_.is_open()) {
      file_tree_.close();
    }
    file_tree_.open((params_->log_path_ + "tree_" +
                     std::to_string(iteration_count_) + ".txt")
                        .c_str(),
                    std::ios::out);
  }

  // Publish visualization of total exploration area
  // TODO: Publish once in the main code
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = 0;
  p.header.frame_id = params_->frame_id_;
  p.id = 0;
  p.ns = "workspace";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  Eigen::Vector3d tmp = 0.5 * (params_->bbx_min_ + params_->bbx_max_);
  p.pose.position.x = tmp.x();
  p.pose.position.y = tmp.y();
  p.pose.position.z = tmp.z();
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, 0.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  tmp = params_->bbx_max_ - params_->bbx_min_;
  p.scale.x = tmp.x();
  p.scale.y = tmp.y();
  p.scale.z = tmp.z();
  p.color.r = 200.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 0.1;
  p.frame_locked = false;
  params_->inspection_path_.publish(p);
}

void RrtTree::getBestBranch(std::vector<geometry_msgs::Pose> &path,
                            std::vector<geometry_msgs::Pose> &trajectory) {
  modifyColorNode(bestNode_->id_);
  // This function returns the best branch
  Node *current = bestNode_->parent_;
  std::vector<Node *> pathNodes;
  pathNodes.emplace_back(bestNode_);
  while (current->parent_ != nullptr) {
    if (not manager_->checkMotion(pathNodes.back()->state_,
                                  current->parent_->state_, true)) {
      pathNodes.emplace_back(current);
    }
    current = current->parent_;
  }
  pathNodes.emplace_back(current);
  sampleBranch(pathNodes, path, trajectory);
  exact_root_ = current->state_;
}

double RrtTree::gain2(Pose &state) {
  static const double voxel_size = manager_lowres_->getResolution();
  static const double voxel_size_inv = 1.0 / voxel_size;
  static const int voxels_per_side = manager_lowres_->getVoxelsPerSide();
  static const double voxels_per_side_inv = 1.0 / voxels_per_side;
  static const double cubic_voxel_size = CUBE(voxel_size);

  const Point position_mav(state[0], state[1], state[2]);
  voxblox::HierarchicalIndexSet checked_voxels_set;
  Eigen::Quaterniond orientation;
  Transformation T_G_B;
  double yaw;
  AlignedVector<Point> plane_points;
  Point u_distance, u_slope, v_center;
  int u_max;
  AlignedVector<double> vertical_gain;
  vertical_gain.reserve(360);
  for (int i = -180; i < 180; ++i) {
    yaw = M_PI * i / 180.0;
    orientation = Eigen::AngleAxisd(yaw, Point::UnitZ());
    T_G_B = Transformation(position_mav, orientation);
    params_->camera_model_.setBodyPose(T_G_B);
    const Point camera_position =
        params_->camera_model_.getCameraPose().getPosition();

    // Get the three points defining the back plane of the camera frustum.
    params_->camera_model_.getFarPlanePoints(&plane_points);

    // We map the plane into u and v coordinates, which are the plane's
    // coordinate system, with the origin at plane_points[1] and outer bounds at
    // plane_points[0] and plane_points[2].
    u_distance = plane_points[0] - plane_points[1];
    u_slope = u_distance.normalized();
    u_max = static_cast<int>(std::ceil(u_distance.norm() * voxel_size_inv));
    v_center = (plane_points[2] - plane_points[1]) / 2.0;

    Point pos;
    double gain = 0.0;
    voxblox::GlobalIndex global_voxel_idx;
    voxblox::BlockIndex block_index;
    voxblox::VoxelIndex voxel_index;
    // We then iterate over all the voxels in the coordinate space of the
    // horizontal center back bounding plane of the frustum.
    for (int u = 0; u < u_max; ++u) {
      // Get the 'real' coordinates back from the plane coordinate space.
      pos = plane_points[1] + u * u_slope * voxel_size + v_center;

      global_voxel_idx =
          (voxel_size_inv * pos).cast<voxblox::LongIndexElement>();
      block_index = voxblox::getBlockIndexFromGlobalVoxelIndex(
          global_voxel_idx, voxels_per_side_inv);
      voxel_index = voxblox::getLocalFromGlobalVoxelIndex(global_voxel_idx,
                                                          voxels_per_side);
      if (checked_voxels_set[block_index].count(voxel_index) > 0) {
        continue;
      }

      const voxblox::Point start_scaled =
          (camera_position * voxel_size_inv).cast<voxblox::FloatingPoint>();
      const voxblox::Point end_scaled =
          (pos * voxel_size_inv).cast<voxblox::FloatingPoint>();

      voxblox::LongIndexVector global_voxel_indices;
      voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

      voxblox::BlockIndex block_index_ray;
      voxblox::VoxelIndex voxel_index_ray;

      for (const voxblox::GlobalIndex &global_index_ray :
           global_voxel_indices) {
        block_index_ray = voxblox::getBlockIndexFromGlobalVoxelIndex(
            global_index_ray, voxels_per_side_inv);
        voxel_index_ray = voxblox::getLocalFromGlobalVoxelIndex(
            global_index_ray, voxels_per_side);

        bool voxel_checked = false;
        if (checked_voxels_set[block_index_ray].count(voxel_index_ray) > 0) {
          voxel_checked = true;
        }

        Point recovered_pos = global_index_ray.cast<double>() * voxel_size;
        if (not voxel_checked and
            params_->camera_model_.isPointInView(recovered_pos)) {
          VoxbloxManager::VoxelStatus status =
              manager_lowres_->getVoxelStatus(block_index_ray, voxel_index_ray);
          if (status == VoxbloxManager::kUnknown) {
            gain += params_->gain_unmapped_;
          } else if (status == VoxbloxManager::kOccupied) {
            gain += params_->gain_occupied_;
            break;
          } else {
            gain += params_->gain_free_;
          }
        }
      }
    }
    vertical_gain[i + 180] = gain;
  }
  double max_gain = 0.0;
  int max_gain_yaw = 0;
  double current_gain;
  static const int half_hfov = std::floor(params_->camera_hfov_ / 2.0);
  for (int i = -180; i < 180; i += 5) {
    current_gain = 0.0;
    int left_idx = (i + 180) - half_hfov;
    if (left_idx < 0) {
      for (int j = 360 + left_idx; j < 360; ++j) {
        current_gain += vertical_gain[j];
      }
      left_idx = 0;
    }
    int right_idx = (i + 180) + half_hfov;
    if (right_idx >= 360) {
      for (int j = 0; j < right_idx - 359; ++j) {
        current_gain += vertical_gain[j];
      }
      right_idx = 359;
    }
    for (int j = left_idx; j <= right_idx; ++j) {
      current_gain += vertical_gain[j];
    }
    if (current_gain > max_gain) {
      max_gain = current_gain;
      max_gain_yaw = i;
    }
  }
  state[3] = max_gain_yaw * M_PI / 180.0;
  return max_gain * cubic_voxel_size;
}

void RrtTree::gain(Pose state, double &maxGainFound, double &orientationFound) {
  double gain = 0.0;
  int checked_voxels = 0;

  Point origin(state[0], state[1], state[2]);
  Eigen::Quaterniond quaternion;
  quaternion = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ());
  Transformation T_G_B(origin, quaternion);
  params_->camera_model_.setBodyPose(T_G_B);

  // Get the boundaries of the current view.
  Point aabb_min, aabb_max;
  params_->camera_model_.getAabb(&aabb_min, &aabb_max);

  // Get the center of the camera to raycast to.
  // Transformation camera_pose = params_->camera_model_.getCameraPose();
  // Point camera_center = camera_pose.getPosition();

  Point point;
  // double rangeSq = pow(params_->gainRange_, 2.0);
  double res = manager_lowres_->getResolution();
  // Iterate over all nodes within the allowed distance
  for (point.x() = aabb_min.x(); point.x() < aabb_max.x(); point.x() += res) {
    for (point.y() = aabb_min.y(); point.y() < aabb_max.y(); point.y() += res) {
      for (point.z() = aabb_min.z(); point.z() < aabb_max.z();
           point.z() += res) {
        // TODO: Check if frustum is inside bounding box
        if (params_->camera_model_.isPointInView(point)) {
          if (VoxbloxManager::kOccupied !=
              this->manager_lowres_->getVisibility(origin, point, false)) {
            VoxbloxManager::VoxelStatus node =
                manager_lowres_->getVoxelStatus(point);
            if (node == VoxbloxManager::kUnknown) {
              gain += params_->gain_unmapped_;
            } else if (node == VoxbloxManager::kOccupied) {
              gain += params_->gain_occupied_;
            } else {
              gain += params_->gain_free_;
            }
          }
        }
      }
    }
  }
  gain *= pow(res, 3.0);
  // gain /= checked_voxels;
  compareGain(state, gain, maxGainFound, orientationFound);
}

void RrtTree::compareGain(Pose &state, double gain, double &maxGainFound,
                          double &orientationFound) {
  std::lock_guard<std::mutex> guard(myMutex);
  if (gain > maxGainFound) {
    maxGainFound = gain;
    orientationFound = state[3];
  }
}

std::vector<geometry_msgs::Pose> RrtTree::getPathBackToPrevious(
    std::string targetFrame) {
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
  params_->exploration_tree_.publish(tree_msg_);
  tree_msg_.markers.clear();
  counter_ = 0;
  bestGain_ = params_->zero_gain_;
  bestNode_ = nullptr;

  kd_free(kdTree_);
}

void RrtTree::reset() {
  tree_msg_.markers.clear();
  visualization_msgs::Marker p;
  p.action = visualization_msgs::Marker::DELETEALL;
  tree_msg_.markers.emplace_back(p);
  params_->exploration_tree_.publish(tree_msg_);
  tree_msg_.markers.clear();
  std::stack<Pose>().swap(history_);
}

void RrtTree::publishNode(Node *node) {
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID_;
  p.header.frame_id = params_->frame_id_;
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
  p.scale.x = std::min(node->gain_, 0.5);
  p.scale.y = 0.1;
  p.scale.z = 0.1;
  p.color.r = 167.0 / 255.0;
  p.color.g = 167.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 1.0;
  // p.lifetime = ros::Duration(20.0);
  tree_msg_.markers.emplace_back(p);
  // params_->inspectionPath_.publish(p);

  if (!node->parent_) {
    params_->exploration_tree_.publish(tree_msg_);
    return;
  }
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID_;
  p.header.frame_id = params_->frame_id_;
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
  params_->exploration_tree_.publish(tree_msg_);
}

void RrtTree::modifyColorNode(const int id) {
  tree_msg_.markers[id].color.r = 0.0;
  tree_msg_.markers[id].color.g = 1.0;
  tree_msg_.markers[id].color.b = 0.0;
  params_->exploration_tree_.publish(tree_msg_);
}

std::vector<geometry_msgs::Pose> RrtTree::samplePath(
    Pose start, Pose end, const std::string &targetFrame) {
  std::vector<geometry_msgs::Pose> ret;
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
      std::min(params_->dt_ * params_->v_max_ / distance.norm(),
               params_->dt_ * params_->dyaw_max_ / abs(yaw_direction));
  assert(disc > 0.0);
  for (double it = 0.0; it <= 1.0; it += disc) {
    tf::Vector3 origin((1.0 - it) * start[0] + it * end[0],
                       (1.0 - it) * start[1] + it * end[1],
                       (1.0 - it) * start[2] + it * end[2]);
    double yaw = start[3] + yaw_direction * it;
    if (yaw > M_PI) yaw -= 2.0 * M_PI;
    if (yaw < -M_PI) yaw += 2.0 * M_PI;
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, yaw);
    tf::Pose poseTF(quat, origin);
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(poseTF, pose);
    ret.push_back(pose);

    // Logging the best path selected
    if (params_->log_) {
      for (size_t i = 0; i < 3; ++i) {
        file_path_ << origin[i] << ";";
      }
      file_path_ << yaw << "\n";
    }
  }
  return ret;
}

void RrtTree::sampleBranch(const std::vector<Node *> &pathNodes,
                           std::vector<geometry_msgs::Pose> &result,
                           std::vector<geometry_msgs::Pose> &trajectory) {
  if (not trajectory.empty()) {
    (*pathNodes.rbegin())->state_.w() =
        tf::getYaw(trajectory.back().orientation);
  }
  for (auto iter = pathNodes.rbegin(); (iter + 1) != pathNodes.rend(); ++iter) {
    Pose start = (*iter)->state_;
    Pose end = (*(iter + 1))->state_;

    history_.push(start);

    Eigen::Vector3d distance(end[0] - start[0], end[1] - start[1],
                             end[2] - start[2]);
    double yaw_direction = end[3] - start[3];
    if (yaw_direction > M_PI) {
      yaw_direction -= 2.0 * M_PI;
    } else if (yaw_direction < -M_PI) {
      yaw_direction += 2.0 * M_PI;
    }
    double disc =
        std::min(params_->dt_ * params_->v_max_ / distance.norm(),
                 params_->dt_ * params_->dyaw_max_ / abs(yaw_direction));
    assert(disc > 0.0);
    for (double it = 0.0; it <= 1.0; it += disc) {
      Point origin((1.0 - it) * start[0] + it * end[0],
                   (1.0 - it) * start[1] + it * end[1],
                   (1.0 - it) * start[2] + it * end[2]);
      double yaw = start[3] + yaw_direction * it;
      if (yaw > M_PI)
        yaw -= 2.0 * M_PI;
      else if (yaw < -M_PI)
        yaw += 2.0 * M_PI;
      Eigen::Affine3d aff;
      aff.translation() = origin;
      Eigen::Quaterniond quat(Eigen::AngleAxisd(yaw, Point::UnitZ()));
      aff.linear() = quat.toRotationMatrix();
      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(aff, pose);
      result.emplace_back(pose);

      // Logging the best path selected
      if (params_->log_) {
        for (size_t i = 0; i < 3; ++i) {
          file_path_ << origin[i] << ";";
        }
        file_path_ << yaw << "\n";
      }
    }
  }
}

}  // namespace nbveplanner

#include "nbveplanner/history.h"

namespace nbveplanner {

History::History(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                 HighResManager *manager, LowResManager *manager_lowres,
                 Params *params)
    : nh_(nh),
      nh_private_(nh_private),
      manager_(CHECK_NOTNULL(manager)),
      manager_lowres_(CHECK_NOTNULL(manager_lowres)),
      params_(CHECK_NOTNULL(params)) {
  graph_nodes_pub_ =
      nh_.advertise<visualization_msgs::Marker>("historyGraph/nodes", 100);
  graph_edges_pub_ =
      nh_.advertise<visualization_msgs::Marker>("historyGraph/edges", 100);
  trajectory_pub_ =
      nh_.advertise<visualization_msgs::Marker>("historyGraph/trajectory", 1);
  gradient_pub_ =
      nh_.advertise<visualization_msgs::Marker>("historyGraph/gradient", 1);

  poseClient_ = nh_.subscribe("pose_cov", 1, &History::poseCallback, this);
  odomClient_ = nh_.subscribe("odometry", 1, &History::odomCallback, this);

  setUpPublisherMsg();
  drone_exploring_ = true;
  iteration_ = 0;
  point_id_ = 0;
  edge_id_ = 0;
  home_pos_ = {0.0, 0.0, 0.0};
  kdTree_ = kd_create(3);
  initialized_ = false;
  stopped_ = false;
}

History::~History() = default;

void History::setUpPublisherMsg() {
  gain_color_.r = 0.0;
  gain_color_.g = 1.0;
  gain_color_.b = 0.0;
  gain_color_.a = 0.5;

  dead_color_.r = 200.0 / 255.0;
  dead_color_.g = 100.0 / 255.0;
  dead_color_.b = 0.0;
  dead_color_.a = 0.5;

  point_msg.header.frame_id = line.header.frame_id =
      trajectory.header.frame_id = gradient.header.frame_id =
          params_->frame_id_;
  point_msg.ns = "point";
  point_msg.type = visualization_msgs::Marker::SPHERE;
  point_msg.action = line.action = trajectory.action = gradient.action =
      visualization_msgs::Marker::ADD;
  point_msg.pose.orientation.w = trajectory.pose.orientation.w =
      line.pose.orientation.w = gradient.pose.orientation.w = 1.0;
  point_msg.scale.x = point_msg.scale.y = point_msg.scale.z = 0.2;

  line.id = 0;
  line.ns = "line";
  line.type = trajectory.type = visualization_msgs::Marker::LINE_LIST;
  line.points.resize(2);
  line.scale.x = 0.02;
  line.color.r = 1.0;
  line.color.g = 0.0;
  line.color.b = 0.0;
  line.color.a = 0.5;

  trajectory.id = 0;
  trajectory.ns = "trajectory";
  trajectory.scale.x = 0.05;
  trajectory.color.r = 1.0;
  trajectory.color.g = 165.0 / 255.0;
  trajectory.color.b = 0.0;
  trajectory.color.a = 1.0;

  gradient.ns = "gradient";
  gradient.scale.x = 0.02;
  gradient.scale.y = 0.05;
  gradient.color.r = 0.0;
  gradient.color.g = 0.0;
  gradient.color.b = 1.0;
  gradient.color.a = 1.0;
}

void History::poseCallback(
    const geometry_msgs::PoseWithCovarianceStamped &pose) {
  current_pose_ = pose.pose.pose;
}

void History::odomCallback(const nav_msgs::Odometry &pose) {
  current_pose_ = pose.pose.pose;
}

std::string convertToString(const Eigen::Vector3d &vec) {
  std::ostringstream s;
  s << std::to_string(vec.x()) << "/" << std::to_string(vec.y()) << "/"
    << std::to_string(vec.z());
  return s.str();
}

double History::bfs(const Eigen::Vector3d &point) {
  std::unordered_set<std::string> visited;
  visited.insert(convertToString(point));

  std::queue<Point> queue;
  queue.emplace(point);
  Point new_pos;
  const double voxel_size = manager_lowres_->getResolution();
  static const std::vector<Point> connNeigh{
      Point{voxel_size, 0.0, 0.0}, Point{-voxel_size, 0.0, 0.0},
      Point{0.0, voxel_size, 0.0}, Point{0.0, -voxel_size, 0.0},
      Point{0.0, 0.0, voxel_size}, Point{0.0, 0.0, -voxel_size}};
  int frontierVoxels = 0;
  double distance;
  VoxbloxManager::VoxelStatus status;
  while (not queue.empty()) {
    new_pos = queue.front();
    queue.pop();
    for (const auto &c : connNeigh) {
      Point candidate{new_pos + c};
      if (visited.find(convertToString(candidate)) == visited.end() and
          (candidate - point).norm() <= 6.0 and
          isInsideBounds(params_->bbx_min_, params_->bbx_max_, candidate)) {
        status = manager_lowres_->getVoxelStatus(candidate);
        if (status == VoxbloxManager::VoxelStatus::kFree /*and
            manager_lowres_->getDistanceAtPosition(candidate, &distance) and
            distance > params_->robot_radius_*/) {
          visited.insert(convertToString(candidate));
          queue.emplace(candidate);
        } else if (status == VoxbloxManager::VoxelStatus::kUnknown) {
          ++frontierVoxels;
        }
      }
    }
  }
  return frontierVoxels * pow(voxel_size, 3.0);
}

void History::clear() {
  visualization_msgs::Marker p;
  p.action = visualization_msgs::Marker::DELETEALL;
  trajectory_pub_.publish(p);
}

void History::reset() {
  if (initialized_) {
    initialized_ = false;
    while (not stopped_) {
      ros::Duration(0.5).sleep();
    }
    graph.clear();
    activeNodes.clear();
    iteration_ = 0;
    point_id_ = 0;
    edge_id_ = 0;
    kd_free(kdTree_);
    kdTree_ = kd_create(3);

    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::DELETEALL;
    graph_nodes_pub_.publish(m);
    graph_edges_pub_.publish(m);
    trajectory_pub_.publish(m);
    gradient_pub_.publish(m);
  }
}

void History::historyMaintenance() {
  while (not initialized_) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("Start History Graph Maintenance");
  stopped_ = false;
  Point prevPos = graph.back().pos;
  Point currPosEigen;
  int iter = 0;
  while (ros::ok()) {
    if (not initialized_) {
      stopped_ = true;
      return;
    }
    tf::pointMsgToEigen(current_pose_.position, currPosEigen);
    if (drone_exploring_ and (prevPos - currPosEigen).norm() >= 0.5) {
      addVertexAndConnect(current_pose_.position, currPosEigen, -1);
      prevPos = currPosEigen;
    }
    kdres *nearest_set = kd_nearest_range3(
        kdTree_, currPosEigen.x(), currPosEigen.y(), currPosEigen.z(), 6.0);
    int n = kd_res_size(nearest_set);
    if (n <= 0) {
      ROS_WARN("No node nearby the area");
    }
    Vertex *tmp;
    for (size_t i = 0; i < n; ++i) {
      if (not initialized_) {
        stopped_ = true;
        return;
      }
      tmp = (Vertex *)kd_res_item_data(nearest_set);
      if (graph.size() > 1 and not refineVertexPosition(tmp) and
          (tmp->potential_gain > params_->zero_frontier_voxels_ or
           tmp->potential_gain == -1)) {
        recalculatePotential(tmp);
      }
      kd_res_next(nearest_set);
    }
    kd_res_free(nearest_set);
    ++iteration_;
    ros::spinOnce();
  }
}

void History::recalculatePotential(Vertex *v) {
  v->potential_gain = bfs(v->pos);
  if (v->potential_gain <= params_->zero_frontier_voxels_) {
    point_msg.header.stamp = ros::Time();
    point_msg.action = visualization_msgs::Marker::ADD;
    point_msg.id = v->id;
    point_msg.color = dead_color_;
    point_msg.pose.position = getPointFromEigen(v->pos);
    graph_nodes_pub_.publish(point_msg);
    activeNodes.erase(v);
  }
}

bool History::refineVertexPosition(Vertex *v) {
  if (v->id != 0) {
    double distance;
    Eigen::Vector3d grad;
    Eigen::Vector3d refinedPos = v->pos;
    bool refined = false;
    ros::Time timeout = ros::Time::now() + ros::Duration(1.0);
    static const Point robot_radius_vec{
        params_->robot_radius_, params_->robot_radius_, params_->robot_radius_};
    static const Point lower_bound = params_->bbx_min_ + robot_radius_vec;
    static const Point upper_bound = params_->bbx_max_ - robot_radius_vec;
    while (manager_->getDistanceAndGradientAtPosition(refinedPos, &distance,
                                                      &grad) and
           grad.norm() > 0.001 and ros::Time::now() < timeout) {
      refinedPos += 0.2 * grad;
      if (not isInsideBounds(lower_bound, upper_bound, refinedPos)) {
        refinedPos -= 0.2 * grad;
        break;
      } else {
        refined = true;
      }
    }
    if (refined) {
      bool connected = true;
      for (const auto &n : v->adj) {
        if (not manager_->checkMotion(refinedPos, n.first->pos, true)) {
          connected = false;
          break;
        }
      }
      if (connected) {
        // Delete the old position in the kd-tree
        kd_delete3(kdTree_, v->pos.x(), v->pos.y(), v->pos.z());
        kdres *nearest = kd_nearest3(kdTree_, refinedPos.x(), refinedPos.y(),
                                     refinedPos.z());
        auto *tmp = (Vertex *)kd_res_item_data(nearest);
        kd_res_free(nearest);
        if ((tmp->pos - refinedPos).norm() > 0.5) {
          kd_insert3(kdTree_, refinedPos.x(), refinedPos.y(), refinedPos.z(),
                     v);
          v->pos = refinedPos;

          geometry_msgs::Point p = getPointFromEigen(refinedPos);

          point_msg.header.stamp = ros::Time();
          point_msg.action = visualization_msgs::Marker::ADD;
          point_msg.id = v->id;
          if (v->potential_gain > params_->zero_frontier_voxels_ or
              v->potential_gain == -1) {
            point_msg.color = gain_color_;
          } else {
            point_msg.color = dead_color_;
          }
          point_msg.pose.position = p;
          graph_nodes_pub_.publish(point_msg);
          for (auto &n : v->adj) {
            line.header.stamp = ros::Time::now();
            line.action = visualization_msgs::Marker::ADD;
            line.id = n.second;
            line.points[0] = p;
            line.points[1] = getPointFromEigen(n.first->pos);
            graph_edges_pub_.publish(line);
            ++edge_id_;
          }
        } else {
          collapseVertices(tmp, v);
          return true;
        }
      }
    }
  }
  return false;
}

void History::collapseVertices(Vertex *v1, Vertex *v2) {
  std::unordered_set<unsigned int> visited;
  for (const auto &adj : v1->adj) {
    if (adj.first->pos != v2->pos) {
      visited.insert(v2->id);
    }
  }
  geometry_msgs::Point p = getPointFromEigen(v1->pos);
  for (const auto &adj : v2->adj) {
    if (visited.find(adj.first->id) == visited.end() and
        adj.first->pos != v1->pos) {
      v1->adj.insert(adj);
      if (not adj.first->adj.erase(std::make_pair(v2, adj.second))) {
        ROS_WARN("Vertex not found 1");
      }
      adj.first->adj.insert(std::make_pair(v1, adj.second));
      line.action = visualization_msgs::Marker::ADD;
    } else {
      if (not adj.first->adj.erase(std::make_pair(v2, adj.second))) {
        ROS_WARN("Vertex not found 2");
      }
      line.action = visualization_msgs::Marker::DELETE;
    }
    line.header.stamp = ros::Time::now();
    line.id = adj.second;
    line.points[0] = p;
    line.points[1] = getPointFromEigen(adj.first->pos);
    graph_edges_pub_.publish(line);
  }
  activeNodes.erase(v2);
  point_msg.id = v2->id;
  point_msg.action = visualization_msgs::Marker::DELETE;
  graph_nodes_pub_.publish(point_msg);
  /*for (auto it = graph.begin(); it != graph.end(); ++it) {
    if (it->id == v2->id) {
      graph.erase(it);
      break;
    }
  }*/
}

bool compareFunction(std::pair<Vertex *, unsigned int> p1,
                     std::pair<Vertex *, unsigned int> p2) {
  return p1.second < p2.second;
}

bool History::getNearestActiveNode(
    std::vector<std::pair<Vertex *, unsigned int>> &res) {
  auto prev_iter = iteration_;
  while ((prev_iter + 1) > iteration_) {
    ros::Duration(0.1).sleep();
  }
  if (activeNodes.empty()) {
    return false;
  }
  /*auto minDistance = DBL_MAX;
  Eigen::Vector3d currentPosEigen;
  tf::pointMsgToEigen(current_pose_.position, currentPosEigen);
  double dist;
  for (const auto &n : activeNodes) {
    dist = (n->pos - currentPosEigen).norm();
    if (dist > minDistance) {
      minDistance = dist;
      res = *n;
    }
  }*/
  res.resize(activeNodes.size());
  Eigen::Vector3d currentPosEigen;
  tf::pointMsgToEigen(current_pose_.position, currentPosEigen);
  int i = 0;
  for (const auto &n : activeNodes) {
    res[i] = std::make_pair(n, (n->pos - currentPosEigen).norm());
    ++i;
  }
  std::sort(res.begin(), res.end(), compareFunction);
  return true;
}

std::vector<geometry_msgs::Pose> History::getPathToNode(Eigen::Vector3d &goal) {
  std::vector<geometry_msgs::Pose> result;
  // Get the closest node to the current position
  kdres *nearest =
      kd_nearest3(kdTree_, current_pose_.position.x, current_pose_.position.y,
                  current_pose_.position.z);
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    ROS_WARN("No node nearby the area");
    return result;
  }
  auto *nearest_vertex = (Vertex *)kd_res_item_data(nearest);
  kd_res_free(nearest);
  AStarNode initial{
      .v = nearest_vertex, .parent = nullptr, .g = 0.0, .h = 0.0, .f = 0.0};
  if (initial.v->pos != goal) {
    std::priority_queue<AStarNode> priorityQueue;
    std::deque<AStarNode> closedNodes;
    std::unordered_set<unsigned int> visited;

    priorityQueue.push(initial);
    AStarNode *node;
    while (not priorityQueue.empty()) {
      closedNodes.emplace_back(priorityQueue.top());
      priorityQueue.pop();
      node = &closedNodes.back();
      visited.insert(node->v->id);
      for (const auto &successor : node->v->adj) {
        if (successor.first->pos == goal) {
          std::vector<Vertex *> res;
          res.emplace_back(successor.first);
          AStarNode *current = node;
          while (current->parent != nullptr) {
            /*if (not manager_->checkMotion(res.back()->pos,
                                          current->parent->v->pos)) {
              res.emplace_back(current->v);
            }*/
            res.emplace_back(current->v);
            current = current->parent;
          }
          res.emplace_back(current->v);
          Eigen::Vector3d tmp;
          tf::pointMsgToEigen(current_pose_.position, tmp);
          Vertex exact_pos{.pos = tmp};
          res.emplace_back(&exact_pos);
          sampleBranch(res, result);
          return result;
        } else if (visited.find(successor.first->id) == visited.end()) {
          double g = node->g + (successor.first->pos - node->v->pos).norm();
          double h = (goal - successor.first->pos).norm();
          priorityQueue.push(AStarNode{.v = successor.first,
                                       .parent = node,
                                       .g = g,
                                       .h = h,
                                       .f = g + h});
        }
      }
    }
  } else {
    std::vector<Vertex *> res;
    res.emplace_back(initial.v);
    Eigen::Vector3d tmp;
    tf::pointMsgToEigen(current_pose_.position, tmp);
    Vertex exact_pos{.pos = tmp};
    res.emplace_back(&exact_pos);
    sampleBranch(res, result);
    return result;
  }
}

void History::sampleBranch(const std::vector<Vertex *> &pathNodes,
                           std::vector<geometry_msgs::Pose> &result) {
  Eigen::Vector4d start;
  Eigen::Vector4d end;
  Eigen::Vector3d dir;
  Eigen::Vector3d distance;
  double previous_yaw = tf::getYaw(current_pose_.orientation);
  trajectory.points.clear();

  // for (auto iter = pathNodes.rbegin() + 1; iter != pathNodes.rend(); ++iter)
  // {
  auto iter_start = pathNodes.rbegin();
  auto iter_end = pathNodes.rend() - 1;
  while (iter_start != iter_end) {
    while (
        iter_end != (iter_start + 1) and
        not manager_->checkMotion((*iter_start)->pos, (*iter_end)->pos, true)) {
      iter_end -= 1;
    }
    start = {(*iter_start)->pos.x(), (*iter_start)->pos.y(),
             (*iter_start)->pos.z(), previous_yaw};
    end = {(*iter_end)->pos.x(), (*iter_end)->pos.y(), (*iter_end)->pos.z(),
           previous_yaw};

    trajectory.points.emplace_back(getPointFromEigen((*iter_start)->pos));
    trajectory.points.emplace_back(getPointFromEigen((*iter_end)->pos));

    double yaw_direction = end[3] - start[3];
    if (yaw_direction > M_PI) {
      yaw_direction -= 2.0 * M_PI;
    }
    if (yaw_direction < -M_PI) {
      yaw_direction += 2.0 * M_PI;
    }
    distance = {end[0] - start[0], end[1] - start[1], end[2] - start[2]};
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
      result.emplace_back(pose);
    }
    iter_start = iter_end;
    iter_end = pathNodes.rend() - 1;
  }
  if (trajectory_pub_.getNumSubscribers() > 0) {
    trajectory_pub_.publish(trajectory);
  }
}

void History::addVertex(const geometry_msgs::Point &point) {
  Eigen::Vector3d pos;
  tf::pointMsgToEigen(point, pos);
  home_pos_ = pos;
  graph.emplace_back(Vertex{.pos = pos, .potential_gain = -1, .id = point_id_});
  Vertex *v = &(*graph.rbegin());

  point_msg.header.stamp = ros::Time::now();
  point_msg.id = point_id_;
  point_msg.action = visualization_msgs::Marker::ADD;
  point_msg.pose.position = point;
  point_msg.color = gain_color_;
  graph_nodes_pub_.publish(point_msg);
  ++point_id_;
  activeNodes.insert(v);
  kd_insert3(kdTree_, pos.x(), pos.y(), pos.z(), v);

  initialized_ = true;
}

void History::addVertexAndConnect(const geometry_msgs::Point &point,
                                  const Eigen::Vector3d &state,
                                  double potential_gain) {
  // First we should check if there is a node nearby
  kdres *nearest_set =
      kd_nearest_range3(kdTree_, state.x(), state.y(), state.z(), 3.0);
  int n = kd_res_size(nearest_set);
  if (n <= 0) {
    kd_res_free(nearest_set);
    ROS_WARN("No node nearby the area");
    return;
  }

  graph.emplace_back(
      Vertex{.pos = state, .potential_gain = potential_gain, .id = point_id_});

  Vertex *newVertex = &(*graph.rbegin());

  // Check if we can connect the new node to the history graph
  unsigned int copy_edge_id = edge_id_;
  Vertex *v;
  for (int i = 0; i < n; ++i) {
    v = (Vertex *)kd_res_item_data(nearest_set);
    if (manager_->checkMotion(state, v->pos, true)) {
      newVertex->adj.insert(std::make_pair(v, edge_id_));
      v->adj.insert(std::make_pair(newVertex, edge_id_));
      line.header.stamp = ros::Time::now();
      line.action = visualization_msgs::Marker::ADD;
      line.id = edge_id_;
      line.points[0] = point;
      line.points[1] = getPointFromEigen(v->pos);
      graph_edges_pub_.publish(line);
      ++edge_id_;
    }
    kd_res_next(nearest_set);
  }
  kd_res_free(nearest_set);

  if (copy_edge_id == edge_id_) {
    kdres *nearest = kd_nearest3(kdTree_, state.x(), state.y(), state.z());
    auto *nearest_vertex = (Vertex *)kd_res_item_data(nearest);
    kd_res_free(nearest);
    if (manager_->checkMotion(state, v->pos, false)) {
      // Force connection with the closest node if no other node succeeded
      newVertex->adj.insert(std::make_pair(nearest_vertex, edge_id_));
      nearest_vertex->adj.insert(std::make_pair(newVertex, edge_id_));
      line.header.stamp = ros::Time::now();
      line.action = visualization_msgs::Marker::ADD;
      line.id = edge_id_;
      line.points[0] = point;
      line.points[1] = getPointFromEigen(nearest_vertex->pos);
      graph_edges_pub_.publish(line);
      ++edge_id_;
    } else {
      // Delete node from the graph because no collision-free connection was
      // possible
      graph.pop_back();
    }
  } else {
    point_msg.header.stamp = ros::Time::now();
    point_msg.action = visualization_msgs::Marker::ADD;
    point_msg.id = point_id_;
    point_msg.pose.position = point;
    point_msg.color = gain_color_;
    graph_nodes_pub_.publish(point_msg);
    ++point_id_;
    activeNodes.insert(newVertex);
    kd_insert3(kdTree_, state.x(), state.y(), state.z(), newVertex);
  }
}

geometry_msgs::Point History::getPointFromEigen(const Eigen::Vector3d &vec) {
  geometry_msgs::Point point;
  tf::pointEigenToMsg(vec, point);
  return point;
}

void History::publishVertex() {
  if (graph_edges_pub_.getNumSubscribers() > 0) {
    graph_edges_pub_.publish(line);
  }
}

}  // namespace nbveplanner
#ifndef NBVEPLANNER_HISTORY_H
#define NBVEPLANNER_HISTORY_H

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kdtree/kdtree.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <boost/functional/hash.hpp>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <list>
#include <queue>
#include <string>
#include <unordered_set>

#include <nbveplanner/params.h>
#include <nbveplanner/voxblox_manager.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <utility>
#include <vector>

namespace nbveplanner {

struct Vertex {
  Eigen::Vector3d pos;
  double potential_gain;
  unsigned int id;
  std::unordered_set<std::pair<Vertex *, unsigned int>,
                     boost::hash<std::pair<Vertex *, unsigned int>>>
      adj;
};

struct AStarNode {
  Vertex *v;
  AStarNode *parent;
  double g, h, f;

  bool operator<(const AStarNode &node) const { return f < node.f; }
};

class History {
 public:
  History(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
          std::shared_ptr<VoxbloxManager> manager,
          std::shared_ptr<VoxbloxManager> manager_lowres,
          std::shared_ptr<Params> params);

  ~History();

  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose);

  void odomCallback(const nav_msgs::Odometry &pose);

  void historyMaintenance();

  void collapseVertices(Vertex *v1, Vertex *v2);

  bool refineVertexPosition(Vertex *v);

  void recalculatePotential(Vertex *v);

  void addVertex(const geometry_msgs::Point &point);

  void addVertexAndConnect(const geometry_msgs::Point &point,
                           const Eigen::Vector3d &state, double potential_gain);

  void setDroneExploring(bool drone_exploring) {
    drone_exploring_ = drone_exploring;
  }

  void clear();

  void reset();

  bool getDroneExploring() const { return drone_exploring_; }

  void publishVertex();

  bool getNearestActiveNode(
      std::vector<std::pair<Vertex *, unsigned int>> &res);

  std::vector<geometry_msgs::Pose> getPathToNode(Eigen::Vector3d &goal);

  void sampleBranch(const std::vector<Vertex *> &pathNodes,
                    std::vector<geometry_msgs::Pose> &result);

  Eigen::Vector3d home_pos_;

 protected:
  void setUpPublisherMsg();

  double bfs(const Eigen::Vector3d &point);

  static geometry_msgs::Point getPointFromEigen(const Eigen::Vector3d &vec);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::shared_ptr<VoxbloxManager> manager_;
  std::shared_ptr<VoxbloxManager> manager_lowres_;
  std::shared_ptr<Params> params_;
  kdtree *kdTree_;

  std::deque<Vertex> graph;
  visualization_msgs::Marker point_msg, line, trajectory, gradient;
  std_msgs::ColorRGBA gain_color_, dead_color_;
  geometry_msgs::Pose current_pose_;
  ros::Subscriber odomClient_;
  ros::Subscriber poseClient_;
  ros::Publisher graph_nodes_pub_;
  ros::Publisher graph_edges_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher gradient_pub_;
  std::unordered_set<Vertex *> activeNodes;

  bool drone_exploring_;
  bool initialized_;
  unsigned int iteration_;
  double zero_frontier_voxels_;
  double vicinity_range_;
  unsigned int point_id_;
  unsigned int edge_id_;
  bool stopped_;
};
}  // namespace nbveplanner

#endif  // NBVEPLANNER_HISTORY_H

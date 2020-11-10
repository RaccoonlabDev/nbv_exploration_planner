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
  Point pos;
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

/**
 * Class to create a History Graph with vertices and nodes, added incrementally,
 * and maintained during the exploration process
 */
class History {
 public:
  History(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
          VoxbloxManager *manager, VoxbloxManager *manager_lowres,
          Params *params);

  ~History();

  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose);

  void odomCallback(const nav_msgs::Odometry &pose);

  /**
   * Keeps the History Graph updated while the exploration process is running
   */
  void historyMaintenance();

  /**
   * Collapses v2 and its edges into v1. The edges are merged if they form a
   * collision-free path from v1 to the goal node.
   * @param v1 Pointer to a vertex in the history graph
   * @param v2 Pointer to a vertex in the history graph
   */
  void collapseVertices(Vertex *v1, Vertex *v2);

  /**
   * Pushes away from obstacles the position of v using the direction of
   * the ESDF gradient
   * @param v Pointer to a vertex in the history graph
   * @return True if v position was refined, False otherwise
   */
  bool refineVertexPosition(Vertex *v);

  /**
   * Recalculates the potential gain of the vertex v
   * @param v Pointer to a vertex in the history graph
   */
  void recalculatePotential(Vertex *v);

  /**
   * Adds the point to the history graph without connecting it to the graph.
   * Mainly used for creating the root node.
   * @param point Position in Cartesian space
   */
  void addVertex(const geometry_msgs::Point &point);

  /**
   *
   * @param point Geometry message of the position in Cartesian space
   * @param state Vector of the position in Cartesian space
   * @param potential_gain Potential gain to associate with the new node
   */
  void addVertexAndConnect(const geometry_msgs::Point &point,
                           const Point &state, double potential_gain);

  void setDroneExploring(bool drone_exploring) {
    drone_exploring_ = drone_exploring;
  }

  /**
   * Deletes all the visualization markers published
   */
  void clear();

  /**
   * Restarts the history graph and its maintenance. This function erases
   * the previous graph built.
   */
  void reset();

  bool getDroneExploring() const { return drone_exploring_; }

  void publishVertex();

  /**
   * Computes a vector of active nodes sorted in increasing order of distance
   * from the current position, and stores it in res
   * @param res The resulting sorted vector is stored here
   * @return True if an active node exists, False otherwise
   */
  bool getNearestActiveNode(
      std::vector<std::pair<Vertex *, unsigned int>> &res);

  /**
   * Computes the path, if exists, from the current position to the goal node,
   * using A* algorithm.
   * @param goal Desired goal node in 3D space
   * @return The vector of the waypoints to reach the goal
   */
  std::vector<geometry_msgs::Pose> getPathToNode(Point &goal);

  /**
   * Discretizes the path in small steps to be sent to the controller later
   * @param pathNodes The vector of vertex waypoints
   * @param result The resulting sampled path in ROS message format
   */
  void sampleBranch(const std::vector<Vertex *> &pathNodes,
                    std::vector<geometry_msgs::Pose> &result);

  Point home_pos_;

 protected:
  void setUpPublisherMsg();

  double bfs(const Point &point);

  static geometry_msgs::Point getPointFromEigen(const Point &vec);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  VoxbloxManager *manager_;
  VoxbloxManager *manager_lowres_;
  Params *params_;
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
  unsigned int point_id_;
  unsigned int edge_id_;
  bool stopped_;
};
}  // namespace nbveplanner

#endif  // NBVEPLANNER_HISTORY_H

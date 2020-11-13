#ifndef NBVEPLANNER_RRTTREE_H_
#define NBVEPLANNER_RRTTREE_H_

#include "nbveplanner/tree.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kdtree/kdtree.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <random>
#include <sstream>
#include <thread>

#define SQ(x) ((x) * (x))
#define CUBE(x) ((x) * (x) * (x))

namespace nbveplanner {

class RrtTree : public TreeBase {
 public:
  RrtTree(VoxbloxManager *manager,
          VoxbloxManager *manager_lowres,
          Params *params);

  ~RrtTree();

  void visualizeFrustum();

  void setStateFromPoseCovMsg(
      const geometry_msgs::PoseWithCovarianceStamped &pose) override;

  void setStateFromOdometryMsg(const nav_msgs::Odometry &pose) override;

  void initialize(bool seedHistory) override;

  void iterate() override;

  void getBestBranch(std::vector<geometry_msgs::Pose> &path,
                     std::vector<geometry_msgs::Pose> &trajectory) override;

  void clear() override;

  void reset() override;

  std::vector<geometry_msgs::Pose> getPathBackToPrevious(
      std::string targetFrame) override;

  void setRootVicinity(double rootVicinity) override;

  void publishNode(Node *node);

  void modifyColorNode(int id);

  double optimized_gain(Pose &state);

  double gain(Pose &state);

  std::vector<geometry_msgs::Pose> samplePath(Pose start, Pose end,
                                              const std::string &targetFrame);

  void sampleBranch(const std::vector<Node *> &pathNodes,
                    std::vector<geometry_msgs::Pose> &result,
                    std::vector<geometry_msgs::Pose> &trajectory);

 protected:
  kdtree *kdTree_;
  std::stack<Pose> history_;
  int g_ID_;
  double root_vicinity_;
  int iteration_count_;
  std::mutex myMutex;
  visualization_msgs::MarkerArray tree_msg_;
  std::fstream file_response_;
  std::fstream file_tree_;
  std::fstream file_path_;
};

}  // namespace nbveplanner
#endif

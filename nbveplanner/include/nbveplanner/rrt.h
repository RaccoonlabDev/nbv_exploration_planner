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

namespace nbveplanner {

class RrtTree : public TreeBase {
 public:
  RrtTree();

  RrtTree(std::shared_ptr<VoxbloxManager> manager,
          std::shared_ptr<VoxbloxManager> manager_lowres,
          std::shared_ptr<Params> params);

  ~RrtTree();

  void visualizeFrustum();

  void setStateFromPoseCovMsg(
      const geometry_msgs::PoseWithCovarianceStamped &pose) override;

  void setStateFromOdometryMsg(const nav_msgs::Odometry &pose) override;

  void initialize(bool seedHistory) override;

  void iterate() override;

  std::vector<geometry_msgs::Pose> getBestEdge(
      std::string targetFrame) override;

  void getBestBranch(std::vector<geometry_msgs::Pose> &path,
                     std::vector<geometry_msgs::Pose> &trajectory) override;

  void clear() override;

  void reset() override;

  std::vector<geometry_msgs::Pose> getPathBackToPrevious(
      std::string targetFrame) override;

  void setRootVicinity(double rootVicinity) override;

  void publishNode(Node *node);

  double gain2(Pose &state);

  void gain(Pose state, double &maxGainFound, double &orientationFound);

  void compareGain(Pose &state, double gain, double &maxGainFound,
                   double &orientationFound);

  std::vector<geometry_msgs::Pose> samplePath(Pose start, Pose end,
                                              const std::string &targetFrame);

  void sampleBranch(const std::vector<Node *> &pathNodes,
                    std::vector<geometry_msgs::Pose> &result,
                    std::vector<geometry_msgs::Pose> &trajectory);

 protected:
  kdtree *kdTree_;
  std::shared_ptr<VoxbloxManager> manager_;
  std::shared_ptr<VoxbloxManager> manager_lowres_;
  double rootVicinity_;
  std::stack<Pose> history_;
  int g_ID_;
  int iterationCount_;
  std::fstream fileTree_;
  std::fstream filePath_;
  std::fstream fileResponse_;
  std::string logFilePath_;
  std::mutex myMutex;
  visualization_msgs::MarkerArray tree_msg_;
};

}  // namespace nbveplanner
#endif

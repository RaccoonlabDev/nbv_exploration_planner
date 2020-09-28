#ifndef RRTTREE_H_
#define RRTTREE_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kdtree/kdtree.h>
#include <nav_msgs/Odometry.h>
#include <nbveplanner/tree.h>
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
#define SQRT2 0.70711

class RrtTree : public TreeBase<Eigen::Vector4d> {
 public:
  RrtTree();

  RrtTree(VoxbloxManager *manager, VoxbloxManager *manager_lowres, Params *params);

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

  void publishNode(Node<StateVec> *node);

  void gain2(StateVec &state);

  void gain(StateVec state, double &maxGainFound, double &orientationFound);

  void compareGain(StateVec &state, double gain, double &maxGainFound,
                   double &orientationFound);

  std::vector<geometry_msgs::Pose> samplePath(StateVec start, StateVec end,
                                              const std::string &targetFrame);

  void sampleBranch(const std::vector<Node<StateVec> *> &pathNodes,
                    std::vector<geometry_msgs::Pose> &result,
                    std::vector<geometry_msgs::Pose> &trajectory);

  // virtual int getUnknownCells();

 protected:
  kdtree *kdTree_;
  double rootVicinity_;
  std::stack<StateVec> history_;
  int g_ID_;
  int iterationCount_;
  std::fstream fileTree_;
  std::fstream filePath_;
  std::fstream fileResponse_;
  std::string logFilePath_;
  std::mutex myMutex;
  visualization_msgs::MarkerArray tree_msg_;
};

#endif

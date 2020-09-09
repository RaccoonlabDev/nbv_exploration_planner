#ifndef NBVEP_H_
#define NBVEP_H_

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nbveplanner/history.h>
#include <nbveplanner/rrt.h>
#include <nbveplanner/tree.hpp>
#include <nbveplanner/voxblox_manager.h>
#include <std_srvs/Empty.h>

#include <ros/ros.h>
#include <string>
#include <vector>

#include <chrono>
#include <ctime>

#define SQ(x) ((x) * (x))
#define SQRT2 0.70711

template <typename stateVec> class nbvePlanner {
protected:
  enum SamplingStatus { initial, reseeded, full };

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber posClient_;
  ros::Subscriber posCovClient_;
  ros::Subscriber odomClient_;
  ros::ServiceServer plannerService_;
  ros::ServiceServer goHomeService_;
  ros::ServiceServer resetService_;
  ros::ServiceServer historyGraphService;
  ros::Subscriber pointcloud_sub_;

  // Map Manager
  VoxbloxManager *manager_;

  bool exploration_complete_;
  bool ready_;

public:
  TreeBase<stateVec> *tree_;
  History *hist_;
  std::ofstream dataFile_;
  std::string path_;
  ros::Time initialTime;
  Params params_;
  geometry_msgs::Pose local_position_;


  nbvePlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  ~nbvePlanner();

  bool setParams();

  void poseCovCallback(const geometry_msgs::PoseWithCovarianceStamped &pose);

  void odomCallback(const nav_msgs::Odometry &pose);

  bool isExplorationComplete();

  bool isReady();

  bool plannerCallback(std::vector<geometry_msgs::Pose> &path,
                       std::vector<geometry_msgs::Pose> &trajectory);

  bool goHome(std::vector<geometry_msgs::Pose> &path);

  bool reset();

  void setDroneExploration(bool drone_exploring);

  void initializeHistoryGraph(geometry_msgs::Point initial_position);

};

#endif // NBVEP_H_

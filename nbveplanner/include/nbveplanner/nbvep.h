#ifndef NBVEPLANNER_NBVEP_H_
#define NBVEPLANNER_NBVEP_H_

#include "nbveplanner/history.h"
#include "nbveplanner/params.h"
#include "nbveplanner/rrt.h"
#include "nbveplanner/tree.h"
#include "nbveplanner/voxblox_manager.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <chrono>
#include <ctime>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>
#include <memory>

#define SQ(x) ((x) * (x))
#define SQRT2 0.70711

namespace nbveplanner {

class nbvePlanner {
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
  std::shared_ptr<VoxbloxManager> manager_;
  std::shared_ptr<VoxbloxManager> manager_lowres_;
  std::unique_ptr<RrtTree> tree_;

  bool exploration_complete_;
  bool ready_;
  std::string ns_map_;

 public:
  std::unique_ptr<History> hist_;
  std::shared_ptr<Params> params_;
  geometry_msgs::Pose local_position_;

  nbvePlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  ~nbvePlanner();

  void poseCovCallback(const geometry_msgs::PoseWithCovarianceStamped &pose);

  void odomCallback(const nav_msgs::Odometry &pose);

  bool isExplorationComplete() const;

  bool isReady() const;

  bool plannerCallback(std::vector<geometry_msgs::Pose> &path,
                       std::vector<geometry_msgs::Pose> &trajectory);

  bool goHome(std::vector<geometry_msgs::Pose> &path) const;

  bool reset();

  void setDroneExploration(bool drone_exploring) const;

  void initializeHistoryGraph(geometry_msgs::Point initial_position) const;
};

}  // namespace nbveplanner

#endif  // NBVEPLANNER_NBVEP_H_

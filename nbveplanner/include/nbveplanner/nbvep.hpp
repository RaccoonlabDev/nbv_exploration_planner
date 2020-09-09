#ifndef NBVEP_HPP_
#define NBVEP_HPP_

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <nbveplanner/nbvep.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

template <typename stateVec>
nbvePlanner<stateVec>::nbvePlanner(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  manager_ = new VoxbloxManager(nh, nh_private);
  hist_ = new History(nh, nh_private, manager_);

  // Set up the topics and services
  params_.inspectionPath_ =
      nh_.advertise<visualization_msgs::Marker>("inspectionPath", 1);
  params_.explorationTree_ =
      nh_.advertise<visualization_msgs::MarkerArray>("explorationTree", 100);
  posCovClient_ = nh_.subscribe("pose_cov", 10,
                                &nbvePlanner<stateVec>::poseCovCallback, this);
  odomClient_ =
      nh_.subscribe("odometry", 10, &nbvePlanner<stateVec>::odomCallback, this);

  if (!setParams()) {
    ROS_ERROR("Could not start the planner. Parameters missing!");
  }

  // Precompute the sensor field of view boundaries. The normals of the
  // separating hyperplanes are stored
  double pitch = M_PI * params_.camPitch_[0] / 180.0;
  double camTop = (M_PI * params_.camVertical_[0] / 360.0 - pitch) - M_PI / 2.0;
  double camBottom =
      (-M_PI * params_.camVertical_[0] / 360.0 - pitch) + M_PI / 2.0;

  if (params_.camHorizontal_[0] != 360) {
    double side = M_PI * (params_.camHorizontal_[0]) / 360.0 - M_PI / 2.0;
    Vector3d bottom(cos(camBottom), 0.0, -sin(camBottom));
    Vector3d top(cos(camTop), 0.0, -sin(camTop));
    Vector3d right(cos(side), sin(side), 0.0);
    Vector3d left(cos(side), -sin(side), 0.0);
    AngleAxisd m = AngleAxisd(pitch, Vector3d::UnitY());
    Vector3d rightR = m * right;
    Vector3d leftR = m * left;
    rightR.normalize();
    leftR.normalize();
    std::vector<Eigen::Vector3d> camBoundNormals;
    camBoundNormals.push_back(bottom);
    camBoundNormals.push_back(top);
    camBoundNormals.push_back(rightR);
    camBoundNormals.push_back(leftR);
    params_.camBoundNormals_.push_back(camBoundNormals);
  } else {
    double side = M_PI * 90.0 / 360.0 - M_PI / 2.0;

    // X axis
    Vector3d bottom_x(cos(camBottom), 0.0, sin(camBottom));
    Vector3d top_x(cos(camTop), 0.0, sin(camTop));
    Vector3d right(cos(side), sin(side), 0.0);
    Vector3d left(cos(side), -sin(side), 0.0);
    AngleAxisd m = AngleAxisd(pitch, Vector3d::UnitY());
    Vector3d rightR_x = m * right;
    Vector3d leftR_x = m * left;
    rightR_x.normalize();
    leftR_x.normalize();
    std::vector<Eigen::Vector3d> camBoundNormals;
    camBoundNormals.push_back(bottom_x);
    camBoundNormals.push_back(top_x);
    camBoundNormals.push_back(rightR_x);
    camBoundNormals.push_back(leftR_x);
    params_.camBoundNormals_.push_back(camBoundNormals);

    // Y axis
    double camBottom2 = (-M_PI * params_.camVertical_[0] / 360.0) + M_PI / 2.0;
    double camTop2 = (M_PI * params_.camVertical_[0] / 360.0) - M_PI / 2.0;
    Vector3d bottom = Vector3d(0.0, cos(camBottom2), sin(camBottom2));
    Vector3d top = Vector3d(0.0, cos(camTop2), sin(camTop2));
    double side2 = side + M_PI / 2;
    Vector3d rightR_y = leftR_x;
    Vector3d leftR_y = -rightR_x;
    Vector3d bottomR_y = m * bottom;
    Vector3d topR_y = m * top;
    bottomR_y.normalize();
    topR_y.normalize();
    camBoundNormals.clear();
    camBoundNormals.push_back(bottomR_y);
    camBoundNormals.push_back(topR_y);
    camBoundNormals.push_back(rightR_y);
    camBoundNormals.push_back(leftR_y);
    params_.camBoundNormals_.push_back(camBoundNormals);

    // -X axis
    camBoundNormals.clear();
    camBoundNormals.emplace_back(-top_x);    // Bottom neg. x axis
    camBoundNormals.emplace_back(-bottom_x); // Top neg. x axis
    camBoundNormals.emplace_back(-leftR_x);  // Right neg. x axis
    camBoundNormals.emplace_back(-rightR_x); // Left neg. x axis
    params_.camBoundNormals_.push_back(camBoundNormals);

    // -Y axis
    camBoundNormals.clear();
    camBoundNormals.emplace_back(-topR_y);    // Bottom neg. y axis
    camBoundNormals.emplace_back(-bottomR_y); // Top neg. y axis
    camBoundNormals.emplace_back(-leftR_y);   // Right neg. y axis
    camBoundNormals.emplace_back(-rightR_y);  // Left neg. y axis
    params_.camBoundNormals_.push_back(camBoundNormals);
  }

  hist_->setParams(params_);

  // Initialize the tree instance.
  tree_ = new RrtTree(manager_);
  tree_->setParams(params_);

  // Not yet ready. Needs a position message first.
  ready_ = false;
  exploration_complete_ = false;
  std::string path_plot;
  if (!ros::param::get(ros::this_node::getName() + "/path_plot", path_plot)) {
    ROS_WARN("No path for storing the plot provided");
  }
  time_t t = time(nullptr);
  struct tm *now = localtime(&t);
  char buffer[80];
  strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", now);
  path_ = path_plot + "/data-" + buffer + ".txt";
  dataFile_.open(path_);
  dataFile_ << "0;0;" << manager_->getNumberOfMappedVoxels() << "\n";
  dataFile_.close();
  ROS_INFO("Planner All Set");
}

template <typename stateVec> nbvePlanner<stateVec>::~nbvePlanner() = default;

template <typename stateVec>
void nbvePlanner<stateVec>::poseCovCallback(
    const geometry_msgs::PoseWithCovarianceStamped &pose) {
  local_position_ = pose.pose.pose;
  tree_->setStateFromPoseCovMsg(pose);
  // Planner is now ready to plan.
  if (!ready_) {
    initialTime = ros::Time::now();
  }
  ready_ = true;
}

template <typename stateVec>
void nbvePlanner<stateVec>::odomCallback(const nav_msgs::Odometry &pose) {
  local_position_ = pose.pose.pose;
  tree_->setStateFromOdometryMsg(pose);
  // Planner is now ready to plan.
  if (!ready_) {
    initialTime = ros::Time::now();
  }
  ready_ = true;
}

template <typename stateVec> bool nbvePlanner<stateVec>::isReady() {
  return ready_;
}

template <typename stateVec>
bool nbvePlanner<stateVec>::isExplorationComplete() {
  return exploration_complete_;
}

template <typename stateVec>
bool nbvePlanner<stateVec>::plannerCallback(
    std::vector<geometry_msgs::Pose> &path,
    std::vector<geometry_msgs::Pose> &trajectory) {
  ros::Time computationTime = ros::Time::now();
  // Check that planner is ready to compute path.
  if (exploration_complete_) {
    ROS_INFO_THROTTLE(1,
                      "Exploration finished. Not planning any further moves.");
    return true;
  }
  if (manager_ == nullptr) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: No Voxblox available!");
    return true;
  }
  if (manager_->isEsdfEmpty() or manager_->isTsdfEmpty()) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: Tsdf or Esdf map is empty!");
    return true;
  }
  path.clear();
  // Clear old tree and reinitialize.
  tree_->clear();
  hist_->clear();
  tree_->initialize(false);
  tree_->setRootVicinity(params_.vicinityRange_);

  // Iterate the tree construction method.
  SamplingStatus samplingStatus = initial;
  hist_->setDroneExploring(true);
  int loopCount = 0;
  std::vector<std::pair<Vertex *, unsigned int>> hist_nodes;
  int hist_idx;

  while (
      (!tree_->gainFound() || tree_->getCounter() < params_.initIterations_) and
      ros::ok()) {
    if (tree_->getCounter() > params_.cuttoffIterations_) {
      switch (samplingStatus) {
      case initial: {
        ROS_INFO("No gain found, seeding history graph");
        tree_->clear();
        loopCount = 0;
        if (hist_->getNearestActiveNode(hist_nodes)) {
          hist_idx = 1;
          tree_->setHistRoot(Eigen::Vector4d{
              hist_nodes[0].first->pos.x(), hist_nodes[0].first->pos.y(),
              hist_nodes[0].first->pos.z(), 0.0});
          tree_->initialize(true);
          samplingStatus = reseeded;
        } else {
          ROS_INFO("Empty History Graph, seeding full search space");
          tree_->initialize(false);
          samplingStatus = full;
        }
        break;
      }
      case reseeded: {
        tree_->clear();
        loopCount = 0;
        if (hist_idx < hist_nodes.size()) {
          ROS_INFO("No gain found, seeding next active node");
          tree_->setHistRoot(
              Eigen::Vector4d{hist_nodes[hist_idx].first->pos.x(),
                              hist_nodes[hist_idx].first->pos.y(),
                              hist_nodes[hist_idx].first->pos.z(), 0.0});
          ++hist_idx;
          tree_->initialize(true);
        } else {
          ROS_INFO("No gain found, seeding full search space");
          tree_->initialize(false);
          double radius = sqrt(SQ(params_.minX_ - params_.maxX_) +
                               SQ(params_.minY_ - params_.maxY_) +
                               SQ(params_.minZ_ - params_.maxZ_));
          tree_->setRootVicinity(radius);
          samplingStatus = full;
        }
        break;
      }
      case full: {
        ROS_INFO("No gain found, shutting down");
        // ros::shutdown();
        exploration_complete_ = true;
        return true;
      }
      }
    }
    if (loopCount > 1000 * (tree_->getCounter() + 1)) {
      if (samplingStatus == initial) {
        ROS_INFO_THROTTLE(
            1,
            "Exceeding maximum failed iterations, return to previous point!");
        path = tree_->getPathBackToPrevious(params_.navigationFrame_);
        // hist_->setDroneExploring(false);
        return true;
      } else if (hist_idx < hist_nodes.size()) {
        tree_->clear();
        loopCount = 0;
        if (hist_idx < hist_nodes.size()) {
          ROS_INFO("No gain found, seeding next active node");
          tree_->setHistRoot(
              Eigen::Vector4d{hist_nodes[hist_idx].first->pos.x(),
                              hist_nodes[hist_idx].first->pos.y(),
                              hist_nodes[hist_idx].first->pos.z(), 0.0});
          ++hist_idx;
          tree_->initialize(true);
        }
      }
    }
    tree_->iterate();
    loopCount++;
  }
  if (samplingStatus == reseeded) {
    trajectory = hist_->getPathToNode((hist_nodes[hist_idx - 1].first)->pos);
    hist_->setDroneExploring(false);
  }
  // Extract the best edge.
  tree_->getBestBranch(path, trajectory);

  double cTime = (ros::Time::now() - computationTime).toSec();
  double executionTime = (ros::Time::now() - initialTime).toSec();
  ROS_INFO("Path computation lasted %2.3fs", cTime);
  // writeNumberUnknown(executionTime, cTime);
  /*dataFile_.open(path_, std::ostream::app);
  dataFile_ << executionTime << ";" << cTime << ";"
            << manager_->getNumberOfMappedVoxels() << "\n";
  dataFile_.close();*/
  return true;
}

template <typename stateVec>
bool nbvePlanner<stateVec>::goHome(std::vector<geometry_msgs::Pose> &path) {
  path = hist_->getPathToNode(hist_->home_pos_);
  if (path.empty()) {
    return false;
  }
  return true;
}

template <typename stateVec> bool nbvePlanner<stateVec>::reset() {
  tree_->reset();
  hist_->reset();
  manager_->clear();
  return true;
}

template <typename stateVec>
void nbvePlanner<stateVec>::setDroneExploration(bool drone_exploring) {
  hist_->setDroneExploring(drone_exploring);
}

template <typename stateVec> bool nbvePlanner<stateVec>::setParams() {
  const std::string &ns = ros::this_node::getName();
  bool ret = true;
  params_.v_max_ = 1.0;
  if (!ros::param::get(ns + "/system/v_max", params_.v_max_)) {
    ROS_WARN(
        "No maximal system speed specified. Looking for %s. Default is 0.25.",
        (ns + "/system/v_max").c_str());
  }
  params_.dyaw_max_ = 0.5;
  if (!ros::param::get(ns + "/system/dyaw_max", params_.dyaw_max_)) {
    ROS_WARN("No maximal yaw speed specified. Looking for %s. Default is 0.5.",
             (ns + "/system/yaw_max").c_str());
  }
  params_.camPitch_ = {15.0};
  if (!ros::param::get(ns + "/system/camera/pitch", params_.camPitch_)) {
    ROS_WARN("No camera pitch specified. Looking for %s. Default is 15deg.",
             (ns + "/system/camera/pitch").c_str());
  }
  params_.camHorizontal_ = {90.0};
  if (!ros::param::get(ns + "/system/camera/horizontal",
                       params_.camHorizontal_)) {
    ROS_WARN("No camera horizontal opening specified. Looking for %s. Default "
             "is 90deg.",
             (ns + "/system/camera/horizontal").c_str());
  }
  params_.camVertical_ = {60.0};
  if (!ros::param::get(ns + "/system/camera/vertical", params_.camVertical_)) {
    ROS_WARN("No camera vertical opening specified. Looking for %s. Default is "
             "60deg.",
             (ns + "/system/camera/vertical").c_str());
  }
  if (params_.camPitch_.size() != params_.camVertical_.size()) {
    ROS_WARN("Specified camera fields of view unclear: Not all parameter "
             "vectors have same length! Setting to default.");
    params_.camPitch_.clear();
    params_.camPitch_ = {15.0};
    params_.camHorizontal_.clear();
    params_.camHorizontal_ = {90.0};
    params_.camVertical_.clear();
    params_.camVertical_ = {60.0};
  }
  params_.igFree_ = 0.0;
  if (!ros::param::get(ns + "/nbvep/gain/free", params_.igFree_)) {
    ROS_WARN("No gain coefficient for free cells specified. Looking for %s. "
             "Default is 0.0.",
             (ns + "/nbvep/gain/free").c_str());
  }
  params_.igOccupied_ = 0.0;
  if (!ros::param::get(ns + "/nbvep/gain/occupied", params_.igOccupied_)) {
    ROS_WARN("No gain coefficient for occupied cells specified. Looking for "
             "%s. Default is 0.0.",
             (ns + "/nbvep/gain/occupied").c_str());
  }
  params_.igUnmapped_ = 1.0;
  if (!ros::param::get(ns + "/nbvep/gain/unmapped", params_.igUnmapped_)) {
    ROS_WARN("No gain coefficient for unmapped cells specified. Looking for "
             "%s. Default is 1.0.",
             (ns + "/nbvep/gain/unmapped").c_str());
  }
  params_.degressiveCoeff_ = 0.0;
  if (!ros::param::get(ns + "/nbvep/gain/degressive_coeff",
                       params_.degressiveCoeff_)) {
    ROS_WARN("No degressive factor for gain accumulation specified. Looking "
             "for %s. Default is 0.25.",
             (ns + "/nbvep/gain/degressive_coeff").c_str());
  }
  params_.initIterations_ = 15;
  if (!ros::param::get(ns + "/nbvep/tree/init_iterations",
                       params_.initIterations_)) {
    ROS_WARN("No initial iterations value specified. Looking for %s. Default "
             "is 15.",
             (ns + "/nbvep/tree/init_iterations").c_str());
  }
  params_.extensionRange_ = 1.0;
  if (!ros::param::get(ns + "/nbvep/tree/extension_range",
                       params_.extensionRange_)) {
    ROS_WARN("No value for maximal extension range specified. Looking for %s. "
             "Default is 1.0m.",
             (ns + "/nbvep/tree/extension_range").c_str());
  }
  params_.vicinityRange_ = 5.0;
  if (!ros::param::get(ns + "/nbvep/tree/vicinity_range",
                       params_.vicinityRange_)) {
    ROS_WARN("No value for vicinity range specified. Looking for %s. "
             "Default is 5.0m.",
             (ns + "/nbvep/tree/vicinity_range").c_str());
  }
  params_.dt_ = 0.1;
  if (!ros::param::get(ns + "/nbvep/dt", params_.dt_)) {
    ROS_WARN(
        "No sampling time step specified. Looking for %s. Default is 0.1s.",
        (ns + "/nbvep/dt").c_str());
  }
  params_.gainRange_ = 1.0;
  if (!ros::param::get(ns + "/nbvep/gain/range", params_.gainRange_)) {
    ROS_WARN("No gain range specified. Looking for %s. Default is 1.0m.",
             (ns + "/nbvep/gain/range").c_str());
  }
  if (!ros::param::get(ns + "/bbx/minX", params_.minX_)) {
    ROS_WARN("No x-min value specified. Looking for %s",
             (ns + "/bbx/minX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/minY", params_.minY_)) {
    ROS_WARN("No y-min value specified. Looking for %s",
             (ns + "/bbx/minY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/minZ", params_.minZ_)) {
    ROS_WARN("No z-min value specified. Looking for %s",
             (ns + "/bbx/minZ").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxX", params_.maxX_)) {
    ROS_WARN("No x-max value specified. Looking for %s",
             (ns + "/bbx/maxX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxY", params_.maxY_)) {
    ROS_WARN("No y-max value specified. Looking for %s",
             (ns + "/bbx/maxY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxZ", params_.maxZ_)) {
    ROS_WARN("No z-max value specified. Looking for %s",
             (ns + "/bbx/maxZ").c_str());
    ret = false;
  }
  params_.boundingBox_[0] = 0.5;
  if (!ros::param::get(ns + "/system/bbx/x", params_.boundingBox_[0])) {
    ROS_WARN("No x size value specified. Looking for %s. Default is 0.5m.",
             (ns + "/system/bbx/x").c_str());
  }
  params_.boundingBox_[1] = 0.5;
  if (!ros::param::get(ns + "/system/bbx/y", params_.boundingBox_[1])) {
    ROS_WARN("No y size value specified. Looking for %s. Default is 0.5m.",
             (ns + "/system/bbx/y").c_str());
  }
  params_.boundingBox_[2] = 0.3;
  if (!ros::param::get(ns + "/system/bbx/z", params_.boundingBox_[2])) {
    ROS_WARN("No z size value specified. Looking for %s. Default is 0.3m.",
             (ns + "/system/bbx/z").c_str());
  }
  params_.cuttoffIterations_ = 200;
  if (!ros::param::get(ns + "/nbvep/tree/cuttoff_iterations",
                       params_.cuttoffIterations_)) {
    ROS_WARN("No cuttoff iterations value specified. Looking for %s. Default "
             "is 200.",
             (ns + "/nbvep/tree/cuttoff_iterations").c_str());
  }
  params_.zero_gain_ = 0.0;
  if (!ros::param::get(ns + "/nbvep/gain/zero", params_.zero_gain_)) {
    ROS_WARN("No zero gain value specified. Looking for %s. Default is 0.0.",
             (ns + "/nbvep/gain/zero").c_str());
  }
  params_.dOvershoot_ = 0.5;
  if (!ros::param::get(ns + "/system/bbx/overshoot", params_.dOvershoot_)) {
    ROS_WARN("No estimated overshoot value for collision avoidance specified. "
             "Looking for %s. Default is 0.5m.",
             (ns + "/system/bbx/overshoot").c_str());
  }
  params_.navigationFrame_ = "world";
  if (!ros::param::get(ns + "/tf_frame", params_.navigationFrame_)) {
    ROS_WARN(
        "No navigation frame specified. Looking for %s. Default is 'world'.",
        (ns + "/tf_frame").c_str());
  }
  params_.voxelSize_ = 0.2;
  if (!ros::param::get(ns + "/tsdf_voxel_size", params_.voxelSize_)) {
    ROS_WARN("No option for voxel size specified. Looking for %s. "
             "Default is 0.2.",
             (ns + "/tsdf_voxel_size").c_str());
  }
  params_.robot_radius_ = params_.boundingBox_.norm() / 2.0;
  manager_->setRobotRadius(params_.robot_radius_);
  return ret;
}

template <typename stateVec>
void nbvePlanner<stateVec>::initializeHistoryGraph(
    geometry_msgs::Point initial_position) {
  hist_->addVertex(initial_position);
}

#endif // NBVEP_HPP_

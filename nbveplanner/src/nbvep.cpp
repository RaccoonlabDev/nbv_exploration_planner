#include "nbveplanner/nbvep.h"
#include <minkindr_conversions/kindr_tf.h>

namespace nbveplanner {

nbvePlanner::nbvePlanner(const ros::NodeHandle &nh,
                         const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), params_(std::make_unique<Params>()) {
  params_->setParametersFromRos(nh_private_);
  setUpCamera();
  params_->inspection_path_ =
      nh_.advertise<visualization_msgs::Marker>("inspectionPath", 1);
  params_->exploration_tree_ =
      nh_.advertise<visualization_msgs::MarkerArray>("explorationTree", 1);
  posCovClient_ =
      nh_.subscribe("pose_cov", 10, &nbvePlanner::poseCovCallback, this);
  odomClient_ = nh_.subscribe("odometry", 10, &nbvePlanner::odomCallback, this);

  nh_private_.param<std::string>("namespace_lowres_map", ns_map_, "lowres/");

  manager_ = std::make_unique<HighResManager>(nh, nh_private, params_.get());
  manager_lowres_ =
      std::make_unique<LowResManager>(nh, nh_private, params_.get(), ns_map_);
  hist_ = std::make_unique<History>(nh, nh_private, manager_.get(),
                                    manager_lowres_.get(), params_.get());
  tree_ = std::make_unique<RrtTree>(manager_.get(), manager_lowres_.get(),
                                    params_.get());
  if (params_->log_) {
    file_exploration_.open((params_->log_path_ + "exploration.txt").c_str(),
                           std::ios::out);
    file_exploration_ << ros::Time::now().toSec() << ";0;0\n";
    file_exploration_.close();
  }
  // Not yet ready. Needs a position message first.
  ready_ = false;
  exploration_complete_ = false;
  ROS_INFO("Planner All Set");
}

void nbvePlanner::setUpCamera() const {
  // Set camera FOV
  params_->camera_model_.setIntrinsicsFromFoV(
      params_->camera_hfov_, params_->camera_vfov_, params_->sensor_min_range_,
      params_->gain_range_);
  // Set Boundaries of Exploration
  params_->camera_model_.setBoundingBox(params_->bbx_min_, params_->bbx_max_);
  // Set camera frame transformation
  static tf::TransformListener listener;
  tf::StampedTransform stamped_transform;

  try {
    listener.waitForTransform(params_->camera_frame_, "base_link",
                              ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform(params_->camera_frame_, "base_link", ros::Time(0),
                             stamped_transform);
  } catch (tf::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
  Transformation T_C_B;
  tf::transformTFToKindr(stamped_transform, &T_C_B);
  params_->camera_model_.setExtrinsics(T_C_B);
}

nbvePlanner::~nbvePlanner() {
  if (file_exploration_.is_open()) {
    file_exploration_.close();
  }
}

void nbvePlanner::poseCovCallback(
    const geometry_msgs::PoseWithCovarianceStamped &pose) {
  local_position_ = pose.pose.pose;
  tree_->setStateFromPoseCovMsg(pose);
  // Planner is now ready to plan.
  ready_ = true;
}

void nbvePlanner::odomCallback(const nav_msgs::Odometry &pose) {
  local_position_ = pose.pose.pose;
  tree_->setStateFromOdometryMsg(pose);
  // Planner is now ready to plan.
  ready_ = true;
}

bool nbvePlanner::isReady() const { return ready_; }

bool nbvePlanner::isExplorationComplete() const {
  return exploration_complete_;
}

bool nbvePlanner::plannerCallback(
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
  ros::spinOnce();
  tree_->initialize(false);
  tree_->setRootVicinity(params_->vicinity_range_);

  // Iterate the tree construction method.
  SamplingStatus samplingStatus = initial;
  hist_->setDroneExploring(true);
  int loopCount = 0;
  std::vector<std::pair<Vertex *, unsigned int>> hist_nodes;
  int hist_idx;

  while ((!tree_->gainFound() ||
          tree_->getCounter() < params_->init_iterations_) and
         ros::ok()) {
    if (tree_->getCounter() > params_->cutoff_iterations_) {
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
            double radius = (params_->bbx_min_ - params_->bbx_max_).norm();
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
        path = tree_->getPathBackToPrevious(params_->frame_id_);
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
  ROS_INFO("Path computation lasted %2.3fs", cTime);
  if (params_->log_) {
    file_exploration_.open((params_->log_path_ + "exploration.txt").c_str(),
                           std::ios::app);
    file_exploration_ << ros::Time::now().toSec() << ";" << cTime << ";"
                      << manager_->getNumberMappedVoxels() << "\n";
    file_exploration_.close();
  }
  return true;
}

bool nbvePlanner::goHome(std::vector<geometry_msgs::Pose> &path) const {
  path = hist_->getPathToNode(hist_->home_pos_);
  if (path.empty()) {
    return false;
  }
  return true;
}

bool nbvePlanner::reset() {
  tree_->reset();
  hist_->reset();
  manager_->clear();
  manager_lowres_->clear();
  exploration_complete_ = false;
  return true;
}

void nbvePlanner::setDroneExploration(bool drone_exploring) const {
  hist_->setDroneExploring(drone_exploring);
}

void nbvePlanner::initializeHistoryGraph(
    geometry_msgs::Point initial_position) const {
  hist_->addVertex(initial_position);
}

}  // namespace nbveplanner
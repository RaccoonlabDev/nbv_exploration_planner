#include <chrono>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nbv_msgs/SetState.h>
#include <nbv_msgs/State.h>
#include <nbv_msgs/Status.h>
#include <nbveplanner/history.h>
#include <nbveplanner/nbvep.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>
#include <thread>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

bool access_local_pos = false;
bool stopped;
double dt;
double wp_z;
double shift_initial_x, shift_initial_y, shift_initial_z;
double speed_rotate;
std::string frame_id;

geometry_msgs::PoseStamped position_control_msg;
ros::Publisher pose_pub;
ros::Publisher status_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
mavros_msgs::State current_state;
nbv_msgs::State exploration_state;
nbv_msgs::Status request_state;
nbvePlanner<Eigen::Matrix<double, 4, 1>> *planner;

void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

void send_path(std::vector<geometry_msgs::Pose> &path) {
  geometry_msgs::PoseStamped aux_msg;
  ros::Rate loop_rate(1 / dt);
  for (auto &pos : path) {
    if (request_state.status == nbv_msgs::Status::STATUS_STOP) {
      return;
    }
    aux_msg.header.stamp = ros::Time::now();
    aux_msg.header.frame_id = frame_id;

    aux_msg.pose.position.x = pos.position.x;
    aux_msg.pose.position.y = pos.position.y;
    aux_msg.pose.position.z = pos.position.z;

    aux_msg.pose.orientation.x = pos.orientation.x;
    aux_msg.pose.orientation.y = pos.orientation.y;
    aux_msg.pose.orientation.z = pos.orientation.z;
    aux_msg.pose.orientation.w = pos.orientation.w;

    position_control_msg = aux_msg;
    loop_rate.sleep();
  }
  ros::Duration(2.0).sleep();
}

bool stop_planner() {
  while (not stopped) {
    ros::Duration(dt).sleep();
  }
  return true;
}

bool go_home() {
  std::vector<geometry_msgs::Pose> path;
  if (planner->goHome(path)) {
    exploration_state.status.status = nbv_msgs::Status::STATUS_GO_HOME;
    send_path(path);
    exploration_state.status.status = nbv_msgs::Status::STATUS_GO_HOME_COMPLETE;
    return true;
  }
  return false;
}

void start_planner() {
  int iteration = 0;
  std::vector<geometry_msgs::Pose> path;
  std::vector<geometry_msgs::Pose> trajectory;
  ros::Rate loop_rate(1);
  while (ros::ok() and
         (request_state.status == nbv_msgs::Status::STATUS_RUN or
          request_state.status == nbv_msgs::Status::STATUS_WAIT)) {
    ROS_INFO_THROTTLE(0.5, "Planning iteration %i", iteration);
    exploration_state.data = "Planning iteration " + std::to_string(iteration);
    if (planner->plannerCallback(path, trajectory)) {
      if (planner->isExplorationComplete()) {
        exploration_state.status.status = nbv_msgs::Status::STATUS_COMPLETE;
        exploration_state.data = "Exploration complete";
        ROS_INFO("Exploration complete");
      }
      if (not trajectory.empty()) {
        send_path(trajectory);
        trajectory.clear();
        // planner->setDroneExploration(true);
      }
      if (not path.empty()) {
        send_path(path);
      }
    } else {
      ROS_WARN_THROTTLE(1, "Planner not reachable");
      exploration_state.data = "Planner service not reachable";
    }
    iteration++;
    loop_rate.sleep();
  }
  stopped = true;
}

void initializationMotion() {

  while (ros::ok() && !current_state.connected) {
    ros::Duration(dt).sleep();
  }

  ROS_INFO("Started Exploration");

  while (not planner->isReady()) {
    ROS_INFO("Waiting initialization Local position");
    ros::Duration(1.0).sleep();
  }

  position_control_msg.header.frame_id = frame_id;
  position_control_msg.pose.position.x = planner->local_position_.position.x;
  position_control_msg.pose.position.y = planner->local_position_.position.y;
  position_control_msg.pose.position.z = wp_z;
/*
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  while (current_state.mode != "OFFBOARD" or !current_state.armed) {
    if( current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(1.0))){
      if( set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if( !current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(1.0))){
        if( arming_client.call(arm_cmd) &&
            arm_cmd.response.success){
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    position_control_msg.header.stamp = ros::Time::now();
    //pose_pub.publish(position_control_msg);

    ros::spinOnce();
    ros::Duration(dt).sleep();
  }

  for (int i = 0; i < 100; ++i) {
    position_control_msg.header.stamp = ros::Time::now();
    pose_pub.publish(position_control_msg);
    ros::Duration(dt).sleep();
  }
*/
  // This is the initialization motion, necessary to known free space that
  // allows the planning of initial paths.

  ROS_INFO("Starting the planner: Performing initialization motion");

  double step = (2 * M_PI) / (dt * speed_rotate);
  step = 2 * M_PI / step;

  ros::Rate loop_rate(1 / dt);
  for (double i = 0.; i < 2 * M_PI; i += step) {
    if (request_state.status != nbv_msgs::Status::STATUS_RUN and
        request_state.status != nbv_msgs::Status::STATUS_WAIT) {
      stopped = true;
      return;
    }
    position_control_msg.header.frame_id = frame_id;
    position_control_msg.header.stamp = ros::Time::now();

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, i);
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q, odom_quat);
    position_control_msg.pose.orientation = odom_quat;

    pose_pub.publish(position_control_msg);
    loop_rate.sleep();
  }
  planner->initializeHistoryGraph(position_control_msg.pose.position);
  ROS_INFO("History graph initialized");

  position_control_msg.pose.position.x += shift_initial_x;
  position_control_msg.pose.position.y += shift_initial_y;
  position_control_msg.pose.position.z += shift_initial_z;
  position_control_msg.header.stamp = ros::Time::now();
  pose_pub.publish(position_control_msg);
  ros::Duration(1.0).sleep();
  start_planner();
}

void publish_pose() {
  ros::Rate rate(20.0);
  while (ros::ok()) {
    if (exploration_state.status.status == nbv_msgs::Status::STATUS_RUN or
        exploration_state.status.status == nbv_msgs::Status::STATUS_GO_HOME) {
      pose_pub.publish(position_control_msg);
    }
    status_pub.publish(exploration_state);
    rate.sleep();
  }
}

void private_manager() {
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    switch (request_state.status) {
    case nbv_msgs::Status::STATUS_RUN: {
      if (exploration_state.status.status == nbv_msgs::Status::STATUS_WAIT) {
        std::thread t1(initializationMotion);
        t1.detach();
        std::thread t2(&History::historyMaintenance, planner->hist_);
        t2.detach();
      } else if (exploration_state.status.status ==
                     nbv_msgs::Status::STATUS_STOP or
                 exploration_state.status.status ==
                     nbv_msgs::Status::STATUS_GO_HOME_COMPLETE) {
        std::thread t3(start_planner);
        t3.detach();
      }
      stopped = false;
      exploration_state.status.status = request_state.status;
      request_state.status = nbv_msgs::Status::STATUS_WAIT;
      break;
    }
    case nbv_msgs::Status::STATUS_STOP: {
      stop_planner();
      exploration_state.status.status = nbv_msgs::Status::STATUS_STOP;
      request_state.status = nbv_msgs::Status::STATUS_WAIT;
      break;
    }
    case nbv_msgs::Status::STATUS_RESET: {
      stop_planner();
      exploration_state.status.status = nbv_msgs::Status::STATUS_STOP;
      planner->reset();
      exploration_state.status.status = nbv_msgs::Status::STATUS_WAIT;
      request_state.status = nbv_msgs::Status::STATUS_WAIT;
      break;
    }
    case nbv_msgs::Status::STATUS_GO_HOME: {
      stop_planner();
      exploration_state.status.status = nbv_msgs::Status::STATUS_STOP;
      std::thread t4(go_home);
      t4.detach();
      request_state.status = nbv_msgs::Status::STATUS_WAIT;
      break;
    }
    }
    loop_rate.sleep();
  }
}

bool manager_srv(nbv_msgs::SetState::Request &req,
                 nbv_msgs::SetState::Response &res) {
  if (req.status.status == nbv_msgs::Status::STATUS_RUN) {
    // Start planner from the beginning (empty maps)
    if (exploration_state.status.status != nbv_msgs::Status::STATUS_RUN and
        exploration_state.status.status != nbv_msgs::Status::STATUS_GO_HOME) {
      request_state.status = nbv_msgs::Status::STATUS_RUN;
      res.success = true;
      res.message = "Running planner";
      return true;
    }
    res.success = false;
    res.message = "Planner is already running or going home";
    return true;
  } else if (req.status.status == nbv_msgs::Status::STATUS_STOP) {
    request_state.status = req.status.status;
    res.success = true;
    res.message = "Stopping Planner";
    return true;
  } else if (req.status.status == nbv_msgs::Status::STATUS_RESET) {
    request_state.status = req.status.status;
    res.success = true;
    res.message = "Planner is restarting";
    return true;
  } else if (req.status.status == nbv_msgs::Status::STATUS_GO_HOME) {
    if (exploration_state.status.status == nbv_msgs::Status::STATUS_RUN or
        exploration_state.status.status == nbv_msgs::Status::STATUS_COMPLETE) {
      request_state.status = req.status.status;
      res.success = true;
      res.message = "Going Home";
      return true;
    }
    res.success = false;
    res.message = "Exploration is not running";
    return true;
  }
  res.success = false;
  res.message = "Status does not exist";
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "nbvePlanner");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  planner = new nbvePlanner<Eigen::Vector4d>(nh, nh_private);
  // PUBLISHERS
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("position_control", 10);
  status_pub = nh.advertise<nbv_msgs::State>("/nbv_planner/state", 10);

  // SUBSCRIBERS
  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

  // ADVERTISER SERVICES
  ros::ServiceServer state_srv =
      nh.advertiseService("/nbv_planner/set_mode", manager_srv);

  // CLIENT SERVICES
  arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  const std::string &ns = ros::this_node::getName();
  nh.param<double>(ns + "/wp_z", wp_z, 1.0);
  nh.param<double>(ns + "/speed_rotate", speed_rotate, 0.5);
  nh.param<double>(ns + "/shift_initial_x", shift_initial_x, 0.0);
  nh.param<double>(ns + "/shift_initial_y", shift_initial_y, 0.0);
  nh.param<double>(ns + "/shift_initial_z", shift_initial_z, 0.0);
  dt = planner->params_.dt_;
  frame_id = planner->params_.navigationFrame_;

  exploration_state.status.status = nbv_msgs::Status::STATUS_WAIT;

  std::thread pose_t(publish_pose);
  std::thread manager_t(private_manager);
  ros::spin();
}

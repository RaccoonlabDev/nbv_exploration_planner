//
// Created by op on 11/25/20.
//

#include "frontier_clustering/frontier_clustering.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "frontierClustering");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  frontiers::FrontierClustering node(nh, nh_private);

  ros::spin();
  return 0;
}
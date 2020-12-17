//
// Created by op on 12/17/20.
//

#ifndef SRC_VIEWPOINTS_H
#define SRC_VIEWPOINTS_H

#include <frontier_clustering/common.h>
#include <random>

namespace frontiers {

/*
 * Class that represents a sampled point in cylindrical coordinate system
 */
class Sample {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Sample(double r, double theta, double z);

  Sample(Pose &pose);

  virtual ~Sample() = default;

 private:
  // Sample position in cylindrical coordinate system (r,theta,z,yaw)
  Pose pose_;
};

/*
 * This class represents the uniformly distributed viewpoints over a
 * cylindrical coordinate system centered at the origin of the cluster
 */
class Viewpoints {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Viewpoints(double radius, double h);
  virtual ~Viewpoints() = default;

  void generateSamples(int n);

 private:
  double radius_;
  double h_;

  AlignedVector<Sample> samples_;

  std::default_random_engine generator_;
  std::uniform_real_distribution<double> radius_distribution_;
  std::uniform_real_distribution<double> theta_distribution_;
  std::uniform_real_distribution<double> z_distribution_;
};

}  // namespace frontiers

#endif  // SRC_VIEWPOINTS_H

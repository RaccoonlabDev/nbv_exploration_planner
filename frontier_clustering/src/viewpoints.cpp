//
// Created by op on 12/17/20.
//

#include "frontier_clustering/viewpoints.h"

namespace frontiers {

Sample::Sample(double r, double theta, double z) {
  pose_.x() = r;
  pose_.y() = theta;
  pose_.z() = z;
}

Sample::Sample(Pose &pose) : pose_(pose) {}

Viewpoints::Viewpoints(double radius, double h)
    : radius_(radius),
      h_(h),
      radius_distribution_(-radius_, radius_),
      theta_distribution_(-M_PI, M_PI),
      z_distribution_(-h_ / 2.0, h_ / 2.0) {}

void Viewpoints::generateSamples(int n) {
  for (int i = 0; i < n; ++i) {
    samples_.emplace_back(Sample(radius_distribution_(generator_),
                                 theta_distribution_(generator_),
                                 z_distribution_(generator_)));
  }
}

}  // namespace frontiers

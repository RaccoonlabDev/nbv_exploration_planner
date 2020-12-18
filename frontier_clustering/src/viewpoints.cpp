//
// Created by op on 12/17/20.
//

#include "frontier_clustering/viewpoints.h"

namespace frontiers {

Viewpoints::Viewpoints() : radius_(0.0), h_(0.0) {}

void Viewpoints::setShape(const double radius, const double h) {
  radius_ = radius;
  h_ = h;
}

void Viewpoints::generateSamples(int n, const double voxel_size,
                                 const Point &center, HighResManager *manager,
                                 AlignedVector<Point> &samples) {
  CHECK_NOTNULL(manager);
  samples.reserve(n);
  Point center_global = (center + Point(0.5, 0.5, 0.5)) * voxel_size;
  Point p;
  double r;
  double theta;
  for (int i = 0; i < n; ++i) {
    r = radius_distribution_(generator_);
    theta = theta_distribution_(generator_);
    p.x() = r * cos(theta) + center_global.x();
    p.y() = r * sin(theta) + center_global.y();
    p.z() = z_distribution_(generator_) + center_global.z();
    if (manager->checkCollisionWithRobotAtVoxel(p) == VoxelStatus::kFree) {
      samples.emplace_back(p);
    }
  }
}

}  // namespace frontiers

//
// Created by op on 12/17/20.
//

#ifndef SRC_VIEWPOINTS_H
#define SRC_VIEWPOINTS_H

#include <frontier_clustering/common.h>
#include <frontier_clustering/voxblox_manager.h>
#include <random>

namespace frontiers {
/*
 * This class represents the uniformly distributed viewpoints over a
 * cylindrical coordinate system centered at the origin of the cluster
 */
class Viewpoints {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Viewpoints();
  virtual ~Viewpoints() = default;

  void setShape(const double radius, const double h);

  void generateSamples(int n, const double voxel_size, const Point& center,
                       HighResManager* manager,
                       AlignedVector<Point>& samples);

  // const AlignedVector<Pose>& samples() const { return samples_; }

  const double& radius() const { return radius_; }

  const double& h() const { return h_; }

 private:
  double radius_;
  double h_;

  // AlignedVector<Pose> samples_;

  std::default_random_engine generator_;
  std::uniform_real_distribution<double> radius_distribution_;
  std::uniform_real_distribution<double> theta_distribution_;
  std::uniform_real_distribution<double> z_distribution_;
};

}  // namespace frontiers

#endif  // SRC_VIEWPOINTS_H

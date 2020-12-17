//
// Created by op on 12/17/20.
//

#ifndef SRC_VIEWPOINTS_H
#define SRC_VIEWPOINTS_H

#include <frontier_clustering/common.h>

/*
 * Class that represents a sampled point in cylindrical coordinate system
 */
class Sample {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Sample();
  virtual ~Sample() = default;

 private:

};

/*
 * This class represents the uniformly distributed viewpoints over a
 * cylindrical coordinate system centered at the origin of the cluster
 */
class Viewpoints {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Viewpoints();
  virtual ~Viewpoints() = default;

 private:

};

#endif  // SRC_VIEWPOINTS_H

#ifndef NBVEPLANNER_COMMON_H
#define NBVEPLANNER_COMMON_H

#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <Eigen/Core>
#include <vector>

namespace nbveplanner {

typedef Eigen::Vector3d Point;
typedef kindr::minimal::QuatTransformationTemplate<double> Transformation;
typedef Eigen::Vector4d Pose;

template <typename Type>
using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type>>;

/*
 * Returns true if all elements of p are smaller or equal than upper_bound and
 * greater or equal than lower_bound
 */
template <typename Type>
inline bool isInsideBounds(const Point &lower_bound, const Point &upper_bound,
                           const Type &p) {
  if (p.x() >= lower_bound.x() and p.x() <= upper_bound.x() and
      p.y() >= lower_bound.y() and p.y() <= upper_bound.y() and
      p.z() >= lower_bound.z() and p.z() <= upper_bound.z()) {
    return true;
  }
  return false;
}
}  // namespace nbveplanner

#endif  // NBVEPLANNER_COMMON_H

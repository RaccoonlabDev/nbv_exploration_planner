//
// Created by op on 05.10.2020.
//

#ifndef SRC_RRT_STAR_H
#define SRC_RRT_STAR_H

#include <kdtree/kdtree.h>
#include "nbveplanner/tree.h"

namespace nbveplanner {

class RRTStar : public TreeBase {
 public:
  RRTStar(VoxbloxManager *manager, VoxbloxManager *manager_lowres,
          Params *params);

  ~RRTStar();

  void setStateFromPoseCovMsg(
      const geometry_msgs::PoseWithCovarianceStamped &pose) override;

  void setStateFromOdometryMsg(const nav_msgs::Odometry &pose) override;

  /*
  void initialize(bool seedHistory) override;

  void iterate() override;
*/
 protected:
  kdtree *kdTree_;
};

}  // namespace nbveplanner
#endif  // SRC_RRT_STAR_H

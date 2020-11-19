#include "nbveplanner/tree.h"

namespace nbveplanner {

Node::Node() {
  id_ = -1;
  parent_ = nullptr;
  distance_ = DBL_MAX;
  num_unmapped_ = 0.0;
  time_to_reach_ = 0.0;
  gain_ = 0.0;
}

Node::~Node() {
  for (auto &it : children_) {
    delete it;
    it = nullptr;
  }
}

TreeBase::TreeBase(HighResManager *manager, LowResManager *manager_lowres,
                   Params *params)
    : manager_(CHECK_NOTNULL(manager)),
      manager_lowres_(CHECK_NOTNULL(manager_lowres)),
      params_(CHECK_NOTNULL(params)) {
  bestGain_ = params_->zero_gain_;
  bestNode_ = nullptr;
  counter_ = 0;
  rootNode_ = nullptr;
}

TreeBase::~TreeBase() = default;

int TreeBase::getCounter() const { return counter_; }

bool TreeBase::gainFound() { return bestGain_ > params_->zero_gain_; }

void TreeBase::setHistRoot(const Pose &root) { hist_root_ = root; }

}  // namespace nbveplanner

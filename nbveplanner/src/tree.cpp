#include "nbveplanner/tree.h"

namespace nbveplanner {

Node::Node() {
  parent_ = nullptr;
  distance_ = DBL_MAX;
  gain_ = 0.0;
}

Node::~Node() {
  for (auto & it : children_) {
    delete it;
    it = nullptr;
  }
}

TreeBase::TreeBase() {
  bestNode_ = nullptr;
  counter_ = 0;
  rootNode_ = nullptr;
  bestGain_ = 0;
}

TreeBase::TreeBase(std::shared_ptr<Params> params)
    : params_(std::move(params)) {
  bestGain_ = params_->zero_gain_;
  bestNode_ = nullptr;
  counter_ = 0;
  rootNode_ = nullptr;
}

TreeBase::~TreeBase() = default;

int TreeBase::getCounter() const {
  return counter_;
}

bool TreeBase::gainFound() {
  return bestGain_ > params_->zero_gain_;
}

void TreeBase::setHistRoot(const Pose &root) {
  hist_root_ = root;
}

}  // namespace nbveplanner

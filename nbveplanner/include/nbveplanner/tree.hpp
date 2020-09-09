/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TREE_HPP_
#define TREE_HPP_

#include <nbveplanner/tree.h>

template <typename stateVec> Node<stateVec>::Node() {
  parent_ = NULL;
  distance_ = DBL_MAX;
  gain_ = 0.0;
}

template <typename stateVec> Node<stateVec>::~Node() {
  for (typename std::vector<Node<stateVec> *>::iterator it = children_.begin();
       it != children_.end(); it++) {
    delete (*it);
    (*it) = NULL;
  }
}

template <typename stateVec> TreeBase<stateVec>::TreeBase() {
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;
  counter_ = 0;
  rootNode_ = NULL;
}

template <typename stateVec>
TreeBase<stateVec>::TreeBase(VoxbloxManager *manager) {
  manager_ = manager;
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;
  counter_ = 0;
  rootNode_ = NULL;
}

template <typename stateVec> TreeBase<stateVec>::~TreeBase() {}

template <typename stateVec>
void TreeBase<stateVec>::setParams(const Params &params) {
  params_ = params;
}

template <typename stateVec>
int TreeBase<stateVec>::getCounter() {
  return counter_;
}

template <typename stateVec>
bool TreeBase<stateVec>::gainFound() {
  return bestGain_ > params_.zero_gain_;
}

template <typename stateVec>
void TreeBase<stateVec>::setHistRoot(
    const Eigen::Vector4d &root) {
  hist_root_ = root;
}

#endif

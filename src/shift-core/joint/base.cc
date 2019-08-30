#include <assert.h>
#include <iostream>

#include "shift-core/context.h"
#include "shift-core/joint/base.h"
#include "shift-core/joint/float.h"
#include "shift-core/joint/prismatic.h"
#include "shift-core/joint/revolute.h"
#include "shift-core/joint/rigid.h"
#include "shift-core/joint/spherical.h"

bool Shift::Joint::Modules::initialized() {
  if(!Shift::Joint::Float::isRegistered()) {
    return false;
  }
  if(!Shift::Joint::Prismatic::isRegistered()) {
    return false;
  }
  if(!Shift::Joint::Revolute::isRegistered()) {
    return false;
  }
  if(!Shift::Joint::Rigid::isRegistered()) {
    return false;
  }
  if(!Shift::Joint::Spherical::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Joint::Base::addParameters() {
  addParameter("name", &name_, "descriptive joint name", true);
  addParameter("type", &type_, "joint type", true);
  addParameter("parent", &parent_, "parent body/frame pair", true)
    .setRowDim(2)
    .setColumnDim(-1);
  addParameter("child", &child_, "child body/frame pair", true)
    .setRowDim(2)
    .setColumnDim(-1);
}

void Shift::Joint::Base::load() {
  // connect();
}

void Shift::Joint::Base::connect() {

  if(context_ && parent_.size() > 0 && child_.size() > 0) {
    // connect parent

    if(parent_.at(0).size() == 1 && parent_.at(0).at(0) != "fixed") {
      /* name is not a fully qualified name */
      std::vector<std::string> name = fully_qualified_name_;
      name.erase(name.end() - 1);
      name.push_back(parent_.at(0).at(0));
      // std::cout << "looking for parent body: [ ";
      // for(auto& i : name) {
      //   std::cout << i << " ";
      // }
      // std::cout << "]" << std::endl;
      parentBodyPointer_ = context_->getBody(name);
    } else {
      /* name is a fully qualified name */
      // std::cout << "looking for parent body: [ ";
      // for(auto& i : parent_.at(0)) {
      //   std::cout << i << " ";
      // }
      // std::cout << "]" << std::endl;
      parentBodyPointer_ = context_->getBody(parent_.at(0));
    }

    if(!parentBodyPointer_) {
      std::cerr << "PARENT BODY NOT FOUND!" << std::endl;
    } else {
      parentFramePointer_ = parentBodyPointer_->getFrame(parent_.at(1).at(0));
      if(!parentFramePointer_) {
        std::cerr << "PARENT FRAME NOT FOUND!" << std::endl;
      }
    }

    if(child_.at(0).size() == 1 && child_.at(0).at(0) != "fixed") {
      /* child body name is not a fully qualified name */
      std::vector<std::string> name = fully_qualified_name_;
      name.erase(name.end() - 1);
      name.push_back(child_.at(0).at(0));
      // std::cout << "looking for child body: [ ";
      // for(auto& i : name) {
      //   std::cout << i << " ";
      // }
      // std::cout << "]" << std::endl;
      childBodyPointer_ = context_->getBody(name);
    } else {
      /* child name is already a fully qualified name */
      // std::cout << "looking for child body: [ ";
      // for(auto& i : child_.at(0)) {
      //   std::cout << i << " ";
      // }
      // std::cout << "]" << std::endl;
      childBodyPointer_ = context_->getBody(child_.at(0));
    }
    if(!childBodyPointer_) {
      std::cerr << "CHILD BODY NOT FOUND!" << std::endl;
      assert(0);
    } else {
      childFramePointer_ = childBodyPointer_->getFrame(child_.at(1).at(0));
      if(!childFramePointer_) {
        std::cerr << "CHILD FRAME NOT FOUND!" << std::endl;
        assert(0);
      }
    }
  }
}

void Shift::Joint::Base::setInitialDisplacement(const double* q) {
  if(q_0_) {
    memcpy(q_0_, q, qCount_ * sizeof(double));
  }
}

void Shift::Joint::Base::setInitialVelocity(const double* dq) {
  if(dq_0_) {
    memcpy(dq_0_, dq, dqCount_ * sizeof(double));
  }
}

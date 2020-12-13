#include <assert.h>
#include <iostream>

#include "shift/core/Context.h"
#include "shift/core/Exception.h"
#include "shift/core/joint/Base.h"
#include "shift/core/joint/Float.h"
#include "shift/core/joint/Prismatic.h"
#include "shift/core/joint/Revolute.h"
#include "shift/core/joint/Rigid.h"
#include "shift/core/joint/Spherical.h"

bool shift::core::joint::Modules::initialized() {
  if(!shift::core::joint::Float::isRegistered()) {
    return false;
  }
  if(!shift::core::joint::Prismatic::isRegistered()) {
    return false;
  }
  if(!shift::core::joint::Revolute::isRegistered()) {
    return false;
  }
  if(!shift::core::joint::Rigid::isRegistered()) {
    return false;
  }
  if(!shift::core::joint::Spherical::isRegistered()) {
    return false;
  }
  return true;
}

void shift::core::joint::Base::addParameters() {
  addParameter("name", &name_, "descriptive joint name", true);
  addParameter("type", &type_, "joint type", true);
  addParameter("parent", &parent_, "parent body/frame pair", true)
    .setRowDim(2)
    .setColumnDim(-1);
  addParameter("child", &child_, "child body/frame pair", true)
    .setRowDim(2)
    .setColumnDim(-1);
}

void shift::core::joint::Base::load() {
  // connect();
}

void shift::core::joint::Base::connect() {

  if(context_ && parent_.size() > 0 && child_.size() > 0) {
    // connect parent
    if(parent_.at(0).size() == 1) {
      if(parent_.at(0).at(0) == "fixed") {
        parent_body_ptr_ = context_->fixedBody();
      } else {
        if(parent_assembly_ptr_ == nullptr) {
          throw shift::core::Exception("No parent assembly set for joint " + name());
        }
        try {
          parent_body_ptr_ =
            (*parent_assembly_ptr_)["bodies"][parent_.at(0).at(0)]
              .data<shift::core::body::Base>();
        } catch(const char* msg) {
          std::cerr << "Error: could not find parent body for joint: " << name()
                    << ". Loading the simulation will attempt to continue."
                    << std::endl;
          std::cerr << msg << std::endl;
          parent_frame_ptr_ = nullptr;
          active_ = false;
        }
      }
    } else if(parent_.at(0).at(0) == "fixed") {
      assert(context_);
      parent_body_ptr_ = context_->fixedBody();
    } else {
      assert(context_);
      try {
        auto& value = (*context_)[parent_.at(0).at(0)];
        for(auto it = parent_.at(0).begin() + 1; it <= parent_.at(0).end();
            it++) {
          value = value[*it];
        }
        parent_body_ptr_ = value.data<shift::core::body::Base>();
      } catch(const char* msg) {
        std::cerr << "Error: could not find parent body for joint: " << name()
                  << ". Loading the simulation will attempt to continue."
                  << std::endl;
        std::cerr << msg << std::endl;
        parent_frame_ptr_ = nullptr;
        active_ = false;
      }
    }

    if(!parent_body_ptr_) {
      std::cerr << "Error: PARENT BODY NOT FOUND!" << std::endl;
    } else {
      try {
        parent_frame_ptr_ = (*parent_body_ptr_)["frames"][parent_.at(1).at(0)]
                              .data<shift::core::Frame>();
      } catch(const char* msg) {
        std::cerr << "Could not find parent frame for joint: " << name()
                  << std::endl;
        std::cerr << msg << std::endl;
        parent_frame_ptr_ = nullptr;
        active_ = false;
      }
      if(!parent_frame_ptr_) {
        std::cerr << "Error: PARENT FRAME NOT FOUND!" << std::endl;
      }
    }

    if(child_.at(0).size() == 1) {
      if(child_.at(0).at(0) == "fixed") {
        child_body_ptr_ = context_->fixedBody();
      } else {
        assert(parent_assembly_ptr_);
        try {
          child_body_ptr_ =
            (*parent_assembly_ptr_)["bodies"][child_.at(0).at(0)]
              .data<shift::core::body::Base>();
        } catch(const char* msg) {
          std::cerr << "Could not find child body \"" << child_.at(0).at(0)
                    << "\" for joint: \"" << name() << "\"" << std::endl;
          std::cerr << msg << std::endl;
          parent_frame_ptr_ = nullptr;
          active_ = false;
        }
      }
    } else {
      assert(context_);
      auto& value = (*context_)[child_.at(0).at(0)];
      for(auto it = child_.at(0).begin() + 1; it <= parent_.at(0).end(); it++) {
        value = value[*it];
      }
      try {
        child_body_ptr_ = value.data<shift::core::body::Base>();
      } catch(const char* msg) {
        std::cerr << "Could not find child body \"" << child_.at(0).at(0)
                  << "\" for joint: \"" << name() << "\"" << std::endl;
        std::cerr << msg << std::endl;
        child_body_ptr_ = nullptr;
        active_ = false;
      }
    }
    if(!child_body_ptr_) {
      std::cerr << "CHILD BODY NOT FOUND!" << std::endl;
      assert(0);
    } else {
      try {
        child_frame_ptr_ =
          (*child_body_ptr_)["frames"][child_.at(1).at(0)].data<shift::core::Frame>();
      } catch(const char* msg) {
        std::cerr << "Could not find child frame \"" << child_.at(1).at(0)
                  << "\" for joint: \"" << name() << "\"" << std::endl;
        std::cerr << msg << std::endl;
        child_frame_ptr_ = nullptr;
        active_ = false;
      }
      if(!child_frame_ptr_) {
        std::cerr << "CHILD FRAME NOT FOUND!" << std::endl;
        assert(0);
      }
    }
  }
}

void shift::core::joint::Base::setInitialDisplacement(const double* q) {
  if(q_0_) {
    memcpy(q_0_, q, q_count_ * sizeof(double));
  }
}

void shift::core::joint::Base::setInitialVelocity(const double* dq) {
  if(dq_0_) {
    memcpy(dq_0_, dq, dq_count_ * sizeof(double));
  }
}

void shift::core::joint::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}

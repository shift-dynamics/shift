#include <assert.h>
#include <iostream>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/joint/base.h"
#include "shift/core/joint/float.h"
#include "shift/core/joint/prismatic.h"
#include "shift/core/joint/revolute.h"
#include "shift/core/joint/rigid.h"
#include "shift/core/joint/spherical.h"

bool shift::core::joint::Modules::is_initialized() {
  if(!shift::core::joint::Float::is_registered()) {
    return false;
  }
  if(!shift::core::joint::Prismatic::is_registered()) {
    return false;
  }
  if(!shift::core::joint::Revolute::is_registered()) {
    return false;
  }
  if(!shift::core::joint::Rigid::is_registered()) {
    return false;
  }
  if(!shift::core::joint::Spherical::is_registered()) {
    return false;
  }
  return true;
}

void shift::core::joint::Base::DeclareParameters() {
  DeclareParameter("name", &name_, "descriptive joint name", true);
  DeclareParameter("type", &type_, "joint type", true);
  DeclareParameter("parent", &parent_, "parent body/frame pair", true)
    .set_row_dim(2)
    .set_column_dim(-1);
  DeclareParameter("child", &child_, "child body/frame pair", true)
    .set_row_dim(2)
    .set_column_dim(-1);
}

void shift::core::joint::Base::Load() {
  // Connect();
}

void shift::core::joint::Base::Connect() {

  if(context_ && parent_.size() > 0 && child_.size() > 0) {
    // connect parent
    if(parent_.at(0).size() == 1) {
      if(parent_.at(0).at(0) == "fixed") {
        parent_body_ptr_ = context_->get_fixed_body();
      } else {
        if(parent_assembly_ptr_ == nullptr) {
          throw shift::core::Exception("No parent assembly set for joint " + get_name());
        }
        try {
          parent_body_ptr_ =
            (*parent_assembly_ptr_)["bodies"][parent_.at(0).at(0)]
              .get_data<shift::core::body::Base>();
        } catch(const char* msg) {
          std::cerr << "Error: could not find parent body for joint: " << get_name()
                    << ". Loading the simulation will attempt to continue."
                    << std::endl;
          std::cerr << msg << std::endl;
          parent_frame_ptr_ = nullptr;
          active_ = false;
        }
      }
    } else if(parent_.at(0).at(0) == "fixed") {
      assert(context_);
      parent_body_ptr_ = context_->get_fixed_body();
    } else {
      assert(context_);
      try {
        auto* value = &(*context_)[parent_.at(0).at(0)];
        for(auto it = parent_.at(0).begin() + 1; it != parent_.at(0).end();
            it++) {
          value = &(*value)[*it];
        }
        parent_body_ptr_ = value->get_data<shift::core::body::Base>();
      } catch(const char* msg) {
        std::cerr << "Error: could not find parent body for joint: " << get_name()
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
                              .get_data<shift::core::Frame>();
      } catch(const char* msg) {
        std::cerr << "Could not find parent frame for joint: " << get_name()
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
        child_body_ptr_ = context_->get_fixed_body();
      } else {
        assert(parent_assembly_ptr_);
        try {
          child_body_ptr_ =
            (*parent_assembly_ptr_)["bodies"][child_.at(0).at(0)]
              .get_data<shift::core::body::Base>();
        } catch(const char* msg) {
          std::cerr << "Could not find child body \"" << child_.at(0).at(0)
                    << "\" for joint: \"" << get_name() << "\"" << std::endl;
          std::cerr << msg << std::endl;
          parent_frame_ptr_ = nullptr;
          active_ = false;
        }
      }
    } else {
      assert(context_);
      auto* value = &(*context_)[child_.at(0).at(0)];
      for(auto it = child_.at(0).begin() + 1; it != child_.at(0).end(); it++) {
        value = &(*value)[*it];
      }
      try {
        child_body_ptr_ = value->get_data<shift::core::body::Base>();
      } catch(const char* msg) {
        std::cerr << "Could not find child body \"" << child_.at(0).at(0)
                  << "\" for joint: \"" << get_name() << "\"" << std::endl;
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
          (*child_body_ptr_)["frames"][child_.at(1).at(0)].get_data<shift::core::Frame>();
      } catch(const char* msg) {
        std::cerr << "Could not find child frame \"" << child_.at(1).at(0)
                  << "\" for joint: \"" << get_name() << "\"" << std::endl;
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

void shift::core::joint::Base::set_initial_displacement(const double* q) {
  if(q_0_) {
    memcpy(q_0_, q, q_count_ * sizeof(double));
  }
}

void shift::core::joint::Base::set_initial_velocity(const double* dq) {
  if(dq_0_) {
    memcpy(dq_0_, dq, dq_count_ * sizeof(double));
  }
}

void shift::core::joint::Base::ActivateObjects(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, parent_is_active && active_);
}

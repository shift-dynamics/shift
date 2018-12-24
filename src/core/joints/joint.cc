#include <iostream>

#include "shift-core/context.h"
#include "shift-core/joints/float.h"
#include "shift-core/joints/joint.h"
#include "shift-core/joints/prismatic.h"
#include "shift-core/joints/revolute.h"
#include "shift-core/joints/rigid.h"
#include "shift-core/joints/spherical.h"

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

void Shift::Joint::JointBase::addKeys() {
  SerialEntry name("name", SerialType::STRING, &name_);
  addSerialEntry(name);
  SerialEntry type("type", SerialType::STRING, &type_);
  addSerialEntry(type);
  SerialEntry parent("parent", SerialType::STRING_2DARRAY, &parent_, 2, -1);
  addSerialEntry(parent);
  SerialEntry child("child", SerialType::STRING_2DARRAY, &child_, 2, -1);
  addSerialEntry(child);
}

void Shift::Joint::JointBase::readJSON(Json::Value& value) {
  Shift::Serialize::readJSON(value);
  if(context_) {
    context_->setActive(this);
  }
}

void Shift::Joint::JointBase::print() {
  std::cout << "name: " << name_ << std::endl;
  std::cout << "type: " << type_ << std::endl;
  std::cout << "dof: " << dof_ << std::endl;
  std::cout << "parent: [";
  for(unsigned int i = 0; i < parent_.size(); i++) {
    std::cout << "[";
    for(unsigned int j = 0; j < parent_[i].size(); j++) {
      std::cout << parent_[i][j];
      if(j != parent_[i].size() - 1) {
        std::cout << ", ";
      }
    }
    std::cout << "]";
    if(i != parent_.size() - 1) {
      std::cout << ", ";
    }
  }
  std::cout << "]" << std::endl;

  std::cout << "child: [";
  for(unsigned int i = 0; i < child_.size(); i++) {
    std::cout << "[";
    for(unsigned int j = 0; j < child_[i].size(); j++) {
      std::cout << child_[i][j];
      if(j != child_[i].size() - 1) {
        std::cout << ", ";
      }
    }
    std::cout << "]";
    if(i != child_.size() - 1) {
      std::cout << ", ";
    }
  }
  std::cout << "]" << std::endl;
}

void Shift::Joint::JointBase::load() {
  // connect();
}

void Shift::Joint::JointBase::connect() {
  std::cout << "connecting joint: " << name_ << std::endl;
  if(context_) {
    // connect parent
    std::cout << "conecting parent body / frame" << std::endl;
    if(parentBody_.size() == 1) {
      /* name is not a fully qualified name */
      std::vector<std::string> name = fullyQualifiedName_;
      name.erase(name.end() - 1);
      name.push_back(parentBody_.at(0));
      parentBodyPointer_ = context_->getBody(name);
    } else {
      /* name is a fully qualified name */
      parentBodyPointer_ = context_->getBody(parentBody_);
    }
    if(parentBodyPointer_) {
      std::cout << "parent body: " << parentBodyPointer_->getName()
                << std::endl;
    } else {
      std::cout << "PARENT BODY NOT FOUND!" << std::endl;
    }

    std::cout << "conecting child body / frame" << std::endl;
    if(childBody_.size() == 1) {
      /* name is not a fully qualified name */
      std::vector<std::string> name = fullyQualifiedName_;
      name.erase(name.end() - 1);
      name.push_back(parentBody_.at(0));
      childBodyPointer_ = context_->getBody(name);
    } else {
      /* name is a fully qualified name */
      childBodyPointer_ = context_->getBody(childBody_);
    }
    if(childBodyPointer_) {
      std::cout << "child body: " << childBodyPointer_->getName() << std::endl;
    } else {
      std::cout << "CHILD BODY NOT FOUND!" << std::endl;
    }
  }
}

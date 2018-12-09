#include "shift-core/assembly.h"
#include "shift-core/behaviors/behavior.h"
#include "shift-core/bodies/body.h"
#include "shift-core/bodies/fixed.h"
#include "shift-core/factory.h"

#include <iostream>

void Shift::Assembly::printBehaviors() {}

void Shift::Assembly::printBodies() {
  for(unsigned int i = 0; i < bodies_.size(); i++) {
    bodies_[i]->print();
  }
}

void Shift::Assembly::printConstraints() {}

void Shift::Assembly::printForceModules() {}

void Shift::Assembly::printJoints() {}

void Shift::Assembly::print() {
  std::cout << "assembly: " << name_ << std::endl;

  printBehaviors();
  printBodies();
  printConstraints();
  printForceModules();
  printJoints();

  /* print subassemblies */
  for(unsigned int i = 0; i < subassemblies_.size(); i++) {
    subassemblies_[i].print();
  }
}

void Shift::Assembly::addKeys() {
  Shift::SerialEntry name("name", Shift::SerialType::STRING, &name_);
  name.setRequired(true);
  name.setDescription("User defined descriptive assembly name.");
  addSerialEntry(name);

  Shift::SerialEntry active("active", Shift::SerialType::BOOLEAN, &active_);
  active.setRequired(false);
  active.setDescription(
    "Indicates whether current assembly is active in simulation: If "
    "set to false, the data will be loaded into computer working memory,"
    "but will not be processed when setting up and solving the equations of"
    "motion. Use this key to selectively enable or disable portions of the"
    "model.");
  addSerialEntry(active);
}

void Shift::Assembly::readJSON(Json::Value& value, bool fixed) {

  if(fixed) {
    if(value.isMember("fixed")) {
      setName("fixed");
    }

    Shift::Body::BodyBase* body =
      Shift::Factory<Shift::Body::BodyBase>::get()->create("fixed");
    try {
      body->readJSON(value["fixed"]);
    } catch(const char* msg) {
      std::cerr << msg << std::endl;
    }
    bodies_.push_back(body);
  } else {

    if(value["behaviors"] && value["behaviors"].isArray()) {
      for(unsigned int i = 0; i < value["behaviors"].size(); ++i) {
        if(value["behaviors"][i].isMember("type") &&
           value["behaviors"][i]["type"].isString()) {
          std::string type = value["behaviors"][i]["type"].asString();
          Behavior::BehaviorBase* behavior =
            Shift::Factory<Shift::Behavior::BehaviorBase>::get()->create(type);
          if(behavior) {
            try {
              behavior->readJSON(value["behaviors"][i]);
            } catch(const char* msg) {
              std::cerr << msg << std::endl;
            }
            behaviors_.insert(behaviors_.end(), behavior);
          }
        }
      }
    }

    if(value.isMember("bodies") && value["bodies"].isArray()) {
      for(unsigned int i = 0; i < value["bodies"].size(); ++i) {
        std::string bodyType("rigid");
        if(value["bodies"][i].isMember("type") &&
           value["bodies"][i]["type"].isString()) {
          bodyType.assign(value["bodies"][i]["type"].asString());
        }
        Shift::Body::BodyBase* body =
          Shift::Factory<Shift::Body::BodyBase>::get()->create(bodyType);
        if(body) {
          try {
            body->readJSON(value["bodies"][i]);
          } catch(const char* msg) {
            std::cerr << msg << std::endl;
          }
          bodies_.push_back(body);
        }
      }
    }

    if(value.isMember("constraints") && value["constraints"].isArray()) {
      for(unsigned int i = 0; i < value["constraints"].size(); ++i) {
        if(value["constraints"][i].isMember("type") &&
           value["constraints"][i]["type"].isString()) {
          std::string constraintType(
            value["constraints"][i]["type"].asString());
          Shift::Constraint::ConstraintBase* constraint =
            Shift::Factory<Shift::Constraint::ConstraintBase>::get()->create(
              constraintType);
          if(constraint) {
            try {
              constraint->readJSON(value["constraints"][i]);
            } catch(const char* msg) {
              std::cerr << msg << std::endl;
            }
            constraints_.push_back(constraint);
          }
        }
      }
    }

    if(value.isMember("force_modules") && value["force_modules"].isArray()) {
      for(unsigned int i = 0; i < value["force_modules"].size(); ++i) {
        if(value["force_modules"][i].isMember("type") &&
           value["force_modules"][i]["type"].isString()) {
          std::string forceModuleType(
            value["force_modules"][i]["type"].asString());
          Shift::ForceModule::ForceModuleBase* forceModule =
            Shift::Factory<Shift::ForceModule::ForceModuleBase>::get()->create(
              forceModuleType);
          if(forceModule) {
            try {
              forceModule->readJSON(value["force_modules"][i]);
            } catch(const char* msg) {
              std::cerr << msg << std::endl;
            }
            forcemodules_.push_back(forceModule);
          }
        }
      }
    }

    if(value.isMember("joints") && value["joints"].isArray()) {
      for(unsigned int i = 0; i < value["joints"].size(); ++i) {
        if(value["joints"][i].isMember("type") &&
           value["joints"][i]["type"].isString()) {
          std::string jointType(value["joints"][i]["type"].asString());
          Shift::Joint::JointBase* joint =
            Shift::Factory<Shift::Joint::JointBase>::get()->create(jointType);
          if(joint) {
            try {
              joint->readJSON(value["joints"][i]);
            } catch(const char* msg) {
              std::cerr << msg << std::endl;
            }
            joints_.push_back(joint);
          }
        }
      }
    }
  }

  /* read subassemblies */
  if(value.isMember("assemblies") && value["assemblies"].isArray()) {
    std::cout << "reading assemblies" << std::endl;
    for(unsigned int i = 0; i < value["assemblies"].size(); ++i) {
      Assembly assembly;
      assembly.readJSON(value["assemblies"][i]);
      subassemblies_.push_back(assembly);
    }
  }
}

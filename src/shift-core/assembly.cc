#include "shift-core/assembly.h"
#include "shift-core/behavior/base.h"
#include "shift-core/body/base.h"
#include "shift-core/body/fixed.h"
#include "shift-core/context.h"
#include "shift-core/factory.h"

#include <iostream>

void Shift::Assembly::printBehaviors() {}

void Shift::Assembly::printBodies() {
  for(auto& i : bodies_) {
    i->print();
  }
}

void Shift::Assembly::printConstraints() {}

void Shift::Assembly::printForceModules() {}

void Shift::Assembly::printJoints() {
  for(auto& i : joints_) {
    i->print();
  }
}

void Shift::Assembly::print() {
  std::cout << "assembly: " << name_ << std::endl;

  printBehaviors();
  printBodies();
  printConstraints();
  printForceModules();
  printJoints();

  /* print subassemblies */
  for(auto& i : subassemblies_) {
    i->print();
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
    auto body = Shift::Factory<Shift::Body::Base>::get()->create("fixed");
    body->setName("fixed");
    try {
      body->setContext(context_);
      body->readJSON(value["fixed"]);
    } catch(const char* msg) {
      std::cerr << msg << std::endl;
    }
    bodies_.push_back(std::move(body));
  } else {

    try {
      Shift::Serialize::readJSON(value);
    } catch(const char* msg) {
      std::cerr << msg << std::endl;
    }

    if(value.isMember("behaviors") && value["behaviors"].isArray()) {
      for(auto i : value["behaviors"]) {
        if(i.isMember("type") && i["type"].isString()) {
          std::string type = i["type"].asString();
          auto behavior =
            Shift::Factory<Shift::Behavior::Base>::get()->create(type);
          if(behavior) {
            try {
              behavior->setContext(context_);
              behavior->readJSON(i);
            } catch(const char* msg) {
              std::cerr << msg << std::endl;
            }
            // behavior->setContext(context_);
            // behavior->setAssembly(this);
            behaviors_.push_back(std::move(behavior));
          }
        }
      }
    }

    if(value.isMember("bodies") && value["bodies"].isArray()) {
      for(auto& i : value["bodies"]) {
        std::string bodyType("rigid");
        if(i.isMember("type") && i["type"].isString()) {
          bodyType.assign(i["type"].asString());
        }
        auto body = Shift::Factory<Shift::Body::Base>::get()->create(bodyType);
        if(body) {
          try {
            body->setContext(context_);
            body->readJSON(i);
          } catch(const char* msg) {
            std::cerr << msg << std::endl;
          }
          body->setContext(context_);
          body->setAssembly(this);
          bodies_.push_back(std::move(body));
        }
      }
    }

    if(value.isMember("constraints") && value["constraints"].isArray()) {
      for(auto& i : value["constraints"]) {
        if(i.isMember("type") && i["type"].isString()) {
          std::string constraintType(i["type"].asString());
          auto constraint =
            Shift::Factory<Shift::Constraint::Base>::get()->create(
              constraintType);
          if(constraint) {
            try {
              constraint->setContext(context_);
              constraint->readJSON(i);
            } catch(const char* msg) {
              std::cerr << msg << std::endl;
            }
            // constraint->setContext(context_);
            // constraint->setAssembly(this);
            constraints_.push_back(std::move(constraint));
          }
        }
      }
    }

    if(value.isMember("force_modules") && value["force_modules"].isArray()) {
      for(auto& i : value["force_modules"]) {
        if(i.isMember("type") && i["type"].isString()) {
          std::string forceModuleType(i["type"].asString());
          auto forceModule =
            Shift::Factory<Shift::ForceModule::Base>::get()->create(
              forceModuleType);
          if(forceModule) {
            try {
              forceModule->setContext(context_);
              forceModule->readJSON(i);
            } catch(const char* msg) {
              std::cerr << msg << std::endl;
            }
            // forceModule->setContext(context_);
            // forceModule->setAssembly(this);
            forcemodules_.push_back(std::move(forceModule));
          }
        }
      }
    }

    if(value.isMember("joints") && value["joints"].isArray()) {
      for(auto& i : value["joints"]) {
        if(i.isMember("type") && i["type"].isString()) {
          std::string jointType(i["type"].asString());
          auto joint =
            Shift::Factory<Shift::Joint::Base>::get()->create(jointType);
          if(joint) {
            try {
              joint->setContext(context_);
              joint->readJSON(i);
            } catch(const char* msg) {
              std::cerr << msg << std::endl;
            }
            joint->setAssembly(this);
            joints_.push_back(std::move(joint));
          }
        }
      }
    }
  }

  /* read subassemblies */
  if(value.isMember("assemblies") && value["assemblies"].isArray()) {
    for(auto& i : value["assemblies"]) {
      auto assembly = std::make_unique<Assembly>();
      assembly->setContext(context_);
      assembly->readJSON(i);
      subassemblies_.push_back(std::move(assembly));
    }
  }
}

void Shift::Assembly::setFullyQualifiedNames(
  std::vector<std::string>& baseName) {
  setFullyQualifiedName(baseName);
  auto assemblyName = getFullyQualifiedName();

  for(auto& i : behaviors_) {
    i->setFullyQualifiedName(assemblyName);
  }

  for(auto& i : bodies_) {
    i->setFullyQualifiedNames(assemblyName);
  }

  for(auto& i : constraints_) {
    i->setFullyQualifiedName(assemblyName);
  }

  for(auto& i : forcemodules_) {
    i->setFullyQualifiedName(assemblyName);
  }

  for(auto& i : joints_) {
    i->setFullyQualifiedName(assemblyName);
  }

  for(auto& i : subassemblies_) {
    i->setFullyQualifiedNames(assemblyName);
  }
}

void Shift::Assembly::setActiveObjects(bool parentIsActive) {
  if(active_ && context_ && parentIsActive) {
    for(auto& i : behaviors_) {
      context_->setActive(i.get());
    }
    for(auto& i : bodies_) {
      context_->setActive(i.get());
      i->setActiveObjects(parentIsActive);
    }
    for(auto& i : constraints_) {
      context_->setActive(i.get());
    }
    for(auto& i : forcemodules_) {
      context_->setActive(i.get());
    }
    for(auto& i : joints_) {
      context_->setActive(i.get());
    }
  } else {
    if(context_) {
      for(auto& i : behaviors_) {
        context_->removeActive(i.get());
      }
      for(auto& i : bodies_) {
        context_->removeActive(i.get());
      }
      for(auto& i : constraints_) {
        context_->removeActive(i.get());
      }
      for(auto& i : forcemodules_) {
        context_->removeActive(i.get());
      }
      for(auto& i : joints_) {
        context_->removeActive(i.get());
      }
    }
  }

  for(auto& i : subassemblies_) {
    i->setActiveObjects(parentIsActive && active_);
  }
}

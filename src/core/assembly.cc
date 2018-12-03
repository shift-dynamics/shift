#include "shift-core/assembly.h"
#include "shift-core/behaviors/behavior.h"
#include "shift-core/factory.h"

void Shift::Assembly::addKeys() {
  SerialEntry name("name", SerialType::STRING, &name_);
  name.setRequired(true);
  name.setDescription("User defined descriptive assembly name.");
  addSerialEntry(name);

  SerialEntry active("active", SerialType::BOOLEAN, &active_);
  active.setRequired(false);
  active.setDescription(
    "Indicates whether current assembly is active in simulation: If the value "
    "is set to false, the data will be loaded into computer working memory, "
    "but will not be processed when setting up and solving the equations of "
    "motion. Use this key to selectively enable or disable portions of the "
    "model.");
  addSerialEntry(active);
}

void Shift::Assembly::readJSON(Json::Value& value, bool fixed) {

  if(value["assemblies"] && value["assemblies"].isArray()) {
    for(int i = 0; i < value["assemblies"].size(); ++i) {
      Assembly assembly;
      assembly.readJSON(value["assemblies"][i]);
      subassemblies_.insert(subassemblies_.end(), assembly);
    }
  }

  if(value["behaviors"] && value["behaviors"].isArray()) {
    for(int i = 0; i < value["behaviors"].size(); ++i) {
      if(value["behaviors"][i]["type"] &&
         value["behaviors"][i]["type"].isString()) {
        std::string type = value["behaviors"][i]["type"].asString();
        Behavior::BehaviorBase* behavior =
          Shift::Factory<Behavior::BehaviorBase>::create(type);
        behavior->readJSON(value["behaviors"][i]);
        behaviors_.insert(behaviors_.end(), behavior);
      }
    }
  }

  // if(value["assemblies"] && value["assemblies"].isArray()) {
  //   for(int i = 0; i < value["assemblies"].size(); ++i) {
  //     Assembly assembly;
  //     assembly.readJSON(value["assemblies"][i]);
  //     subassemblies_.insert(subassemblies_.end(), assembly);
  //   }
  // }
  //
  // if(value["assemblies"] && value["assemblies"].isArray()) {
  //   for(int i = 0; i < value["assemblies"].size(); ++i) {
  //     Assembly assembly;
  //     assembly.readJSON(value["assemblies"][i]);
  //     subassemblies_.insert(subassemblies_.end(), assembly);
  //   }
  // }
  //
  // if(value["assemblies"] && value["assemblies"].isArray()) {
  //   for(int i = 0; i < value["assemblies"].size(); ++i) {
  //     Assembly assembly;
  //     assembly.readJSON(value["assemblies"][i]);
  //     subassemblies_.insert(subassemblies_.end(), assembly);
  //   }
  // }

  // value["behaviors"] "behaviors" std::vector<Behavior::BehaviorBase*>
  //   behaviors_;
  // "bodies" std::vector<Body::BodyBase*> bodies_;
  // "constraints" std::vector<Constraint::ConstraintBase*> constraints_;
  // "force_modules" std::vector<ForceModule::ForceModuleBase*> forcemodules_;
  // "joints" std::vector<Joint::JointBase*> joints_;
  // "lights" std::vector<Light::LightBase*> lights_;
}

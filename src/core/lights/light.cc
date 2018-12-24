#include "shift-core/lights/light.h"
#include "shift-core/context.h"
#include "shift-core/lights/directional.h"
#include "shift-core/lights/ies.h"
#include "shift-core/lights/point.h"
#include "shift-core/lights/spot.h"

#include "ksl/print.h"

bool Shift::Light::Modules::initialized() {
  if(!Shift::Light::Point::isRegistered()) {
    return false;
  }
  if(!Shift::Light::Spot::isRegistered()) {
    return false;
  }
  if(!Shift::Light::Directional::isRegistered()) {
    return false;
  }
  if(!Shift::Light::IES::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Light::LightBase::readJSON(Json::Value& value) {
  Shift::Serialize::readJSON(value);
  if(context_) {
    context_->setActive(this);
  }
}

void Shift::Light::LightBase::addKeys() {
  SerialEntry name("name", SerialType::STRING, &name_);
  name.setRequired(true);
  name.setDescription("User defined descriptive light name.");
  addSerialEntry(name);

  SerialEntry active("active", SerialType::BOOLEAN, &active_);
  active.setRequired(false);
  active.setDescription(
    "Indicates whether light is active in simulation: If the value is set to "
    "false, the data will be loaded into computer working memory, but will not "
    "be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model.");
  addSerialEntry(active);

  SerialEntry lightType("type", SerialType::STRING, &type_);
  lightType.setRequired(true);
  lightType.setDescription("Indicates type of light.");
  addSerialEntry(lightType);

  SerialEntry parent("parent", SerialType::STRING_ARRAY, &parent_);
  parent.setRequired(true);
  parent.setDescription(
    "Indicates light's parent body and frame using the syntax [\"Body\", "
    "\"Frame\"]");
  addSerialEntry(parent);

  SerialEntry Ka("Ka", SerialType::KSL_VEC3, &Ka_);
  Ka.setRequired(false);
  Ka.setDescription("Indicates light's ambient color component in RGB format "
                    "using values from 0 to 1. ");
  addSerialEntry(Ka);

  SerialEntry Kd("Ka", SerialType::KSL_VEC3, &Kd_);
  Kd.setRequired(false);
  Kd.setDescription("Indicates light's diffuse color component in RGB format "
                    "using values from 0 to 1. ");
  addSerialEntry(Kd);

  SerialEntry Ks("Ka", SerialType::KSL_VEC3, &Ks_);
  Ks.setRequired(false);
  Ks.setDescription("Indicates light's specular color component in RGB format "
                    "using values from 0 to 1. ");
  addSerialEntry(Ks);
}

void Shift::Light::LightBase::print() {
  std::cout << "  name: " << name_ << std::endl;
  std::cout << "  type: " << type_ << std::endl;
  std::cout << "  parent: [";
  for(unsigned int i = 0; i < parent_.size(); i++) {
    std::cout << parent_[i];
    if(i != parent_.size() - 1) {
      std::cout << ", ";
    }
  }
  std::cout << "]" << std::endl;
  ksl_vec3_print(stdout, &Ka_, "  Ka: ");
  ksl_vec3_print(stdout, &Kd_, "  Kd: ");
  ksl_vec3_print(stdout, &Ks_, "  Ks: ");
}

void Shift::Light::LightBase::load() {}

void Shift::Light::LightBase::connect() {}

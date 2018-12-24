#include "shift-core/behaviors/contactdetection.h"
#include "shift-core/factory.h"

bool Shift::Behavior::ContactDetection::registered_ =
  Shift::Factory<Shift::Behavior::BehaviorBase>::get()->registerModule(
    "contact_detection", Shift::Behavior::ContactDetection::create);

std::unique_ptr<Shift::Behavior::ContactDetection>
Shift::Behavior::ContactDetection::create() {
  return std::make_unique<Shift::Behavior::ContactDetection>();
}

class Shift::Registration<Shift::Behavior::BehaviorBase>
  registration("contact_detection", Shift::Behavior::ContactDetection::create);

void Shift::Behavior::ContactDetection::addKeys() {
  SerialEntry name("name", SerialType::STRING, &name_);
  name.setRequired(false);
  name.setDescription("User defined descriptive name.");
  addSerialEntry(name);

  SerialEntry active("active", SerialType::BOOLEAN, &active_);
  active.setRequired(false);
  active.setDescription(
    "Indicates whether behavior is active in simulation: If the value is set "
    "to false, the data will be loaded into computer working memory, but will "
    "not be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model.");
  addSerialEntry(active);
}

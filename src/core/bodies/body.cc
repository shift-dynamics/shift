#include "shift-core/bodies/body.h"

void Shift::Body::Factory::registerModule(const std::string& constraintName,
                                          CreateFunction pfnCreate) {
  factoryMap_.insert(std::make_pair(constraintName, pfnCreate));
}

void Shift::Body::BodyBase::addKeys() {
  SerialEntry name("name", SerialType::STRING, &name_);
  name.setRequired(true);
  name.setDescription("User defined descriptive body name.");
  addSerialEntry(name);

  SerialEntry active("active", SerialType::BOOLEAN, &active_);
  active.setRequired(false);
  active.setDescription(
    "Indicates whether body is active in simulation: If the value is set to "
    "false, the data will be loaded into computer working memory, but will not "
    "be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model.");
  addSerialEntry(active);
}

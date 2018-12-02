#include "shift-core/assembly.h"

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

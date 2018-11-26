#include "shift-core/body.h"

void Shift::Body::addJsonKeys() {
  addSerialEntry("name", SerialType::STRING, &name_);
  addSerialEntry("active", SerialType::BOOLEAN, &active_);
  addSerialEntry("definition", SerialType::STRING_ARRAY, &definition_);
  addSerialEntry("center_of_mass", SerialType::DOUBLE_ARRAY,
                 &centerOfMassWRTOrigin_, 3);
}

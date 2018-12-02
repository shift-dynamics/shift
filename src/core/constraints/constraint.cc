#include "shift-core/constraints/constraint.h"

void Shift::Constraint::Factory::registerModule(const std::string& name,
                                                CreateFunction pfnCreate) {
  factoryMap_.insert(std::make_pair(name, pfnCreate));
}

// void Shift::Constraint::addKeys() {
//   addSerialEntry("name", SerialType::STRING, &name_);
//   addSerialEntry("body_frame_pairs", SerialType::STRING_ARRAY,
//   &bodyFramePairs_,
//                  2, 2);

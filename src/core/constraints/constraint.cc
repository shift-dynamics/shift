#include "shift-core/constraints/constraint.h"

bool Shift::Constraint::Modules::initialized() {
  return true;
}

// void Shift::Constraint::addKeys() {
//   addSerialEntry("name", SerialType::STRING, &name_);
//   addSerialEntry("body_frame_pairs", SerialType::STRING_ARRAY,
//   &bodyFramePairs_,
//                  2, 2);

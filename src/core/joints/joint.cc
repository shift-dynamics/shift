#include "shift-core/joints/joint.h"

void Shift::Joint::Factory::registerModule(const std::string& name,
                                           CreateFunction pfnCreate) {
  factoryMap_.insert(std::make_pair(name, pfnCreate));
}

void Shift::Joint::JointBase::addKeys() {
  // addSerialEntry("name", SerialType::STRING, &name_);
  // addSerialEntry("body_frame_pairs", SerialType::STRING_ARRAY,
  // &bodyFramePairs_,
  //                2, 2);
}

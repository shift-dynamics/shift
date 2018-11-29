#include "shift-core/frame.h"

void Shift::Frame::addJsonKeys() {
  addSerialEntry("name", SerialType::STRING, &name_);
  addSerialEntry("active", SerialType::BOOLEAN, &active_);
  addSerialEntry("pose", SerialType::POSE, &pose_);
}

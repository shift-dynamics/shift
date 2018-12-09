#include <iostream>

#include "shift-core/units.h"

void Shift::Units::addKeys() {
  SerialEntry mass("mass", SerialType::STRING, &mass_);
  mass.setRequired(true);
  mass.setDescription("Mass units, default value is kg if not provided.");
  addSerialEntry(mass);

  SerialEntry length("length", SerialType::STRING, &length_);
  length.setRequired(false);
  length.setDescription(
    "Length units, default value is meters if not provided.");
  addSerialEntry(length);

  SerialEntry time("time", SerialType::STRING, &time_);
  time.setRequired(false);
  time.setDescription("Time units, default value is seconds if not provided.");
  addSerialEntry(time);

  SerialEntry angle("angle", SerialType::STRING, &angle_);
  angle.setRequired(false);
  angle.setDescription(
    "Angle units, default value is radians if not provided.");
  addSerialEntry(angle);
}

void Shift::Units::print() {
  std::cout << "units: " << std::endl;
  std::cout << "  mass: " << mass_ << std::endl;
  std::cout << "  length: " << length_ << std::endl;
  std::cout << "  time: " << time_ << std::endl;
  std::cout << "  angle: " << angle_ << std::endl;
}

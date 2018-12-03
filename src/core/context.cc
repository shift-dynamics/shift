#include <fstream>

#include "shift-core/context.h"

void Shift::Context::readJSON(const std::string filename) {
  std::ifstream file(filename);
  file >> root_;
  fixed_.readJSON(root_);
}

void Shift::Context::writeJSON() {}

void Shift::Context::writeJSON(const std::string filename) {}

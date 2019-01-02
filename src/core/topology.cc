#include "shift-core/topology.h"

bool Shift::Topology::matchFullyQualifiedName(
  const std::vector<std::string>& name1,
  const std::vector<std::string>& name2) {

  /* exceptional case where ["fixed"] matches ["fixed", "fixed"] */
  if(name1.size() == 1 && name1.at(0) == "fixed") {
    if(name2.size() == 2 && name2.at(0) == "fixed" && name2.at(1) == "fixed") {
      return true;
    }
  }

  /* exceptional case where ["fixed", "fixed"] matches ["fixed"] */
  if(name2.size() == 1 && name2.at(0) == "fixed") {
    if(name1.size() == 2 && name1.at(0) == "fixed" && name1.at(1) == "fixed") {
      return true;
    }
  }

  if(name1.size() != name2.size()) {
    return false;
  }

  for(unsigned int i = 0; i < name1.size(); i++) {
    if(name1[i] != name2[i]) {
      return false;
    }
  }

  return true;
}

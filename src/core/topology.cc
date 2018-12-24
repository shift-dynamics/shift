#include "shift-core/topology.h"

bool Shift::Topology::matchFullyQualifiedName(
  const std::vector<std::string>& name1,
  const std::vector<std::string>& name2) {
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

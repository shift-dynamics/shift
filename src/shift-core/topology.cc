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

/*!
@brief Used to detect if k is in the kinematic path of j
@param pcl parent child list
@param j index of body
@param k index of second body to test
*/
bool Shift::Topology::inPath(std::vector<int>& pcl, int j, int k) {
  if(j == k) {
    return true;
  }
  bool inPath = false;
  while(j >= -1) {
    if(j == k) {
      inPath = true;
      break;
    }
    j = pcl[j];
    std::cout << j << std::endl;
  }
  return inPath;
}

/*!
@brief Detect if body/joint k is head of a substructure
@param k index of body / joint
@param list of substructure members
*/
bool Shift::Topology::isSubstructureHead(
  int k, std::vector<std::vector<int>>& substructures) {
  for(auto substructure : substructures) {
    if(k == substructure[0]) {
      return true;
    }
  }
  return false;
}

/*!
@brief Detect if body/joint k is member of a substructure
@param k index of body / joint
@param list of substructure members
*/
bool Shift::Topology::isSubstructureMember(
  int bodyIndex, std::vector<std::vector<int>>& substructures) {
  for(auto substructure : substructures) {
    for(auto member : substructure) {
      if(bodyIndex == member) {
        return true;
      }
    }
  }
  return false;
}

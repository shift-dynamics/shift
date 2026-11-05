/**
@file
@brief Utilities to process the topology of a Shift model 
@author Kristopher Wehage
@remark
THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.
*/

#include <algorithm>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

#include "shift/core/assembly.h"
#include "shift/core/body/rigid.h"
#include "shift/core/constraint/chord.h"
#include "shift/core/context.h"
#include "shift/core/frame.h"
#include "shift/core/topology.h"

#include "shift/core/joint/float.h"
#include "shift/core/joint/prismatic.h"
#include "shift/core/joint/gear.h"
#include "shift/core/joint/revolute.h"
#include "shift/core/joint/screw.h"
#include "shift/core/joint/rigid.h"
#include "shift/core/joint/spherical.h"

bool shift::core::topology::MatchFullyQualifiedName(
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
\brief Used to detect if k is in the kinematic path of j
\param pcl parent child list
\param j index of body
\param k index of second body to test
*/
bool shift::core::topology::InPath(std::vector<int>& pcl, int j, int k) {
  /* walk from j toward the fixed body (-1), which has no parent */
  while(j != k) {
    if(j < 0) {
      return false;
    }
    j = pcl[j];
  }
  return true;
}

/*!
\brief Detect if body/joint k is head of a substructure
\param k index of body / joint
\param list of substructure members
*/
bool shift::core::topology::IsSubstructureHead(
  int k, std::vector<std::vector<int>>& substructures) {
  for(auto substructure : substructures) {
    if(k == substructure[0]) {
      return true;
    }
  }
  return false;
}

/*!
\brief Detect if body/joint k is member of a substructure
\param k index of body / joint
\param list of substructure members
*/
bool shift::core::topology::IsSubstructureMember(
  int body_index, std::vector<std::vector<int>>& substructures) {
  for(auto substructure : substructures) {
    for(auto member : substructure) {
      if(body_index == member) {
        return true;
      }
    }
  }
  return false;
}

void shift::core::topology::SetInversePermutation(const std::vector<int>& p,
                                            std::vector<int>& p_inv) {
  if(p.size() != p_inv.size()) {
    p_inv.resize(p.size());
  }
  for(unsigned int i = 0; i < p.size(); i++) {
    p_inv[p[i]] = i;
  }
}

std::string shift::core::topology::GetJointType(dh_fptr f) {
  if(f == shift::core::joint::spherical_dh) {
    return "SPHERICAL";
  } else if(f == shift::core::joint::sphericalinv_dh) {
    return "SPHERICAL_INVERSE";
  } else if(f == shift::core::joint::tx_dh) {
    return "PRISMATIC_TX";
  } else if(f == shift::core::joint::txinv_dh) {
    return "PRISMATIC_TX_INVERSE";
  } else if(f == shift::core::joint::ty_dh) {
    return "PRISMATIC_TY";
  } else if(f == shift::core::joint::tyinv_dh) {
    return "PRISMATIC_TY_INVERSE";
  } else if(f == shift::core::joint::tz_dh) {
    return "PRISMATIC_TZ";
  } else if(f == shift::core::joint::tzinv_dh) {
    return "PRISMATIC_TZ_INVERSE";
  } else if(f == shift::core::joint::ta_dh) {
    return "PRISMATIC_TA";
  } else if(f == shift::core::joint::tainv_dh) {
    return "PRISMATIC_TA_INVERSE";
  } else if(f == shift::core::joint::rx_dh) {
    return "REVOLUTE_RX";
  } else if(f == shift::core::joint::rxinv_dh) {
    return "REVOLUTE_RX_INVERSE";
  } else if(f == shift::core::joint::ry_dh) {
    return "REVOLUTE_RY";
  } else if(f == shift::core::joint::ryinv_dh) {
    return "REVOLUTE_RY_INVERSE";
  } else if(f == shift::core::joint::rz_dh) {
    return "REVOLUTE_RZ";
  } else if(f == shift::core::joint::rzinv_dh) {
    return "REVOLUTE_RZ_INVERSE";
  } else if(f == shift::core::joint::ra_dh) {
    return "REVOLUTE_RA";
  } else if(f == shift::core::joint::rainv_dh) {
    return "REVOLUTE_RA_INVERSE";
  } else if(f == shift::core::joint::screw_dh) {
    return "SCREW";
  } else if(f == shift::core::joint::screwinv_dh) {
    return "SCREW_INVERSE";
  } else if(f == shift::core::joint::gear_dh) {
    return "GEAR";
  } else if(f == shift::core::joint::gearinv_dh) {
    return "GEAR_INVERSE";
  } else if(f == shift::core::joint::rigid_dh) {
    return "RIGID";
  } else if(f == shift::core::joint::rigidinv_dh) {
    return "RIGID_INVERSE";
  } else if(f == shift::core::joint::float_dh) {
    return "FLOAT";
  } else if(f == shift::core::joint::floatinv_dh) {
    return "FLOAT_INVERSE";
  }
  return "unknown";
}

std::string shift::core::topology::GetBasename(const std::string& s) {
  size_t i = s.rfind('.', s.length());
  if(i != std::string::npos) {
    return (s.substr(0, i));
  } else {
    return std::string("");
  }
}

std::string shift::core::topology::GetFilename(const std::string& s) {
  char sep = '/';
#ifdef _WIN32
  sep = '\\';
#endif
  size_t i = s.rfind(sep, s.length());
  if(i != std::string::npos) {
    return (s.substr(i + 1, s.length() - i));
  }
  // No filename found
  return s;
}

std::string shift::core::topology::GetExtension(std::string& filename) {
  std::string::size_type idx;
  idx = filename.rfind('.');
  if(idx != std::string::npos) {
    std::string extension = filename.substr(idx + 1);
    return extension;
  } else {
    // No extension found
    return ("");
  }
}

std::string shift::core::topology::RemoveExtension(std::string& filename) {
  std::string::size_type idx;
  idx = filename.rfind('.');
  if(idx != std::string::npos) {
    std::string extension = filename.substr(0, idx);
    return extension;
  } else {
    // No extension found
    return ("");
  }
}

std::string ToLower(std::string& s) {
  std::string out;
  out.resize(s.size());
  std::transform(s.begin(), s.end(), out.begin(), ::tolower);
  return out;
}

std::string shift::core::topology::GetValidGraphvizExtension(std::string& extension) {
  std::string ext = ToLower(extension);
  if(ext == "gif") {
    return std::string("gif");
  } else if(ext == "jpg" || ext == "jpeg") {
    return std::string("jpg");
  } else if(ext == "tiff" || ext == "tif") {
    return std::string("svg");
  } else if(ext == "svg") {
    return std::string("svg");
  } else if(ext == "png") {
    return std::string("png");
  } else if(ext == "pdf") {
    return std::string("pdf");
  } else if(ext == "dot") {
    return std::string("dot");
  } else {
    return std::string("");
  }
}

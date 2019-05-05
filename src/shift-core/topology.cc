/**
@file
@brief Utilities to process the topology of a Shift model, including utilities
to display the model as a directed graph.
@author Kristopher Wehage
@copyright Kristopher Wehage 2019
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

#include "shift-core/assembly.h"
#include "shift-core/body/rigid.h"
#include "shift-core/constraint/chord.h"
#include "shift-core/context.h"
#include "shift-core/frame.h"
#include "shift-core/topology.h"

#include "shift-core/joint/float.h"
#include "shift-core/joint/prismatic.h"
#include "shift-core/joint/revolute.h"
#include "shift-core/joint/rigid.h"
#include "shift-core/joint/spherical.h"

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
\brief Used to detect if k is in the kinematic path of j
\param pcl parent child list
\param j index of body
\param k index of second body to test
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
  }
  return inPath;
}

/*!
\brief Detect if body/joint k is head of a substructure
\param k index of body / joint
\param list of substructure members
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
\brief Detect if body/joint k is member of a substructure
\param k index of body / joint
\param list of substructure members
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

void Shift::Topology::setInversePermutation(const std::vector<int>& p,
                                            std::vector<int>& pInverse) {
  if(p.size() != pInverse.size()) {
    pInverse.resize(p.size());
  }
  for(unsigned int i = 0; i < p.size(); i++) {
    pInverse[p[i]] = i;
  }
}

std::string Shift::Topology::getJointType(dh_fptr f) {
  if(f == Shift::Joint::spherical_dh) {
    return "SPHERICAL";
  } else if(f == Shift::Joint::sphericalinv_dh) {
    return "SPHERICAL_INVERSE";
  } else if(f == Shift::Joint::tx_dh) {
    return "PRISMATIC_TX";
  } else if(f == Shift::Joint::txinv_dh) {
    return "PRISMATIC_TX_INVERSE";
  } else if(f == Shift::Joint::ty_dh) {
    return "PRISMATIC_TY";
  } else if(f == Shift::Joint::tyinv_dh) {
    return "PRISMATIC_TY_INVERSE";
  } else if(f == Shift::Joint::tz_dh) {
    return "PRISMATIC_TZ";
  } else if(f == Shift::Joint::tzinv_dh) {
    return "PRISMATIC_TZ_INVERSE";
  } else if(f == Shift::Joint::ta_dh) {
    return "PRISMATIC_TA";
  } else if(f == Shift::Joint::tainv_dh) {
    return "PRISMATIC_TA_INVERSE";
  } else if(f == Shift::Joint::rx_dh) {
    return "REVOLUTE_RX";
  } else if(f == Shift::Joint::rxinv_dh) {
    return "REVOLUTE_RX_INVERSE";
  } else if(f == Shift::Joint::ry_dh) {
    return "REVOLUTE_RY";
  } else if(f == Shift::Joint::ryinv_dh) {
    return "REVOLUTE_RY_INVERSE";
  } else if(f == Shift::Joint::rz_dh) {
    return "REVOLUTE_RZ";
  } else if(f == Shift::Joint::rzinv_dh) {
    return "REVOLUTE_RZ_INVERSE";
  } else if(f == Shift::Joint::ra_dh) {
    return "REVOLUTE_RA";
  } else if(f == Shift::Joint::rainv_dh) {
    return "REVOLUTE_RA_INVERSE";
  } else if(f == Shift::Joint::rigid_dh) {
    return "RIGID";
  } else if(f == Shift::Joint::rigidinv_dh) {
    return "RIGID_INVERSE";
  } else if(f == Shift::Joint::float_dh) {
    return "FLOAT";
  } else if(f == Shift::Joint::floatinv_dh) {
    return "FLOAT_INVERSE";
  }
  return "unknown";
}

std::string Shift::Topology::getBasename(const std::string& s) {
  size_t i = s.rfind('.', s.length());
  if(i != std::string::npos) {
    return (s.substr(0, i));
  } else {
    return std::string("");
  }
}

std::string Shift::Topology::getFilename(const std::string& s) {
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

std::string Shift::Topology::getExtension(std::string& filename) {
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

std::string toLower(std::string& s) {
  std::string out;
  out.resize(s.size());
  std::transform(s.begin(), s.end(), out.begin(), ::tolower);
  return out;
}

std::string Shift::Topology::getGraphvizExtension(std::string& extension) {
  std::string ext = toLower(extension);
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

#if 0
/*!
@brief Write a graphviz dot file for mechanism after converting chords to arcs

Writes a graphviz dot file for original mechanism after adding virtual bodies
and converting chords to arcs

@param context pointer to a Shift::Context data structure.
@param filename dot file will be saved to filename
@return returns -1 on failure, 0 on success
*/
int Shift::Topology::digraphDOT(const Shift::Context& context,
                                std::string filename) {

  FILE* digraph;
  if((digraph = fopen(filename.c_str(), "w")) == NULL) {
    std::cerr << "Cannot open file: " << filename << ". No DOT file generated"
              << std::endl;
    return -1;
  }
  fprintf(digraph, "digraph original {\n");
  fprintf(digraph, "size=\"7,7\";\n");
  fprintf(digraph, "{rank=same; \"fixed\"}\n");

// @TODO convert this to new C++ API
  jointNode* HEAD;
  if(context->jointHead) {
    int level = 1;
    int count = 1;
    HEAD = context->jointHead;
    while(count != 0) {
      count = 0;
      fprintf(digraph, "{rank=same; ");
      HEAD = context->jointHead;
      while(HEAD) {
        if(HEAD->level == level) {
          fprintf(digraph, "\"%s\" ", HEAD->bodyTo);
          count++;
        }
        HEAD = HEAD->next;
      }
      fprintf(digraph, "}\n");
      level++;
    }

    HEAD = context->jointHead;
    while(HEAD) {
      fprintf(digraph, "\"%s\" -> \"%s\"\n", HEAD->bodyFrom, HEAD->bodyTo);
      HEAD = HEAD->next;
    }
    chordConstraintNode* HEAD2 = context->chordConstraintHead;
    while(HEAD2) {
      fprintf(digraph, "\"%s\" [shape=octagon];\n", HEAD2->bodyFrom);
      fprintf(digraph, "\"%s\" -> \"%s\" [style=dashed];", HEAD2->bodyFrom,
              HEAD2->bodyTo);
      HEAD2 = HEAD2->next;
    }
    fprintf(digraph, "\n}\n");
  }
  fclose(digraph);
}

#endif

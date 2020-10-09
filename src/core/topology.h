#ifndef SHIFT_TOPOLOGY_H_
#define SHIFT_TOPOLOGY_H_

#include <assert.h>
#include <iostream>
#include <string.h>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/screw.h"

namespace shift {
class Context;

namespace Topology {

/*!
@brief Displacements (d), influence coefficient matrices (h), velocities (v) and
quadratic accelerations (a).
*/
struct DHVA {
  DHVA() {}
  DHVA(unsigned int frame_count, unsigned int dc_count, unsigned int dq_count,
       unsigned int non_rigid_joint_count) {

    frame_count_ = frame_count;
    dc_count_ = dc_count;
    dq_count_ = dq_count;
    non_rigid_joint_count_ = non_rigid_joint_count;

    size_t dhva_size_ =
      2 * dc_count * sizeof(double) +                     /* dc */
      frame_count * sizeof(ksl_SE3_t) +                   /* d */
      (dq_count + 1) * sizeof(ksl_screw_t) +              /* h */
      (non_rigid_joint_count + 1) * sizeof(ksl_screw_t) + /* v */
      (non_rigid_joint_count + 1) * sizeof(ksl_screw_t);  /* aq */
    dc = new double[dhva_size_];
    if(dc == nullptr) {
      std::cerr << "Error: failed to allocate memory for DHVA" << std::endl;
      exit(-1);
    }
    memset(dc, 0, dhva_size_);
    d = (ksl_SE3_t*)(dc + 2 * dc_count);
    h = (ksl_screw_t*)(d + frame_count);
    v = h + (non_rigid_joint_count + 1);
    aq = v + (non_rigid_joint_count + 1);
    for(unsigned int i = 0; i < frame_count; i++) {
      ksl_SE3_setIdentity(d + i);
    }

    // increment pointers to h, v, and aq. This allows using the index -1 for
    // the fixed inertial reference frame
    h++;
    v++;
    aq++;
  }

  ~DHVA() {
    if(dc != nullptr) {
      delete[] dc;
      dc = nullptr;
      d = nullptr;
      h = nullptr;
      v = nullptr;
      aq = nullptr;
    }
  }

  DHVA& operator=(const DHVA& other) {

    // check for self assignment
    if(&other == this) {
      return *this;
    }

    frame_count_ = other.frame_count_;
    dc_count_ = other.dc_count_;
    dq_count_ = other.dq_count_;
    non_rigid_joint_count_ = other.non_rigid_joint_count_;

    size_t dhva_size_ =
      2 * dc_count_ * sizeof(double) +                     /* dc */
      frame_count_ * sizeof(ksl_SE3_t) +                   /* d */
      (dq_count_ + 1) * sizeof(ksl_screw_t) +              /* h */
      (non_rigid_joint_count_ + 1) * sizeof(ksl_screw_t) + /* v */
      (non_rigid_joint_count_ + 1) * sizeof(ksl_screw_t);  /* aq */
    dc = new double[dhva_size_];
    if(dc == nullptr) {
      std::cerr << "Error: failed to allocate memory for DHVA" << std::endl;
      assert(0);
    }
    memset(dc, 0, dhva_size_);
    d = (ksl_SE3_t*)(dc + 2 * dc_count_);
    h = (ksl_screw_t*)(d + frame_count_);
    v = h + (non_rigid_joint_count_ + 1);
    aq = v + (non_rigid_joint_count_ + 1);

    // increment pointers to h, v, and aq. This allows using the index -1 for
    // the fixed inertial reference frame
    h++;
    v++;
    aq++;

    memcpy(dc, other.dc, dhva_size_);
    return *this;
  }

  double* dc = nullptr;     /*!< direction cosine arrays */
  ksl_SE3_t* d = nullptr;   /*!< array of displacements equal
                              in size to number of frames */
  ksl_screw_t* h = nullptr; /*!< an array of influence coefficient matrices */
  ksl_screw_t* v =
    nullptr; /*!< an array of body velocities equal to the
        number of non rigid joints + 1 for the fixed inertial reference frame */
  ksl_screw_t* aq =
    nullptr; /*!< an array of body accelerations equal to the
        number of non rigid joints + 1 for the fixed inertial reference frame */
private:
  unsigned int frame_count_ = 0;
  unsigned int dc_count_ = 0;
  unsigned int dq_count_ = 0;
  unsigned int non_rigid_joint_count_ = 0;
};

/*!
@brief Struct that defines offsets into DHVA array.
*/
struct Offsets {
  union {
    unsigned int dc;    /*!< offset into direction cosine array */
    unsigned int q = 0; /*!< offset into array of generalized coordinates */
  };
  unsigned int dq = 0; /*!< offset into array of generalized velocities */
  struct {
    int d = 0; /*!< offset for parent body's displacement */
    int v = 0; /*!< offset for parent body's velocity */
  } parent;
  struct {
    int d = 0; /*!< offset for child body's displacement */
    int v = 0; /*!< offset for child body's velocity */
  } child;
  unsigned int data = 0; /*!< offset into data array */
};

typedef void (*dh_fptr)(const shift::Context&, DHVA&, const Offsets&,
                        const double*);

typedef void (*dhva_fptr)(const shift::Context&, DHVA&, const Offsets&,
                          const double*, const double*);

/*!
@defgroup sorting Utilities to support sorting bodies and joints in
level-increasing order and identifying loop members
@{
*/
bool matchFullyQualifiedName(const std::vector<std::string>&,
                             const std::vector<std::string>&);

bool inPath(std::vector<int>& pcl, int j, int k);

bool isSubstructureHead(int, std::vector<std::vector<int>>&);

bool isSubstructureMember(int, std::vector<std::vector<int>>&);

/*!
\brief set inverse permutation
\param p forward permutation array
\param pInverse inverse permutation array
*/
void setInversePermutation(const std::vector<int>& p,
                           std::vector<int>& pInverse);

std::string getJointType(dh_fptr f);

std::string getBasename(const std::string& s);

std::string getExtension(std::string& filename);

std::string removeExtension(std::string& filename);

std::string getFilename(const std::string& filename);

std::string getValidGraphvizExtension(std::string& extension);

/*!
\brief generate a directed graph of a mechanism with bodies as graph nodes
and joints as graph edges and save as a DOT file. This variant does not use the
graphviz API. The DOT file can be used to generate an image using graphviz tools
offline.
*/
int digraphDOT(const shift::Context& context, std::string fileName);

/*!
\brief generate a directed graph of a mechanism with frames as graph nodes
and joints and relative transformations across bodies (i.e. shape matrices) as
graph edges and save as a DOT file. This variant does not use the
graphviz API. The DOT file can be used to generate an image using graphviz tools
*/
int digraphFramesDOT(const shift::Context& context, std::string fileName);

// // identify which frames in the d array correspond to body reference frames
// void setDReferenceFrames();
//
// // should be moved to joint's set up function
// void initializeJointForces();
//
// // identify body reference frames that may have been optimized out when
// // processing forward kinematics and set up operations required to update
// them void initializeGLReferenceFrames();
//
// // plays back operations identified in initializeGLReferenceFrames, updates
// // any GL frames that weren't processed in updateD, updateDH, or updateDHVA
// void updateGLReferenceFrames(shift::Context&, double* dhva);
//
// // sets up array of force modules, this should be moved from here to
// context.h void flattenForceModules();
//
// // basic sweep to set up collision simplices, this should be moved from here
// to
// // collisiongeometry.h or elsewhere.
// void basicSweep();

} // namespace Topology
} // namespace shift
#endif

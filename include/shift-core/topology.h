#ifndef SHIFT_TOPOLOGY_H_
#define SHIFT_TOPOLOGY_H_

#include <iostream>
#include <string.h>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/screw.h"

namespace Shift {
class Context;

namespace Topology {

/*!
@brief Displacements (d), influence coefficient matrices (h), velocities (v) and
quadratic accelerations (a).
*/
struct DHVA {
  DHVA() {}
  DHVA(unsigned int frameCount, unsigned int dcCount, unsigned int dqCount,
       unsigned int nonRigidJointCount) {

    frameCount_ = frameCount;
    dcCount_ = dcCount;
    dqCount_ = dqCount;
    nonRigidJointCount_ = nonRigidJointCount;

    size_t dhvaSize = 2 * dcCount * sizeof(double) +                   /* dc */
                      frameCount * sizeof(ksl_SE3_t) +                 /* d */
                      (dqCount + 1) * sizeof(ksl_screw_t) +            /* h */
                      (nonRigidJointCount + 1) * sizeof(ksl_screw_t) + /* v */
                      (nonRigidJointCount + 1) * sizeof(ksl_screw_t);  /* aq */
    dc = new double[dhvaSize];
    if(dc == NULL) {
      std::cerr << "Error: failed to allocate memory for DHVA" << std::endl;
      exit(-1);
    }
    memset(dc, 0, dhvaSize);
    d = (ksl_SE3_t*)(dc + 2 * dcCount);
    h = (ksl_screw_t*)(d + frameCount);
    v = h + (nonRigidJointCount + 1);
    aq = v + (nonRigidJointCount + 1);
    for(unsigned int i = 0; i < frameCount; i++) {
      ksl_SE3_setIdentity(d + i);
    }

    // increment pointers to h, v, and aq. This allows using the index -1 for
    // the fixed inertial reference frame
    h++;
    v++;
    aq++;
  }

  ~DHVA() {
    if(dc != NULL) {
      delete[] dc;
      dc = NULL;
      d = NULL;
      h = NULL;
      v = NULL;
      aq = NULL;
    }
  }

  DHVA& operator=(const DHVA& other) {

    std::cout << "copying DHVA" << std::endl;
    // check for self assignment
    if(&other == this) {
      return *this;
    }

    frameCount_ = other.frameCount_;
    dcCount_ = other.dcCount_;
    dqCount_ = other.dqCount_;
    nonRigidJointCount_ = other.nonRigidJointCount_;

    size_t dhvaSize = 2 * dcCount_ * sizeof(double) +                   /* dc */
                      frameCount_ * sizeof(ksl_SE3_t) +                 /* d */
                      (dqCount_ + 1) * sizeof(ksl_screw_t) +            /* h */
                      (nonRigidJointCount_ + 1) * sizeof(ksl_screw_t) + /* v */
                      (nonRigidJointCount_ + 1) * sizeof(ksl_screw_t);  /* aq */
    dc = new double[dhvaSize];
    if(dc == NULL) {
      std::cerr << "Error: failed to allocate memory for DHVA" << std::endl;
      exit(-1);
    }
    memset(dc, 0, dhvaSize);
    d = (ksl_SE3_t*)(dc + 2 * dcCount_);
    h = (ksl_screw_t*)(d + frameCount_);
    v = h + (nonRigidJointCount_ + 1);
    aq = v + (nonRigidJointCount_ + 1);

    // increment pointers to h, v, and aq. This allows using the index -1 for
    // the fixed inertial reference frame
    h++;
    v++;
    aq++;

    memcpy(dc, other.dc, dhvaSize);
    return *this;
  }

  double* dc = NULL;     /*!< direction cosine arrays */
  ksl_SE3_t* d = NULL;   /*!< array of displacements equal
                              in size to number of frames */
  ksl_screw_t* h = NULL; /*!< an array of influence coefficient matrices */
  ksl_screw_t* v =
    NULL; /*!< an array of body velocities equal to the
        number of non rigid joints + 1 for the fixed inertial reference frame */
  ksl_screw_t* aq =
    NULL; /*!< an array of body accelerations equal to the
        number of non rigid joints + 1 for the fixed inertial reference frame */
private:
  unsigned int frameCount_ = 0;
  unsigned int dcCount_ = 0;
  unsigned int dqCount_ = 0;
  unsigned int nonRigidJointCount_ = 0;
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

typedef void (*dh_fptr)(const Shift::Context&, DHVA&, const Offsets&,
                        const double*);

typedef void (*dhva_fptr)(const Shift::Context&, DHVA&, const Offsets&,
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

std::string getFilename(const std::string& filename);

std::string getGraphvizExtension(std::string& extension);

/*!
\brief generate a directed graph of a mechanism with bodies as graph nodes
and joints as graph edges and save as a PNG file.
*/
int digraph(const Shift::Context& context, std::string fileName);

/*!
\brief generate a directed graph of a mechanism with frames as graph nodes
and joints and relative transformations across bodies (i.e. shape matrices) as
graph edges and save as a PNG file.
*/
int digraphFrames(const Shift::Context& context, std::string fileName);

/*!
\brief generate a directed graph of a mechanism with bodies as graph nodes
and joints as graph edges and save as a DOT file. This variant does not use the
graphviz API. The DOT file can be used to generate an image using graphviz tools
offline.
*/
int digraphDOT(const Shift::Context& context, std::string fileName);

/*!
\brief generate a directed graph of a mechanism with frames as graph nodes
and joints and relative transformations across bodies (i.e. shape matrices) as
graph edges and save as a DOT file. This variant does not use the
graphviz API. The DOT file can be used to generate an image using graphviz tools
*/
int digraphFramesDOT(const Shift::Context& context, std::string fileName);

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
// void updateGLReferenceFrames(Shift::Context&, double* dhva);
//
// // sets up array of force modules, this should be moved from here to
// context.h void flattenForceModules();
//
// // basic sweep to set up collision simplices, this should be moved from here
// to
// // collisiongeometry.h or elsewhere.
// void basicSweep();

} // namespace Topology
} // namespace Shift
#endif

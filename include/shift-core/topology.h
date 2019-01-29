#ifndef SHIFT_TOPOLOGY_H_
#define SHIFT_TOPOLOGY_H_

#include <string>
#include <vector>

#include "shift-core/context.h"

namespace Shift {
namespace Topology {

bool matchFullyQualifiedName(const std::vector<std::string>&,
                             const std::vector<std::string>&);

bool inPath(std::vector<int>& pcl, int j, int k);

void freeRunTimeArrays();

// count active objects (bodies/joints/lights/frames/io)\n");
void setIndicesInitial();

/* recursively clear inTree flag on all frames */
void clearFramesInTree();

void setFramesInTree();

/* determine number of chord constraints, found by subtracting
number active bodies from number of active joints, (body count does not
include the massless "fixed" body) */
// context->chordConstraintCount =
//   context->activeJointCount - context->activeBodyCount;

/* alot additional space for all frames on fixed body */
// s2_frame_t* frameHead = context->assembly->bodyHead->frameHead;
// while(frameHead) {
//   if(!s2_stack_in(frameStack, frameHead->index)) {
//     s2_stack_append(frameStack, frameHead->index);
//   }
//   frameHead = frameHead->next;
// }

/*
  alot additional space for the virtual bodies/frames that will
  be added, equal to the number of chord constraints.
  context->activeFrameInTreeCount does not contain active Frames on the
  fixed reference frame.
*/
// context->activeFrameInTreeCount =
//   frameStack->count + context->chordConstraintCount;
// context->activeFrameCount += context->chordConstraintCount;
// s2_stack_free(frameStack);

/* malloc storage for d, dWRTReference, q, dq, ddq, h, v, and aq */
void mallocRunTimeArrays();
void mallocDHVAArrays();

/* identify loops and loop members */

/* set level away from fixed body for active bodies and joints */
// void levelOrderSort(); // in context.cc

/* add a virtual body for each chord loop */
// void processVirtualBodies(); // in context.cc

/* update parent child list */
// void processpcl();

/* identify members of each kinematic loop */
// fprintf(
//   debug,
//   "# processing loop members\n");
// s2_stack_t** loopMembers = topology_processLoopMembers(context);

/* carry out kinematic substructuring algorithm, reorder bodies and joints
by increasing level order within each kinematic substructure */

/* identify overlapping loops */
void detectSubstructures();

/* reorder substructures in level increasing order */
void substructureLevelOrderSort();

/* reorder members of substructures in level increasing order */
void substructureBodyJointReorder();

/* update parent child list */
// topology_processpcl(context);

/*
  copy data into contiguous memory location and update memory pointers
*/

// identifying reference frames
void setReferenceFrames();

// setting joint terms (h and dc)
void copyUpdateJoints();

// set dva index
void setDVAIndex();

// copy V and A terms from joint data structures to dhva array and update
// memory pointers
void copyUpdateVA();

// copy D tersmf rom frame data structures to dhva array and update memory
// pointers
void copyUpdateD();

// remove rigid joints from model
void compressRigidJoints();

// set parent joint pointers in bodies
void setParentJointPointers();

// set names and count DHVA operations\n");
void countDHVA();

// set dhva pointers
void setDHVAPointers();

// set -g in fixed frame aq array
void setFixedFrameGravityVector();

// forward kinematics functions

/*!
@defgroup forward_kinematics Forward Kinematics
@{
*/

/*!
@brief update displacements only
*/
void updateD(Shift::Context&, double* dhva, double* q);

/*!
@brief update displacements and influence coefficient matrices
*/
void updateDH(Shift::Context&, double* dhva, double* q);

/*!
@brief update displacements, influence coefficient matrices, velocities
and quadratic acceleration terms
*/
void updateDHVA(Shift::Context&, double* dhva, double* q, double* dq);

/*!
@}
*/

// identify which frames in the d array correspond to body reference frames
void setDReferenceFrames();

// should be moved to joint's set up function
void initializeJointForces();

// identify body reference frames that may have been optimized out when
// processing forward kinematics and set up operations required to update them
void initializeGLReferenceFrames();

// plays back operations identified in initializeGLReferenceFrames, updates
// any GL frames that weren't processed in updateD, updateDH, or updateDHVA
void updateGLReferenceFrames(Shift::Context&, double* dhva);

// sets up array of force modules, this should be moved from here to context.h
void flattenForceModules();

// basic sweep to set up collision simplices, this should be moved from here to
// collisiongeometry.h or elsewhere.
void basicSweep();

} // namespace Topology
} // namespace Shift
#endif

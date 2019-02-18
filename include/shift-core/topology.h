#ifndef SHIFT_TOPOLOGY_H_
#define SHIFT_TOPOLOGY_H_

#include <string>
#include <vector>

#include "shift-core/context.h"

namespace Shift {
namespace Topology {

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
@}
@defgroup forward_kinematics Forward Kinematics functions
@{
*/

/*!
@brief update displacements only
*/
void updateD(Shift::Context&, Shift::Joint::DHVA& dhva, const double* q);

/*!
@brief update displacements and influence coefficient matrices
*/
void updateDH(Shift::Context&, Shift::Joint::DHVA& dhva, const double* q);

/*!
@brief update displacements, influence coefficient matrices, velocities
and quadratic acceleration terms
*/
void updateDHVA(Shift::Context&, Shift::Joint::DHVA& dhva, const double* q,
                const double* dq);

/*!
@}
*/
//
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

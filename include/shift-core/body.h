#ifndef _SHIFT_BODY_H_
#define _SHIFT_BODY_H_

#include <string>
#include <vector>

#include "base.h"
#include "frame.h"

#include "ksl_inertia.h"
#include "ksl_vector.h"

namespace Shift {

class Assembly;
class Joint;

/*!
@brief Parameters that describe a Shift::Body
*/
class Body : Base {

public:
  Body(std::string name)
    : Base(name) {}
  ~Body() {}

  std::string getDefinition();
  void setDefinition(std::string);
  void setParentAssembly(Assembly*);
  void appendFrame(Frame);
  Joint* getParentJoint();
  Body* getParentBody();

private:
  std::string m_definition; /*!< Path to files that define body geometry,
                                  i.e. obj and mtl files */

  Assembly* m_assembly = NULL; /*!< Pointer to assembly that this body is a
                                  member of. */

  std::vector<Frame> m_frames; /*!< head of Frame linked list */

  int m_referenceFrameIndex = -1; /*!< pointer to Body's unique reference
                                    frame, which  may not be the Frame's origin.
                                    Assigned automatically by
                                    s2_ProcessTopology */

  int m_originIndex = -1; /*!< Index of body's unique *origin*
                             coordinate system, which may not be the
                             reference frame */

  Joint* m_parentJointPointer = NULL; /*!< Pointer to Body's unique *parent*
                                         Joint datastructure */

  Body* m_parentBodyPointer = NULL; /*!< A pointer to the unique *parent* Body,
                                       set by s2_ProcessTopology */

  Frame* m_parentFramePointer =
    NULL; /*!< parent frame of body reference frame */

  // s2_material_t* materials;      /*!< s2_material_t datastructure */
  Body* m_geometryInstanceOf = NULL; /*!< if this body has identical geometry to
                              another body, this pointer is set to the original
                              body. Otherwise, if this geometry is unique to
                              this body or is the first instance of this
                              geometry, the pointer is set to NULL */
  Body* m_collisionGeometryInstanceOf = NULL; /*!< if this body has identical
                              collision geometry to another body, this pointer
                              is set to the original body. Otherwise, if the
                              collision geometry is unique to this body or is
                              the first instance of this collision geometry, the
                              pointer is set to NULL */

  int m_index = -1; /*!< index in contiguous *run-time* array of Bodies, set
                  by s2_ProcessTopology */
  int m_level = -1; /*!< level in the spanning tree, set
                  by s2_ProcessTopology */
  int m_materialIndex = -1; /*!< offset into context->materials */
  int m_materialCount = 0;  /*!< number of materials assigned to a body */
  bool m_isVirtual = false; /*!< indicates whether the Body is *virtual*. All
                       user-defined bodies are not virtual. Massless
                       virtual bodies are added to simplify processing
                       kinematics if multiple loops are present in the
                       mechanism */
  ksl_inertia_t m_inertia;  /*!< centroidal Mass and moment of Inertia
                                            properties expressed in origin
                               coordinate system */
  ksl_vec3_t centerOfMassWRTOrigin;    /*!< vector from body origin to
                                                center of mass */
  ksl_vec3_t centerOfMassWRTReference; /*!< vector from body reference
                                                frame to center of mass */
  int collisionObjectCount;
  int collisionObjectOffset; /*!< index to beginning of array of one or more
                       s2_collisionobject_t* datastructures stored in
                       s2_context_t datastructure, s2_collisionobject_t is the
                       level 3 interface to all primitive convex geometries */
  bool drawEnabled;
  bool isVisible;
  bool collisionObjectDrawEnabled;
};

} // namespace Shift

#endif

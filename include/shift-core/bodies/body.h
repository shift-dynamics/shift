#ifndef _SHIFT_BODY_H_
#define _SHIFT_BODY_H_

#include <string>
#include <vector>

#include "ksl/inertia.h"

#include "shift-core/base.h"
#include "shift-core/frame.h"
#include "shift-core/serialize.h"

namespace Shift {

class Assembly;
class Joint;

/*!
@brief Parameters that describe a Shift::Body
*/
class Body
  : public Base
  , public Serialize {

public:
  Body(std::string name)
    : Base(name) {
    addKeys();
  }
  Body()
    : Base("") {
    addKeys();
  }
  ~Body() {}

  void addKeys();
  std::string getDefinition();
  void setDefinition(std::string);
  void setParentAssembly(Assembly*);
  void appendFrame(Frame);
  Joint* getParentJoint();
  Body* getParentBody();

  void processStuff();
  void service();

private:
  std::vector<std::string> definition_; /*!< Path to files that define body
                                  geometry, i.e. obj and mtl files */

  Assembly* assembly_ = NULL; /*!< Pointer to assembly that this body is a
                                  member of. */

  std::vector<Frame> frames_; /*!< Frame list */

  int referenceFrameIndex_ = -1; /*!< pointer to Body's unique reference
                                    frame, which  may not be the Frame's origin.
                                    Assigned automatically by
                                    s2_ProcessTopology */

  int originIndex_ = -1; /*!< Index of body's unique *origin*
                             coordinate system, which may not be the
                             reference frame */

  Joint* parentJointPointer_ = NULL; /*!< Pointer to Body's unique *parent*
                                         Joint datastructure */

  Body* parentBodyPointer_ = NULL; /*!< A pointer to the unique *parent* Body,
                                       set by s2_ProcessTopology */

  Frame* parentFramePointer_ =
    NULL; /*!< parent frame of body reference frame */

  // s2_material_t* materials;      /*!< s2_material_t datastructure */
  Body* geometryInstanceOf_ = NULL; /*!< if this body has identical geometry to
                              another body, this pointer is set to the original
                              body. Otherwise, if this geometry is unique to
                              this body or is the first instance of this
                              geometry, the pointer is set to NULL */
  Body* collisionGeometryInstanceOf_ = NULL; /*!< if this body has identical
                              collision geometry to another body, this pointer
                              is set to the original body. Otherwise, if the
                              collision geometry is unique to this body or is
                              the first instance of this collision geometry, the
                              pointer is set to NULL */

  int index_ = -1; /*!< index in contiguous *run-time* array of Bodies, set
                  by s2_ProcessTopology */
  int level_ = -1; /*!< level in the spanning tree, set
                  by s2_ProcessTopology */
  int materialIndex_ = -1; /*!< offset into context->materials */
  int meshCount_ = 0;      /*!< number of meshes assigned to a body */
  bool isVirtual_ = false; /*!< indicates whether the Body is *virtual*. All
                       user-defined bodies are not virtual. Massless
                       virtual bodies are added to simplify processing
                       kinematics if multiple loops are present in the
                       mechanism */
  ksl_inertia_t inertia_;  /*!< centroidal Mass and moment of Inertia
                                            properties expressed in body origin
                               coordinate system */
  ksl_vec3_t centerOfMassWRTOrigin_;    /*!< vector from body origin to
                                                center of mass */
  ksl_vec3_t centerOfMassWRTReference_; /*!< vector from body reference
                                                frame to center of mass */
  int collisionObjectCount_ = 0;
  int collisionObjectOffset_ = -1; /*!< index to beginning of array of one or
                       more s2_collisionobject_t* datastructures stored in
                       s2_context_t datastructure, s2_collisionobject_t is the
                       level 3 interface to all primitive convex geometries */
  bool drawEnabled_ = true;
  bool isVisible_ = true;
  bool collisionObjectDrawEnabled_ = false;
};

} // namespace Shift

#endif

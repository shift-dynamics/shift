#ifndef SHIFT_CORE_FEA_FLEX_MODEL_H_
#define SHIFT_CORE_FEA_FLEX_MODEL_H_

#include <string>
#include <vector>

#include "ksl/vec3.h"

#include "shift/core/cad/brep.h"
#include "shift/core/fea/element.h"
#include "shift/core/fea/reduction.h"
#include "shift/core/fea/tet_mesh.h"

namespace shift {
namespace core {
namespace fea {

/*!
@brief a solid of a STEP file placed in a body: the body's origin
coordinates of a point p of the solid are \f$s (R \underline{p} +
\underline{t})\f$, with the solid's placement in the file (R, t) and the
geometry's scale s (model length units per meter)
*/
struct PlacedSolid {
  const cad::Model* model = nullptr;
  cad::PlacedBody placed;
  double scale = 1.0;
};

/*!
@brief a mesh of 10-node tetrahedra of a placed solid, in the body's origin
coordinates; \p element_size and \p minimum_element_size are in model units
*/
TetMesh MeshPlacedSolid(const PlacedSolid& solid, double element_size,
                        double minimum_element_size);

/*!
@brief what a frame of a flexible body is tied to, in the body's origin
coordinates

- faces given: the nodes on those B-rep faces;
- kRevolute: the nodes on the cylindrical faces coaxial with the axis
  through position (the bores, whose material lies outside the cylinder,
  with the smallest radius; a pin, if there is no bore); all of them, so a
  frame between two lugs takes both bores;
- kSpherical: the nodes on the spherical faces centred on position (the
  socket before the ball, as for a bore);
- otherwise, or if no such face is found: the nodes within radius of
  position (element_size if radius is 0).
*/
struct AttachmentRequest {
  enum class Joint { kNone, kRevolute, kSpherical };
  std::string name; //!< the frame, for messages
  ksl_vec3_t position = {};
  Joint joint = Joint::kNone;
  ksl_vec3_t axis = {{0.0, 0.0, 1.0}};
  Attachment::Spider spider = Attachment::Spider::kRbe3;
  std::vector<int> faces;
  double radius = 0.0;
};

/*!
@brief the attachment of a request on a mesh of a placed solid; \p
faces_used, if given, receives the B-rep faces tied (empty for a patch round
a point). Throws shift::core::Exception, naming the frame, if no node is
found.
*/
Attachment ChooseAttachment(const TetMesh& mesh, const PlacedSolid& solid,
                            const AttachmentRequest& request,
                            double element_size,
                            std::vector<int>* faces_used = nullptr);

/*! @brief the nodes nearest the given positions (body origin coordinates),
 * one each */
std::vector<int> NearestNodes(const TetMesh& mesh,
                              const std::vector<ksl_vec3_t>& positions);

} // namespace fea
} // namespace core
} // namespace shift

#endif

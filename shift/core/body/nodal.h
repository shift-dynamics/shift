#ifndef SHIFT_BODY_NODAL_H_
#define SHIFT_BODY_NODAL_H_

#include <string>
#include <vector>

#include "ksl/mat3x3.h"
#include "ksl/vec3.h"

#include "shift/core/body/flex.h"

namespace shift {
namespace core {
namespace body {

/*!
@brief A flexible body whose elastic coordinates are the displacements of
retained nodes of a finite element model, by Guyan (static) reduction

The finite element model is lumped: points \f$p\f$ with mass \f$m_p\f$ and
rotary inertia \f$\bar{I}_p\f$, each with \f$d = 3\f$ degrees of freedom (the
translations of a solid element's node) or \f$d = 6\f$ (the translations and
rotations of a beam or shell node), the stiffness matrix \f$K^{\delta}\f$ and
the damping matrix \f$C^{\delta}\f$ over all of them. The fixed points carry
the reference frame: their degrees of freedom are removed, so the shapes hold
the frame still, and they must hold the body still (\f$K^{\delta}\f$ of the
remaining degrees of freedom positive definite). The degrees of freedom of the
retained points are the elastic coordinates \f$\underline{\eta}\f$ (the
masters \f$m\f$, point by point in the order given, d each), and those of the
other points (the slaves \f$s\f$) follow them statically:

\f[
\underline{\delta} = G\underline{\eta}, \qquad
G = \begin{bmatrix} I \\ -(K^{\delta}_{ss})^{-1}K^{\delta}_{sm}
\end{bmatrix}, \qquad
K^{\eta} = G^TK^{\delta}G = K^{\delta}_{mm} -
K^{\delta}_{ms}(K^{\delta}_{ss})^{-1}K^{\delta}_{sm}, \qquad
C^{\eta} = G^TC^{\delta}G
\f]

(Guyan 1965). The rows of \f$G\f$ of each point are its slice of the shape
matrix, \f$\Upsilon_p^t\f$ and, with \f$d = 6\f$, \f$\Upsilon_p^r\f$; a point
of \f$d = 3\f$ has no rotation of its own and so no rotary inertia. The mass
matrix then follows from the points as for any body::Flex. A static load at
the retained points is resolved exactly; the dynamics are those of the
masses driven through \f$G\f$, an upper bound on each frequency that
converges as more points are retained.

The model is given with SetFiniteElementModel, in the coordinates of the
body's origin. Making it from a mesh of the body's geometry (10-node
tetrahedra) or reading it from CalculiX is still to be added, and until then
the type is not registered for model files.
*/
class Nodal : public Flex {
public:
  Nodal()
    : Flex() {}
  Nodal(std::string name)
    : Flex(name) {}
  virtual ~Nodal() override {}

  /*! \brief set the lumped finite element model and reduce it.
   * \p positions, \p masses and \p rotary_inertias of the points (the
   * latter in the origin's axes, about the point, zero for \p dofs = 3);
   * \p dofs, 3 or 6 per point; \p stiffness and \p damping,
   * \f$K^{\delta}\f$ and \f$C^{\delta}\f$, (points * dofs)^2 each,
   * row-major, the degrees of freedom of point p at p * dofs, translations
   * first (damping may be empty); \p fixed_points carry the reference
   * frame; \p retained_points give the elastic coordinates, dofs each, in
   * this order. Throws shift::core::Exception if the sizes do not match, a
   * point is both fixed and retained, or the fixed points do not hold the
   * body still. */
  void SetFiniteElementModel(const std::vector<ksl_vec3_t>& positions,
                             const std::vector<double>& masses,
                             const std::vector<ksl_mat3x3_t>& rotary_inertias,
                             unsigned int dofs,
                             const std::vector<double>& stiffness,
                             const std::vector<double>& damping,
                             const std::vector<unsigned int>& fixed_points,
                             const std::vector<unsigned int>& retained_points);
};

} // namespace body
} // namespace core
} // namespace shift

#endif

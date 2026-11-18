#ifndef SHIFT_CORE_FEA_REDUCTION_H_
#define SHIFT_CORE_FEA_REDUCTION_H_

#include <vector>

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "ksl/screw.h"
#include "ksl/vec3.h"

#include "shift/core/fea/element.h"
#include "shift/core/fea/tet_mesh.h"

namespace shift {
namespace core {
namespace fea {

/*!
@brief a frame of a flexible body tied to nodes of its mesh, with the six
motions (translation, then rotation) of a reference point at position

- kRbe3 (averaging, as Nastran's RBE3): the reference point moves with the
  weighted mean of the nodes, \f$\underline{t} = \underline{\bar{u}} +
  \underline{\theta} \times (\underline{x}_a - \underline{c})\f$ and
  \f$\underline{\theta} = S^{-1} \sum_i (\underline{x}_i - \underline{c})
  \times \underline{u}_i\f$, \f$S = \sum_i (|\underline{d}_i|^2 I -
  \underline{d}_i \underline{d}_i^T)\f$ with \f$\underline{c}\f$ the nodes'
  centroid and \f$\underline{d}_i = \underline{x}_i - \underline{c}\f$ (equal
  weights); it does not stiffen the nodes
- kRbe2 (rigid, as RBE2): the nodes move rigidly with the reference point,
  \f$\underline{u}_i = \underline{t} + \underline{\theta} \times
  (\underline{x}_i - \underline{x}_a)\f$
*/
struct Attachment {
  enum class Spider { kRbe3, kRbe2 };
  ksl_vec3_t position = {};
  Spider spider = Spider::kRbe3;
  std::vector<int> nodes;
};

/*!
@brief the shapes of a flexible body reduced from a finite element model,
ready for body::Flex::SetModel

The reference attachment is held still, so the shapes hold the body's
reference frame still. The elastic coordinates are, in order: the six
motions of each other attachment and the three translations of each
retained node (the static, or constraint, modes: the static deflection of
the mesh for a unit motion of one of them with the others held), then the
kept fixed-interface modes (all attachments and retained nodes held,
normalized to unit modal mass), so that \f$K^{\eta}\f$ is block diagonal
(Craig and Bampton 1968). With no modes kept this is Guyan's (static)
reduction to the attachments and retained nodes.

The points are the mesh's nodes (three translations, lumped masses, no
rotary inertia) followed by one point for each attachment (no mass; its
translation and rotation are those of the reference point).
*/
struct Reduction {
  std::vector<ksl_vec3_t> positions;
  std::vector<double> masses;
  unsigned int elastic_count = 0;
  unsigned int static_count = 0;
  /*! points * elastic_count screws, of point p and coordinate j at
   * p * elastic_count + j */
  std::vector<ksl_screw_t> shapes;
  /*! \f$K^{\eta}\f$ and \f$M^{\eta}\f$, elastic_count^2 each, row-major */
  std::vector<double> stiffness;
  std::vector<double> mass;
  /*! the natural frequencies (Hz) of the fixed-interface modes computed,
   * ascending, whether kept or not */
  std::vector<double> frequencies;
  /*! the translations of the mesh's nodes in each fixed-interface mode
   * computed (unit modal mass), kept or not: x, y, z of node n in mode k at
   * 3 (n * frequencies.size() + k) */
  std::vector<double> computed_shapes;
  /*! the point of each attachment */
  std::vector<unsigned int> attachment_points;
};

/*!
@brief reduce a mesh to the static modes of its attachments (other than \p
reference) and retained nodes, and the fixed-interface modes listed in \p
modes_kept (indices into the lowest \p modes_computed, ascending in
frequency)

Throws shift::core::Exception if the attachments share nodes, an averaging
attachment's nodes lie on a line, or the reference attachment does not hold
the body still.
*/
Reduction Reduce(const TetMesh& mesh, const Assembled& assembled,
                 const std::vector<Attachment>& attachments,
                 unsigned int reference,
                 const std::vector<int>& retained_nodes,
                 unsigned int modes_computed,
                 const std::vector<unsigned int>& modes_kept);

/*!
@brief Reduce, keeping every fixed-interface mode found of the lowest \p
modes_computed (fewer if the mesh has fewer), for KeepModes to choose from
*/
Reduction ReduceKeepingAll(const TetMesh& mesh, const Assembled& assembled,
                           const std::vector<Attachment>& attachments,
                           unsigned int reference,
                           const std::vector<int>& retained_nodes,
                           unsigned int modes_computed);

/*!
@brief the static compliance of a mesh held by its \p reference attachment
at averaging (kRbe3) attachments: the motions (translation, then rotation,
six per attachment in order) per unit load at each,
\f$G K_h^{-1} G^T\f$ of their averaging matrices G (Attachment) and the
inverse of the stiffness held by the reference. Exact for the finite
element model, as the Guyan reduction to the same attachments (Guyan 1965),
whose stiffness is its inverse; but as an averaging attachment does not
stiffen its nodes, the attachments may share nodes, and the held stiffness
is factored once for any number of them.
*/
class HeldCompliance {
public:
  /*! factor the stiffness of \p mesh held by \p reference; keeps a
   * pointer to \p mesh */
  HeldCompliance(const TetMesh& mesh, const Assembled& assembled,
                 const Attachment& reference);
  /*! the compliance at \p attachments, each kRbe3: (6 n)^2, row-major,
   * symmetric */
  std::vector<double>
  Compliance(const std::vector<Attachment>& attachments) const;

private:
  const TetMesh* mesh_;
  /*! the motions of the nodes, u = T u_o, of the free coordinates u_o */
  Eigen::SparseMatrix<double> T_;
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt_;
};

/*!
@brief the damping matrix \f$C^{\eta}\f$ of a reduced body (n^2, row-major)
from damping ratios of its own modes: with the modes \f$\Phi\f$ of \f$K^{\eta},
M^{\eta}\f$ (ascending, unit modal mass) and frequencies \f$\omega_k\f$,
\f$C^{\eta} = M^{\eta} \Phi \, \mathrm{diag}(2 \zeta_k \omega_k) \, \Phi^T
M^{\eta}\f$, \f$\zeta_k\f$ the k-th of \p ratios, or the last of them beyond
their count
*/
std::vector<double> ModalDamping(const Reduction& reduction,
                                 const std::vector<double>& ratios);

} // namespace fea
} // namespace core
} // namespace shift

#endif

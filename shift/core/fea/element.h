#ifndef SHIFT_CORE_FEA_ELEMENT_H_
#define SHIFT_CORE_FEA_ELEMENT_H_

#include <array>
#include <vector>

#include <Eigen/Sparse>

#include "shift/core/fea/tet_mesh.h"

namespace shift {
namespace core {
namespace fea {

/*! @brief an isotropic linear elastic material */
struct Material {
  double youngs_modulus = 0.0; //!< E (Pa)
  double poisson_ratio = 0.0;  //!< nu
  double density = 0.0;        //!< rho (kg/m^3)
};

/*!
@brief the stiffness matrix of a 10-node tetrahedron, 30 x 30 row-major, its
degrees of freedom the translations x, y, z of node a at 3 a

\f$K_e = \int_{V_e} B^T D B \, dV\f$ with the strains \f$\underline{\epsilon}
= B \underline{u}_e\f$ in Voigt order \f$(\epsilon_{xx}, \epsilon_{yy},
\epsilon_{zz}, \gamma_{xy}, \gamma_{yz}, \gamma_{zx})\f$ and \f$D\f$ of the
isotropic material, by the 4-point rule (exact for straight sides, where
\f$B^T D B\f$ is quadratic). The nodes are ordered as in TetMesh. Returns
false if the element's Jacobian is not positive at a quadrature point; \p
volume is the element's volume by the same rule.
*/
bool Tet10Stiffness(const std::array<ksl_vec3_t, 10>& nodes,
                    const Material& material, double* stiffness,
                    double* volume);

/*!
@brief the lumped (diagonal) mass of a 10-node tetrahedron of volume V, by
row-sum scaling of the diagonal of its consistent mass matrix (Hinton, Rock
and Zienkiewicz 1976): \f$\rho V / 36\f$ at each corner and \f$4 \rho V /
27\f$ at each mid-side node, from the diagonal terms \f$6\f$ and \f$32\f$ of
\f$\rho V / 420\f$ (row-sum lumping would give the corners negative mass)
*/
std::array<double, 10> Tet10LumpedMass(double volume, double density);

/*!
@brief the stiffness matrix of a mesh, \f$3n \times 3n\f$ for its n nodes,
and the lumped mass of each node; throws shift::core::Exception naming an
element whose Jacobian is not positive
*/
struct Assembled {
  Eigen::SparseMatrix<double> stiffness;
  std::vector<double> masses;
  double volume = 0.0;
};
Assembled Assemble(const TetMesh& mesh, const Material& material);

} // namespace fea
} // namespace core
} // namespace shift

#endif

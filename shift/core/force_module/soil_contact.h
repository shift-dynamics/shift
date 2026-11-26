#ifndef SHIFT_CORE_FORCE_MODULE_SOIL_CONTACT_H_
#define SHIFT_CORE_FORCE_MODULE_SOIL_CONTACT_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/vec3.h"
#include "shift/core/force_module/base.h"

namespace shift {
namespace core {

namespace body {
class Base;
}
namespace collision_geometry {
class Base;
}
namespace terrain {
class Base;
class Scm;
class Sph;
}

namespace force_module {

/*!
@brief the force of deformable soil (terrain::Scm, the Soil Contact Model,
and terrain::Sph, soil as particles) on bodies pressing into it, which lowers
the soil under them

Each node of the terrain's grid under a body is tested by a ray cast up along
the z axis of the terrain frame, from max_sinkage below the surface to the
surface, against the collision geometry of the bodies, the nodes under each
geometry cast together as a grid (collision_geometry::Base::RayCastGrid; CAD
geometry projects its exact faces onto the grid, Cad::RayCastGrid): the lowest point of a body over the node, if it is below the
surface, gives the sinkage \f$s\f$ of the body there, from the undisturbed
surface. Where several bodies are over a node, the lowest takes it.

The nodes in contact form patches, the sets of nodes joined along the grid.
A patch of area \f$A\f$ and perimeter \f$P\f$ (counted on the grid) has the width
\f$b = 2 A / P\f$ in Bekker's law, the smaller side of a long rectangle.

At each node in contact the pressure \f$p_e\f$ is that of the terrain's
pressure-sinkage law (terrain::Scm::Pressure), with Hunt and Crossley's damping
on the speed of sinkage \f$\dot{s}\f$, so that the force has no jump as contact
starts: \f$p = \max(0, p_e (1 + c \dot{s}))\f$, with the terrain's damping
\f$c\f$. The shear stress follows Mohr-Coulomb with Janosi and Hanamoto's curve,

\f[
\tau = (c + p \tan\phi) \left(1 - e^{-|\underline{j}| / K}\right)
\f]

against the shear displacement \f$\underline{j}\f$ of the soil at the node, in the
plane of the terrain frame. \f$\underline{j}\f$ grows with the slip velocity
\f$\underline{v}\f$ of the body over the node from its accepted value, by the
trapezoidal rule,
\f$\underline{j} = \underline{j}_a + (\underline{v}_a + \underline{v}) (t - t_a) / 2\f$,
so that the force is continuous over the evaluations of a step; it starts
from \f$\underline{v} (t - t_a)\f$ where a body comes onto the node. The pressure acts up along the z axis of the terrain
frame and the shear stress against \f$\underline{j}\f$, each over the node's area
(the spacing squared), at the body's point over the node.

Each evaluation writes the nodes in contact as trials of the terrain (see
terrain::Scm::Trial): their deepest sinkage, their level (the rut sprung back,
terrain::Scm::ReboundLevel), their shear displacement and slip velocity, which
the terrain
keeps when the step is accepted. A model has one soil_contact.

The bodies press on the soil with their collision geometry named in geometry,
or with all of it. Geometry that should touch only the soil, such as the
shoes of a track, can be left out of contact detection ("contact": false).

The soil is the terrain's at each node (its soil or soil map); the
unloading_ratio, damping and max_sinkage are the terrain's.

On SPH soil (a terrain of type sph, or a region of one, see terrain::Sph),
the bodies are markers among the particles, and the load on each is the
soil's on its markers over the last accepted step (terrain::Sph::
get_body_load). Under an SPH region the SCM nodes are the region's.

Outputs: normal_force (the total pressure force of SCM soil), contact_nodes
and sinkage (the deepest, from the undisturbed surface), and sph_force (the
SPH soils' loads on the bodies along the up axis, -z, of each soil's frame).

JSON:
\code
{"type": "soil_contact", "name": "soil", "bodies": ["wheel", "link1", "link2"],
 "geometry": ["tread", "shoe"]}
\endcode
*/
class SoilContact : public Base {
public:
  SoilContact()
    : Base() {
    DeclareParameters();
  }
  SoilContact(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~SoilContact() override {}

  static std::unique_ptr<SoilContact> Create();

  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;
  /*! \brief the stiffness and damping of the SCM soil's force at each node
   * in contact: the pressure with the sinkage (the body's displacement along
   * the terrain's z axis at the point over the node) and with its rate (the
   * damping), the shear stress with that pressure and with the slip
   * velocity through the shear displacement. The ray's point over the node
   * stays on the node's vertical: a displacement \f$\underline{\delta}\f$
   * of the body moves it along the terrain's z axis by \f$\underline{g}^T E
   * \underline{\delta}\f$, \f$\underline{g} = \underline{n} /
   * (\underline{n} \cdot \underline{e}_z)\f$ with the normal
   * \f$\underline{n}\f$ of the body's surface there (by RayCast; the
   * terrain's z axis where the surface is steeper than 84 degrees), which
   * changes the sinkage, the velocity of the body's point there and the
   * moment arm of the force. The SPH soils' loads, of the last accepted step,
   * do not change within a step. */
  bool AddTangents(const double* y, Tangents& tangents) override;

  void UpdateOutputs(double t, const double* y) override;

  double get_normal_force() const {
    return normal_force_;
  }
  double get_sph_force() const {
    return sph_force_;
  }
  double get_contact_nodes() const {
    return contact_nodes_;
  }
  double get_sinkage() const {
    return sinkage_;
  }
  /*! \brief the names of the bodies on the soil (parameter bodies) */
  const std::vector<std::string>& get_body_names() const {
    return body_names_;
  }

protected:
  void DeclareParameters() override;

private:
  /*! \brief a node under a body: the body's lowest point over it */
  struct Hit {
    long i, j;
    double z;               //!< of the point, terrain frame (down)
    int body;               //!< index in bodies_
    ksl_vec3_t point;       //!< inertial frame
    double width;           //!< b of its patch
    int geometry = -1;      //!< index in geometry_ of the body's surface
    double start = 0.0;     //!< z of the start of its ray, terrain frame
  };
  /*! \brief the nodes under a geometry, and its lowest point */
  struct Range {
    long i0, i1, j0, j1;
    double bottom;
  };

  /*! \brief the forces on the bodies, and the trials of the nodes in
   * contact if apply, and their derivatives (if tangents) */
  void Evaluate(bool apply, Tangents* tangents = nullptr);

  static bool registered_;

  std::vector<std::string> body_names_;
  std::vector<std::string> geometry_names_;

  terrain::Scm* terrain_ = nullptr;
  /*! the SPH soils: terrains and regions of type sph */
  std::vector<terrain::Sph*> sph_;
  std::vector<body::Base*> bodies_;
  std::vector<int> load_indices_;
  /*! the collision geometry of the bodies, and the index of its body */
  std::vector<collision_geometry::Base*> geometry_;
  std::vector<int> geometry_body_;

  /* scratch of Evaluate, kept to save allocations */
  std::vector<Range> ranges_;
  std::vector<int> cells_;
  std::vector<Hit> hits_;
  std::vector<double> offsets_;
  std::vector<double> distances_;

  double normal_force_ = 0.0;
  double sph_force_ = 0.0;
  double contact_nodes_ = 0.0;
  double sinkage_ = 0.0;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif

#ifndef SHIFT_FORCEMODULE_TERRAIN_CONTACT_H_
#define SHIFT_FORCEMODULE_TERRAIN_CONTACT_H_

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

namespace force_module {

/*!
@brief contact of a body with the terrain at a set of points on it

The contact force module tests collision geometry, and knows no terrain. This
module tests points of a body against the surface under them, as tires do, for
the skids, feet and corners of a machine or a body that falls on an elevation
map. Each point is tested on its own, against the active terrains of the model
(terrain::Base::Query), or the ground \f$z = 0\f$ without any.

At a point below the surface by the depth \f$d\f$, along the normal
\f$\underline{n}\f$ into the terrain, moving with the velocity
\f$\underline{v}\f$, the normal force follows Hunt and Crossley: it has no jump
as the contact starts, and pushes only,

\f[
f_n = \max\left(0, \; k d^e (1 + a \dot{d})\right), \quad
\dot{d} = \underline{v} \cdot \underline{n}
\f]

with the stiffness \f$k\f$, the exponent \f$e\f$ and the damping \f$a\f$. Friction opposes
the sliding velocity \f$\underline{v}_t\f$ in the plane of the surface, smooth
through zero over the slip_speed \f$v_s\f$, so a body creeps on a slope instead of
holding: \f$-\mu f_n \underline{v}_t / \sqrt{|\underline{v}_t|^2 + v_s^2}\f$. The
force on the body at the point is \f$-f_n \underline{n}\f$ plus the friction.

A point only sees the surface under it: a body with a few points can sink a
bump between them, so space the points closer than the bumps of the terrain.

Outputs: normal_force (the total), contacts (the points in contact) and depth
(the deepest).

JSON (SI units): points are x, y, z of each point in the body's coordinates.
\code
{"type": "terrain_contact", "name": "skids", "body": "sled",
 "points": [1, 0.5, 0.2, 1, -0.5, 0.2, -1, 0.5, 0.2, -1, -0.5, 0.2],
 "stiffness": 2e5, "damping": 1.0, "friction": 0.6}
\endcode
*/
class TerrainContact : public Base {
public:
  TerrainContact()
    : Base() {
    DeclareParameters();
  }
  TerrainContact(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~TerrainContact() override {}

  static std::unique_ptr<TerrainContact> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  std::vector<std::string> get_output_names() const override;
  void UpdateOutputs(double t, const double* y) override;
  void AppendOutputs(std::vector<double>& values) const override;

  double get_normal_force() const {
    return normal_force_;
  }

protected:
  void DeclareParameters() override;

private:
  void Evaluate(bool apply);

  static bool registered_;

  std::string body_name_;
  std::vector<double> points_;
  double stiffness_ = 0.0;
  double exponent_ = 1.0;
  double damping_ = 0.0;
  double friction_ = 0.0;
  double slip_speed_ = 0.01;

  body::Base* body_ = nullptr;
  int load_index_ = -2;
  double normal_force_ = 0.0, contacts_ = 0.0, depth_ = 0.0;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif

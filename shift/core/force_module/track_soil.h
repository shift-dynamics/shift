#ifndef SHIFT_CORE_FORCE_MODULE_TRACK_SOIL_H_
#define SHIFT_CORE_FORCE_MODULE_TRACK_SOIL_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/vec3.h"
#include "shift/core/force_module/base.h"
#include "shift/core/force_module/soil.h"

namespace shift {
namespace core {

namespace body {
class Base;
}

namespace force_module {

/*!
@brief the soil under the shoes of a track: sinkage with loading,
unloading and plasticity, and traction and side forces from shear and earth
pressure

Each shoe is a rectangle in its link's coordinates, the plane of its
grousers' tips: center, normal (out of the shoe, into the soil when it is
on the ground), along (across the grousers, along the track), length
(along) and width. The ground is the plane z = ground (z down), the soil
below it. The shoe is sampled at stations_along x stations_across points,
each carrying an equal share of its area A.

Normal pressure, at each station sunk z below the ground: Bekker's
p_load(z) = (k_c / b + k_phi) z^n (b the smaller of the shoe's length and
width) while the soil is loaded beyond the deepest sinkage z_p reached
there, and below z_p the elastic line back from p_load(z_p),

  p = max(0, min(p_load(z), p_load(z_p) - k_u (z_p - z)))

with the unloading stiffness k_u = unloading_ratio p_load(z_p) / z_p (the
soil springs back by z_p / unloading_ratio). The plastic sinkage z_p is a
state that follows the sinkage from below, dz_p / dt = max(0, z - z_p) /
plastic_time, so that a shoe pressed in keeps its rut when lifted and
reloads along the stiff line.

Traction: the soil's shear on the plane of the tips (Mohr-Coulomb with
Janosi and Hanamoto), sum over the stations of (c + p tan(phi)) A_i, and
Rankine's passive earth pressure on the faces of the soil in the shoe's
way: the leading grouser (width x the passive force per width at the
grouser's embedded depth, min(z, grouser_height)) along the track, and a
side of the shoe (length x the passive force at the shoe's depth) across
it. Both are mobilized by the shear displacement j of the shoe on the soil,
by 1 - exp(-|j| / K), and oppose it. j, in the ground plane, is a state: the
slip velocity of the shoe's center while it is on the soil, relaxing to
zero with shear_time when it is lifted, dj/dt = w v - (1 - w) j /
shear_time, with w = 1 - exp(-z_max / contact_depth) for the deepest
station's sinkage z_max.

The normal force of each station acts at it, the shear at the stations in
proportion to their strength, and the earth pressure at the shoe's center.

Outputs, for each shoe: <shoe>_sinkage (the deepest station's), and the
total normal_force, traction (along the track) and side_force (across it)
on the shoes, in the ground plane, from the shoes' along directions.

JSON (SI units):
\code
{"type": "track_soil", "name": "soil", "soil": {"name": "lete_sand"},
 "shoes": ["link1", "link2", ...], "shoe_center": [0.095, 0, 0.082],
 "shoe_normal": [0, 0, 1], "shoe_along": [1, 0, 0], "shoe_length": 0.18,
 "shoe_width": 0.6, "grouser_height": 0.025}
\endcode
*/
class TrackSoil : public Base {
public:
  TrackSoil()
    : Base() {
    DeclareParameters();
  }

  TrackSoil(std::string name)
    : Base(name) {
    DeclareParameters();
  }

  virtual ~TrackSoil() override {}

  static std::unique_ptr<TrackSoil> Create();

  static bool is_registered() {
    return registered_;
  }

  void DeclareParameters() override;
  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  std::vector<std::string> get_output_names() const override;
  void UpdateOutputs(double t, const double* y) override;
  void AppendOutputs(std::vector<double>& values) const override;

  const SoilProperties& get_soil() const {
    return soil_properties_;
  }

  /*! \brief the pressure at sinkage z of a station whose plastic sinkage
   * is z_p */
  double Pressure(double z, double z_p) const;

  /*! \brief number of states of each shoe: one per station, then j */
  int get_shoe_state_count() const {
    return stations_along_ * stations_across_ + 2;
  }

private:
  /*! \brief the forces on the shoes and the state rates (if dy) */
  void Evaluate(const double* y, double* dy, bool apply);

  static bool registered_;

  Soil soil_;
  std::vector<std::string> shoe_names_;
  ksl_vec3_t center_ = {{0.0, 0.0, 0.0}};
  ksl_vec3_t normal_ = {{0.0, 0.0, 1.0}};
  ksl_vec3_t along_ = {{1.0, 0.0, 0.0}};
  double length_ = 0.0, width_ = 0.0;
  double grouser_height_ = 0.0;
  double ground_ = 0.0;
  int stations_along_ = 3, stations_across_ = 2;
  double unloading_ratio_ = 10.0;
  double plastic_time_ = 0.01;
  double shear_time_ = 0.01;
  double contact_depth_ = 1e-3;

  SoilProperties soil_properties_;
  std::vector<body::Base*> shoes_;
  std::vector<int> load_indices_;
  ksl_vec3_t across_;
  std::vector<double> outputs_;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif

#ifndef SHIFT_TERRAIN_BASE_H_
#define SHIFT_TERRAIN_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <limits>
#include <string>

#include "ksl/types.h"
#include "shift/core/base.h"
#include "shift/core/parameter/factory_array.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

namespace body {
class Base;
}

class Frame;
class Context;

namespace force_module {
struct SoilProperties;
}

namespace terrain {

/*!
@brief Abstract terrain: a surface attached to a frame of the fixed body

Terrains are queried for the surface under a point (see Query), e.g. by the
tire force modules.

A terrain may hold other terrains in its regions: each covers its extent, x
and y in the frame of the terrain that holds it, which it shares. Query and
SoilAt of the terrain answer with the region under the point, so a deformable
terrain can be refined where it matters, e.g. an SPH soil (Sph) inside an SCM
terrain (Scm) where a bucket digs. A region starts from the surface and the
soils of the terrain that holds it; its soil, if given, changes properties of
those (see force_module::Soil::Override). The terrain that holds the regions
loads, connects and updates them (LoadAll, ConnectAll, AcceptStepAll,
UpdateOutputAll); only the terrains of the fixed body are in
Context::get_active_terrain.

\code
"terrain": [{"type": "scm", "name": "ground", "x": [0, 20], "y": [0, 10],
             "spacing": 0.05, "soil": {"name": "LETE sand"},
             "regions": [{"type": "sph", "name": "pit", "x": [8, 10],
                          "y": [4, 6], "depth": 0.3, "spacing": 0.02,
                          "soil": {"youngs_modulus": 2e6,
                                   "poisson_ratio": 0.3}}]}]
\endcode
*/
class Base
  : public shift::core::Base
  , public shift::core::parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name) {
    DeclareParameters();
  }
  Base()
    : shift::core::Base("") {
    DeclareParameters();
  }
  virtual ~Base() override {}

  const std::string& get_type() {
    return type_;
  }

  static const std::string get_array_key() {
    return "terrain";
  }

  static const std::string get_default_type() {
    return "";
  }

  virtual void ActivateObjects(bool parent_is_active) override;

  virtual void Load();
  virtual void Connect();

  /*! @name The terrain and its regions, the terrain first; called by
   * Context on its terrains */
  /*! @{ */
  void LoadAll();
  void ConnectAll();
  void AcceptStepAll();
  void UpdateOutputAll();
  /*! \brief the smallest StepLimit of the terrain and its regions */
  double StepLimitAll() const;
  /*! @} */

  /*! \brief the longest integration step the terrain allows, after Load;
   * infinite by default. A terrain coupled explicitly to the bodies (Sph)
   * limits it, so that its force on them is not too far behind them (see
   * Context::GetStepLimit) */
  virtual double StepLimit() const {
    return std::numeric_limits<double>::infinity();
  }

  /*! \brief the terrain that holds this one in its regions; null for a
   * terrain of the fixed body */
  Base* get_parent_terrain() const {
    return parent_terrain_;
  }

  /*! \brief the active regions of the terrain */
  const std::vector<Base*>& get_regions() const {
    return regions_;
  }

  /*! \brief the terrains, each followed by its regions and theirs, e.g.
   * to find a deformable terrain (Scm) that is a region of a rigid one */
  static std::vector<Base*> WithRegions(const std::vector<Base*>& terrains);

  /*! \brief whether the terrain has an extent, x and y */
  bool has_extent() const {
    return x_range_.size() == 2 && y_range_.size() == 2;
  }

  /*! \brief the extent, [first, last] of x and of y in the terrain frame;
   * empty without one */
  const std::vector<double>& get_x_range() const {
    return x_range_;
  }
  const std::vector<double>& get_y_range() const {
    return y_range_;
  }

  /*! \brief whether x, y of the terrain frame are inside the extent x, y;
   * true everywhere for a terrain without one */
  bool Contains(double x, double y) const;

  /*! \brief the region whose extent holds x, y of the terrain frame; null
   * if none */
  Base* RegionAt(double x, double y) const;

  /*! \brief the surface point on the line through point along the z axis
   * of the terrain frame, and the unit surface normal pointing into the
   * terrain there, both in the inertial frame. Returns false if the
   * terrain has no surface there. */
  virtual bool Query(const ksl_vec3_t& point, ksl_vec3_t& surface,
                     ksl_vec3_t& normal) const {
    (void)point;
    (void)surface;
    (void)normal;
    return false;
  }

  /*! \brief the soil of the terrain under point (inertial frame), along the
   * z axis of the terrain frame; null if the terrain has no soil there */
  virtual const force_module::SoilProperties*
  SoilAt(const ksl_vec3_t& point) const {
    (void)point;
    return nullptr;
  }

  /*! \brief called by Context::AcceptStep after each accepted integration
   * step: a terrain whose shape or soil changes (see Scm) takes the changes
   * of the accepted state here, and only here, so that the evaluations of a
   * step that is rejected leave it as it was */
  virtual void AcceptStep() {}

  /*! \brief called by Context::WriteStates at each output time, before the
   * loggers and the state observer: a terrain whose shape changes gathers
   * the changes since the previous output time here (see
   * Scm::get_output_changes) */
  virtual void UpdateOutput() {}

protected:
  virtual void DeclareParameters() override;

  std::string type_; //!< descriptive name of TerrainBase type
  std::string filename_;
  std::vector<std::vector<std::string>> parent_;
  /*! \brief [first, last] x and y of the terrain frame: the extent of the
   * terrain, optional for some types and required of a region */
  std::vector<double> x_range_;
  std::vector<double> y_range_;

private:
  /*! \brief that each region has an extent inside this one, and that no
   * two overlap */
  void CheckRegions() const;

  shift::core::parameter::FactoryArray<shift::core::terrain::Base>
    region_array_;
  std::vector<Base*> regions_; //!< the active regions
  Base* parent_terrain_ = nullptr;
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace terrain
}
} // namespace shift

#endif

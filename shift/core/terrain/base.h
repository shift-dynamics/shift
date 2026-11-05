#ifndef SHIFT_TERRAIN_BASE_H_
#define SHIFT_TERRAIN_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "ksl/types.h"
#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

namespace body {
class Base;
}

class Frame;
class Context;

namespace terrain {

/*!
@brief Abstract terrain: a surface attached to a frame of the fixed body

Terrains are queried for the surface under a point (see Query), e.g. by the
tire force modules.
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

protected:
  virtual void DeclareParameters() override;

  std::string type_; //!< descriptive name of TerrainBase type
  std::string filename_;
  std::vector<std::vector<std::string>> parent_;
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace terrain
}
} // namespace shift

#endif

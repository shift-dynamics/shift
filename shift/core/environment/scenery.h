#ifndef SHIFT_ENVIRONMENT_SCENERY_H_
#define SHIFT_ENVIRONMENT_SCENERY_H_

#include <memory>
#include <string>
#include <vector>

#include <ksl/SE3.h>

#include "shift/core/geometry/base.h"
#include "shift/core/parameter/factory_array.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

namespace terrain {
class Basic;
}

namespace environment {

/*!
@brief Objects strewn over a terrain ("scenery" at the root of the model
file): trees, bushes, grass, rocks

Each item of the scenery is one of the meshes of geometry, chosen at random,
in its own coordinates with z down and the origin where it meets the ground,
turned at random about the vertical, scaled by a random factor between
scale[0] and scale[1], and set on the surface of the terrain with its
origin depth below it. Its vertical leans toward the normal of the surface
by tilt: 0 stands upright (trees), 1 lies on the slope (grass, stones).

Items are tried at random over the terrain, density per square meter, and
kept where the terrain's material map has one of materials (anywhere when
none are given, or the terrain has no map), with no other material within
margin, and no other item of this scenery, or of the sceneries named in
avoid, nearer than spacing. The same seed always gives the same items.

The scenery is drawn and seen by cameras; it does not collide. A view may
leave out the items beyond draw_distance from its camera (grass that would
cover too few pixels to matter), the ray tracer excepted.

JSON (SI units, angles in the model's units):
\code
"scenery": [
  {"name": "firs", "terrain": "forest",
   "geometry": [{"type": "mesh", "filename": ["qrc:/environments/fir_a.glb"]},
                {"type": "mesh", "filename": ["qrc:/environments/fir_b.glb"]}],
   "materials": ["forest_floor"], "margin": 3.0,
   "density": 0.02, "spacing": 3.5, "scale": [0.7, 1.2], "seed": 1}
]
\endcode
*/
class Scenery : public parameter::List {
public:
  /*! \brief an item of the scenery */
  struct Item {
    int variant = 0; /*!< index in get_geometry() */
    /*! the mesh's coordinates in the inertial frame, without the scale */
    ksl_SE3_t pose;
    double scale = 1.0;
  };

  Scenery() {
    DeclareParameters();
  }
  virtual ~Scenery() override {}

  static const std::string get_array_key() {
    return "scenery";
  }

  /*! \brief activates the meshes, so that the context loads them */
  virtual void ActivateObjects(bool parent_is_active) override;

  /*! \brief throws if a parameter is out of range */
  void Check();

  /*! \brief places the items on terrain, away from those of the sceneries
   * that avoid names (placed before); throws if the terrain has no surface
   * to stand on */
  void Place(const terrain::Basic& terrain,
             const std::vector<const Scenery*>& avoided);

  const std::string& get_name() const {
    return name_;
  }
  const std::string& get_terrain_name() const {
    return terrain_;
  }
  const std::vector<std::string>& get_materials() const {
    return materials_;
  }
  const std::vector<std::string>& get_avoid() const {
    return avoid_;
  }
  std::vector<std::unique_ptr<geometry::Base>>& get_geometry() {
    return geometry_.get_data();
  }
  double get_spacing() const {
    return spacing_;
  }
  /*! \brief distance from the camera beyond which a view may leave the
   * items out (m), 0 for none */
  double get_draw_distance() const {
    return draw_distance_;
  }
  /*! \brief the items, after Place */
  const std::vector<Item>& get_items() const {
    return items_;
  }

protected:
  void DeclareParameters() override;

private:
  std::string name_;
  std::string terrain_;
  parameter::FactoryArray<geometry::Base> geometry_;
  std::vector<std::string> materials_;
  std::vector<std::string> avoid_;
  double density_ = 0.0;
  double spacing_ = 0.0;
  double margin_ = 0.0;
  std::vector<double> scale_ = {1.0, 1.0};
  double tilt_ = 0.0;
  double depth_ = 0.0;
  int seed_ = 1;
  double draw_distance_ = 0.0;
  std::vector<Item> items_;
};

} // namespace environment
} // namespace core
} // namespace shift

#endif

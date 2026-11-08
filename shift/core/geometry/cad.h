#ifndef SHIFT_GEOMETRY_CAD_H_
#define SHIFT_GEOMETRY_CAD_H_

#include "shift/core/cad/brep.h"
#include "shift/core/cad/repair.h"
#include "shift/core/geometry/base.h"

#include <memory>
#include <string>
#include <vector>

namespace shift {
namespace core {
namespace geometry {

/*! \brief a STEP file read and repaired (cad::ReadStep, cad::Repair), and
 * the bodies of it a geometry uses; shared by geometries that load the same
 * file with the same options */
struct CadAsset {
  cad::Model model;
  cad::RepairReport report;
  /*! the bodies of the file used, with their placements in the file's
   * root coordinates */
  std::vector<cad::PlacedBody> bodies;
};

/*!
@brief exact geometry of a body from a STEP file: faces bounded by trimmed
analytic and NURBS surfaces, drawn by evaluating the surfaces on the GPU

The body's origin frame coordinates are the file's coordinates, in meters
whatever the units of the file, times scale.
*/
class Cad : public Base {
public:
  Cad()
    : Base() {
    DeclareParameters();
  }
  Cad(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Cad() override {}

  static std::unique_ptr<Cad> Create();

  static bool is_registered() {
    return registered_;
  }

  virtual void Load() override;

  /*! \brief the file read and repaired; geometries that load the same file
   * with the same options return the same asset */
  const CadAsset* get_asset() const {
    return asset_.get();
  }
  std::shared_ptr<const CadAsset> get_shared_asset() const {
    return asset_;
  }

  /*! \brief model length units per meter */
  double get_scale() const {
    return scale_;
  }

  const std::string& get_filename() const {
    return filename_;
  }

  /*! \brief color (linear RGB) of the faces without a color of their own in
   * the file, empty for the viewer's default */
  const std::vector<double>& get_color() const {
    return color_;
  }

  /*! \brief opacity in the ray tracer, 1 for opaque */
  double get_opacity() const {
    return opacity_;
  }

private:
  void DeclareParameters() override;
  static bool registered_;
  std::string filename_;
  std::string part_;
  double scale_ = 1.0;
  std::vector<double> color_;
  double opacity_ = 1.0;
  std::shared_ptr<const CadAsset> asset_;
};

} // namespace geometry
} // namespace core
} // namespace shift

#endif

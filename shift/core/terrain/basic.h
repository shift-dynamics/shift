#ifndef SHIFT_TERRAIN_BASIC_H_
#define SHIFT_TERRAIN_BASIC_H_

#include <vector>

#include "shift/core/force_module/soil.h"
#include "shift/core/terrain/base.h"
#include "shift/core/terrain/soil_map.h"
#include "shift/core/terrain/surface_texture.h"

namespace shift {
namespace core {
namespace terrain {

/*!
@brief Terrain from an elevation map

The elevation map is a single channel TIFF image (8, 16 or 32 bit integer,
or 16, 32 or 64 bit float samples). Its samples are the control points of the
surface on a grid of x_scale by y_scale (m) in the terrain frame (the frame
named by parent, on the fixed body), with the first sample at its origin.
Integer samples are normalized to 0..1; every sample is then multiplied by
z_scale.

coordinate_type:
- "sae" (default): image columns along x, rows along y; a sample is the z
  of the surface in the terrain frame, z down, so larger samples are lower.
- "iso": image rows along x, columns along y; a sample is the height of the
  surface above the terrain frame's xy plane (its -z, z down).

interpolation_type:
- "bilinear": the surface passes through the control points, and is only C0
  continuous (its normal jumps between cells)
- "bicubic" (default): the natural cubic spline that interpolates the
  samples, a uniform cubic B-spline whose control points are solved for when
  the map is loaded so that the surface passes through every sample. It is C2
  continuous, so tire forces on it change smoothly, and reproduces a plane
  exactly.

Beyond the map the bilinear surface continues with the samples of its edges.
The bicubic surface continues along its slope at the edges for about a cell,
and then levels off.

texture_variation (optional) breaks up the repetition of the texture: the
viewer shifts the tiles by random amounts in patches a few tiles across,
blending between them, and varies the brightness over tens of meters.
surfaces (optional, see SurfaceTexture, at most kMaxSurfaces) are other
textures drawn over the terrain's in patches, e.g. dirt among gravel.

soil (optional, see force_module::Soil) is the soil of the terrain, and
soil_map (optional, see SoilMap) gives soils by area from an integer TIFF;
SoilAt returns the soil under a point, for the models that act on soil.

\\code
"terrain": [{"type": "basic", "name": "hills", "filename": "hills.tiff",
             "parent": [["fixed"], "terrain"], "x_scale": 0.5,
             "y_scale": 0.5, "z_scale": 1.0,
             "texture_filename": "gravel.jpg", "texture_scale": 4.0,
             "normal_map_filename": "gravel_normal.png"}]
\\endcode
*/
class Basic : public Base {
public:
  Basic()
    : Base() {
    DeclareParameters();
  }
  Basic(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Basic() override {}

  static std::unique_ptr<Basic> Create();

  static bool is_registered() {
    return registered_;
  }

  void Load() override;
  void Connect() override;

  /*! \brief Query on the undisturbed surface, the elevation map, which
   * never changes: safe to call from another thread while a deformable
   * terrain (Scm) changes */
  bool QueryUndisturbed(const ksl_vec3_t& point, ksl_vec3_t& surface,
                        ksl_vec3_t& normal) const;

  bool Query(const ksl_vec3_t& point, ksl_vec3_t& surface,
             ksl_vec3_t& normal) const override;

  const force_module::SoilProperties*
  SoilAt(const ksl_vec3_t& point) const override;

  /*! \brief z of the surface (terrain frame, z down) at x, y of the terrain
   * frame, and its slopes dz/dx and dz/dy, as Query finds it: the elevation
   * map (LocalZ) here, and with its deformation for a deformable terrain */
  virtual double SurfaceZ(double x, double y, double& z_x,
                          double& z_y) const {
    return LocalZ(x, y, z_x, z_y);
  }

  /*! \brief the soil at x, y of the terrain frame: of the nearest sample of
   * the soil map, or the terrain's soil without a map; for a region, the
   * soil there of the terrain that holds it, changed by the region's soil;
   * null if the terrain has no soil */
  const force_module::SoilProperties* LocalSoil(double x, double y) const;

  /*! \brief the index in get_soils() of LocalSoil(x, y), -1 for none */
  int LocalSoilIndex(double x, double y) const;

  /*! \brief the soils of the terrain: its own soil first, if given, then
   * those of the soil map */
  const std::vector<force_module::SoilProperties>& get_soils() const {
    return soils_;
  }

  /*! \brief whether an elevation map was loaded; without one (a flat
   * deformable terrain) the surface is the xy plane of the terrain frame */
  bool has_elevation_map() const {
    return !z_.empty();
  }

  /*! \brief z of the elevation map (terrain frame, z down) at x, y of the
   * terrain frame, and its slopes dz/dx and dz/dy; without a map, that of
   * the terrain that holds this region, or 0 */
  double LocalZ(double x, double y, double& z_x, double& z_y) const;

  /*! @name The elevation map, e.g. for drawing */
  /*! @{ */
  /*! \brief extent of the map in the terrain frame: x and y of the last
   * control point (the first is at the origin) */
  double get_x_extent() const;
  double get_y_extent() const;
  /*! \brief the control points of the surface as z (down) of the terrain
   * frame, get_x_samples() along x by get_y_samples() along y, x fastest:
   * the B-spline control points that interpolate the samples for the bicubic
   * surface, the samples themselves for the bilinear */
  const std::vector<double>& get_control_points() const {
    return bicubic_ ? control_ : z_;
  }
  /*! \brief the samples of the map as z (down) of the terrain frame, laid
   * out as get_control_points(); the surface passes through them */
  const std::vector<double>& get_samples() const {
    return z_;
  }
  /*! \brief spacing of the control points along x and y (m) */
  double get_x_scale() const {
    return x_scale_;
  }
  double get_y_scale() const {
    return y_scale_;
  }
  /*! \brief true for the cubic B-spline surface, false for bilinear */
  bool is_bicubic() const {
    return bicubic_;
  }
  /*! \brief control points of the map along x and y */
  long get_x_samples() const {
    return nx_;
  }
  long get_y_samples() const {
    return ny_;
  }
  /*! \brief pose of the terrain frame in the inertial frame */
  const ksl_SE3_t& get_pose() const;
  /*! \brief path of the texture image, empty if none */
  const std::string& get_texture_path() const {
    return texture_path_;
  }
  /*! \brief size of one tile of the texture (m) */
  double get_texture_scale() const {
    return texture_scale_;
  }
  /*! \brief path of the normal map, empty if none */
  const std::string& get_normal_map_path() const {
    return normal_map_path_;
  }
  /*! \brief factor of the slopes of the normal map */
  double get_normal_scale() const {
    return normal_scale_;
  }
  /*! \brief whether the viewer breaks up the repetition of the textures */
  bool get_texture_variation() const {
    return texture_variation_;
  }
  /*! \brief the surfaces drawn over the terrain in patches, in order */
  const std::vector<const SurfaceTexture*>& get_surfaces() const {
    return surface_list_;
  }
  /*! \brief the most surfaces a terrain may have */
  static constexpr int kMaxSurfaces = 2;
  /*! @} */

protected:
  virtual void DeclareParameters() override;
  /*! \brief path of a file next to the model file or in an include
   * directory */
  std::string FindFile(const std::string& name) const;
  /*! \brief whether the terrain needs an elevation map (filename); a
   * deformable terrain is flat without one */
  virtual bool requires_elevation_map() const {
    return true;
  }
  /*! \brief whether the soil of the terrain needs Bekker's pressure-sinkage
   * properties (n, k_c, k_phi); not for a continuum soil (Sph) */
  virtual bool requires_bekker() const {
    return true;
  }
  /*! \brief the Basic terrain that holds this one in its regions, null for
   * a terrain of the fixed body; a region shares its frame, and starts from
   * its surface and soils */
  const Basic* get_parent_basic() const {
    return parent_basic_;
  }
  /*! \brief Query, on the deformed surface (SurfaceZ) or the undisturbed
   * one (LocalZ) */
  bool QuerySurface(const ksl_vec3_t& point, bool deformed, ksl_vec3_t& surface,
                    ksl_vec3_t& normal) const;
  /*! \brief x, y and z of point (inertial frame) in the terrain frame */
  void ToLocal(const ksl_vec3_t& point, double& x, double& y,
               double& z) const;

  const Frame* frame_ = nullptr; //!< terrain frame, on the fixed body
  const Basic* parent_basic_ = nullptr;

private:
  /*! \brief solve for the control points of the B-spline that interpolates
   * the samples */
  void SolveControlPoints();
  /*! \brief control point (i, j) along the x and y axes of the terrain
   * frame as z (down): beyond the map, the sample of its edge for the
   * bilinear surface, and for the bicubic the control point extrapolated
   * linearly by one more along each axis */
  double ControlZ(long i, long j) const;

  std::string texture_filename_;
  double x_scale_ = 1.0;
  double y_scale_ = 1.0;
  double z_scale_ = 1.0;
  double texture_scale_ = 1.0;
  std::string normal_map_filename_;
  double normal_scale_ = 1.0;
  bool texture_variation_ = false;
  parameter::Array<SurfaceTexture> surfaces_;
  std::vector<const SurfaceTexture*> surface_list_;
  std::string coordinate_type_ = "sae";
  std::string interpolation_type_ = "bicubic";

  /* the samples of the map as z (down) of the terrain frame, nx along x by
  ny along y, x fastest */
  std::vector<double> z_;
  /* the control points of the bicubic surface, laid out as z_ */
  std::vector<double> control_;
  long nx_ = 0;
  long ny_ = 0;
  bool bicubic_ = true;
  std::string texture_path_;
  std::string normal_map_path_;

  force_module::Soil soil_;
  SoilMap soil_map_;
  std::vector<force_module::SoilProperties> soils_;
  int base_soil_ = -1; //!< index of soil_ in soils_, -1 if not given

  static bool registered_;
};

} // namespace terrain
} // namespace core
} // namespace shift

#endif

#include "shift/core/terrain/basic.h"

#include <algorithm>
#include <cmath>

#include "ksl/vec3.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"
#include "shift/core/log.h"
#include "shift/core/terrain/tiff.h"
#include "shift/core/utility.h"

bool shift::core::terrain::Basic::registered_ =
  shift::core::Factory<shift::core::terrain::Base>::get()->RegisterModule(
    "basic", shift::core::terrain::Basic::Create);

std::unique_ptr<shift::core::terrain::Basic> shift::core::terrain::Basic::Create() {
  return std::make_unique<shift::core::terrain::Basic>();
}

void shift::core::terrain::Basic::DeclareParameters() {
  shift::core::terrain::Base::DeclareParameters();
  DeclareParameter("texture_filename", &texture_filename_,
               "image drawn on the terrain: a file next to the model or in "
               "an include directory, or a texture built into the viewer, "
               "e.g. qrc:/textures/asphalt.jpg; optional",
               false);
  DeclareParameter("x_scale", &x_scale_,
               "spacing of the elevation map samples along x (m), defaults "
               "to 1 m if not specified",
               false);
  DeclareParameter("y_scale", &y_scale_,
               "spacing of the elevation map samples along y (m), defaults "
               "to 1 m if not specified",
               false);
  DeclareParameter("z_scale", &z_scale_,
               "factor of the elevation map samples (m), after integer "
               "samples are normalized to 0..1; defaults to 1",
               false);
  DeclareParameter("texture_scale", &texture_scale_,
               "size of one tile of the texture image (m), defaults to 1",
               false);
  DeclareParameter("normal_map_filename", &normal_map_filename_,
               "path to a tangent space normal map of the texture (as a "
               "bump map: it changes how light falls on the surface, not "
               "its shape), tiled like the texture: a file, or a built-in "
               "texture such as qrc:/textures/asphalt_normal.jpg; optional",
               false);
  DeclareParameter("normal_scale", &normal_scale_,
               "strength of the normal map: its slopes are multiplied by "
               "this factor; defaults to 1",
               false);
  DeclareParameter("texture_variation", &texture_variation_,
               "break up the repetition of the texture in the viewer: tiles "
               "shifted by random amounts in patches, and the brightness "
               "varied over tens of meters; defaults to false",
               false);
  DeclareParameter("surfaces", &surfaces_,
               "other textures drawn over the terrain's in patches, at most "
               "2, each with \"texture_filename\", \"coverage\" and "
               "\"patch_size\" (see terrain::SurfaceTexture); optional",
               false);
  DeclareParameter(
    "coordinate_type", &coordinate_type_,
    "specify coordinate convention to be used in model: acceptable values are "
    "\"sae\" and \"iso\". The \"sae\" coordinate convention has positive z "
    "pointing down, which is commonly used for space craft, robotics and "
    "vehicle dynamics applications. The \"iso\" coordinate convention has "
    "positive z pointing up; defaults to \"sae\" if not specified",
    false);
  DeclareParameter(
    "interpolation_type", &interpolation_type_,
    "specify method to interpolate between control points: acceptable values "
    "are \"bilinear\" and \"bicubic\"; \"bilinear\" sampling is faster, but is "
    "only C0 continuous. \"bicubic\" sampling is C2 continuous but slower to "
    "compute. defaults to \"bicubic\" if not specified.",
    false);
  DeclareParameter("soil", &soil_,
                   "the soil of the terrain (see force_module::Soil), for the "
                   "models that act on soil; optional",
                   false);
  DeclareParameter("soil_map", &soil_map_,
                   "soils by area from an integer TIFF: \"filename\", "
                   "\"x_scale\", \"y_scale\" and \"soils\", each a soil "
                   "with the \"value\" of the samples that select it "
                   "(see terrain::SoilMap); optional",
                   false);
}

void shift::core::terrain::Base::Load() {}

void shift::core::terrain::Base::Connect() {}

std::string shift::core::terrain::Basic::FindFile(const std::string& name) const {
  if(shift::core::utility::IsResourcePath(name)) {
    return name;
  }
  const std::string file = context_->FindFile(name);
  if(!file.empty()) {
    return file;
  }
  throw shift::core::Exception("Terrain " + get_name() + ": file " + name +
                               " not found");
}

void shift::core::terrain::Basic::Load() {
  const std::string prefix = "Terrain " + get_name() + ": ";
  if(coordinate_type_ != "sae" && coordinate_type_ != "iso") {
    throw shift::core::Exception(prefix + "coordinate_type must be \"sae\" "
                                 "or \"iso\", not \"" + coordinate_type_ + "\"");
  }
  if(interpolation_type_ != "bilinear" && interpolation_type_ != "bicubic") {
    throw shift::core::Exception(prefix + "interpolation_type must be "
                                 "\"bilinear\" or \"bicubic\", not \"" +
                                 interpolation_type_ + "\"");
  }
  if(!(x_scale_ > 0.0) || !(y_scale_ > 0.0)) {
    throw shift::core::Exception(prefix + "x_scale and y_scale must be positive");
  }
  bicubic_ = interpolation_type_ == "bicubic";
  const bool sae = coordinate_type_ == "sae";
  z_.clear();
  control_.clear();
  nx_ = ny_ = 0;
  if(filename_.empty()) {
    if(requires_elevation_map() && !parent_basic_) {
      throw shift::core::Exception(prefix + "needs filename, the elevation "
                                            "map");
    }
  } else {
    const std::string path = FindFile(filename_);
    TiffImage image;
    std::string error;
    if(!ReadTiff(path, image, error)) {
      throw shift::core::Exception(prefix + path + ": " + error);
    }
    /* integer samples normalized to 0..1, then to z (down) on the x, y
    grid of the terrain frame */
    const double range = image.Range();
    nx_ = sae ? image.width : image.height;
    ny_ = sae ? image.height : image.width;
    z_.resize(size_t(nx_) * ny_);
    for(long row = 0; row < image.height; row++) {
      for(long column = 0; column < image.width; column++) {
        const double value =
          z_scale_ * image.samples[size_t(row) * image.width + column] / range;
        const long i = sae ? column : row; // along x
        const long j = sae ? row : column; // along y
        z_[size_t(j) * nx_ + i] = sae ? value : -value;
      }
    }
    if(bicubic_) {
      SolveControlPoints();
    }
  }
  /* the soils: the terrain's own first, then those of the map; a region's are
  those of the terrain that holds it, changed by its own soil */
  soils_.clear();
  base_soil_ = -1;
  if(parent_basic_) {
    if(soil_map_.is_given()) {
      throw shift::core::Exception(prefix + "a region takes the soil map of "
                                            "the terrain that holds it");
    }
    soils_ = parent_basic_->soils_;
    base_soil_ = parent_basic_->base_soil_;
    if(soil_.is_given()) {
      for(auto& soil : soils_) {
        soil_.Override(soil, "Terrain " + get_name());
      }
    }
  } else if(soil_.is_given()) {
    soils_.push_back(soil_.Resolve("Terrain " + get_name(), requires_bekker()));
    base_soil_ = 0;
  }
  if(soil_map_.is_given() && !parent_basic_) {
    soil_map_.Load(FindFile(soil_map_.get_filename()), sae, base_soil_,
                   "Terrain " + get_name(), soils_);
  }
  /* the texture only changes the look: draw without it if it is missing */
  texture_path_.clear();
  if(!texture_filename_.empty()) {
    try {
      texture_path_ = FindFile(texture_filename_);
    } catch(const shift::core::Exception& e) {
      SHIFT_WARNING(std::string(e.what()) + "; drawn without a texture");
    }
  }
  normal_map_path_.clear();
  if(!normal_map_filename_.empty()) {
    try {
      normal_map_path_ = FindFile(normal_map_filename_);
    } catch(const shift::core::Exception& e) {
      SHIFT_WARNING(std::string(e.what()) + "; drawn without a normal map");
    }
  }
  /* the surfaces, like the texture, only change the look */
  const std::string owner = "Terrain " + get_name();
  if(int(surfaces_.get_data().size()) > kMaxSurfaces) {
    throw shift::core::Exception(owner + ": at most " +
                                 std::to_string(kMaxSurfaces) +
                                 " surfaces can be drawn");
  }
  surface_list_.clear();
  for(auto& surface : surfaces_.get_data()) {
    surface->Check(owner);
    std::string texture, normal_map;
    try {
      texture = FindFile(surface->get_texture_filename());
      if(!surface->get_normal_map_filename().empty()) {
        normal_map = FindFile(surface->get_normal_map_filename());
      }
    } catch(const shift::core::Exception& e) {
      SHIFT_WARNING(std::string(e.what()) + "; drawn without it");
    }
    surface->set_paths(texture, normal_map);
    surface_list_.push_back(surface.get());
  }
}

void shift::core::terrain::Basic::Connect() {
  const std::string prefix = "Terrain " + get_name() + ": ";
  parent_basic_ = nullptr;
  if(get_parent_terrain()) {
    /* a region: the frame of the terrain that holds it, connected first */
    parent_basic_ = dynamic_cast<const Basic*>(get_parent_terrain());
    if(!parent_basic_) {
      throw shift::core::Exception(prefix + "a region of a terrain of type "
                                            "\"basic\", \"scm\" or \"sph\" "
                                            "only");
    }
    if(!parent_.empty()) {
      throw shift::core::Exception(prefix + "a region takes the frame of the "
                                            "terrain that holds it: no parent");
    }
    frame_ = parent_basic_->frame_;
    return;
  }
  /* parent: [["fixed"], "frame"] or [["fixed"], ["frame"]] */
  std::string frame_name = "origin";
  if(parent_.size() >= 2 && !parent_[1].empty()) {
    frame_name = parent_[1][0];
  }
  frame_ = nullptr;
  for(auto& frame : context_->get_fixed_body()->get_frames()) {
    if(frame->get_name() == frame_name || (frame_name == "origin" && frame->is_origin())) {
      frame_ = frame.get();
    }
  }
  if(!frame_) {
    throw shift::core::Exception(prefix + "frame " + frame_name +
                                 " not found on the fixed body");
  }
}

const ksl_SE3_t& shift::core::terrain::Basic::get_pose() const {
  return context_->get_dhva().d[frame_->get_index()];
}

double shift::core::terrain::Basic::get_x_extent() const {
  return (nx_ - 1) * x_scale_;
}

double shift::core::terrain::Basic::get_y_extent() const {
  return (ny_ - 1) * y_scale_;
}

/*!
The bicubic surface at sample k of a row is (c[k - 1] + 4 c[k] + c[k + 1]) / 6,
from the basis of LocalZ at s = 0. Beyond the map the control points are
extrapolated linearly, c[-1] = 2 c[0] - c[1] and c[n] = 2 c[n - 1] - c[n - 2]
(see ControlZ): the natural end condition, a second derivative of zero at the
edges, under which the surface at the first and last samples is c[0] and
c[n - 1]. Setting the surface to the samples z[k] gives the tridiagonal system

  c[0] = z[0],  (c[k - 1] + 4 c[k] + c[k + 1]) / 6 = z[k],  c[n - 1] = z[n - 1]

diagonally dominant, solved by the Thomas algorithm along each row and then
along each column, since the surface is a tensor product. Samples on a plane
give control points on the same plane, so a plane is reproduced exactly.
*/
void shift::core::terrain::Basic::SolveControlPoints() {
  control_ = z_;
  auto solve = [](std::vector<double>& c, size_t start, size_t stride,
                  long n) {
    if(n < 3) {
      return; // c = z
    }
    auto at = [&](long k) -> double& { return c[start + size_t(k) * stride]; };
    /* the interior rows times 6, [1 4 1], with c[0] and c[n - 1] known:
    forward elimination of k = 1 .. n - 2, then back substitution */
    std::vector<double> upper(n, 0.0), right(n, 0.0);
    right[0] = at(0);
    for(long k = 1; k <= n - 2; k++) {
      const double denominator = 4.0 - upper[k - 1];
      upper[k] = 1.0 / denominator;
      right[k] = (6.0 * at(k) - right[k - 1]) / denominator;
    }
    /* the last interior row also has c[n - 1] on its right */
    for(long k = n - 2; k >= 1; k--) {
      const double next = at(k + 1);
      at(k) = right[k] - upper[k] * next;
    }
  };
  for(long j = 0; j < ny_; j++) {
    solve(control_, size_t(j) * nx_, 1, nx_);
  }
  for(long i = 0; i < nx_; i++) {
    solve(control_, size_t(i), size_t(nx_), ny_);
  }
}

/*!
Bilinear: the sample, clamped to the map. Bicubic: the control point, and
beyond the map one control point more along each axis, extrapolated linearly
from the two at the edge, which is held further out, so that beyond the map the
surface continues along its slope at the edge for about a cell and then levels
off.
*/
double shift::core::terrain::Basic::ControlZ(long i, long j) const {
  const std::vector<double>& c = get_control_points();
  if(!bicubic_) {
    i = std::clamp(i, 0L, nx_ - 1);
    j = std::clamp(j, 0L, ny_ - 1);
    return c[size_t(j) * nx_ + i];
  }
  /* the indices and weights of the control points of the map that give
  index k along an axis of n */
  auto along = [](long k, long n, long index[2], double weight[2]) {
    if(n < 2 || (k >= 0 && k < n)) {
      index[0] = index[1] = std::clamp(k, 0L, n - 1);
      weight[0] = 1.0;
      weight[1] = 0.0;
    } else if(k < 0) {
      index[0] = 0;
      index[1] = 1;
      weight[0] = 2.0;
      weight[1] = -1.0;
    } else {
      index[0] = n - 1;
      index[1] = n - 2;
      weight[0] = 2.0;
      weight[1] = -1.0;
    }
  };
  long ii[2], jj[2];
  double wi[2], wj[2];
  along(i, nx_, ii, wi);
  along(j, ny_, jj, wj);
  double z = 0.0;
  for(int b = 0; b < 2; b++) {
    for(int a = 0; a < 2; a++) {
      if(wi[a] != 0.0 && wj[b] != 0.0) {
        z += wi[a] * wj[b] * c[size_t(jj[b]) * nx_ + ii[a]];
      }
    }
  }
  return z;
}

double shift::core::terrain::Basic::LocalZ(double x, double y, double& z_x,
                                           double& z_y) const {
  if(z_.empty()) {
    if(parent_basic_) {
      return parent_basic_->LocalZ(x, y, z_x, z_y);
    }
    z_x = z_y = 0.0;
    return 0.0;
  }
  const double u = x / x_scale_;
  const double v = y / y_scale_;
  const long i = static_cast<long>(std::floor(u));
  const long j = static_cast<long>(std::floor(v));
  const double s = u - i;
  const double t = v - j;
  if(!bicubic_) {
    const double z00 = ControlZ(i, j), z10 = ControlZ(i + 1, j);
    const double z01 = ControlZ(i, j + 1), z11 = ControlZ(i + 1, j + 1);
    z_x = ((z10 - z00) * (1.0 - t) + (z11 - z01) * t) / x_scale_;
    z_y = ((z01 - z00) * (1.0 - s) + (z11 - z10) * s) / y_scale_;
    return (z00 * (1.0 - s) + z10 * s) * (1.0 - t) +
           (z01 * (1.0 - s) + z11 * s) * t;
  }
  /* uniform cubic B-spline basis over control points i - 1 .. i + 2, and its
  derivative */
  auto basis = [](double a, double b[4], double db[4]) {
    const double a2 = a * a, a3 = a2 * a;
    b[0] = (1.0 - 3.0 * a + 3.0 * a2 - a3) / 6.0;
    b[1] = (4.0 - 6.0 * a2 + 3.0 * a3) / 6.0;
    b[2] = (1.0 + 3.0 * a + 3.0 * a2 - 3.0 * a3) / 6.0;
    b[3] = a3 / 6.0;
    db[0] = (-3.0 + 6.0 * a - 3.0 * a2) / 6.0;
    db[1] = (-12.0 * a + 9.0 * a2) / 6.0;
    db[2] = (3.0 + 6.0 * a - 9.0 * a2) / 6.0;
    db[3] = 3.0 * a2 / 6.0;
  };
  double bs[4], dbs[4], bt[4], dbt[4];
  basis(s, bs, dbs);
  basis(t, bt, dbt);
  double z = 0.0;
  z_x = z_y = 0.0;
  for(int b = 0; b < 4; b++) {
    for(int a = 0; a < 4; a++) {
      const double c = ControlZ(i - 1 + a, j - 1 + b);
      z += bs[a] * bt[b] * c;
      z_x += dbs[a] * bt[b] * c;
      z_y += bs[a] * dbt[b] * c;
    }
  }
  z_x /= x_scale_;
  z_y /= y_scale_;
  return z;
}

bool shift::core::terrain::Basic::Query(const ksl_vec3_t& point,
                                        ksl_vec3_t& surface,
                                        ksl_vec3_t& normal) const {
  return QuerySurface(point, true, surface, normal);
}

bool shift::core::terrain::Basic::QueryUndisturbed(const ksl_vec3_t& point,
                                                   ksl_vec3_t& surface,
                                                   ksl_vec3_t& normal) const {
  return QuerySurface(point, false, surface, normal);
}

bool shift::core::terrain::Basic::QuerySurface(const ksl_vec3_t& point,
                                               bool deformed,
                                               ksl_vec3_t& surface,
                                               ksl_vec3_t& normal) const {
  if(!frame_ ||
     (z_.empty() && requires_elevation_map() && !parent_basic_)) {
    return false;
  }
  double x, y, z_point;
  ToLocal(point, x, y, z_point);
  if(const Base* region = RegionAt(x, y)) {
    const Basic* basic = dynamic_cast<const Basic*>(region);
    return basic ? basic->QuerySurface(point, deformed, surface, normal)
                 : region->Query(point, surface, normal);
  }
  const ksl_SE3_t& d = get_pose();
  double z_x, z_y;
  const double z =
    deformed ? SurfaceZ(x, y, z_x, z_y) : LocalZ(x, y, z_x, z_y);
  /* the solid is at z > z(x, y) (z down); into it, (-z_x, -z_y, 1) */
  const double length = std::sqrt(z_x * z_x + z_y * z_y + 1.0);
  const double local_normal[3] = {-z_x / length, -z_y / length, 1.0 / length};
  const double local_surface[3] = {x, y, z};
  for(int k = 0; k < 3; k++) {
    normal.at[k] = d.R.v0.at[k] * local_normal[0] +
                   d.R.v1.at[k] * local_normal[1] +
                   d.R.v2.at[k] * local_normal[2];
    surface.at[k] = d.t.at[k] + d.R.v0.at[k] * local_surface[0] +
                    d.R.v1.at[k] * local_surface[1] +
                    d.R.v2.at[k] * local_surface[2];
  }
  return true;
}

void shift::core::terrain::Basic::ToLocal(const ksl_vec3_t& point, double& x,
                                          double& y, double& z) const {
  /* R^T (p - t) */
  const ksl_SE3_t& d = get_pose();
  const ksl_vec3_t r = {{point.x - d.t.x, point.y - d.t.y, point.z - d.t.z}};
  x = ksl_dot_vv(&d.R.v0, &r);
  y = ksl_dot_vv(&d.R.v1, &r);
  z = ksl_dot_vv(&d.R.v2, &r);
}

int shift::core::terrain::Basic::LocalSoilIndex(double x, double y) const {
  if(parent_basic_) {
    return parent_basic_->LocalSoilIndex(x, y);
  }
  return soil_map_.is_given() ? soil_map_.Index(x, y) : base_soil_;
}

const shift::core::force_module::SoilProperties*
shift::core::terrain::Basic::LocalSoil(double x, double y) const {
  const int index = LocalSoilIndex(x, y);
  return index >= 0 ? &soils_[index] : nullptr;
}

const shift::core::force_module::SoilProperties*
shift::core::terrain::Basic::SoilAt(const ksl_vec3_t& point) const {
  if(!frame_) {
    return nullptr;
  }
  double x, y, z;
  ToLocal(point, x, y, z);
  if(const Base* region = RegionAt(x, y)) {
    return region->SoilAt(point);
  }
  return LocalSoil(x, y);
}

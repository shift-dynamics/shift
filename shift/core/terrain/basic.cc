#include "shift/core/terrain/basic.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <tiffio.h>

#include "ksl/vec3.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"
#include "shift/core/log.h"
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
  DeclareParameter("property_map_filename", &property_map_filename_,
               "path to filename that defines terrain texture", false);
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
}

void shift::core::terrain::Base::Load() {}

void shift::core::terrain::Base::Connect() {}

std::string shift::core::terrain::Basic::FindFile(const std::string& name) const {
  if(!name.empty() && name[0] == '/') {
    return name;
  }
  if(shift::core::utility::IsResourcePath(name)) {
    return name;
  }
  const std::string file =
    shift::core::utility::GetParentPath(context_->get_filename()) + "/" + name;
  if(shift::core::utility::FileExists(file)) {
    return file;
  }
  for(const auto& directory :
      (*context_)["model_parameters"]["include_directories"]
        .get<std::vector<std::string>>()) {
    const std::string candidate = directory + "/" + name;
    if(shift::core::utility::FileExists(candidate)) {
      return candidate;
    }
  }
  throw shift::core::Exception("Terrain " + get_name() + ": file " + name +
                               " not found");
}

/* the samples of a single channel TIFF image, row major, integers
normalized to 0..1 */
/* an IEEE 754 half precision (16 bit) float */
static double HalfToDouble(uint16_t h) {
  const int exponent = (h >> 10) & 0x1f;
  const int mantissa = h & 0x3ff;
  double value;
  if(exponent == 0) { // zero or subnormal
    value = std::ldexp(double(mantissa), -24);
  } else if(exponent == 31) { // infinity or NaN
    value = mantissa ? std::nan("") : INFINITY;
  } else {
    value = std::ldexp(double(mantissa | 0x400), exponent - 25);
  }
  return (h & 0x8000) ? -value : value;
}

static std::vector<double> ReadTiff(const std::string& path, long& width,
                                    long& height, std::string& error) {
  std::vector<double> samples;
  TIFFSetWarningHandler(nullptr);
  TIFF* tiff = TIFFOpen(path.c_str(), "r");
  if(!tiff) {
    error = "could not read the TIFF image";
    return samples;
  }
  uint32_t w = 0, h = 0;
  uint16_t bits = 0, channels = 1, format = SAMPLEFORMAT_UINT, planar = 1;
  TIFFGetField(tiff, TIFFTAG_IMAGEWIDTH, &w);
  TIFFGetField(tiff, TIFFTAG_IMAGELENGTH, &h);
  TIFFGetFieldDefaulted(tiff, TIFFTAG_BITSPERSAMPLE, &bits);
  TIFFGetFieldDefaulted(tiff, TIFFTAG_SAMPLESPERPIXEL, &channels);
  TIFFGetFieldDefaulted(tiff, TIFFTAG_SAMPLEFORMAT, &format);
  TIFFGetFieldDefaulted(tiff, TIFFTAG_PLANARCONFIG, &planar);
  const bool is_float = format == SAMPLEFORMAT_IEEEFP;
  const bool is_signed = format == SAMPLEFORMAT_INT;
  if(w < 2 || h < 2 || (is_float && bits != 16 && bits != 32 && bits != 64) ||
     (!is_float && bits != 8 && bits != 16 && bits != 32) ||
     TIFFIsTiled(tiff)) {
    error = "the elevation map must be a striped TIFF of at least 2 x 2 "
            "8, 16 or 32 bit integer or 16, 32 or 64 bit float samples";
    TIFFClose(tiff);
    return samples;
  }
  /* the first channel of each pixel */
  const size_t stride = planar == PLANARCONFIG_CONTIG ? channels : 1;
  std::vector<uint8_t> row(TIFFScanlineSize(tiff));
  samples.resize(size_t(w) * h);
  const double range = is_float ? 1.0 : std::ldexp(1.0, bits) - 1.0;
  for(uint32_t y = 0; y < h; y++) {
    if(TIFFReadScanline(tiff, row.data(), y, 0) < 0) {
      error = "could not read row " + std::to_string(y);
      TIFFClose(tiff);
      return {};
    }
    for(uint32_t x = 0; x < w; x++) {
      const uint8_t* p = row.data() + size_t(x) * stride * (bits / 8);
      double value = 0.0;
      if(is_float && bits == 16) {
        uint16_t u;
        std::memcpy(&u, p, 2);
        value = HalfToDouble(u);
      } else if(is_float && bits == 32) {
        float f;
        std::memcpy(&f, p, 4);
        value = f;
      } else if(is_float) {
        std::memcpy(&value, p, 8);
      } else if(bits == 8) {
        value = is_signed ? int8_t(*p) : *p;
      } else if(bits == 16) {
        uint16_t u;
        std::memcpy(&u, p, 2);
        value = is_signed ? double(int16_t(u)) : u;
      } else {
        uint32_t u;
        std::memcpy(&u, p, 4);
        value = is_signed ? double(int32_t(u)) : u;
      }
      samples[size_t(y) * w + x] = value / range;
    }
  }
  TIFFClose(tiff);
  width = w;
  height = h;
  return samples;
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
  const std::string path = FindFile(filename_);
  long width = 0, height = 0;
  std::string error;
  const std::vector<double> samples = ReadTiff(path, width, height, error);
  if(!error.empty()) {
    throw shift::core::Exception(prefix + path + ": " + error);
  }
  /* to z (down) on the x, y grid of the terrain frame */
  const bool sae = coordinate_type_ == "sae";
  nx_ = sae ? width : height;
  ny_ = sae ? height : width;
  z_.resize(size_t(nx_) * ny_);
  for(long row = 0; row < height; row++) {
    for(long column = 0; column < width; column++) {
      const double value = z_scale_ * samples[size_t(row) * width + column];
      const long i = sae ? column : row; // along x
      const long j = sae ? row : column; // along y
      z_[size_t(j) * nx_ + i] = sae ? value : -value;
    }
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
}

void shift::core::terrain::Basic::Connect() {
  const std::string prefix = "Terrain " + get_name() + ": ";
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

double shift::core::terrain::Basic::ControlZ(long i, long j) const {
  i = std::clamp(i, 0L, nx_ - 1);
  j = std::clamp(j, 0L, ny_ - 1);
  return z_[size_t(j) * nx_ + i];
}

double shift::core::terrain::Basic::LocalZ(double x, double y, double& z_x,
                                           double& z_y) const {
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
  if(!frame_ || z_.empty()) {
    return false;
  }
  /* the point in the terrain frame, R^T (p - t) */
  const ksl_SE3_t& d = get_pose();
  const ksl_vec3_t r = {{point.x - d.t.x, point.y - d.t.y, point.z - d.t.z}};
  const double x = ksl_dot_vv(&d.R.v0, &r);
  const double y = ksl_dot_vv(&d.R.v1, &r);
  double z_x, z_y;
  const double z = LocalZ(x, y, z_x, z_y);
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

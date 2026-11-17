#include "shift/gui/environment.h"

#include <QFile>

#include <algorithm>
#include <cmath>
#include <sstream>

#ifdef SHIFT_HAVE_OPENEXR
#include <ImfArray.h>
#include <ImfRgbaFile.h>
#include <ImfStdIO.h>
#endif


static constexpr float kPi = 3.14159265358979f;
static constexpr int kSampleCount = 512;

/*! low discrepancy point i of n in the unit square */
static void Hammersley(int i, int n, float& u, float& v) {
  unsigned int bits = static_cast<unsigned int>(i);
  bits = (bits << 16u) | (bits >> 16u);
  bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
  bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
  bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
  bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
  u = (static_cast<float>(i) + 0.5f) / static_cast<float>(n);
  v = static_cast<float>(bits) * 2.3283064365386963e-10f;
}

/*! orthonormal basis (t, b, n) */
static void Basis(const ksl_vec3f_t& n, ksl_vec3f_t& t, ksl_vec3f_t& b) {
  const ksl_vec3f_t a = std::fabs(n.x) < 0.9f ? ksl_vec3f(1.f, 0.f, 0.f)
                                              : ksl_vec3f(0.f, 1.f, 0.f);
  ksl_cross_vvf(&a, &n, &t);
  ksl_vec3f_normalize(&t);
  ksl_cross_vvf(&n, &t, &b);
}

/*! the unit direction t x + b y + n z of the basis of n */
static ksl_vec3f_t InBasis(const ksl_vec3f_t& n, float x, float y, float z) {
  ksl_vec3f_t t, b;
  Basis(n, t, b);
  ksl_vec3f_t d;
  ksl_product_avf(z, &n, &d);
  ksl_axpy_vvf(x, &t, &d);
  ksl_axpy_vvf(y, &b, &d);
  ksl_vec3f_normalize(&d);
  return d;
}

/*! GGX distributed half vector about n, alpha = roughness^2 */
static ksl_vec3f_t SampleGgx(float u, float v, float alpha,
                             const ksl_vec3f_t& n) {
  const float phi = 2.f * kPi * u;
  const float cos_theta =
    std::sqrt((1.f - v) / (1.f + (alpha * alpha - 1.f) * v));
  const float sin_theta = std::sqrt(std::max(1.f - cos_theta * cos_theta, 0.f));
  return InBasis(n, sin_theta * std::cos(phi), sin_theta * std::sin(phi),
                 cos_theta);
}

/*! cosine distributed direction about n */
static ksl_vec3f_t SampleCosine(float u, float v, const ksl_vec3f_t& n) {
  const float phi = 2.f * kPi * u;
  const float r = std::sqrt(v);
  return InBasis(n, r * std::cos(phi), r * std::sin(phi),
                 std::sqrt(std::max(1.f - v, 0.f)));
}

/*! the direction 2 (a.h) h - a, a reflected about h */
static ksl_vec3f_t Reflected(const ksl_vec3f_t& a, const ksl_vec3f_t& h) {
  ksl_vec3f_t l;
  ksl_product_avf(2.f * ksl_dot_vvf(&a, &h), &h, &l);
  ksl_nxpy_vvf(&a, &l);
  return l;
}

/*! a (1 - t) + b t */
static ksl_vec3f_t Mix(const ksl_vec3f_t& a, const ksl_vec3f_t& b, float t) {
  ksl_vec3f_t m;
  ksl_product_avf(1.f - t, &a, &m);
  ksl_axpy_vvf(t, &b, &m);
  return m;
}

/*! k v */
static ksl_vec3f_t Scaled(const ksl_vec3f_t& v, float k) {
  ksl_vec3f_t s;
  ksl_vec3f_scaled(&v, k, &s);
  return s;
}

/*! Smith-Schlick visibility term with the image based lighting remapping
 * k = alpha / 2 */
static float GeometrySmithIbl(float n_dot_v, float n_dot_l, float roughness) {
  const float k = roughness * roughness / 2.f;
  return n_dot_v / (n_dot_v * (1.f - k) + k) * n_dot_l /
         (n_dot_l * (1.f - k) + k);
}

static void Store(std::vector<float>& table, const ksl_vec3f_t& value,
                  float w = 1.f) {
  table.push_back(value.x);
  table.push_back(value.y);
  table.push_back(value.z);
  table.push_back(w);
}


ksl_vec3f_t shift::gui::EquirectDirection(float u, float v) {
  const float azimuth = (u - 0.5f) * 2.f * kPi;
  const float altitude = (0.5f - v) * kPi;
  const float c = std::cos(altitude);
  return ksl_vec3f(c * std::cos(azimuth), c * std::sin(azimuth),
                   -std::sin(altitude));
}

void shift::gui::EquirectCoordinates(const ksl_vec3f_t& d, float& u,
                                     float& v) {
  u = std::atan2(d.y, d.x) / (2.f * kPi) + 0.5f;
  v = 0.5f - std::asin(std::clamp(-d.z, -1.f, 1.f)) / kPi;
}

ksl_vec3f_t shift::gui::SkyMap::Sample(const ksl_vec3f_t& d) const {
  float u, v;
  EquirectCoordinates(d, u, v);
  const float x = u * width - 0.5f;
  const float y = std::clamp(v * height - 0.5f, 0.f, height - 1.f);
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const float fx = x - x0;
  const float fy = y - y0;
  const int y1 = std::min(y0 + 1, height - 1);
  auto at = [&](int i, int j) {
    i = ((i % width) + width) % width;
    return texels[static_cast<size_t>(j) * width + i];
  };
  return Mix(Mix(at(x0, y0), at(x0 + 1, y0), fx),
             Mix(at(x0, y1), at(x0 + 1, y1), fx), fy);
}

shift::gui::SkyMap shift::gui::SkyMap::Downsampled(int factor) const {
  SkyMap small;
  small.width = width / factor;
  small.height = height / factor;
  small.texels.assign(static_cast<size_t>(small.width) * small.height,
                      ksl_vec3f(0.f, 0.f, 0.f));
  for(int j = 0; j < small.height; j++) {
    for(int i = 0; i < small.width; i++) {
      ksl_vec3f_t sum = ksl_vec3f(0.f, 0.f, 0.f);
      float weight = 0.f;
      for(int b = 0; b < factor; b++) {
        // texels near the poles cover less of the sphere
        const int row = j * factor + b;
        const float w = std::cos((0.5f - (row + 0.5f) / height) * kPi);
        for(int a = 0; a < factor; a++) {
          ksl_axpy_vvf(
            w, &texels[static_cast<size_t>(row) * width + i * factor + a],
            &sum);
          weight += w;
        }
      }
      small.texels[static_cast<size_t>(j) * small.width + i] =
        weight > 0.f ? Scaled(sum, 1.f / weight) : ksl_vec3f(0.f, 0.f, 0.f);
    }
  }
  return small;
}

/* the gradient sky, the viewer's own */
static ksl_vec3f_t GradientRadiance(const shift::gui::SceneEnvironment& e,
                                    const ksl_vec3f_t& d) {
  const float altitude = -d.z; // +z is down
  if(altitude >= 0.f) {
    return Mix(e.horizon, e.zenith, std::sqrt(altitude));
  }
  return Mix(e.horizon, e.ground, std::clamp(-altitude * 4.f, 0.f, 1.f));
}

/* the Perez distribution of the luminance or a chromaticity of the sky, for
the zenith angle theta of the view and its angle gamma from the sun */
static float Perez(const float c[5], float cos_theta, float gamma,
                   float cos_gamma) {
  return (1.f + c[0] * std::exp(c[1] / std::max(cos_theta, 0.01f))) *
         (1.f + c[2] * std::exp(c[3] * gamma) +
          c[4] * cos_gamma * cos_gamma);
}

/* luminance (kcd/m^2) and chromaticity of the sky toward a direction with
zenith angle theta and angle gamma from the sun, the sun at zenith angle
theta_s (Preetham et al. 1999, appendix A) */
static void PreethamYxy(float cos_theta, float gamma, float cos_gamma,
                        float theta_s, float t, float& big_y, float& x,
                        float& y) {
  const float cy[5] = {0.1787f * t - 1.4630f, -0.3554f * t + 0.4275f,
                       -0.0227f * t + 5.3251f, 0.1206f * t - 2.5771f,
                       -0.0670f * t + 0.3703f};
  const float cx[5] = {-0.0193f * t - 0.2592f, -0.0665f * t + 0.0008f,
                       -0.0004f * t + 0.2125f, -0.0641f * t - 0.8989f,
                       -0.0033f * t + 0.0452f};
  const float cc[5] = {-0.0167f * t - 0.2608f, -0.0950f * t + 0.0092f,
                       -0.0079f * t + 0.2102f, -0.0441f * t - 1.6537f,
                       -0.0109f * t + 0.0529f};
  const float chi = (4.f / 9.f - t / 120.f) * (kPi - 2.f * theta_s);
  const float zenith_y =
    (4.0453f * t - 4.9710f) * std::tan(chi) - 0.2155f * t + 2.4192f;
  const float s = theta_s, s2 = s * s, s3 = s2 * s, t2 = t * t;
  const float zenith_x =
    t2 * (0.00166f * s3 - 0.00375f * s2 + 0.00209f * s) +
    t * (-0.02903f * s3 + 0.06377f * s2 - 0.03202f * s + 0.00394f) +
    (0.11693f * s3 - 0.21196f * s2 + 0.06052f * s + 0.25886f);
  const float zenith_c =
    t2 * (0.00275f * s3 - 0.00610f * s2 + 0.00317f * s) +
    t * (-0.04214f * s3 + 0.08970f * s2 - 0.04153f * s + 0.00516f) +
    (0.15346f * s3 - 0.26756f * s2 + 0.06670f * s + 0.26688f);
  const float cos_s = std::cos(theta_s);
  big_y = zenith_y * Perez(cy, cos_theta, gamma, cos_gamma) /
          Perez(cy, 1.f, theta_s, cos_s);
  x = zenith_x * Perez(cx, cos_theta, gamma, cos_gamma) /
      Perez(cx, 1.f, theta_s, cos_s);
  y = zenith_c * Perez(cc, cos_theta, gamma, cos_gamma) /
      Perez(cc, 1.f, theta_s, cos_s);
}

static ksl_vec3f_t YxyToLinearRgb(float big_y, float x, float y) {
  const float big_x = x * big_y / y;
  const float big_z = (1.f - x - y) * big_y / y;
  return ksl_vec3f(3.2406f * big_x - 1.5372f * big_y - 0.4986f * big_z,
                   -0.9689f * big_x + 1.8758f * big_y + 0.0415f * big_z,
                   0.0557f * big_x - 0.2040f * big_y + 1.0570f * big_z);
}

/* the zenith angle of the sun in the model: at least a little above the
horizon, where the model holds */
static float SunZenithAngle(const ksl_vec3f_t& sun) {
  return std::min(std::acos(std::clamp(-sun.z, -1.f, 1.f)),
                  kPi / 2.f - 0.02f);
}

ksl_vec3f_t shift::gui::PreethamRadiance(const ksl_vec3f_t& d,
                                         const ksl_vec3f_t& sun,
                                         float turbidity) {
  // the luminance of the default gradient sky's zenith, reached by the
  // zenith of a sky of turbidity 3 for the default sun (57 degrees high)
  static const float scale = [] {
    float big_y, x, y;
    const float theta_s = SunZenithAngle(SceneEnvironment().sun_direction);
    PreethamYxy(1.f, theta_s, std::cos(theta_s), theta_s, 3.f, big_y, x, y);
    const ksl_vec3f_t zenith = SceneEnvironment().zenith;
    const float luminance =
      0.2126f * zenith.x + 0.7152f * zenith.y + 0.0722f * zenith.z;
    return luminance / big_y;
  }();
  const float cos_theta = std::max(-d.z, 0.f);
  const float cos_gamma = std::clamp(ksl_dot_vvf(&d, &sun), -1.f, 1.f);
  const float gamma = std::acos(cos_gamma);
  float big_y, x, y;
  PreethamYxy(cos_theta, gamma, cos_gamma, SunZenithAngle(sun), turbidity,
              big_y, x, y);
  const ksl_vec3f_t rgb = YxyToLinearRgb(big_y * scale, x, y);
  return ksl_vec3f(std::max(rgb.x, 0.f), std::max(rgb.y, 0.f),
                   std::max(rgb.z, 0.f));
}

float shift::gui::AngstromBeta(float turbidity) {
  return std::max(0.04608366f * turbidity - 0.04586026f, 0.f);
}

ksl_vec3f_t shift::gui::SunlightColor(const ksl_vec3f_t& sun,
                                      float turbidity) {
  const float elevation = std::asin(std::clamp(-sun.z, -1.f, 1.f));
  if(elevation <= 0.f) {
    return ksl_vec3f(0.f, 0.f, 0.f);
  }
  // optical depths at 680, 550 and 440 nm: Rayleigh scattering, and the
  // aerosols of the turbidity (Angstrom's law, Preetham et al. 1999)
  const float lambda[3] = {0.68f, 0.55f, 0.44f};
  const float beta = AngstromBeta(turbidity);
  auto transmittance = [&](float air_mass) {
    float t[3];
    for(int i = 0; i < 3; i++) {
      const float l4 = std::pow(lambda[i], -4.f);
      const float rayleigh =
        0.008569f * l4 *
        (1.f + 0.0113f / (lambda[i] * lambda[i]) + 0.00013f * l4);
      const float aerosol = beta * std::pow(lambda[i], -1.3f);
      t[i] = std::exp(-(rayleigh + aerosol) * air_mass);
    }
    return ksl_vec3f(t[0], t[1], t[2]);
  };
  // the air mass of Kasten and Young (1989)
  const float degrees = elevation * 180.f / kPi;
  const float air_mass =
    1.f / (std::sin(elevation) +
           0.50572f * std::pow(degrees + 6.07995f, -1.6364f));
  const ksl_vec3f_t overhead = transmittance(1.f);
  const ksl_vec3f_t through = transmittance(air_mass);
  const ksl_vec3f_t white = SceneEnvironment().sun_color;
  return ksl_vec3f(white.x * through.x / overhead.x,
                   white.y * through.y / overhead.y,
                   white.z * through.z / overhead.z);
}

bool shift::gui::LoadSkyImage(const std::string& path, float rotation,
                              SkyMap& map, std::string& error) {
#ifdef SHIFT_HAVE_OPENEXR
  QFile file(QString::fromStdString(path));
  if(!file.open(QIODevice::ReadOnly)) {
    error = "cannot open " + path;
    return false;
  }
  const QByteArray bytes = file.readAll();
  try {
    Imf::StdISStream stream;
    stream.str(std::string(bytes.constData(), bytes.size()));
    Imf::RgbaInputFile input(stream);
    const Imath::Box2i window = input.dataWindow();
    const int width = window.max.x - window.min.x + 1;
    const int height = window.max.y - window.min.y + 1;
    Imf::Array2D<Imf::Rgba> pixels(height, width);
    input.setFrameBuffer(&pixels[0][0] - window.min.x - window.min.y * width,
                         1, width);
    input.readPixels(window.min.y, window.max.y);
    /* resample to the map, turned about the vertical */
    map.width = EnvironmentTables::kRadianceWidth;
    map.height = EnvironmentTables::kRadianceHeight;
    map.texels.assign(static_cast<size_t>(map.width) * map.height,
                      ksl_vec3f(0.f, 0.f, 0.f));
    auto texel = [&](int i, int j) {
      i = ((i % width) + width) % width;
      j = std::clamp(j, 0, height - 1);
      const Imf::Rgba& p = pixels[j][i];
      return ksl_vec3f(std::max(float(p.r), 0.f), std::max(float(p.g), 0.f),
                       std::max(float(p.b), 0.f));
    };
    // box filter the source over each texel of the map
    const int step_x = std::max(1, width / map.width);
    const int step_y = std::max(1, height / map.height);
    for(int j = 0; j < map.height; j++) {
      for(int i = 0; i < map.width; i++) {
        const float u = (i + 0.5f) / map.width - rotation / (2.f * kPi);
        const float v = (j + 0.5f) / map.height;
        const float x = u * width - 0.5f * step_x;
        const float y = v * height - 0.5f * step_y;
        const int x0 = static_cast<int>(std::floor(x));
        const int y0 = static_cast<int>(std::floor(y));
        ksl_vec3f_t sum = ksl_vec3f(0.f, 0.f, 0.f);
        for(int b = 0; b < step_y; b++) {
          for(int a = 0; a < step_x; a++) {
            const ksl_vec3f_t t = texel(x0 + a, y0 + b);
            ksl_xpy_vvf(&t, &sum);
          }
        }
        map.texels[static_cast<size_t>(j) * map.width + i] =
          Scaled(sum, 1.f / float(step_x * step_y));
      }
    }
  } catch(const std::exception& e) {
    error = path + ": " + e.what();
    return false;
  }
  return true;
#else
  (void)rotation;
  (void)map;
  error = path + ": this build of the viewer reads no OpenEXR images";
  return false;
#endif
}

shift::gui::SkyMap shift::gui::BuildSkyMap(const SceneEnvironment& e,
                                           std::string* error) {
  SkyMap map;
  if(e.sky_type == SceneEnvironment::kImage) {
    std::string message;
    if(LoadSkyImage(e.image_path, e.image_rotation, map, message)) {
      for(auto& t : map.texels) {
        ksl_vec3f_scale(&t, e.sky_intensity);
      }
      return map;
    }
    if(error) {
      *error = message;
    }
  }
  map.width = EnvironmentTables::kRadianceWidth;
  map.height = EnvironmentTables::kRadianceHeight;
  map.texels.resize(static_cast<size_t>(map.width) * map.height);
  const bool physical = e.sky_type == SceneEnvironment::kPhysical;
  // the physical sky meets the ground color below the horizon, as the
  // gradient sky does
  for(int j = 0; j < map.height; j++) {
    for(int i = 0; i < map.width; i++) {
      const ksl_vec3f_t d =
        EquirectDirection((i + 0.5f) / map.width, (j + 0.5f) / map.height);
      ksl_vec3f_t radiance;
      if(!physical) {
        radiance = GradientRadiance(e, d);
      } else if(d.z <= 0.f) {
        radiance = PreethamRadiance(d, e.sun_direction, e.turbidity);
      } else {
        ksl_vec3f_t level = ksl_vec3f(d.x, d.y, 0.f);
        ksl_vec3f_normalize(&level);
        const ksl_vec3f_t horizon =
          PreethamRadiance(level, e.sun_direction, e.turbidity);
        radiance =
          Mix(horizon, e.ground, std::clamp(d.z * 4.f, 0.f, 1.f));
      }
      map.texels[static_cast<size_t>(j) * map.width + i] =
        Scaled(radiance, e.sky_intensity);
    }
  }
  return map;
}

static void StoreMap(std::vector<float>& table, const shift::gui::SkyMap& map) {
  for(const ksl_vec3f_t& t : map.texels) {
    Store(table, t);
  }
}

shift::gui::EnvironmentTables
shift::gui::ComputeEnvironmentTables(const SkyMap& sky) {
  using Tables = EnvironmentTables;
  EnvironmentTables tables;
  StoreMap(tables.radiance, sky);
  // smaller maps for wide lobes: a lobe then covers a few texels, not
  // hundreds that the samples would miss
  const SkyMap medium = sky.Downsampled(sky.width / 128); // 128 x 64
  const SkyMap small = sky.Downsampled(sky.width / 32);   // 32 x 16

  // prefiltered specular, with the usual n = v = r assumption
  const int sample_count = 256;
  for(int level = 0; level < Tables::kRoughnessCount; level++) {
    const float roughness = float(level) / (Tables::kRoughnessCount - 1);
    const float alpha = std::max(roughness * roughness, 1e-3f);
    const SkyMap& source =
      roughness < 0.3f ? sky : (roughness < 0.6f ? medium : small);
    for(int j = 0; j < Tables::kSpecularHeight; j++) {
      for(int i = 0; i < Tables::kSpecularWidth; i++) {
        const ksl_vec3f_t n = EquirectDirection(
          (i + 0.5f) / Tables::kSpecularWidth,
          (j + 0.5f) / Tables::kSpecularHeight);
        ksl_vec3f_t sum = ksl_vec3f(0.f, 0.f, 0.f);
        float weight = 0.f;
        for(int k = 0; k < sample_count; k++) {
          float u, v;
          Hammersley(k, sample_count, u, v);
          const ksl_vec3f_t l = Reflected(n, SampleGgx(u, v, alpha, n));
          const float n_dot_l = ksl_dot_vvf(&n, &l);
          if(n_dot_l > 0.f) {
            const ksl_vec3f_t radiance = source.Sample(l);
            ksl_axpy_vvf(n_dot_l, &radiance, &sum);
            weight += n_dot_l;
          }
        }
        Store(tables.specular,
              weight > 0.f ? Scaled(sum, 1.f / weight) : source.Sample(n));
      }
    }
  }

  // cosine weighted mean radiance, so that diffuse = albedo * irradiance
  for(int j = 0; j < Tables::kIrradianceHeight; j++) {
    for(int i = 0; i < Tables::kIrradianceWidth; i++) {
      const ksl_vec3f_t n =
        EquirectDirection((i + 0.5f) / Tables::kIrradianceWidth,
                          (j + 0.5f) / Tables::kIrradianceHeight);
      ksl_vec3f_t sum = ksl_vec3f(0.f, 0.f, 0.f);
      const int count = 512;
      for(int k = 0; k < count; k++) {
        float u, v;
        Hammersley(k, count, u, v);
        const ksl_vec3f_t radiance = small.Sample(SampleCosine(u, v, n));
        ksl_xpy_vvf(&radiance, &sum);
      }
      Store(tables.irradiance, Scaled(sum, 1.f / count));
    }
  }

  // scale and bias to F0 of the directional albedo of the specular lobe
  const int brdf_count = Tables::kBrdfCount;
  const ksl_vec3f_t n = ksl_vec3f(0.f, 0.f, 1.f);
  for(int j = 0; j < brdf_count; j++) {
    const float roughness = (j + 0.5f) / brdf_count;
    const float alpha = roughness * roughness;
    for(int i = 0; i < brdf_count; i++) {
      const float n_dot_v = (i + 0.5f) / brdf_count;
      const ksl_vec3f_t view =
        ksl_vec3f(std::sqrt(1.f - n_dot_v * n_dot_v), 0.f, n_dot_v);
      float scale = 0.f;
      float bias = 0.f;
      for(int k = 0; k < kSampleCount; k++) {
        float u, v;
        Hammersley(k, kSampleCount, u, v);
        const ksl_vec3f_t h = SampleGgx(u, v, alpha, n);
        const float v_dot_h = ksl_dot_vvf(&view, &h);
        const ksl_vec3f_t l = Reflected(view, h);
        const float n_dot_l = l.z;
        const float n_dot_h = h.z;
        if(n_dot_l > 0.f && v_dot_h > 0.f) {
          const float g = GeometrySmithIbl(n_dot_v, n_dot_l, roughness);
          const float g_visible = g * v_dot_h / (n_dot_h * n_dot_v);
          const float fresnel = std::pow(1.f - v_dot_h, 5.f);
          scale += (1.f - fresnel) * g_visible;
          bias += fresnel * g_visible;
        }
      }
      Store(tables.brdf,
            ksl_vec3f(scale / kSampleCount, bias / kSampleCount, 0.f));
    }
  }

  // the zenith, and the mean radiance just above the horizon
  tables.zenith = sky.Sample(ksl_vec3f(0.f, 0.f, -1.f));
  ksl_vec3f_t horizon = ksl_vec3f(0.f, 0.f, 0.f);
  const int around = 64;
  for(int k = 0; k < around; k++) {
    const float a = 2.f * kPi * k / around;
    ksl_vec3f_t d = ksl_vec3f(std::cos(a), std::sin(a), -0.02f);
    ksl_vec3f_normalize(&d);
    const ksl_vec3f_t radiance = sky.Sample(d);
    ksl_xpy_vvf(&radiance, &horizon);
  }
  tables.horizon = Scaled(horizon, 1.f / around);
  return tables;
}

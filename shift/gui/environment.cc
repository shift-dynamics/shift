#include "shift/gui/environment.h"

#include <algorithm>
#include <cmath>


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

/*! Smith-Schlick visibility term with the image based lighting remapping
 * k = alpha / 2 */
static float GeometrySmithIbl(float n_dot_v, float n_dot_l, float roughness) {
  const float k = roughness * roughness / 2.f;
  return n_dot_v / (n_dot_v * (1.f - k) + k) * n_dot_l /
         (n_dot_l * (1.f - k) + k);
}

/*! a unit direction with z component z */
static ksl_vec3f_t DirectionWithZ(float z) {
  return ksl_vec3f(std::sqrt(std::max(1.f - z * z, 0.f)), 0.f, z);
}

static void Store(std::vector<float>& table, const ksl_vec3f_t& value,
                  float w = 1.f) {
  table.push_back(value.x);
  table.push_back(value.y);
  table.push_back(value.z);
  table.push_back(w);
}

/*! a (1 - t) + b t */
static ksl_vec3f_t Mix(const ksl_vec3f_t& a, const ksl_vec3f_t& b, float t) {
  ksl_vec3f_t m;
  ksl_product_avf(1.f - t, &a, &m);
  ksl_axpy_vvf(t, &b, &m);
  return m;
}


ksl_vec3f_t shift::gui::Sky::Radiance(const ksl_vec3f_t& d) const {
  const float altitude = -d.z; // +z is down
  if(altitude >= 0.f) {
    return Mix(horizon, zenith, std::sqrt(altitude));
  }
  return Mix(horizon, ground, std::clamp(-altitude * 4.f, 0.f, 1.f));
}

shift::gui::EnvironmentTables
shift::gui::ComputeEnvironmentTables(const Sky& sky) {
  EnvironmentTables tables;
  const int z_count = EnvironmentTables::kZCount;
  const int roughness_count = EnvironmentTables::kRoughnessCount;
  const int brdf_count = EnvironmentTables::kBrdfCount;

  // prefiltered specular, with the usual n = v = r assumption
  tables.specular.reserve(4 * z_count * roughness_count);
  for(int j = 0; j < roughness_count; j++) {
    const float roughness = (j + 0.5f) / roughness_count;
    const float alpha = roughness * roughness;
    for(int i = 0; i < z_count; i++) {
      const ksl_vec3f_t n = DirectionWithZ(-1.f + 2.f * (i + 0.5f) / z_count);
      ksl_vec3f_t sum = ksl_vec3f(0.f, 0.f, 0.f);
      float weight = 0.f;
      for(int k = 0; k < kSampleCount; k++) {
        float u, v;
        Hammersley(k, kSampleCount, u, v);
        const ksl_vec3f_t l = Reflected(n, SampleGgx(u, v, alpha, n));
        const float n_dot_l = ksl_dot_vvf(&n, &l);
        if(n_dot_l > 0.f) {
          const ksl_vec3f_t radiance = sky.Radiance(l);
          ksl_axpy_vvf(n_dot_l, &radiance, &sum);
          weight += n_dot_l;
        }
      }
      if(weight > 0.f) {
        ksl_vec3f_scale(&sum, 1.f / weight);
      } else {
        sum = sky.Radiance(n);
      }
      Store(tables.specular, sum);
    }
  }

  // cosine weighted mean radiance, so that diffuse = albedo * irradiance
  tables.irradiance.reserve(4 * z_count);
  for(int i = 0; i < z_count; i++) {
    const ksl_vec3f_t n = DirectionWithZ(-1.f + 2.f * (i + 0.5f) / z_count);
    ksl_vec3f_t sum = ksl_vec3f(0.f, 0.f, 0.f);
    for(int k = 0; k < kSampleCount; k++) {
      float u, v;
      Hammersley(k, kSampleCount, u, v);
      const ksl_vec3f_t radiance = sky.Radiance(SampleCosine(u, v, n));
      ksl_xpy_vvf(&radiance, &sum);
    }
    ksl_vec3f_scale(&sum, 1.f / kSampleCount);
    Store(tables.irradiance, sum);
  }

  // scale and bias to F0 of the directional albedo of the specular lobe
  tables.brdf.reserve(4 * brdf_count * brdf_count);
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
  return tables;
}

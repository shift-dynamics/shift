#include "shift/gui/environment.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr int kSampleCount = 512;

/*! low discrepancy point i of n in the unit square */
void Hammersley(int i, int n, float& u, float& v) {
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
void Basis(const QVector3D& n, QVector3D& t, QVector3D& b) {
  const QVector3D a =
    std::fabs(n.x()) < 0.9f ? QVector3D(1, 0, 0) : QVector3D(0, 1, 0);
  t = QVector3D::crossProduct(a, n).normalized();
  b = QVector3D::crossProduct(n, t);
}

/*! GGX distributed half vector about n, alpha = roughness^2 */
QVector3D SampleGgx(float u, float v, float alpha, const QVector3D& n) {
  const float phi = 2.f * kPi * u;
  const float cos_theta =
    std::sqrt((1.f - v) / (1.f + (alpha * alpha - 1.f) * v));
  const float sin_theta = std::sqrt(std::max(1.f - cos_theta * cos_theta, 0.f));
  QVector3D t, b;
  Basis(n, t, b);
  return (t * (sin_theta * std::cos(phi)) + b * (sin_theta * std::sin(phi)) +
          n * cos_theta)
    .normalized();
}

/*! cosine distributed direction about n */
QVector3D SampleCosine(float u, float v, const QVector3D& n) {
  const float phi = 2.f * kPi * u;
  const float r = std::sqrt(v);
  QVector3D t, b;
  Basis(n, t, b);
  return (t * (r * std::cos(phi)) + b * (r * std::sin(phi)) +
          n * std::sqrt(std::max(1.f - v, 0.f)))
    .normalized();
}

/*! Smith-Schlick visibility term with the image based lighting remapping
 * k = alpha / 2 */
float GeometrySmithIbl(float n_dot_v, float n_dot_l, float roughness) {
  const float k = roughness * roughness / 2.f;
  return n_dot_v / (n_dot_v * (1.f - k) + k) * n_dot_l /
         (n_dot_l * (1.f - k) + k);
}

/*! a unit direction with z component z */
QVector3D DirectionWithZ(float z) {
  return QVector3D(std::sqrt(std::max(1.f - z * z, 0.f)), 0.f, z);
}

void Store(std::vector<float>& table, const QVector3D& value, float w = 1.f) {
  table.push_back(value.x());
  table.push_back(value.y());
  table.push_back(value.z());
  table.push_back(w);
}

} // namespace

QVector3D shift::gui::Sky::Radiance(const QVector3D& d) const {
  const float altitude = -d.z(); // +z is down
  if(altitude >= 0.f) {
    const float t = std::sqrt(altitude);
    return horizon * (1.f - t) + zenith * t;
  }
  const float t = std::clamp(-altitude * 4.f, 0.f, 1.f);
  return horizon * (1.f - t) + ground * t;
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
      const QVector3D n = DirectionWithZ(-1.f + 2.f * (i + 0.5f) / z_count);
      QVector3D sum;
      float weight = 0.f;
      for(int k = 0; k < kSampleCount; k++) {
        float u, v;
        Hammersley(k, kSampleCount, u, v);
        const QVector3D h = SampleGgx(u, v, alpha, n);
        const QVector3D l = 2.f * QVector3D::dotProduct(n, h) * h - n;
        const float n_dot_l = QVector3D::dotProduct(n, l);
        if(n_dot_l > 0.f) {
          sum += sky.Radiance(l) * n_dot_l;
          weight += n_dot_l;
        }
      }
      Store(tables.specular, weight > 0.f ? sum / weight : sky.Radiance(n));
    }
  }

  // cosine weighted mean radiance, so that diffuse = albedo * irradiance
  tables.irradiance.reserve(4 * z_count);
  for(int i = 0; i < z_count; i++) {
    const QVector3D n = DirectionWithZ(-1.f + 2.f * (i + 0.5f) / z_count);
    QVector3D sum;
    for(int k = 0; k < kSampleCount; k++) {
      float u, v;
      Hammersley(k, kSampleCount, u, v);
      sum += sky.Radiance(SampleCosine(u, v, n));
    }
    Store(tables.irradiance, sum / kSampleCount);
  }

  // scale and bias to F0 of the directional albedo of the specular lobe
  tables.brdf.reserve(4 * brdf_count * brdf_count);
  const QVector3D n(0.f, 0.f, 1.f);
  for(int j = 0; j < brdf_count; j++) {
    const float roughness = (j + 0.5f) / brdf_count;
    const float alpha = roughness * roughness;
    for(int i = 0; i < brdf_count; i++) {
      const float n_dot_v = (i + 0.5f) / brdf_count;
      const QVector3D view(std::sqrt(1.f - n_dot_v * n_dot_v), 0.f, n_dot_v);
      float scale = 0.f;
      float bias = 0.f;
      for(int k = 0; k < kSampleCount; k++) {
        float u, v;
        Hammersley(k, kSampleCount, u, v);
        const QVector3D h = SampleGgx(u, v, alpha, n);
        const float v_dot_h = QVector3D::dotProduct(view, h);
        const QVector3D l = 2.f * v_dot_h * h - view;
        const float n_dot_l = l.z();
        const float n_dot_h = h.z();
        if(n_dot_l > 0.f && v_dot_h > 0.f) {
          const float g = GeometrySmithIbl(n_dot_v, n_dot_l, roughness);
          const float g_visible = g * v_dot_h / (n_dot_h * n_dot_v);
          const float fresnel = std::pow(1.f - v_dot_h, 5.f);
          scale += (1.f - fresnel) * g_visible;
          bias += fresnel * g_visible;
        }
      }
      Store(tables.brdf,
            QVector3D(scale / kSampleCount, bias / kSampleCount, 0.f));
    }
  }
  return tables;
}

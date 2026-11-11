#include "shift/core/sph/sph.h"

#include <algorithm>
#include <cmath>

#include "shift/core/exception.h"

/* the rotation vector of a rotation matrix (columns, R[3 c + r]), and the
matrix of a rotation vector */
static std::array<double, 3> RotationVector(const std::array<double, 9>& R) {
  auto at = [&](int r, int c) { return R[3 * c + r]; };
  const double trace = at(0, 0) + at(1, 1) + at(2, 2);
  const double c = std::clamp(0.5 * (trace - 1.0), -1.0, 1.0);
  const double angle = std::acos(c);
  const std::array<double, 3> axis = {at(2, 1) - at(1, 2), at(0, 2) - at(2, 0),
                                      at(1, 0) - at(0, 1)};
  if(angle < 1e-6) {
    /* R = I + [phi]x to first order */
    return {0.5 * axis[0], 0.5 * axis[1], 0.5 * axis[2]};
  }
  const double s = std::sin(angle);
  if(s > 1e-6) {
    const double f = angle / (2.0 * s);
    return {f * axis[0], f * axis[1], f * axis[2]};
  }
  /* near pi: the axis from the diagonal */
  std::array<double, 3> u;
  for(int k = 0; k < 3; k++) {
    u[k] = std::sqrt(std::max(0.0, 0.5 * (at(k, k) + 1.0)));
  }
  const int big = u[0] >= u[1] && u[0] >= u[2] ? 0 : (u[1] >= u[2] ? 1 : 2);
  for(int k = 0; k < 3; k++) {
    if(k != big) {
      u[k] = (at(k, big) + at(big, k)) / (4.0 * u[big]);
    }
  }
  return {angle * u[0], angle * u[1], angle * u[2]};
}

static std::array<double, 9> RotationMatrix(const std::array<double, 3>& phi) {
  const double angle =
    std::sqrt(phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2]);
  /* R = I + a [phi]x + b [phi]x^2 */
  double a = 1.0, b = 0.5;
  if(angle > 1e-8) {
    a = std::sin(angle) / angle;
    b = (1.0 - std::cos(angle)) / (angle * angle);
  }
  const double k[3][3] = {{0.0, -phi[2], phi[1]},
                          {phi[2], 0.0, -phi[0]},
                          {-phi[1], phi[0], 0.0}};
  std::array<double, 9> R;
  for(int r = 0; r < 3; r++) {
    for(int c = 0; c < 3; c++) {
      double k2 = 0.0;
      for(int m = 0; m < 3; m++) {
        k2 += k[r][m] * k[m][c];
      }
      R[3 * c + r] = (r == c ? 1.0 : 0.0) + a * k[r][c] + b * k2;
    }
  }
  return R;
}

/* A B, and A B^T, of rotations as columns */
static std::array<double, 9> Multiply(const std::array<double, 9>& A,
                                      const std::array<double, 9>& B,
                                      bool transpose_b) {
  std::array<double, 9> C;
  for(int r = 0; r < 3; r++) {
    for(int c = 0; c < 3; c++) {
      double sum = 0.0;
      for(int m = 0; m < 3; m++) {
        sum += A[3 * m + r] * (transpose_b ? B[3 * m + c] : B[3 * c + m]);
      }
      C[3 * c + r] = sum;
    }
  }
  return C;
}

shift::core::sph::BodyPose
shift::core::sph::Interpolate(const BodyPose& a, const BodyPose& b,
                              double duration, double s) {
  const double s2 = s * s, s3 = s2 * s;
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0, h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2, h11 = s3 - s2;
  const double d00 = 6.0 * s2 - 6.0 * s, d10 = 3.0 * s2 - 4.0 * s + 1.0;
  const double d01 = -6.0 * s2 + 6.0 * s, d11 = 3.0 * s2 - 2.0 * s;
  const double T = duration;
  BodyPose pose;
  const std::array<double, 3> theta =
    RotationVector(Multiply(b.R, a.R, true));
  std::array<double, 3> phi;
  for(int k = 0; k < 3; k++) {
    pose.t[k] = h00 * a.t[k] + h10 * T * a.v[k] + h01 * b.t[k] +
                h11 * T * b.v[k];
    pose.v[k] = T > 0.0 ? (d00 * a.t[k] + d01 * b.t[k]) / T + d10 * a.v[k] +
                            d11 * b.v[k]
                        : a.v[k];
    phi[k] = h10 * T * a.w[k] + h01 * theta[k] + h11 * T * b.w[k];
    pose.w[k] = T > 0.0 ? d10 * a.w[k] + d01 * theta[k] / T + d11 * b.w[k]
                        : a.w[k];
  }
  pose.R = Multiply(RotationMatrix(phi), a.R, false);
  return pose;
}

void shift::core::sph::Solver::SetBodyPaths(const std::vector<BodyPose>& start,
                                            const std::vector<BodyPose>& end,
                                            double t0, double t1) {
  if(start.size() != bodies_ || end.size() != bodies_) {
    throw shift::core::Exception("SPH: a path for each body is needed");
  }
  path_start_ = start;
  path_end_ = end;
  path_t0_ = t0;
  path_t1_ = t1;
}

void shift::core::sph::Solver::PosesAt(double time,
                                       std::vector<BodyPose>& poses) const {
  if(path_end_.empty()) {
    poses.clear();
    return;
  }
  if(!(path_t1_ > path_t0_)) {
    poses = path_end_;
    return;
  }
  const double duration = path_t1_ - path_t0_;
  const double s = (time - path_t0_) / duration;
  poses.resize(bodies_);
  for(size_t b = 0; b < bodies_; b++) {
    poses[b] = Interpolate(path_start_[b], path_end_[b], duration, s);
  }
}

void shift::core::sph::State::AddBodyMarker(const std::array<float, 3>& local,
                                            uint32_t body,
                                            const Material& properties,
                                            float spacing) {
  if(body_offsets.empty()) {
    first_body_marker = static_cast<uint32_t>(id.size());
    body_offsets.push_back(0);
  }
  const size_t bodies = body_offsets.size() - 1;
  if(body == bodies) {
    body_offsets.push_back(body_offsets.back()); // the next body
  } else if(body + 1 != bodies) {
    throw shift::core::Exception("SPH: the bodies' markers must be added in "
                                 "the order of the bodies, after the rest");
  }
  AddParticle(local, 0, properties, spacing);
  kind.back() = kMarker | (body + 1);
  body_local.push_back({local[0], local[1], local[2], 0.0f});
  body_offsets.back()++;
}

void shift::core::sph::CheckBodyMarkers(const State& state) {
  const size_t markers = state.body_local.size();
  const size_t bodies = state.body_count();
  bool valid = (markers == 0) == (bodies == 0) &&
               (bodies == 0 || state.body_offsets.back() == markers);
  for(size_t i = 0; valid && i < state.size(); i++) {
    const uint32_t kind = state.kind[i];
    const bool body = (kind & State::kMarker) && (kind & ~State::kMarker);
    const bool in_range = state.id[i] >= state.first_body_marker &&
                          state.id[i] - state.first_body_marker < markers;
    if(body != (in_range && bodies > 0)) {
      valid = false;
    } else if(body) {
      const uint32_t b = (kind & ~State::kMarker) - 1;
      const uint32_t slot = state.id[i] - state.first_body_marker;
      valid = b < bodies && slot >= state.body_offsets[b] &&
              slot < state.body_offsets[b + 1];
    }
  }
  if(!valid) {
    throw shift::core::Exception("SPH: the bodies' markers are not laid out "
                                 "by State::AddBodyMarker");
  }
}

void shift::core::sph::State::resize(size_t n) {
  position.resize(n);
  velocity.resize(n);
  density.resize(n);
  stress.resize(n);
  id.resize(n);
  kind.resize(n);
}

void shift::core::sph::State::AddParticle(const std::array<float, 3>& x,
                                          uint32_t material,
                                          const Material& properties,
                                          float spacing) {
  const float mass = properties.density * spacing * spacing * spacing;
  position.push_back({x[0], x[1], x[2], mass});
  velocity.push_back({0.0f, 0.0f, 0.0f, 0.0f});
  density.push_back(properties.density);
  stress.push_back({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
  id.push_back(static_cast<uint32_t>(id.size()));
  kind.push_back(material);
}

void shift::core::sph::State::AddMarker(const std::array<float, 3>& x,
                                        const Material& properties,
                                        float spacing) {
  AddParticle(x, 0, properties, spacing);
  kind.back() = kMarker;
}

long shift::core::sph::State::Find(uint32_t wanted) const {
  for(size_t k = 0; k < id.size(); k++) {
    if(id[k] == wanted) {
      return static_cast<long>(k);
    }
  }
  return -1;
}

/* Wendland C2 in 3D: W = 21 / (16 pi h^3) (1 - q/2)^4 (2 q + 1), q = r / h */
float shift::core::sph::Kernel(float r, float h) {
  const float q = r / h;
  if(q >= 2.0f) {
    return 0.0f;
  }
  const float a = 1.0f - 0.5f * q;
  const float a2 = a * a;
  return 21.0f / (16.0f * 3.14159265f * h * h * h) * a2 * a2 * (2.0f * q + 1.0f);
}

float shift::core::sph::KernelDerivative(float r, float h) {
  const float q = r / h;
  if(q >= 2.0f) {
    return 0.0f;
  }
  const float a = 1.0f - 0.5f * q;
  return 21.0f / (16.0f * 3.14159265f * h * h * h) * (-5.0f * q) * a * a * a /
         h;
}

float shift::core::sph::StableStep(const Settings& settings,
                                   const std::vector<Material>& materials) {
  float c = settings.sound_speed;
  if(!(c > 0.0f)) {
    for(const Material& m : materials) {
      const float g = m.youngs_modulus / (2.0f * (1.0f + m.poisson_ratio));
      const float k = m.youngs_modulus / (3.0f * (1.0f - 2.0f * m.poisson_ratio));
      c = std::max(c, std::sqrt((k + 4.0f * g / 3.0f) / m.density));
    }
  }
  if(!(c > 0.0f)) {
    throw shift::core::Exception("SPH: no speed of sound");
  }
  return settings.cfl * settings.smoothing_ratio * settings.spacing / c;
}

uint32_t shift::core::sph::HashTableSize(size_t count) {
  uint32_t size = 1024;
  while(size < 2 * count && size < (1u << 30)) {
    size <<= 1;
  }
  return size;
}

long shift::core::sph::Solver::Advance(double duration) {
  if(!(duration > 0.0)) {
    return 0;
  }
  const double most = StableStep(settings_, materials_);
  const long steps = static_cast<long>(std::ceil(duration / most - 1e-9));
  const float dt = static_cast<float>(duration / double(steps));
  Steps(dt, steps);
  return steps;
}

std::vector<std::array<float, 8>> shift::core::sph::MaterialTable(
  const Settings& settings, const std::vector<Material>& materials) {
  std::vector<std::array<float, 8>> table;
  for(const Material& m : materials) {
    if(!(m.density > 0.0f) || !(m.youngs_modulus > 0.0f) ||
       !(m.poisson_ratio >= 0.0f && m.poisson_ratio < 0.5f) ||
       !(m.friction_angle >= 0.0f && m.friction_angle < 1.5f) ||
       !(m.dilatancy_angle >= 0.0f && m.dilatancy_angle <= m.friction_angle) ||
       m.cohesion < 0.0f) {
      throw shift::core::Exception(
        "SPH: a material needs a positive density and Young's modulus, a "
        "Poisson's ratio from 0 to 0.5, a friction angle from 0 to 1.5 rad, "
        "a dilatancy angle from 0 to the friction angle and no negative "
        "cohesion");
    }
    const float g = m.youngs_modulus / (2.0f * (1.0f + m.poisson_ratio));
    const float k = m.youngs_modulus / (3.0f * (1.0f - 2.0f * m.poisson_ratio));
    const float c = settings.sound_speed > 0.0f
                      ? settings.sound_speed
                      : std::sqrt((k + 4.0f * g / 3.0f) / m.density);
    const float sp = std::sin(m.friction_angle);
    const float sq = std::sin(m.dilatancy_angle);
    const float root3 = std::sqrt(3.0f);
    table.push_back({m.density, g, k, c, 2.0f * sp / (root3 * (3.0f - sp)),
                     6.0f * m.cohesion * std::cos(m.friction_angle) /
                       (root3 * (3.0f - sp)),
                     2.0f * sq / (root3 * (3.0f - sq)), 0.0f});
  }
  return table;
}

std::array<int32_t, 3> shift::core::sph::Cell(const std::array<float, 4>& x,
                                              float cell) {
  return {static_cast<int32_t>(std::floor(x[0] / cell)),
          static_cast<int32_t>(std::floor(x[1] / cell)),
          static_cast<int32_t>(std::floor(x[2] / cell))};
}

uint32_t shift::core::sph::CellHash(const std::array<int32_t, 3>& c,
                                    uint32_t mask) {
  return ((static_cast<uint32_t>(c[0]) * 73856093u) ^
          (static_cast<uint32_t>(c[1]) * 19349663u) ^
          (static_cast<uint32_t>(c[2]) * 83492791u)) &
         mask;
}

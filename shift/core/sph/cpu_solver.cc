#include "shift/core/sph/cpu_solver.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "shift/core/exception.h"

shift::core::sph::CpuSolver::CpuSolver(const Settings& settings,
                                       const std::vector<Material>& materials,
                                       const State& state)
  : Solver(settings, materials, state.body_count()), state_(state) {
  CheckBodyMarkers(state);
  table_ = MaterialTable(settings, materials);
  impulse_.assign(6 * state.body_local.size(), 0.0f);
  for(uint32_t kind : state.kind) {
    if(!(kind & State::kMarker) && kind >= table_.size()) {
      throw shift::core::Exception("SPH: a particle of no material");
    }
  }
  h_ = settings.smoothing_ratio * settings.spacing;
  cell_ = 2.0f * h_;
  mask_ = HashTableSize(state.size()) - 1;
}

void shift::core::sph::CpuSolver::GetState(State& state) {
  state = state_;
}

void shift::core::sph::CpuSolver::RemoveParticles(
  const std::vector<uint32_t>& ids) {
  state_.RemoveParticles(ids);
}

int shift::core::sph::CpuSolver::NeighborCells(const std::array<float, 4>& x,
                                               uint32_t hashes[27]) const {
  const std::array<int32_t, 3> c = Cell(x, cell_);
  int n = 0;
  for(int dz = -1; dz <= 1; dz++) {
    for(int dy = -1; dy <= 1; dy++) {
      for(int dx = -1; dx <= 1; dx++) {
        const uint32_t hash = CellHash({c[0] + dx, c[1] + dy, c[2] + dz}, mask_);
        bool seen = false;
        for(int k = 0; k < n; k++) {
          seen = seen || hashes[k] == hash;
        }
        if(!seen) {
          hashes[n++] = hash;
        }
      }
    }
  }
  return n;
}

void shift::core::sph::CpuSolver::ClearImpulses() {
  std::fill(impulse_.begin(), impulse_.end(), 0.0f);
}

void shift::core::sph::CpuSolver::GetImpulses(
  std::vector<std::array<double, 6>>& impulses) {
  const size_t n = state_.body_local.size();
  impulses.assign(bodies_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  for(size_t b = 0; b < bodies_; b++) {
    for(uint32_t m = state_.body_offsets[b]; m < state_.body_offsets[b + 1];
        m++) {
      for(int k = 0; k < 6; k++) {
        impulses[b][k] += impulse_[k * n + m];
      }
    }
  }
}

/* the bodies' markers on their poses, in float as the GPU's table (see
sph_bodies.comp) */
void shift::core::sph::CpuSolver::PlaceBodies() {
  PosesAt(time_, poses_);
  if(poses_.empty()) {
    return;
  }
  for(size_t i = 0; i < state_.size(); i++) {
    const uint32_t kind = state_.kind[i];
    if(!(kind & State::kMarker) || (kind & ~State::kMarker) == 0) {
      continue;
    }
    const BodyPose& pose = poses_[(kind & ~State::kMarker) - 1];
    float R[9], t[3], v[3], w[3];
    for(int k = 0; k < 9; k++) {
      R[k] = static_cast<float>(pose.R[k]);
    }
    for(int k = 0; k < 3; k++) {
      t[k] = static_cast<float>(pose.t[k]);
      v[k] = static_cast<float>(pose.v[k]);
      w[k] = static_cast<float>(pose.w[k]);
    }
    const std::array<float, 4>& l =
      state_.body_local[state_.id[i] - state_.first_body_marker];
    const float r[3] = {R[0] * l[0] + R[3] * l[1] + R[6] * l[2],
                        R[1] * l[0] + R[4] * l[1] + R[7] * l[2],
                        R[2] * l[0] + R[5] * l[1] + R[8] * l[2]};
    state_.position[i] = {r[0] + t[0], r[1] + t[1], r[2] + t[2],
                          state_.position[i][3]};
    state_.velocity[i] = {v[0] + (w[1] * r[2] - w[2] * r[1]),
                          v[1] + (w[2] * r[0] - w[0] * r[2]),
                          v[2] + (w[0] * r[1] - w[1] * r[0]), 0.0f};
  }
}

void shift::core::sph::CpuSolver::Sort() {
  const size_t n = state_.size();
  std::vector<uint32_t> hash(n);
  start_.assign(size_t(mask_) + 2, 0);
  for(size_t i = 0; i < n; i++) {
    hash[i] = CellHash(Cell(state_.position[i], cell_), mask_);
    start_[hash[i] + 1]++;
  }
  std::partial_sum(start_.begin(), start_.end(), start_.begin());
  std::vector<uint32_t> order(n);
  std::iota(order.begin(), order.end(), 0u);
  std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
    return hash[a] != hash[b] ? hash[a] < hash[b]
                              : state_.id[a] < state_.id[b];
  });
  sorted_.resize(n);
  for(size_t k = 0; k < n; k++) {
    const uint32_t i = order[k];
    sorted_.position[k] = state_.position[i];
    sorted_.velocity[k] = state_.velocity[i];
    sorted_.density[k] = state_.density[i];
    sorted_.stress[k] = state_.stress[i];
    sorted_.id[k] = state_.id[i];
    sorted_.kind[k] = state_.kind[i];
  }
}

/* the markers' stresses, from the particles around them (Adami et al.
2012): sigma_w = (sum sigma_f W - I (g . sum rho_f (x_w - x_f) W)) / sum W */
void shift::core::sph::CpuSolver::Markers() {
  const State& s = sorted_;
  const std::array<float, 3>& g = settings_.gravity;
  uint32_t hashes[27];
  for(size_t i = 0; i < s.size(); i++) {
    if(!(s.kind[i] & State::kMarker)) {
      continue;
    }
    const std::array<float, 4>& xi = s.position[i];
    float weight = 0.0f, head = 0.0f, apex = 0.0f;
    bool cone = true;
    float sigma[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const int cells = NeighborCells(xi, hashes);
    for(int c = 0; c < cells; c++) {
      for(uint32_t j = start_[hashes[c]]; j < start_[hashes[c] + 1]; j++) {
        if(s.kind[j] & State::kMarker) {
          continue;
        }
        const float dx = xi[0] - s.position[j][0];
        const float dy = xi[1] - s.position[j][1];
        const float dz = xi[2] - s.position[j][2];
        const float r = std::sqrt(dx * dx + dy * dy + dz * dz);
        const float w = Kernel(r, h_);
        if(w == 0.0f) {
          continue;
        }
        weight += w;
        for(int k = 0; k < 6; k++) {
          sigma[k] += s.stress[j][k] * w;
        }
        head += s.density[j] * (g[0] * dx + g[1] * dy + g[2] * dz) * w;
        const std::array<float, 8>& m = table_[s.kind[j]];
        if(m[4] > 0.0f) {
          apex += m[5] / m[4] / 3.0f * w;
        } else {
          cone = false;
        }
      }
    }
    std::array<float, 8>& out = sorted_.stress[i];
    for(int k = 0; k < 8; k++) {
      out[k] = 0.0f;
    }
    if(weight > 0.0f) {
      for(int k = 0; k < 6; k++) {
        out[k] = sigma[k] / weight;
      }
      for(int k = 0; k < 3; k++) {
        out[k] -= head / weight;
      }
      /* no more tension than the soil around it carries, the apex of its
      cone: above the soil, the head would put the marker in tension and
      pull the soil to it */
      const float excess = (out[0] + out[1] + out[2]) / 3.0f - apex / weight;
      if(cone && excess > 0.0f) {
        for(int k = 0; k < 3; k++) {
          out[k] -= excess;
        }
      }
    }
  }
}

/* the density and stress rates, and the return to the Drucker-Prager cone,
into state_ */
void shift::core::sph::CpuSolver::Rates(float dt) {
  const State& s = sorted_;
  const size_t n = s.size();
  state_.resize(n);
  uint32_t hashes[27];
  for(size_t i = 0; i < n; i++) {
    state_.density[i] = s.density[i];
    state_.stress[i] = s.stress[i];
    if(s.kind[i] & State::kMarker) {
      continue;
    }
    const std::array<float, 4>& xi = s.position[i];
    const std::array<float, 4>& vi = s.velocity[i];
    float drho = 0.0f;
    float l[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    const int cells = NeighborCells(xi, hashes);
    for(int c = 0; c < cells; c++) {
      for(uint32_t j = start_[hashes[c]]; j < start_[hashes[c] + 1]; j++) {
        if(j == i) {
          continue;
        }
        const float dx = xi[0] - s.position[j][0];
        const float dy = xi[1] - s.position[j][1];
        const float dz = xi[2] - s.position[j][2];
        const float r = std::sqrt(dx * dx + dy * dy + dz * dz);
        if(!(r < 2.0f * h_) || r < 1e-6f * h_) {
          continue;
        }
        const float f = KernelDerivative(r, h_) / r;
        const float grad[3] = {f * dx, f * dy, f * dz};
        /* a wall's velocity is 0, a body's its own */
        const float vj[3] = {s.velocity[j][0], s.velocity[j][1],
                             s.velocity[j][2]};
        const float mj = s.position[j][3];
        const float dv[3] = {vi[0] - vj[0], vi[1] - vj[1], vi[2] - vj[2]};
        drho += mj * (dv[0] * grad[0] + dv[1] * grad[1] + dv[2] * grad[2]);
        const float volume = mj / s.density[j];
        for(int a = 0; a < 3; a++) {
          for(int b = 0; b < 3; b++) {
            l[a][b] -= volume * dv[a] * grad[b];
          }
        }
      }
    }
    const std::array<float, 8>& m = table_[s.kind[i]];
    const float shear = m[1], bulk = m[2], alpha = m[4], k_c = m[5],
                beta = m[6];
    /* strain rate and spin */
    float e[3][3], w[3][3];
    for(int a = 0; a < 3; a++) {
      for(int b = 0; b < 3; b++) {
        e[a][b] = 0.5f * (l[a][b] + l[b][a]);
        w[a][b] = 0.5f * (l[a][b] - l[b][a]);
      }
    }
    const std::array<float, 8>& t = s.stress[i];
    const float sig[3][3] = {{t[0], t[3], t[5]}, {t[3], t[1], t[4]},
                             {t[5], t[4], t[2]}};
    const float trace = e[0][0] + e[1][1] + e[2][2];
    float rate[3][3];
    for(int a = 0; a < 3; a++) {
      for(int b = 0; b < 3; b++) {
        /* sigma w^T + w sigma */
        float jaumann = 0.0f;
        for(int k = 0; k < 3; k++) {
          jaumann += sig[a][k] * w[b][k] + w[a][k] * sig[k][b];
        }
        rate[a][b] = jaumann + 2.0f * shear * e[a][b];
      }
      rate[a][a] += (bulk - 2.0f * shear / 3.0f) * trace;
    }
    float out[6] = {t[0] + dt * rate[0][0], t[1] + dt * rate[1][1],
                    t[2] + dt * rate[2][2], t[3] + dt * rate[0][1],
                    t[4] + dt * rate[1][2], t[5] + dt * rate[2][0]};
    /* the return to the cone */
    const float i1 = out[0] + out[1] + out[2];
    const float mean = i1 / 3.0f;
    const float s0 = out[0] - mean, s1 = out[1] - mean, s2 = out[2] - mean;
    const float j2 = 0.5f * (s0 * s0 + s1 * s1 + s2 * s2) +
                     out[3] * out[3] + out[4] * out[4] + out[5] * out[5];
    const float root = std::sqrt(j2);
    const float yield = alpha * i1 + root - k_c;
    if(yield > 0.0f) {
      const float lambda = yield / (shear + 9.0f * bulk * alpha * beta);
      const float reduced = root - shear * lambda;
      if(reduced <= 0.0f && alpha > 0.0f) {
        /* to the apex */
        const float apex = k_c / alpha / 3.0f;
        out[0] = out[1] = out[2] = apex;
        out[3] = out[4] = out[5] = 0.0f;
      } else {
        const float scale = reduced > 0.0f ? reduced / root : 0.0f;
        const float new_mean = (i1 - 9.0f * bulk * beta * lambda) / 3.0f;
        out[0] = s0 * scale + new_mean;
        out[1] = s1 * scale + new_mean;
        out[2] = s2 * scale + new_mean;
        out[3] *= scale;
        out[4] *= scale;
        out[5] *= scale;
      }
    }
    for(int k = 0; k < 6; k++) {
      state_.stress[i][k] = out[k];
    }
    /* no less than at rest: a particle drawn away from its neighbours is
    loose soil, not lighter soil, whose volume and acceleration (m / rho,
    sigma / rho^2) would grow as it thins and blow it apart */
    state_.density[i] = std::max(s.density[i] + dt * drho, m[0]);
  }
}

/* the acceleration, from the stresses of state_ and the rest of sorted_; then
the velocity and position into state_ */
void shift::core::sph::CpuSolver::Momentum(float dt) {
  const State& s = sorted_;
  const size_t n = s.size();
  const std::array<float, 3>& g = settings_.gravity;
  const float viscosity = settings_.artificial_viscosity;
  uint32_t hashes[27];
  for(size_t i = 0; i < n; i++) {
    state_.id[i] = s.id[i];
    state_.kind[i] = s.kind[i];
    state_.position[i] = s.position[i];
    state_.velocity[i] = s.velocity[i];
    if(s.kind[i] & State::kMarker) {
      if(s.kind[i] & ~State::kMarker) {
        BodyMarker(i, dt);
      }
      continue;
    }
    const std::array<float, 4>& xi = s.position[i];
    const std::array<float, 4>& vi = s.velocity[i];
    const std::array<float, 8>& ti = state_.stress[i];
    const float rhoi = state_.density[i];
    const float ci = table_[s.kind[i]][3];
    float a[3] = {g[0], g[1], g[2]};
    const int cells = NeighborCells(xi, hashes);
    for(int c = 0; c < cells; c++) {
      for(uint32_t j = start_[hashes[c]]; j < start_[hashes[c] + 1]; j++) {
        if(j == i) {
          continue;
        }
        const float dx = xi[0] - s.position[j][0];
        const float dy = xi[1] - s.position[j][1];
        const float dz = xi[2] - s.position[j][2];
        const float r2 = dx * dx + dy * dy + dz * dz;
        const float r = std::sqrt(r2);
        if(!(r < 2.0f * h_) || r < 1e-6f * h_) {
          continue;
        }
        const float f = KernelDerivative(r, h_) / r;
        const float grad[3] = {f * dx, f * dy, f * dz};
        const bool marker = (s.kind[j] & State::kMarker) != 0;
        /* a wall's velocity is 0, a body's its own */
        const float vj[3] = {s.velocity[j][0], s.velocity[j][1],
                             s.velocity[j][2]};
        const float mj = s.position[j][3];
        const float rhoj = state_.density[j];
        const std::array<float, 8>& tj = state_.stress[j];
        const float cj = marker ? ci : table_[s.kind[j]][3];
        /* Monaghan's artificial viscosity, where the two approach */
        const float approach = (vi[0] - vj[0]) * dx + (vi[1] - vj[1]) * dy +
                               (vi[2] - vj[2]) * dz;
        float pi = 0.0f;
        if(approach < 0.0f) {
          const float mu = h_ * approach / (r2 + 0.01f * h_ * h_);
          pi = -viscosity * 0.5f * (ci + cj) * mu / (0.5f * (rhoi + rhoj));
        }
        const float pi2 = 1.0f / (rhoi * rhoi), pj2 = 1.0f / (rhoj * rhoj);
        const float t[6] = {ti[0] * pi2 + tj[0] * pj2 - pi,
                            ti[1] * pi2 + tj[1] * pj2 - pi,
                            ti[2] * pi2 + tj[2] * pj2 - pi,
                            ti[3] * pi2 + tj[3] * pj2,
                            ti[4] * pi2 + tj[4] * pj2,
                            ti[5] * pi2 + tj[5] * pj2};
        a[0] += mj * (t[0] * grad[0] + t[3] * grad[1] + t[5] * grad[2]);
        a[1] += mj * (t[3] * grad[0] + t[1] * grad[1] + t[4] * grad[2]);
        a[2] += mj * (t[5] * grad[0] + t[4] * grad[1] + t[2] * grad[2]);
      }
    }
    for(int k = 0; k < 3; k++) {
      const float v = vi[k] + dt * a[k];
      state_.velocity[i][k] = v;
      state_.position[i][k] = xi[k] + dt * v;
    }
  }
}

/* as sph_momentum.comp: the particles' accelerations from the marker, as
Momentum computes them, the opposite times their masses */
void shift::core::sph::CpuSolver::BodyMarker(size_t i, float dt) {
  const State& s = sorted_;
  const float viscosity = settings_.artificial_viscosity;
  const std::array<float, 4>& xm = s.position[i];
  const std::array<float, 4>& vm = s.velocity[i];
  const std::array<float, 8>& tm = state_.stress[i];
  const float rhom = state_.density[i];
  const float mm = xm[3];
  float force[3] = {0.0f, 0.0f, 0.0f};
  uint32_t hashes[27];
  const int cells = NeighborCells(xm, hashes);
  for(int c = 0; c < cells; c++) {
    for(uint32_t j = start_[hashes[c]]; j < start_[hashes[c] + 1]; j++) {
      if(s.kind[j] & State::kMarker) {
        continue;
      }
      const std::array<float, 4>& xj = s.position[j];
      const float dx = xj[0] - xm[0];
      const float dy = xj[1] - xm[1];
      const float dz = xj[2] - xm[2];
      const float r2 = dx * dx + dy * dy + dz * dz;
      const float r = std::sqrt(r2);
      if(!(r < 2.0f * h_) || r < 1e-6f * h_) {
        continue;
      }
      const float f = KernelDerivative(r, h_) / r;
      const float grad[3] = {f * dx, f * dy, f * dz};
      const std::array<float, 4>& vj = s.velocity[j];
      const float rhoj = state_.density[j];
      const std::array<float, 8>& tj = state_.stress[j];
      const float cj = table_[s.kind[j]][3];
      const float approach =
        (vj[0] - vm[0]) * dx + (vj[1] - vm[1]) * dy + (vj[2] - vm[2]) * dz;
      float pi = 0.0f;
      if(approach < 0.0f) {
        const float mu = h_ * approach / (r2 + 0.01f * h_ * h_);
        pi = -viscosity * 0.5f * (cj + cj) * mu / (0.5f * (rhoj + rhom));
      }
      const float pj2 = 1.0f / (rhoj * rhoj), pm2 = 1.0f / (rhom * rhom);
      const float t[6] = {tj[0] * pj2 + tm[0] * pm2 - pi,
                          tj[1] * pj2 + tm[1] * pm2 - pi,
                          tj[2] * pj2 + tm[2] * pm2 - pi,
                          tj[3] * pj2 + tm[3] * pm2,
                          tj[4] * pj2 + tm[4] * pm2,
                          tj[5] * pj2 + tm[5] * pm2};
      const float a[3] = {
        mm * (t[0] * grad[0] + t[3] * grad[1] + t[5] * grad[2]),
        mm * (t[3] * grad[0] + t[1] * grad[1] + t[4] * grad[2]),
        mm * (t[5] * grad[0] + t[4] * grad[1] + t[2] * grad[2])};
      for(int k = 0; k < 3; k++) {
        force[k] -= xj[3] * a[k];
      }
    }
  }
  const size_t n = state_.body_local.size();
  const size_t slot = s.id[i] - state_.first_body_marker;
  const float moment[3] = {xm[1] * force[2] - xm[2] * force[1],
                           xm[2] * force[0] - xm[0] * force[2],
                           xm[0] * force[1] - xm[1] * force[0]};
  for(int k = 0; k < 3; k++) {
    impulse_[k * n + slot] += dt * force[k];
    impulse_[(k + 3) * n + slot] += dt * moment[k];
  }
}

void shift::core::sph::CpuSolver::Step(float dt) {
  PlaceBodies();
  Sort();
  Markers();
  Rates(dt);
  Momentum(dt);
  time_ += dt;
}

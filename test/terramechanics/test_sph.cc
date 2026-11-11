/* the SPH soil (shift/core/sph): the CPU reference and the GPU solver agree
and the GPU gives the same bits on every run; a block in a box carries its
weight; a collapsing column runs out as far as Lube et al.'s experiments; a
heap settles near the angle of friction. The GPU tests are skipped when
shift is built without Vulkan, or when no device is found unless
SHIFT_GPU_REQUIRED is set. */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/exception.h"
#include "shift/core/gpu/device.h"
#include "shift/core/sph/cpu_solver.h"
#include "shift/core/sph/gpu_solver.h"
#include "shift/core/sph/sph.h"

static const float kPi = 3.14159265f;

/* dry sand, softened to E = 2 MPa so that the substeps are longer */
static shift::core::sph::Material Sand(float friction_angle = 0.52f) {
  shift::core::sph::Material sand;
  sand.density = 1600.0f;
  sand.youngs_modulus = 2.0e6f;
  sand.poisson_ratio = 0.3f;
  sand.friction_angle = friction_angle;
  sand.dilatancy_angle = 0.0f;
  sand.cohesion = 0.0f;
  return sand;
}

static shift::core::sph::Settings Settings(float spacing) {
  shift::core::sph::Settings settings;
  settings.spacing = spacing;
  return settings;
}

/* a floor of markers under z = 0 (z down): layers deep, over |x|, |y| <=
half */
static void AddFloor(shift::core::sph::State& state, float half, float spacing,
                     int layers, const shift::core::sph::Material& material) {
  const int n = static_cast<int>(std::ceil(half / spacing));
  for(int k = 0; k < layers; k++) {
    for(int j = -n; j <= n; j++) {
      for(int i = -n; i <= n; i++) {
        state.AddMarker({i * spacing, j * spacing, (k + 0.5f) * spacing},
                        material, spacing);
      }
    }
  }
}

/* a box of particles, nx by ny by nz, its bottom on z = 0 */
static void AddBlock(shift::core::sph::State& state, int nx, int ny, int nz,
                     float spacing, const shift::core::sph::Material& material) {
  for(int k = 0; k < nz; k++) {
    for(int j = 0; j < ny; j++) {
      for(int i = 0; i < nx; i++) {
        state.AddParticle({(i - 0.5f * (nx - 1)) * spacing,
                           (j - 0.5f * (ny - 1)) * spacing,
                           -(k + 0.5f) * spacing},
                          0, material, spacing);
      }
    }
  }
}

/* a cylinder of particles of radius r0 and height h0, its bottom on z = 0 */
static void AddColumn(shift::core::sph::State& state, float r0, float h0,
                      float spacing, const shift::core::sph::Material& material) {
  const int n = static_cast<int>(r0 / spacing) + 1;
  const int layers = static_cast<int>(std::round(h0 / spacing));
  for(int k = 0; k < layers; k++) {
    for(int j = -n; j <= n; j++) {
      for(int i = -n; i <= n; i++) {
        const float x = (i + 0.5f) * spacing, y = (j + 0.5f) * spacing;
        if(x * x + y * y <= r0 * r0) {
          state.AddParticle({x, y, -(k + 0.5f) * spacing}, 0, material,
                            spacing);
        }
      }
    }
  }
}

/* the particles (not markers) of a state, by id */
static std::vector<std::array<float, 4>> ByIdParticles(
  const shift::core::sph::State& state) {
  std::vector<std::array<float, 4>> out(state.size());
  for(size_t k = 0; k < state.size(); k++) {
    out[state.id[k]] = state.position[k];
  }
  return out;
}

/* the GPU device for the tests, or null with the test skipped */
static std::unique_ptr<shift::core::gpu::Device> TestDevice() {
  if(!shift::core::gpu::Device::is_built()) {
    return nullptr;
  }
  try {
    return std::make_unique<shift::core::gpu::Device>();
  } catch(const shift::core::Exception& e) {
    if(std::getenv("SHIFT_GPU_REQUIRED")) {
      ADD_FAILURE() << e.what();
    }
    return nullptr;
  }
}

#define REQUIRE_DEVICE(device)                                                \
  auto device = TestDevice();                                                \
  if(!device) {                                                               \
    GTEST_SKIP() << "no GPU device";                                          \
  }

TEST(Sph, KernelIntegratesToOne) {
  /* the kernel over a fine grid sums to 1, and its derivative is its slope */
  const float h = 0.013f, d = 0.0025f;
  double sum = 0.0;
  for(int k = -12; k <= 12; k++) {
    for(int j = -12; j <= 12; j++) {
      for(int i = -12; i <= 12; i++) {
        sum += shift::core::sph::Kernel(
                 d * std::sqrt(float(i * i + j * j + k * k)), h) *
               d * d * d;
      }
    }
  }
  EXPECT_NEAR(sum, 1.0, 1e-3);
  for(float r : {0.2f * h, 0.9f * h, 1.7f * h}) {
    const float e = 1e-4f * h;
    const float slope = (shift::core::sph::Kernel(r + e, h) -
                         shift::core::sph::Kernel(r - e, h)) /
                        (2.0f * e);
    EXPECT_NEAR(shift::core::sph::KernelDerivative(r, h), slope,
                1e-2f * std::abs(slope));
  }
  EXPECT_EQ(shift::core::sph::Kernel(2.0f * h, h), 0.0f);
}

/* a block 6 particles deep in a box of markers: once settled, it is at rest
and the vertical stress of its middle grows with depth as the weight above,
sigma_zz = -rho g depth */
TEST(Sph, BlockCarriesItsWeight) {
  REQUIRE_DEVICE(device);
  const float s = 0.02f;
  const shift::core::sph::Material sand = Sand();
  shift::core::sph::State state;
  AddFloor(state, 0.11f, s, 3, sand);
  /* the walls: 3 layers around the 8 by 8 block (within 4 s of the axis),
  8 particles high */
  for(int k = 0; k < 8; k++) {
    for(int j = -7; j < 7; j++) {
      for(int i = -7; i < 7; i++) {
        const float x = (i + 0.5f) * s, y = (j + 0.5f) * s;
        if(std::max(std::abs(x), std::abs(y)) > 4.0f * s) {
          state.AddMarker({x, y, -(k + 0.5f) * s}, sand, s);
        }
      }
    }
  }
  AddBlock(state, 8, 8, 6, s, sand);
  shift::core::sph::Settings settings = Settings(s);
  shift::core::sph::GpuSolver solver(*device, settings, {sand}, state);
  solver.Advance(0.4);
  shift::core::sph::State end;
  solver.GetState(end);
  float fastest = 0.0f;
  double weight_error = 0.0;
  int middle = 0;
  for(size_t k = 0; k < end.size(); k++) {
    if(end.kind[k] & shift::core::sph::State::kMarker) {
      continue;
    }
    const auto& v = end.velocity[k];
    fastest = std::max(fastest, std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));
    const auto& x = end.position[k];
    /* the middle of the block, away from its sides, top and bottom */
    if(std::abs(x[0]) < 0.03f && std::abs(x[1]) < 0.03f && x[2] < -0.03f &&
       x[2] > -0.09f) {
      const float depth = x[2] + 6 * s; // below the top, about
      const float expected = -sand.density * 9.81f * depth;
      weight_error += std::abs(end.stress[k][2] - expected) /
                      (sand.density * 9.81f * 6 * s);
      middle++;
    }
  }
  ASSERT_GT(middle, 0);
  std::printf("fastest particle %.4f m/s; vertical stress off by %.1f %% of "
              "the weight\n",
              fastest, 100.0 * weight_error / middle);
  EXPECT_LT(fastest, 0.02f);
  EXPECT_LT(weight_error / middle, 0.1);
}

/* the GPU solver does the CPU's arithmetic: after 40 substeps of a block
slumping on a floor, the particles are where the CPU puts them */
TEST(Sph, GpuMatchesCpu) {
  REQUIRE_DEVICE(device);
  const float s = 0.02f;
  const shift::core::sph::Material sand = Sand();
  shift::core::sph::State state;
  AddFloor(state, 0.12f, s, 3, sand);
  AddBlock(state, 4, 4, 10, s, sand);
  const shift::core::sph::Settings settings = Settings(s);
  const float dt = shift::core::sph::StableStep(settings, {sand});
  shift::core::sph::CpuSolver cpu(settings, {sand}, state);
  shift::core::sph::GpuSolver gpu(*device, settings, {sand}, state);
  cpu.Steps(dt, 40);
  gpu.Steps(dt, 40);
  shift::core::sph::State a, b;
  cpu.GetState(a);
  gpu.GetState(b);
  const auto pa = ByIdParticles(a), pb = ByIdParticles(b);
  float moved = 0.0f, difference = 0.0f;
  for(size_t k = 0; k < state.size(); k++) {
    for(int c = 0; c < 3; c++) {
      moved = std::max(moved, std::abs(pa[k][c] - state.position[k][c]));
      difference = std::max(difference, std::abs(pa[k][c] - pb[k][c]));
    }
  }
  std::printf("moved %.3g m, CPU and GPU differ by %.3g m\n", moved,
              difference);
  EXPECT_GT(moved, 1e-4f);
  EXPECT_LT(difference, 1e-3f * moved);
}

/* two runs on the GPU give the same bits */
TEST(Sph, GpuRepeats) {
  REQUIRE_DEVICE(device);
  const float s = 0.02f;
  const shift::core::sph::Material sand = Sand();
  shift::core::sph::State state;
  AddFloor(state, 0.2f, s, 3, sand);
  AddColumn(state, 0.08f, 0.16f, s, sand);
  const shift::core::sph::Settings settings = Settings(s);
  shift::core::sph::State runs[2];
  for(auto& run : runs) {
    shift::core::sph::GpuSolver gpu(*device, settings, {sand}, state);
    gpu.Advance(0.05);
    gpu.GetState(run);
  }
  ASSERT_EQ(runs[0].size(), runs[1].size());
  EXPECT_EQ(std::memcmp(runs[0].position.data(), runs[1].position.data(),
                        runs[0].size() * 16),
            0);
  EXPECT_EQ(std::memcmp(runs[0].stress.data(), runs[1].stress.data(),
                        runs[0].size() * 32),
            0);
}

/* the radius within which nearly all the particles lie (the 99th
percentile), about the z axis */
static float Runout(const shift::core::sph::State& state) {
  std::vector<float> radii;
  for(size_t k = 0; k < state.size(); k++) {
    if(!(state.kind[k] & shift::core::sph::State::kMarker)) {
      radii.push_back(std::hypot(state.position[k][0], state.position[k][1]));
    }
  }
  std::sort(radii.begin(), radii.end());
  return radii[static_cast<size_t>(0.99 * (radii.size() - 1))];
}

/* an axisymmetric column of sand of aspect ratio a = h0 / r0 = 1 collapses
and runs out to (r - r0) / r0 = 1.24 a, the fit of Lube et al. (2004,
J. Fluid Mech. 508) to their experiments for a < 1.7 */
TEST(Sph, ColumnCollapseRunout) {
  REQUIRE_DEVICE(device);
  const float s = 0.01f, r0 = 0.1f, h0 = 0.1f;
  const shift::core::sph::Material sand = Sand();
  shift::core::sph::State state;
  AddFloor(state, 0.45f, s, 3, sand);
  AddColumn(state, r0, h0, s, sand);
  shift::core::sph::GpuSolver gpu(*device, Settings(s), {sand}, state);
  shift::core::sph::State end;
  float runout = 0.0f;
  for(int k = 0; k < 8; k++) {
    gpu.Advance(0.1);
    gpu.GetState(end);
    runout = Runout(end);
  }
  const float expected = r0 * (1.0f + 1.24f * (h0 / r0));
  std::printf("runout %.3f m, Lube et al. %.3f m\n", runout, expected);
  EXPECT_NEAR(runout, expected, 0.2f * expected);
}

/* a cone of sand steeper than its friction angle slumps to a heap whose
sides lie near the friction angle */
TEST(Sph, HeapSettlesNearTheFrictionAngle) {
  REQUIRE_DEVICE(device);
  const float s = 0.01f, phi = 0.52f; // 30 degrees
  const shift::core::sph::Material sand = Sand(phi);
  shift::core::sph::State state;
  AddFloor(state, 0.3f, s, 3, sand);
  /* a cone of base radius 0.12 m and sides at 50 degrees */
  const float base = 0.12f, steep = 50.0f * kPi / 180.0f;
  const int n = static_cast<int>(base / s) + 1;
  for(int k = 0;; k++) {
    const float z = (k + 0.5f) * s;
    const float radius = base - z / std::tan(steep);
    if(radius <= 0.0f) {
      break;
    }
    for(int j = -n; j <= n; j++) {
      for(int i = -n; i <= n; i++) {
        const float x = (i + 0.5f) * s, y = (j + 0.5f) * s;
        if(x * x + y * y <= radius * radius) {
          state.AddParticle({x, y, -z}, 0, sand, s);
        }
      }
    }
  }
  shift::core::sph::GpuSolver gpu(*device, Settings(s), {sand}, state);
  gpu.Advance(1.0);
  shift::core::sph::State end;
  gpu.GetState(end);
  /* the top of the heap in rings of the radius, and the slope of a line
  through the middle rings (least squares) */
  const int rings = 40;
  const float ring = 0.01f;
  std::vector<float> top(rings, 0.0f);
  for(size_t k = 0; k < end.size(); k++) {
    if(end.kind[k] & shift::core::sph::State::kMarker) {
      continue;
    }
    const float r = std::hypot(end.position[k][0], end.position[k][1]);
    const int b = static_cast<int>(r / ring);
    if(b < rings) {
      top[b] = std::max(top[b], -end.position[k][2]);
    }
  }
  /* the rings from a fifth to four fifths of the top's height */
  const float peak = *std::max_element(top.begin(), top.end());
  double sr = 0.0, sz = 0.0, srr = 0.0, srz = 0.0;
  int count = 0;
  for(int b = 0; b < rings; b++) {
    if(top[b] > 0.2f * peak && top[b] < 0.8f * peak) {
      const double r = (b + 0.5) * ring;
      sr += r;
      sz += top[b];
      srr += r * r;
      srz += r * top[b];
      count++;
    }
  }
  ASSERT_GE(count, 3);
  const double slope = (count * srz - sr * sz) / (count * srr - sr * sr);
  const double angle = std::atan(-slope) * 180.0 / kPi;
  std::printf("heap %.3f m high, sides at %.1f degrees; friction angle %.1f\n",
              peak, angle, phi * 180.0 / kPi);
  EXPECT_NEAR(angle, phi * 180.0 / kPi, 5.0);
}

/* a plate of body markers, 4 by 4 by 2, pressed down into a block of
particles at 0.5 m/s: the GPU and the CPU agree on the particles and on the
plate's impulse, which pushes the plate back up (-z, z down) and has no
moment about the plate's centre to speak of, the setup being symmetric */
TEST(Sph, BodyPressesIntoTheSoil) {
  REQUIRE_DEVICE(device);
  const float s = 0.02f;
  const shift::core::sph::Material sand = Sand();
  shift::core::sph::State state;
  AddFloor(state, 0.16f, s, 3, sand);
  AddBlock(state, 8, 8, 5, s, sand);
  /* the plate's markers, centred on its origin */
  for(int k = 0; k < 2; k++) {
    for(int j = 0; j < 4; j++) {
      for(int i = 0; i < 4; i++) {
        state.AddBodyMarker({(i - 1.5f) * s, (j - 1.5f) * s, (k - 0.5f) * s},
                            0, sand, s);
      }
    }
  }
  const shift::core::sph::Settings settings = Settings(s);
  const float dt = shift::core::sph::StableStep(settings, {sand});
  const long n = 100;
  const double duration = n * double(dt), speed = 0.5;
  /* from just above the block (its top at z = -0.1), down */
  shift::core::sph::BodyPose start, end;
  start.t = {0.0, 0.0, -0.1 - 1.5 * s};
  start.v = end.v = {0.0, 0.0, speed};
  end.t = {0.0, 0.0, start.t[2] + speed * duration};
  shift::core::sph::CpuSolver cpu(settings, {sand}, state);
  shift::core::sph::GpuSolver gpu(*device, settings, {sand}, state);
  std::vector<std::array<double, 6>> impulse[2];
  shift::core::sph::Solver* solvers[2] = {&cpu, &gpu};
  for(int k = 0; k < 2; k++) {
    solvers[k]->SetBodyPaths({start}, {end}, 0.0, duration);
    solvers[k]->Steps(dt, n);
    solvers[k]->GetImpulses(impulse[k]);
    ASSERT_EQ(impulse[k].size(), 1u);
  }
  shift::core::sph::State a, b;
  cpu.GetState(a);
  gpu.GetState(b);
  const auto pa = ByIdParticles(a), pb = ByIdParticles(b);
  float difference = 0.0f;
  for(size_t k = 0; k < state.size(); k++) {
    for(int c = 0; c < 3; c++) {
      difference = std::max(difference, std::abs(pa[k][c] - pb[k][c]));
    }
  }
  /* the markers are placed at the start of each substep: the plate ends a
  substep short of the end of its path */
  const long m = a.Find(state.first_body_marker);
  ASSERT_GE(m, 0);
  EXPECT_NEAR(a.position[m][2], end.t[2] - speed * dt - 0.5 * s, 1e-6);
  const std::array<double, 6>& j = impulse[0][0];
  std::printf("CPU and GPU differ by %.3g m; impulse %.4g %.4g %.4g N s, "
              "GPU %.4g N s\n",
              difference, j[0], j[1], j[2], impulse[1][0][2]);
  EXPECT_LT(difference, 1e-6f);
  EXPECT_LT(j[2], 0.0);
  EXPECT_LT(std::abs(j[0]), 0.01 * std::abs(j[2]));
  EXPECT_LT(std::abs(j[1]), 0.01 * std::abs(j[2]));
  for(int c = 0; c < 6; c++) {
    EXPECT_NEAR(impulse[1][0][c], j[c], 1e-4 * std::abs(j[2]) + 1e-12) << c;
  }
  /* the moment about the plate's mean depth on its axis: none but for the
  float sums */
  const double zc = 0.5 * (start.t[2] + end.t[2]);
  const double moment_x = j[3] - (0.0 * j[2] - zc * j[1]);
  const double moment_y = j[4] - (zc * j[0] - 0.0 * j[2]);
  EXPECT_LT(std::abs(moment_x), 1e-3 * std::abs(j[2]) * s);
  EXPECT_LT(std::abs(moment_y), 1e-3 * std::abs(j[2]) * s);
}

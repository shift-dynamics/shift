/* the DEM (shift/core/dem): a sphere on a wall rebounds with its coefficient
of restitution and rests at Hertz's overlap; it rolls down a slope at
5/7 g sin(angle) without slipping, and stops on one gentler than its rolling
resistance; a collision keeps the momentum and the angular momentum; a free
clump keeps its angular momentum (the gyroscopic term); a grain far from the
origin behaves as one near it; and the GPU does the CPU's arithmetic and
gives the same bits on every run. A sphere rests on the triangles of a body
as on a wall, once where triangles meet, and its weight is the body's
impulse; a moving body pushes grains and takes the opposite impulse, on the
GPU as on the CPU. Grains of convex hulls rest flat on a floor and on
each other, keep momentum in collisions with each other and with spheres,
settle in a box, rest on a body's triangles as on a wall, are pushed by a
moving body, and do on the GPU what they do on the CPU. The GPU tests
are skipped when shift is
built without Vulkan, or when no device is found unless SHIFT_GPU_REQUIRED
is set. */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/dem/cpu_solver.h"
#include "shift/core/dem/dem.h"
#include "shift/core/dem/gpu_solver.h"
#include "shift/core/exception.h"
#include "shift/core/gpu/device.h"

static const double kPi = 3.14159265358979323846;

/* glass beads, softened to 50 MPa so that the steps are longer */
static shift::core::dem::Material Glass(float restitution = 0.5f,
                                        float friction = 0.5f,
                                        float rolling = 0.0f) {
  shift::core::dem::Material m;
  m.youngs_modulus = 5.0e7f;
  m.poisson_ratio = 0.3f;
  m.restitution = restitution;
  m.friction = friction;
  m.rolling_resistance = rolling;
  return m;
}

static const float kRadius = 0.005f;
static const float kDensity = 2500.0f;

/* a floor through the origin, the solid below it (z down) */
static shift::core::dem::Wall Floor() {
  shift::core::dem::Wall w;
  w.normal = {0.0, 0.0, -1.0};
  w.offset = 0.0;
  return w;
}

static shift::core::dem::Settings NoGravity() {
  shift::core::dem::Settings s;
  s.gravity = {0.0f, 0.0f, 0.0f};
  return s;
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

/* the speed after a sphere at speed hits the floor, with gravity off */
static double Rebound(float restitution, double speed) {
  const auto sphere =
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0);
  const std::vector<shift::core::dem::Material> materials = {
    Glass(restitution)};
  shift::core::dem::Settings settings = NoGravity();
  settings.walls = {Floor()};
  shift::core::dem::State state;
  state.AddGrain(0, {0.0, 0.0, -kRadius - 1e-4});
  state.velocity[0] = {0.0f, 0.0f, static_cast<float>(speed)};
  shift::core::dem::CpuSolver solver(settings, materials, {sphere}, state);
  const float dt = static_cast<float>(
    shift::core::dem::StableStep({sphere}, materials, speed, 0.05));
  for(int k = 0; k < 200000; k++) {
    solver.Step(dt);
    solver.GetState(state);
    if(state.velocity[0][2] < 0.0f && state.position[0][2] < -kRadius) {
      return -state.velocity[0][2];
    }
  }
  return 0.0;
}

TEST(Dem, ReboundsWithRestitution) {
  for(float e : {0.2f, 0.5f, 0.9f}) {
    for(double speed : {0.1, 1.0}) {
      const double ratio = Rebound(e, speed) / speed;
      std::printf("restitution %.2f at %.1f m/s: rebounds at %.4f\n", e, speed,
                  ratio);
      EXPECT_NEAR(ratio, e, 0.03);
    }
  }
}

/* a sphere comes to rest on the floor at Hertz's overlap for its weight */
TEST(Dem, RestsAtHertzOverlap) {
  const auto sphere =
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0);
  const std::vector<shift::core::dem::Material> materials = {Glass(0.3f)};
  shift::core::dem::Settings settings;
  settings.walls = {Floor()};
  shift::core::dem::State state;
  state.AddGrain(0, {0.0, 0.0, -kRadius});
  shift::core::dem::CpuSolver solver(settings, materials, {sphere}, state);
  const float dt =
    static_cast<float>(shift::core::dem::StableStep({sphere}, materials, 0.1));
  solver.Steps(dt, static_cast<long>(0.2 / dt));
  solver.GetState(state);
  const double nu = materials[0].poisson_ratio;
  /* the wall is of the same material: 1/e* = 2 (1 - nu^2) / e */
  const double e = materials[0].youngs_modulus / (2.0 * (1.0 - nu * nu));
  const double weight = sphere.mass * 9.81;
  const double expected =
    std::pow(weight / (4.0 / 3.0 * e * std::sqrt(double(kRadius))), 2.0 / 3.0);
  const double overlap = kRadius + state.position[0][2];
  std::printf("overlap %.4g m, Hertz %.4g m\n", overlap, expected);
  EXPECT_NEAR(overlap, expected, 0.01 * expected);
  EXPECT_LT(std::abs(state.velocity[0][2]), 1e-4f);
}

/* gravity tilted by angle about y, the floor level: the sphere rolls along
x; its speed after duration */
static shift::core::dem::State Slope(double angle, float rolling,
                                     double duration) {
  const auto sphere =
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0);
  const std::vector<shift::core::dem::Material> materials = {
    Glass(0.3f, 0.5f, rolling)};
  shift::core::dem::Settings settings;
  settings.gravity = {static_cast<float>(9.81 * std::sin(angle)), 0.0f,
                      static_cast<float>(9.81 * std::cos(angle))};
  settings.walls = {Floor()};
  shift::core::dem::State state;
  state.AddGrain(0, {0.0, 0.0, -kRadius});
  shift::core::dem::CpuSolver solver(settings, materials, {sphere}, state);
  const float dt =
    static_cast<float>(shift::core::dem::StableStep({sphere}, materials, 0.5));
  solver.Steps(dt, static_cast<long>(duration / dt));
  solver.GetState(state);
  return state;
}

TEST(Dem, RollsDownSlope) {
  const double angle = 10.0 * kPi / 180.0, duration = 0.2;
  const shift::core::dem::State state = Slope(angle, 0.0f, duration);
  const double expected = 5.0 / 7.0 * 9.81 * std::sin(angle) * duration;
  const double v = state.velocity[0][0];
  /* rolling about y: v = -w_y r along +x with z down */
  const double rim = -state.angular_velocity[0][1] * kRadius;
  std::printf("speed %.4f m/s (5/7 g sin: %.4f), rim speed %.4f\n", v,
              expected, rim);
  EXPECT_NEAR(v, expected, 0.02 * expected);
  EXPECT_NEAR(rim, v, 0.01 * v);
}

TEST(Dem, RollingResistanceHoldsOnGentleSlope) {
  /* the moment mu_r f r holds the sphere while tan(angle) < mu_r (rolling
  without slip; mu_r = 0.1) */
  const shift::core::dem::State held = Slope(3.0 * kPi / 180.0, 0.1f, 0.3);
  const shift::core::dem::State rolls = Slope(10.0 * kPi / 180.0, 0.1f, 0.3);
  std::printf("3 degrees: %.5f m/s, 10 degrees: %.4f m/s\n",
              held.velocity[0][0], rolls.velocity[0][0]);
  EXPECT_LT(std::abs(held.velocity[0][0]), 0.003f);
  EXPECT_GT(rolls.velocity[0][0], 0.1f);
}

/* the linear and angular momentum (about the origin) of the grains */
static void Momentum(const shift::core::dem::State& state,
                     const std::vector<shift::core::dem::Template>& templates,
                     double p[3], double l[3]) {
  for(int k = 0; k < 3; k++) {
    p[k] = l[k] = 0.0;
  }
  for(size_t g = 0; g < state.size(); g++) {
    const auto& t = templates[state.template_index[g]];
    const auto& x = state.position[g];
    const auto& v = state.velocity[g];
    const auto& q = state.orientation[g];
    double mv[3], spin[3], h[3];
    for(int k = 0; k < 3; k++) {
      mv[k] = t.mass * v[k];
      spin[k] = t.inertia[k] * state.angular_velocity[g][k];
    }
    /* R h, h in the principal axes */
    const double e[3] = {q[0], q[1], q[2]}, w = q[3];
    const double c1[3] = {e[1] * spin[2] - e[2] * spin[1],
                          e[2] * spin[0] - e[0] * spin[2],
                          e[0] * spin[1] - e[1] * spin[0]};
    const double t2[3] = {2 * c1[0], 2 * c1[1], 2 * c1[2]};
    const double c2[3] = {e[1] * t2[2] - e[2] * t2[1],
                          e[2] * t2[0] - e[0] * t2[2],
                          e[0] * t2[1] - e[1] * t2[0]};
    for(int k = 0; k < 3; k++) {
      h[k] = spin[k] + w * t2[k] + c2[k];
    }
    p[0] += mv[0];
    p[1] += mv[1];
    p[2] += mv[2];
    l[0] += x[1] * mv[2] - x[2] * mv[1] + h[0];
    l[1] += x[2] * mv[0] - x[0] * mv[2] + h[1];
    l[2] += x[0] * mv[1] - x[1] * mv[0] + h[2];
  }
}

TEST(Dem, CollisionKeepsMomentum) {
  /* two spinning spheres of different sizes and materials meet off center,
  with friction and rolling resistance */
  std::vector<shift::core::dem::Template> templates = {
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0),
    shift::core::dem::Template::SingleSphere(1.6f * kRadius, kDensity, 1)};
  const std::vector<shift::core::dem::Material> materials = {
    Glass(0.4f, 0.6f, 0.05f), Glass(0.7f, 0.3f, 0.02f)};
  shift::core::dem::State state;
  state.AddGrain(0, {-0.02, 0.003, 0.001});
  state.AddGrain(1, {0.02, -0.002, 0.0});
  state.velocity[0] = {1.0f, 0.0f, 0.1f};
  state.velocity[1] = {-0.5f, 0.1f, 0.0f};
  state.angular_velocity[0] = {10.0f, -40.0f, 25.0f};
  state.angular_velocity[1] = {-5.0f, 15.0f, 30.0f};
  shift::core::dem::CpuSolver solver(NoGravity(), materials, templates, state);
  double p0[3], l0[3], p1[3], l1[3];
  Momentum(state, templates, p0, l0);
  const float dt =
    static_cast<float>(shift::core::dem::StableStep(templates, materials, 2.0));
  solver.Steps(dt, static_cast<long>(0.04 / dt));
  solver.GetState(state);
  Momentum(state, templates, p1, l1);
  /* they met and parted */
  EXPECT_LT(state.velocity[0][0], 0.9f);
  const double scale_p = templates[0].mass * 1.0;
  const double scale_l = scale_p * 0.02;
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(p1[k], p0[k], 1e-5 * scale_p) << k;
    EXPECT_NEAR(l1[k], l0[k], 1e-4 * scale_l) << k;
  }
}

/* a clump of three spheres in a line spinning about an axis near its
intermediate one, with no forces, keeps its angular momentum in the fixed
frame and its energy, while it tumbles; without the gyroscopic term it would
not */
TEST(Dem, FreeClumpKeepsAngularMomentum) {
  shift::core::dem::Template clump;
  const float r = kRadius;
  for(int k = -1; k <= 1; k++) {
    shift::core::dem::Sphere s;
    s.center = {1.5f * r * k, 0.0f, 0.0f};
    s.radius = (k == 0 ? 1.2f : 1.0f) * r;
    clump.spheres.push_back(s);
  }
  clump.mass = 0.004f;
  clump.inertia = {2.0e-8f, 6.0e-8f, 7.0e-8f};
  const std::vector<shift::core::dem::Template> templates = {clump};
  const std::vector<shift::core::dem::Material> materials = {Glass()};
  shift::core::dem::State state;
  state.AddGrain(0, {0.0, 0.0, 0.0});
  state.angular_velocity[0] = {0.5f, 30.0f, 0.5f};
  shift::core::dem::CpuSolver solver(NoGravity(), materials, templates, state);
  double p0[3], l0[3];
  Momentum(state, templates, p0, l0);
  const float dt = 1e-5f;
  double smallest_wy = 30.0;
  for(int k = 0; k < 200; k++) {
    solver.Steps(dt, 500);
    solver.GetState(state);
    smallest_wy =
      std::min(smallest_wy, double(state.angular_velocity[0][1]));
  }
  double p1[3], l1[3];
  Momentum(state, templates, p1, l1);
  const double size = std::sqrt(l0[0] * l0[0] + l0[1] * l0[1] + l0[2] * l0[2]);
  double energy[2] = {0.0, 0.0};
  shift::core::dem::State start;
  start.AddGrain(0, {0.0, 0.0, 0.0});
  start.angular_velocity[0] = {0.5f, 30.0f, 0.5f};
  for(int k = 0; k < 3; k++) {
    energy[0] += 0.5 * clump.inertia[k] * std::pow(start.angular_velocity[0][k], 2);
    energy[1] += 0.5 * clump.inertia[k] * std::pow(state.angular_velocity[0][k], 2);
  }
  std::printf("angular momentum %.4g -> (%.4g, %.4g, %.4g); energy %.4g -> "
              "%.4g; w_y fell to %.2f\n",
              size, l1[0], l1[1], l1[2], energy[0], energy[1], smallest_wy);
  /* it tumbles: rotation about the intermediate axis is unstable */
  EXPECT_LT(smallest_wy, 0.0);
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(l1[k], l0[k], 0.01 * size) << k;
  }
  EXPECT_NEAR(energy[1], energy[0], 0.01 * energy[0]);
}

/* a sphere resting on a floor 100 km from the origin rests as one at the
origin: the anchors keep the overlap's precision */
TEST(Dem, FarFromOriginAsNear) {
  const auto sphere =
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0);
  const std::vector<shift::core::dem::Material> materials = {Glass(0.3f)};
  double overlap[2];
  const double far = 1.0e5;
  for(int k = 0; k < 2; k++) {
    const double o = k * far;
    shift::core::dem::Settings settings;
    shift::core::dem::Wall floor = Floor();
    floor.offset = -(o - 0.1); // z = o - 0.1
    settings.walls = {floor};
    shift::core::dem::State state;
    state.AddGrain(0, {o + 0.3, o - 0.2, o - 0.1 - kRadius});
    shift::core::dem::CpuSolver solver(settings, materials, {sphere}, state);
    const float dt = static_cast<float>(
      shift::core::dem::StableStep({sphere}, materials, 0.1));
    solver.Steps(dt, static_cast<long>(0.1 / dt));
    solver.GetState(state);
    overlap[k] = kRadius + (state.position[0][2] - (o - 0.1));
  }
  std::printf("overlap at the origin %.6g m, 100 km away %.6g m\n", overlap[0],
              overlap[1]);
  EXPECT_NEAR(overlap[1], overlap[0], 1e-3 * overlap[0]);
}

/* grains of two shapes, spheres and three-sphere clumps, poured into a box
of walls: n per side, spaced apart, with small random velocities */
static void Box(int n, std::vector<shift::core::dem::Template>& templates,
                std::vector<shift::core::dem::Material>& materials,
                shift::core::dem::Settings& settings,
                shift::core::dem::State& state) {
  templates = {shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0)};
  shift::core::dem::Template clump;
  for(int k = -1; k <= 1; k++) {
    shift::core::dem::Sphere s;
    s.center = {0.6f * kRadius * k, 0.0f, 0.0f};
    s.radius = 0.8f * kRadius;
    s.material = 1;
    clump.spheres.push_back(s);
  }
  clump.mass = 0.0015f;
  clump.inertia = {1.2e-8f, 2.6e-8f, 2.6e-8f};
  templates.push_back(clump);
  materials = {Glass(0.5f, 0.5f, 0.02f), Glass(0.3f, 0.7f, 0.05f)};
  settings = shift::core::dem::Settings();
  const double half = 0.5 * n * 3.0 * kRadius;
  settings.walls = {Floor()};
  for(int axis = 0; axis < 2; axis++) {
    for(int side = -1; side <= 1; side += 2) {
      shift::core::dem::Wall w;
      w.normal = {0.0, 0.0, 0.0};
      w.offset = -half - 0.5 * kRadius; // the solid is outside the box
      w.normal[axis] = -side;
      settings.walls.push_back(w);
    }
  }
  uint32_t seed = 12345u;
  auto random = [&seed]() {
    seed = seed * 1664525u + 1013904223u;
    return (seed >> 8) / 16777216.0f - 0.5f;
  };
  int count = 0;
  for(int k = 0; k < n; k++) {
    for(int j = 0; j < n; j++) {
      for(int i = 0; i < n; i++) {
        const double s = 3.0 * kRadius;
        state.AddGrain(count % 2,
                       {-half + (i + 0.5) * s, -half + (j + 0.5) * s,
                        -(k + 0.6) * s});
        state.velocity.back() = {0.1f * random(), 0.1f * random(),
                                 0.1f * random()};
        state.angular_velocity.back() = {10.0f * random(), 10.0f * random(),
                                         10.0f * random()};
        count++;
      }
    }
  }
}

/* the grains in the box fall onto the floor and stay inside the walls,
slowing down; the spheres on the floor still roll after 0.3 s */
TEST(Dem, GrainsSettleInBox) {
  std::vector<shift::core::dem::Template> templates;
  std::vector<shift::core::dem::Material> materials;
  shift::core::dem::Settings settings;
  shift::core::dem::State state;
  Box(5, templates, materials, settings, state);
  shift::core::dem::CpuSolver solver(settings, materials, templates, state);
  const float dt =
    static_cast<float>(shift::core::dem::StableStep(templates, materials, 1.0));
  solver.Steps(dt, static_cast<long>(0.3 / dt));
  solver.GetState(state);
  const auto stats = solver.GetStatistics();
  const double half = 0.5 * 5 * 3.0 * kRadius;
  double mean = 0.0, lowest = -1.0, highest = 0.0, widest = 0.0;
  for(size_t g = 0; g < state.size(); g++) {
    const auto& v = state.velocity[g];
    mean += std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) / state.size();
    lowest = std::max(lowest, state.position[g][2]);
    highest = std::min(highest, state.position[g][2]);
    widest = std::max(widest, std::max(std::abs(state.position[g][0]),
                                       std::abs(state.position[g][1])));
  }
  std::printf("%llu steps, %llu searches, at most %u neighbors; mean speed "
              "%.4f m/s; grains from z = %.4f to %.4f m, |x|, |y| to %.4f "
              "(walls at %.4f)\n",
              (unsigned long long)stats.steps,
              (unsigned long long)stats.searches, stats.most_neighbors, mean,
              highest, lowest, widest, half + 0.5 * kRadius);
  EXPECT_LT(mean, 0.05);
  EXPECT_LT(lowest, 0.0);
  EXPECT_LT(widest, half + 0.5 * kRadius);
  /* fallen: the pile is lower than it started (5 layers of 3 r) */
  EXPECT_GT(highest, -5 * 3.0 * kRadius + 2 * kRadius);
  EXPECT_GT(stats.searches, 1u);
  EXPECT_LT(stats.searches, stats.steps);
}

/* the GPU solver does the CPU's arithmetic: after 1000 steps of grains
falling in the box, the grains are where the CPU puts them */
TEST(Dem, GpuMatchesCpu) {
  REQUIRE_DEVICE(device);
  std::vector<shift::core::dem::Template> templates;
  std::vector<shift::core::dem::Material> materials;
  shift::core::dem::Settings settings;
  shift::core::dem::State state;
  Box(4, templates, materials, settings, state);
  const float dt =
    static_cast<float>(shift::core::dem::StableStep(templates, materials, 1.0));
  shift::core::dem::CpuSolver cpu(settings, materials, templates, state);
  shift::core::dem::GpuSolver gpu(*device, settings, materials, templates,
                                  state);
  cpu.Steps(dt, 1000);
  gpu.Steps(dt, 1000);
  shift::core::dem::State a, b;
  cpu.GetState(a);
  gpu.GetState(b);
  double moved = 0.0, difference = 0.0;
  for(size_t g = 0; g < state.size(); g++) {
    for(int k = 0; k < 3; k++) {
      moved = std::max(moved, std::abs(a.position[g][k] - state.position[g][k]));
      difference = std::max(difference, std::abs(a.position[g][k] - b.position[g][k]));
    }
  }
  const auto sa = cpu.GetStatistics(), sb = gpu.GetStatistics();
  std::printf("moved %.3g m, CPU and GPU differ by %.3g m; searches %llu and "
              "%llu\n",
              moved, difference, (unsigned long long)sa.searches,
              (unsigned long long)sb.searches);
  EXPECT_GT(moved, 1e-3);
  EXPECT_LT(difference, 1e-3 * moved);
  EXPECT_EQ(sa.searches, sb.searches);
}

/* two runs on the GPU give the same bits */
TEST(Dem, GpuRepeats) {
  REQUIRE_DEVICE(device);
  std::vector<shift::core::dem::Template> templates;
  std::vector<shift::core::dem::Material> materials;
  shift::core::dem::Settings settings;
  shift::core::dem::State state;
  Box(6, templates, materials, settings, state);
  const float dt =
    static_cast<float>(shift::core::dem::StableStep(templates, materials, 1.0));
  shift::core::dem::State runs[2];
  for(auto& run : runs) {
    shift::core::dem::GpuSolver gpu(*device, settings, materials, templates,
                                    state);
    gpu.Steps(dt, 2000);
    gpu.GetState(run);
  }
  ASSERT_EQ(runs[0].size(), runs[1].size());
  EXPECT_EQ(std::memcmp(runs[0].position.data(), runs[1].position.data(),
                        runs[0].size() * sizeof(runs[0].position[0])),
            0);
  EXPECT_EQ(std::memcmp(runs[0].orientation.data(),
                        runs[1].orientation.data(),
                        runs[0].size() * sizeof(runs[0].orientation[0])),
            0);
}

/* a square in the plane z = 0 of a body from -half to half about center,
four triangles meeting at the center */
static shift::core::dem::Body Plate(std::array<float, 2> center, float half) {
  shift::core::dem::Body b;
  const float x = center[0], y = center[1];
  b.vertices = {{x, y, 0.0f},
                {x - half, y - half, 0.0f},
                {x + half, y - half, 0.0f},
                {x + half, y + half, 0.0f},
                {x - half, y + half, 0.0f}};
  b.triangles = {{0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 1}};
  return b;
}

/* a sphere on the vertex where the plate's triangles meet rests at Hertz's
overlap, touching it once; the body's impulse is the weight over the time,
and its moment about the body's origin, 0.02 m from the sphere, the
weight's */
TEST(Dem, RestsOnBodyAsOnWall) {
  const auto sphere =
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0);
  const std::vector<shift::core::dem::Material> materials = {Glass(0.3f)};
  shift::core::dem::Settings settings;
  shift::core::dem::Body plate = Plate({-0.02f, 0.0f}, 0.03f);
  plate.pose.t = {0.02, 0.0, 0.0};
  shift::core::dem::State state;
  state.AddGrain(0, {0.0, 0.0, -kRadius});
  shift::core::dem::CpuSolver solver(settings, materials, {sphere}, state,
                                     {plate});
  const float dt =
    static_cast<float>(shift::core::dem::StableStep({sphere}, materials, 0.1));
  solver.Steps(dt, static_cast<long>(0.2 / dt));
  solver.ClearImpulses();
  const long n = static_cast<long>(0.05 / dt);
  solver.Steps(dt, n);
  solver.GetState(state);
  std::vector<std::array<double, 6>> impulses;
  solver.GetImpulses(impulses);
  const double nu = materials[0].poisson_ratio;
  const double e = materials[0].youngs_modulus / (2.0 * (1.0 - nu * nu));
  const double weight = sphere.mass * 9.81;
  const double expected =
    std::pow(weight / (4.0 / 3.0 * e * std::sqrt(double(kRadius))), 2.0 / 3.0);
  const double overlap = kRadius + state.position[0][2];
  const double duration = n * double(dt);
  std::printf("overlap %.4g m, Hertz %.4g m; impulse %.4g N s, weight over "
              "the time %.4g N s, moment %.4g N m s; at most %u triangles\n",
              overlap, expected, impulses[0][2], weight * duration,
              impulses[0][4], solver.GetStatistics().most_triangles);
  EXPECT_NEAR(overlap, expected, 0.01 * expected);
  ASSERT_EQ(impulses.size(), 1u);
  EXPECT_NEAR(impulses[0][2], weight * duration, 1e-3 * weight * duration);
  EXPECT_NEAR(impulses[0][0], 0.0, 1e-6 * weight * duration);
  EXPECT_NEAR(impulses[0][1], 0.0, 1e-6 * weight * duration);
  /* the force (0, 0, w) at (-0.02, 0, 0) from the body's origin */
  EXPECT_NEAR(impulses[0][4], 0.02 * weight * duration,
              2e-3 * 0.02 * weight * duration);
  EXPECT_NEAR(impulses[0][3], 0.0, 1e-4 * 0.02 * weight * duration);
}

/* a plate normal to x moving at speed v hits a sphere at rest, with gravity
off: the body moves as given, as if of infinite mass, so that the sphere
leaves at (1 + e) v, and the body's impulse is the sphere's momentum,
opposite */
TEST(Dem, MovingBodyPushesSphere) {
  const auto sphere =
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0);
  const float restitution = 0.5f;
  const std::vector<shift::core::dem::Material> materials = {
    Glass(restitution)};
  const shift::core::dem::Settings settings = NoGravity();
  /* the plate in the body's y, z plane: its z = 0 plane turned about y */
  shift::core::dem::Body plate = Plate({0.0f, 0.0f}, 0.02f);
  plate.pose.R = {0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0};
  const double speed = 0.5, t1 = 0.1;
  plate.pose.t = {-kRadius - 1e-3, 0.0, 0.0};
  plate.pose.v = {speed, 0.0, 0.0};
  shift::core::sph::BodyPose end = plate.pose;
  end.t[0] += speed * t1;
  shift::core::dem::State state;
  state.AddGrain(0, {0.0, 0.0, 0.0});
  shift::core::dem::CpuSolver solver(settings, materials, {sphere}, state,
                                     {plate});
  solver.SetBodyPaths({plate.pose}, {end}, 0.0, t1);
  solver.ClearImpulses();
  const float dt = static_cast<float>(
    shift::core::dem::StableStep({sphere}, materials, speed, 0.05));
  solver.Steps(dt, static_cast<long>(0.02 / dt));
  solver.GetState(state);
  std::vector<std::array<double, 6>> impulses;
  solver.GetImpulses(impulses);
  const double momentum = sphere.mass * state.velocity[0][0];
  std::printf("sphere leaves at %.4f m/s, (1 + e) v = %.4f; impulse %.5g "
              "N s, momentum %.5g N s\n",
              state.velocity[0][0], (1.0 + restitution) * speed,
              impulses[0][0], momentum);
  EXPECT_NEAR(state.velocity[0][0], (1.0 + restitution) * speed, 0.03 * speed);
  EXPECT_NEAR(impulses[0][0], -momentum, 1e-4 * momentum);
  EXPECT_LT(std::abs(state.velocity[0][1]) + std::abs(state.velocity[0][2]),
            1e-6f);
}

/* the box of grains, and a plate tilted 30 degrees about x, turning about z
and moving down into them along a path of steps of dt */
static void BoxAndPlate(int n,
                        std::vector<shift::core::dem::Template>& templates,
                        std::vector<shift::core::dem::Material>& materials,
                        shift::core::dem::Settings& settings,
                        shift::core::dem::State& state,
                        shift::core::dem::Body& plate,
                        shift::core::sph::BodyPose& end, long steps,
                        float& dt) {
  Box(n, templates, materials, settings, state);
  dt =
    static_cast<float>(shift::core::dem::StableStep(templates, materials, 1.0));
  const double duration = steps * double(dt);
  /* small enough that its corners keep a grain's width from the walls */
  plate = Plate({0.0f, 0.0f}, 0.012f);
  plate.material = 1;
  const double a = 30.0 * kPi / 180.0, w = 20.0, v = 1.0;
  const double c = std::cos(a), s = std::sin(a);
  /* columns: R = Rx(a) */
  plate.pose.R = {1.0, 0.0, 0.0, 0.0, c, s, 0.0, -s, c};
  plate.pose.t = {0.0, 0.0, -0.072};
  plate.pose.v = {0.0, 0.0, v};
  plate.pose.w = {0.0, 0.0, w};
  end = plate.pose;
  end.t[2] += v * duration;
  /* R_end = Rz(w T) R */
  const double cz = std::cos(w * duration), sz = std::sin(w * duration);
  for(int col = 0; col < 3; col++) {
    const double x = plate.pose.R[3 * col], y = plate.pose.R[3 * col + 1];
    end.R[3 * col] = cz * x - sz * y;
    end.R[3 * col + 1] = sz * x + cz * y;
  }
}

/* the GPU does the CPU's arithmetic with a body too: after the plate has
moved 1.6 cm, into the top grains (later the hits make the runs part as any
two runs of grains do, by their last bits), the grains and the impulse on the
plate are the CPU's */
TEST(Dem, GpuMatchesCpuWithBody) {
  REQUIRE_DEVICE(device);
  std::vector<shift::core::dem::Template> templates;
  std::vector<shift::core::dem::Material> materials;
  shift::core::dem::Settings settings;
  shift::core::dem::State state;
  shift::core::dem::Body plate;
  shift::core::sph::BodyPose end;
  const long steps = 600;
  float dt;
  BoxAndPlate(4, templates, materials, settings, state, plate, end, steps, dt);
  shift::core::dem::CpuSolver cpu(settings, materials, templates, state,
                                  {plate});
  shift::core::dem::GpuSolver gpu(*device, settings, materials, templates,
                                  state, {plate});
  for(shift::core::dem::Solver* solver :
      {static_cast<shift::core::dem::Solver*>(&cpu),
       static_cast<shift::core::dem::Solver*>(&gpu)}) {
    solver->SetBodyPaths({plate.pose}, {end}, 0.0, steps * double(dt));
    solver->ClearImpulses();
    solver->Steps(dt, steps);
  }
  shift::core::dem::State a, b;
  cpu.GetState(a);
  gpu.GetState(b);
  double moved = 0.0, difference = 0.0;
  for(size_t g = 0; g < state.size(); g++) {
    for(int k = 0; k < 3; k++) {
      moved = std::max(moved, std::abs(a.position[g][k] - state.position[g][k]));
      difference =
        std::max(difference, std::abs(a.position[g][k] - b.position[g][k]));
    }
  }
  std::vector<std::array<double, 6>> ia, ib;
  cpu.GetImpulses(ia);
  gpu.GetImpulses(ib);
  const auto sa = cpu.GetStatistics(), sb = gpu.GetStatistics();
  std::printf("moved %.3g m, CPU and GPU differ by %.3g m; impulse on the "
              "plate %.4g %.4g %.4g N s (GPU %.4g %.4g %.4g), moment %.4g "
              "%.4g %.4g N m s (GPU %.4g %.4g %.4g); at most %u and %u "
              "triangles; searches %llu and %llu\n",
              moved, difference, ia[0][0], ia[0][1], ia[0][2], ib[0][0],
              ib[0][1], ib[0][2], ia[0][3], ia[0][4], ia[0][5], ib[0][3],
              ib[0][4], ib[0][5], sa.most_triangles, sb.most_triangles,
              (unsigned long long)sa.searches,
              (unsigned long long)sb.searches);
  EXPECT_GT(moved, 1e-3);
  EXPECT_LT(difference, 1e-3 * moved);
  EXPECT_GT(sa.most_triangles, 0u);
  EXPECT_EQ(sa.most_triangles, sb.most_triangles);
  EXPECT_EQ(sa.searches, sb.searches);
  /* the grains push the plate back up */
  EXPECT_LT(ia[0][2], 0.0);
  double size = 0.0;
  for(int c = 0; c < 3; c++) {
    size = std::max(size, std::abs(ia[0][c]));
  }
  for(int c = 0; c < 3; c++) {
    EXPECT_NEAR(ia[0][c], ib[0][c], 1e-3 * size);
    EXPECT_NEAR(ia[0][3 + c], ib[0][3 + c], 1e-3 * size * 0.02);
  }
}

/* too many neighbors for max_neighbors is an error, not a lost contact */
TEST(Dem, TooManyNeighborsThrows) {
  std::vector<shift::core::dem::Template> templates;
  std::vector<shift::core::dem::Material> materials;
  shift::core::dem::Settings settings;
  shift::core::dem::State state;
  Box(3, templates, materials, settings, state);
  /* pack the grains: every sphere within reach of many */
  for(size_t g = 0; g < state.size(); g++) {
    for(int k = 0; k < 3; k++) {
      state.position[g][k] *= 0.5;
    }
  }
  settings.max_neighbors = 2;
  shift::core::dem::CpuSolver solver(settings, materials, templates, state);
  EXPECT_THROW(solver.Step(1e-6f), shift::core::Exception);
}

/* the speed of the GPU solver on n^3 grains (half of them three-sphere
clumps) falling in the box: a benchmark (meson benchmark dem_speed), not run
with the tests */
TEST(DemSpeed, Gpu) {
  REQUIRE_DEVICE(device);
  for(int n : {32, 64, 96}) {
    std::vector<shift::core::dem::Template> templates;
    std::vector<shift::core::dem::Material> materials;
    shift::core::dem::Settings settings;
    shift::core::dem::State state;
    Box(n, templates, materials, settings, state);
    const float dt = static_cast<float>(
      shift::core::dem::StableStep(templates, materials, 1.0));
    shift::core::dem::GpuSolver gpu(*device, settings, materials, templates,
                                    state);
    gpu.Steps(dt, 256); // warm up
    const long steps = 2048;
    const auto start = std::chrono::steady_clock::now();
    gpu.Steps(dt, steps);
    const double seconds = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - start)
                             .count();
    size_t spheres = 0;
    for(uint32_t t : state.template_index) {
      spheres += templates[t].spheres.size();
    }
    const auto stats = gpu.GetStatistics();
    std::printf("%zu grains, %zu spheres: %.3f ms a step, %.3g sphere steps "
                "a second; %llu searches in %llu steps\n",
                state.size(), spheres, 1e3 * seconds / steps,
                spheres * steps / seconds, (unsigned long long)stats.searches,
                (unsigned long long)stats.steps);
  }
}

/* the non-smooth method (NSC, see shift::core::dem::Nsc), on the CPU */

/* settings for NSC: the solver, of iterations */
static shift::core::dem::Settings Nsc(
  shift::core::dem::Settings settings,
  shift::core::dem::NscSolver solver = shift::core::dem::NscSolver::kApgd,
  uint32_t iterations = 100) {
  settings.method = shift::core::dem::ContactMethod::kNsc;
  settings.nsc.solver = solver;
  settings.nsc.iterations = iterations;
  return settings;
}

/* the speed after a sphere at speed hits the floor, gravity off, in steps
of 1e-4 s; 0 if it does not leave the floor */
static double NscRebound(float restitution, double speed) {
  const auto sphere =
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0);
  const std::vector<shift::core::dem::Material> materials = {
    Glass(restitution)};
  shift::core::dem::Settings settings = Nsc(NoGravity());
  settings.walls = {Floor()};
  shift::core::dem::State state;
  state.AddGrain(0, {0.0, 0.0, -kRadius - 1e-3});
  state.velocity[0] = {0.0f, 0.0f, static_cast<float>(speed)};
  shift::core::dem::CpuSolver solver(settings, materials, {sphere}, state);
  for(int k = 0; k < 2000; k++) {
    solver.Step(1e-4f);
  }
  solver.GetState(state);
  return state.position[0][2] < -kRadius - 1e-3 ? -state.velocity[0][2] : 0.0;
}

/* Newton's restitution: the rebound is the restitution of the approach,
above min_bounce_speed (0.15 m/s), to the residual of APGD (0.03 %);
slower, the sphere stays */
TEST(DemNsc, ReboundsWithRestitution) {
  for(float e : {0.2f, 0.5f, 0.9f}) {
    for(double speed : {0.5, 1.0}) {
      const double ratio = NscRebound(e, speed) / speed;
      std::printf("restitution %.2f at %.1f m/s: rebounds at %.4f\n", e, speed,
                  ratio);
      EXPECT_NEAR(ratio, e, 1e-3);
    }
  }
  EXPECT_EQ(NscRebound(0.9f, 0.1), 0.0);
}

/* a column of ten spheres on the floor, in steps of 5e-4 s over 0.2 s,
the iterations of solver: how far the top is from where rigid spheres rest,
and the fastest sphere at the end */
static void NscStack(shift::core::dem::NscSolver solver_type,
                     uint32_t iterations, double& top, double& fastest,
                     bool mass_splitting = true) {
  const auto sphere =
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0);
  const std::vector<shift::core::dem::Material> materials = {Glass(0.3f)};
  shift::core::dem::Settings settings = Nsc({}, solver_type, iterations);
  settings.nsc.mass_splitting = mass_splitting;
  settings.walls = {Floor()};
  shift::core::dem::State state;
  const int n = 10;
  for(int k = 0; k < n; k++) {
    state.AddGrain(0, {0.0, 0.0, -(2 * k + 1) * kRadius - 2e-4 * k});
  }
  shift::core::dem::CpuSolver solver(settings, materials, {sphere}, state);
  solver.Steps(5e-4f, 400);
  solver.GetState(state);
  top = std::abs(state.position[n - 1][2] + (2 * n - 1) * kRadius);
  fastest = 0.0;
  for(int k = 0; k < n; k++) {
    fastest = std::max(fastest, double(std::abs(state.velocity[k][2])));
  }
}

/* a column of ten spheres rests on the floor without overlap: the contacts
are rigid, once the iterations have carried the weight of the column down
it. Gauss-Seidel takes iterations in proportion to the square of its
height, 400; Jacobi with mass splitting 400 too, without it 2000; APGD
fewer, 100. With 100 the column of Gauss-Seidel or Jacobi stands within
micrometers but jitters at millimeters a second (Jacobi without mass
splitting at tens) */
TEST(DemNsc, StackRestsWithoutOverlap) {
  const struct {
    shift::core::dem::NscSolver solver;
    const char* name;
    uint32_t iterations;
    bool mass_splitting;
  } cases[] = {
    {shift::core::dem::NscSolver::kApgd, "APGD", 100, true},
    {shift::core::dem::NscSolver::kGaussSeidel, "Gauss-Seidel", 400, true},
    {shift::core::dem::NscSolver::kJacobi, "Jacobi", 400, true},
    {shift::core::dem::NscSolver::kJacobi, "Jacobi without mass splitting",
     2000, false}};
  for(const auto& c : cases) {
    double top, fastest, top_100, fastest_100;
    NscStack(c.solver, c.iterations, top, fastest, c.mass_splitting);
    NscStack(c.solver, 100, top_100, fastest_100, c.mass_splitting);
    std::printf("%s, %u iterations: ten spheres stacked: the top %.3g mm "
                "from where rigid spheres rest, at most %.3g mm/s; with 100 "
                "iterations %.3g mm and %.3g mm/s\n",
                c.name, c.iterations, 1e3 * top, 1e3 * fastest, 1e3 * top_100,
                1e3 * fastest_100);
    EXPECT_LT(top, 1e-6) << c.name;
    /* APGD keeps its best iterate, 0.02 mm/s */
    EXPECT_LT(fastest, 5e-5) << c.name;
  }
}

/* gravity tilted by angle about y over a level floor, in steps of 5e-4 s:
the state after duration */
static shift::core::dem::State NscSlope(double angle, float friction,
                                        float rolling, double duration) {
  const auto sphere =
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0);
  const std::vector<shift::core::dem::Material> materials = {
    Glass(0.3f, friction, rolling)};
  shift::core::dem::Settings settings;
  settings.gravity = {static_cast<float>(9.81 * std::sin(angle)), 0.0f,
                      static_cast<float>(9.81 * std::cos(angle))};
  settings.walls = {Floor()};
  settings = Nsc(settings);
  shift::core::dem::State state;
  state.AddGrain(0, {0.0, 0.0, -kRadius});
  shift::core::dem::CpuSolver solver(settings, materials, {sphere}, state);
  solver.Steps(5e-4f, static_cast<long>(duration / 5e-4 + 0.5));
  solver.GetState(state);
  return state;
}

/* rolls without slip at 5/7 g sin; held by rolling friction on a gentle
slope */
TEST(DemNsc, RollsDownSlope) {
  const double angle = 10.0 * kPi / 180.0, duration = 0.2;
  const shift::core::dem::State state = NscSlope(angle, 0.5f, 0.0f, duration);
  const double expected = 5.0 / 7.0 * 9.81 * std::sin(angle) * duration;
  const double v = state.velocity[0][0];
  const double rim = -state.angular_velocity[0][1] * kRadius;
  std::printf("speed %.4f m/s (5/7 g sin: %.4f), rim speed %.4f\n", v,
              expected, rim);
  EXPECT_NEAR(v, expected, 1e-3 * expected);
  EXPECT_NEAR(rim, v, 1e-3 * v);
  const shift::core::dem::State held =
    NscSlope(3.0 * kPi / 180.0, 0.5f, 0.1f, 0.3);
  const shift::core::dem::State rolls =
    NscSlope(10.0 * kPi / 180.0, 0.5f, 0.1f, 0.3);
  std::printf("rolling friction 0.1: 3 degrees: %.5f m/s, 10 degrees: "
              "%.4f m/s\n",
              held.velocity[0][0], rolls.velocity[0][0]);
  EXPECT_LT(std::abs(held.velocity[0][0]), 1e-4f);
  EXPECT_GT(rolls.velocity[0][0], 0.1f);
}

/* on a slope steeper than the friction holds (tan > 7/2 mu), the sphere
slides and spins up, its speed g (sin - mu cos) t; Anitescu's relaxation
lifts a sliding contact off the floor by h mu |u_t| a step, a known
artifact of the method, so the sphere hops along it. SMC, for comparison,
slides on the floor */
TEST(DemNsc, SlidesWithCoulombFriction) {
  const double angle = 30.0 * kPi / 180.0, mu = 0.1, duration = 0.1;
  const double expected =
    9.81 * (std::sin(angle) - mu * std::cos(angle)) * duration;
  const shift::core::dem::State nsc =
    NscSlope(angle, float(mu), 0.0f, duration);
  const auto sphere =
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0);
  const std::vector<shift::core::dem::Material> materials = {
    Glass(0.3f, float(mu), 0.0f)};
  shift::core::dem::Settings settings;
  settings.gravity = {static_cast<float>(9.81 * std::sin(angle)), 0.0f,
                      static_cast<float>(9.81 * std::cos(angle))};
  settings.walls = {Floor()};
  shift::core::dem::State smc;
  smc.AddGrain(0, {0.0, 0.0, -kRadius});
  shift::core::dem::CpuSolver solver(settings, materials, {sphere}, smc);
  const float dt =
    static_cast<float>(shift::core::dem::StableStep({sphere}, materials, 0.5));
  solver.Steps(dt, static_cast<long>(duration / dt + 0.5));
  solver.GetState(smc);
  std::printf("sliding at %.4f m/s expected: NSC %.4f m/s, %.3g mm off the "
              "floor; SMC %.4f m/s, %.3g mm off it\n",
              expected, nsc.velocity[0][0],
              -1e3 * (nsc.position[0][2] + kRadius), smc.velocity[0][0],
              -1e3 * (smc.position[0][2] + kRadius));
  EXPECT_NEAR(smc.velocity[0][0], expected, 0.02 * expected);
  EXPECT_NEAR(nsc.velocity[0][0], expected, 0.02 * expected);
}

/* two spinning spheres meet: the impulses of a pair are equal and opposite
at one point, so momentum is kept exactly */
TEST(DemNsc, CollisionKeepsMomentum) {
  std::vector<shift::core::dem::Template> templates = {
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0),
    shift::core::dem::Template::SingleSphere(1.6f * kRadius, kDensity, 1)};
  const std::vector<shift::core::dem::Material> materials = {
    Glass(0.4f, 0.6f, 0.05f), Glass(0.7f, 0.3f, 0.02f)};
  shift::core::dem::State state;
  state.AddGrain(0, {-0.02, 0.003, 0.001});
  state.AddGrain(1, {0.02, -0.002, 0.0});
  state.velocity[0] = {1.0f, 0.0f, 0.1f};
  state.velocity[1] = {-0.5f, 0.1f, 0.0f};
  state.angular_velocity[0] = {10.0f, -40.0f, 25.0f};
  state.angular_velocity[1] = {-5.0f, 15.0f, 30.0f};
  shift::core::dem::Settings settings = Nsc(NoGravity());
  settings.margin = 0.003f; // 1.5 m/s closing over a step of 1e-3 s
  shift::core::dem::CpuSolver solver(settings, materials, templates, state);
  double p0[3], l0[3], p1[3], l1[3];
  Momentum(state, templates, p0, l0);
  solver.Steps(1e-3f, 40);
  solver.GetState(state);
  Momentum(state, templates, p1, l1);
  EXPECT_LT(state.velocity[0][0], 0.9f);
  const double scale_p = templates[0].mass * 1.0;
  const double scale_l = scale_p * 0.02;
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(p1[k], p0[k], 1e-5 * scale_p) << k;
    EXPECT_NEAR(l1[k], l0[k], 1e-4 * scale_l) << k;
  }
}

/* the grains of the box, 4 by 4 by 4, settle as with SMC, in steps of 0.4 ms
longer, with 30 iterations of APGD: inside the walls and slowed */
TEST(DemNsc, GrainsSettleInBox) {
  std::vector<shift::core::dem::Template> templates;
  std::vector<shift::core::dem::Material> materials;
  shift::core::dem::Settings settings;
  shift::core::dem::State state;
  Box(4, templates, materials, settings, state);
  settings = Nsc(settings, shift::core::dem::NscSolver::kApgd, 30);
  shift::core::dem::CpuSolver solver(settings, materials, templates, state);
  const float h = 4e-4f;
  solver.Steps(h, static_cast<long>(0.3 / h + 0.5));
  solver.GetState(state);
  const auto stats = solver.GetStatistics();
  const double half = 0.5 * 4 * 3.0 * kRadius;
  double mean = 0.0, lowest = -1.0, highest = 0.0, widest = 0.0;
  for(size_t g = 0; g < state.size(); g++) {
    const auto& v = state.velocity[g];
    mean += std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) / state.size();
    lowest = std::max(lowest, state.position[g][2]);
    highest = std::min(highest, state.position[g][2]);
    widest = std::max(widest, std::max(std::abs(state.position[g][0]),
                                       std::abs(state.position[g][1])));
  }
  std::printf("%llu steps, %llu contacts, residual %.3g; mean speed %.4f "
              "m/s; grains from z = %.4f to %.4f m, |x|, |y| to %.4f\n",
              (unsigned long long)stats.steps,
              (unsigned long long)stats.contacts, stats.residual, mean,
              highest, lowest, widest);
  EXPECT_LT(mean, 0.05);
  EXPECT_LT(lowest, 0.0);
  EXPECT_LT(widest, half + 0.5 * kRadius);
  EXPECT_GT(highest, -4 * 3.0 * kRadius + 2 * kRadius);
}

/* the non-smooth method on the GPU */

/* the largest difference of the grains' positions and of their velocities
between two solvers, and how far the first's moved from state */
static void Differences(shift::core::dem::Solver& a,
                        shift::core::dem::Solver& b,
                        const shift::core::dem::State& state, double& moved,
                        double& position, double& velocity) {
  shift::core::dem::State sa, sb;
  a.GetState(sa);
  b.GetState(sb);
  moved = position = velocity = 0.0;
  for(size_t g = 0; g < state.size(); g++) {
    for(int k = 0; k < 3; k++) {
      moved = std::max(moved, std::abs(sa.position[g][k] - state.position[g][k]));
      position =
        std::max(position, std::abs(sa.position[g][k] - sb.position[g][k]));
      velocity = std::max(
        velocity, double(std::abs(sa.velocity[g][k] - sb.velocity[g][k])));
    }
  }
}

/* the GPU's APGD and Jacobi do the CPU's arithmetic in the CPU's order:
grains and clumps falling into the box in steps of 4e-4 s, as the first
grains land, have the same contacts and are where the CPU puts them. They
differ by the multiplications and additions the GPU fuses, which Jacobi
carries over 0.06 s (150 steps) to 1e-7 m. APGD stops at a residual of
about 1e-5 in float, and keeps as its best iterate one of those near it, so
that the runs differ by that much from the first impacts; at 0.044 s a
grain landing at about min_bounce_speed bounces in one run and not in the
other, and the runs part, as any two runs of grains do. APGD is compared
over 0.04 s (100 steps) */
TEST(DemNsc, GpuMatchesCpu) {
  REQUIRE_DEVICE(device);
  for(auto solver_type : {shift::core::dem::NscSolver::kApgd,
                          shift::core::dem::NscSolver::kJacobi}) {
    std::vector<shift::core::dem::Template> templates;
    std::vector<shift::core::dem::Material> materials;
    shift::core::dem::Settings settings;
    shift::core::dem::State state;
    Box(4, templates, materials, settings, state);
    settings = Nsc(settings, solver_type, 30);
    shift::core::dem::CpuSolver cpu(settings, materials, templates, state);
    shift::core::dem::GpuSolver gpu(*device, settings, materials, templates,
                                    state);
    const float h = 4e-4f;
    double moved, position, velocity;
    const int steps =
      solver_type == shift::core::dem::NscSolver::kApgd ? 100 : 150;
    for(int k = 0; k < steps; k++) {
      cpu.Step(h);
      gpu.Step(h);
      if(k == 0 || k + 1 == steps) {
        Differences(cpu, gpu, state, moved, position, velocity);
        const auto sa = cpu.GetStatistics(), sb = gpu.GetStatistics();
        std::printf("%s, step %d: moved %.3g m, CPU and GPU differ by %.3g m "
                    "and %.3g m/s; %llu and %llu contacts, residual %.3g and "
                    "%.3g\n",
                    solver_type == shift::core::dem::NscSolver::kApgd
                      ? "APGD" : "Jacobi",
                    k + 1, moved, position, velocity,
                    (unsigned long long)sa.contacts,
                    (unsigned long long)sb.contacts, sa.residual, sb.residual);
        EXPECT_EQ(sa.contacts, sb.contacts);
      }
    }
    EXPECT_GT(moved, 5e-3);
    EXPECT_LT(position, 1e-3 * moved);
  }
}

/* the grains settle in the box on the GPU as on the CPU (DemNsc.
GrainsSettleInBox), and two runs give the same bits */
TEST(DemNsc, GpuGrainsSettleInBox) {
  REQUIRE_DEVICE(device);
  std::vector<shift::core::dem::Template> templates;
  std::vector<shift::core::dem::Material> materials;
  shift::core::dem::Settings settings;
  shift::core::dem::State state;
  Box(4, templates, materials, settings, state);
  settings = Nsc(settings, shift::core::dem::NscSolver::kApgd, 30);
  const float h = 4e-4f;
  shift::core::dem::State runs[2];
  shift::core::dem::Statistics stats;
  for(auto& run : runs) {
    shift::core::dem::GpuSolver solver(*device, settings, materials,
                                       templates, state);
    solver.Steps(h, static_cast<long>(0.3 / h + 0.5));
    solver.GetState(run);
    stats = solver.GetStatistics();
  }
  const shift::core::dem::State& end = runs[0];
  const double half = 0.5 * 4 * 3.0 * kRadius;
  double mean = 0.0, lowest = -1.0, highest = 0.0, widest = 0.0;
  for(size_t g = 0; g < end.size(); g++) {
    const auto& v = end.velocity[g];
    mean += std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) / end.size();
    lowest = std::max(lowest, end.position[g][2]);
    highest = std::min(highest, end.position[g][2]);
    widest = std::max(widest, std::max(std::abs(end.position[g][0]),
                                       std::abs(end.position[g][1])));
  }
  std::printf("%llu steps, %llu contacts, residual %.3g; mean speed %.4f "
              "m/s; grains from z = %.4f to %.4f m, |x|, |y| to %.4f\n",
              (unsigned long long)stats.steps,
              (unsigned long long)stats.contacts, stats.residual, mean,
              highest, lowest, widest);
  EXPECT_LT(mean, 0.05);
  EXPECT_LT(lowest, 0.0);
  EXPECT_LT(widest, half + 0.5 * kRadius);
  EXPECT_GT(highest, -4 * 3.0 * kRadius + 2 * kRadius);
  EXPECT_GT(stats.contacts, 0u);
  EXPECT_EQ(std::memcmp(runs[0].position.data(), runs[1].position.data(),
                        end.size() * sizeof(end.position[0])),
            0);
  EXPECT_EQ(std::memcmp(runs[0].orientation.data(),
                        runs[1].orientation.data(),
                        end.size() * sizeof(end.orientation[0])),
            0);
}

/* the plate of DemNsc.GpuMatchesCpuWithBody, of mass, moving into the
grains: on the GPU, as on the CPU, it takes part in each step as a free body,
and the grains' impulse on it and the grains are the CPU's until the plate
has gone 1 cm into them */
TEST(DemNsc, GpuMatchesCpuWithBody) {
  REQUIRE_DEVICE(device);
  for(auto solver_type : {shift::core::dem::NscSolver::kApgd,
                          shift::core::dem::NscSolver::kJacobi}) {
    std::vector<shift::core::dem::Template> templates;
    std::vector<shift::core::dem::Material> materials;
    shift::core::dem::Settings settings;
    shift::core::dem::State state;
    shift::core::dem::Body plate;
    shift::core::sph::BodyPose end;
    const long steps = 50;
    float dt;
    BoxAndPlate(4, templates, materials, settings, state, plate, end, steps,
                dt);
    settings = Nsc(settings, solver_type, 30);
    plate.mass = 0.05f;
    plate.inertia = {2e-6f, 2e-6f, 4e-6f, 0.0f, 0.0f, 0.0f};
    /* steps of 2e-4 s, the plate 1 cm in 50 */
    dt = 2e-4f;
    end.t[2] = plate.pose.t[2] + plate.pose.v[2] * steps * double(dt);
    shift::core::dem::CpuSolver cpu(settings, materials, templates, state,
                                    {plate});
    shift::core::dem::GpuSolver gpu(*device, settings, materials, templates,
                                    state, {plate});
    for(shift::core::dem::Solver* solver :
        {static_cast<shift::core::dem::Solver*>(&cpu),
         static_cast<shift::core::dem::Solver*>(&gpu)}) {
      solver->SetBodyPaths({plate.pose}, {end}, 0.0, steps * double(dt));
      solver->ClearImpulses();
      solver->Steps(dt, steps);
    }
    double moved, position, velocity;
    Differences(cpu, gpu, state, moved, position, velocity);
    std::vector<std::array<double, 6>> ia, ib;
    cpu.GetImpulses(ia);
    gpu.GetImpulses(ib);
    const auto sa = cpu.GetStatistics(), sb = gpu.GetStatistics();
    std::printf("%s: moved %.3g m, CPU and GPU differ by %.3g m; %llu and "
                "%llu contacts; impulse on the plate %.4g %.4g %.4g N s (GPU "
                "%.4g %.4g %.4g), moment %.4g %.4g %.4g N m s (GPU %.4g %.4g "
                "%.4g)\n",
                solver_type == shift::core::dem::NscSolver::kApgd ? "APGD"
                                                                  : "Jacobi",
                moved, position, (unsigned long long)sa.contacts,
                (unsigned long long)sb.contacts, ia[0][0], ia[0][1], ia[0][2],
                ib[0][0], ib[0][1], ib[0][2], ia[0][3], ia[0][4], ia[0][5],
                ib[0][3], ib[0][4], ib[0][5]);
    EXPECT_LT(ia[0][2], 0.0);
    EXPECT_LT(position, 1e-3 * std::max(moved, 1e-3));
    double size = 0.0;
    for(int c = 0; c < 3; c++) {
      size = std::max(size, std::abs(ia[0][c]));
    }
    for(int c = 0; c < 3; c++) {
      EXPECT_NEAR(ia[0][c], ib[0][c], 1e-3 * size);
      EXPECT_NEAR(ia[0][3 + c], ib[0][3 + c], 1e-3 * size * 0.02);
    }
  }
}

/* the speed of the non-smooth method on the GPU, APGD and Jacobi of 100
iterations, against the CPU's: the grains of the box of n by n by n fall
for 0.2 s in steps of 4e-4 s, then the next 50 steps are timed */
TEST(DemSpeed, NscGpu) {
  REQUIRE_DEVICE(device);
  for(int n : {10, 20, 40}) {
    for(auto solver_type : {shift::core::dem::NscSolver::kApgd,
                            shift::core::dem::NscSolver::kJacobi}) {
      for(bool on_gpu : {true, false}) {
        /* the CPU is slow */
        if(!on_gpu && n > 10) {
          continue;
        }
        std::vector<shift::core::dem::Template> templates;
        std::vector<shift::core::dem::Material> materials;
        shift::core::dem::Settings settings;
        shift::core::dem::State state;
        Box(n, templates, materials, settings, state);
        settings = Nsc(settings, solver_type, 100);
        /* the clumps packed at the bottom of the tall box */
        settings.max_neighbors = 24;
        std::unique_ptr<shift::core::dem::Solver> solver;
        if(on_gpu) {
          solver = std::make_unique<shift::core::dem::GpuSolver>(
            *device, settings, materials, templates, state);
        } else {
          solver = std::make_unique<shift::core::dem::CpuSolver>(
            settings, materials, templates, state);
        }
        const float h = 4e-4f;
        solver->Steps(h, 500);
        const long steps = 50;
        const auto start = std::chrono::steady_clock::now();
        solver->Steps(h, steps);
        const double seconds = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
        const auto stats = solver->GetStatistics();
        shift::core::dem::State end;
        solver->GetState(end);
        double fastest = 0.0;
        for(const auto& v : end.velocity) {
          fastest = std::max(fastest, double(std::sqrt(
                                        v[0] * v[0] + v[1] * v[1] + v[2] * v[2])));
        }
        std::printf("%s %s: %zu grains, %llu contacts: %.2f ms a step "
                    "(residual %.3g)%s\n",
                    on_gpu ? "GPU" : "CPU",
                    solver_type == shift::core::dem::NscSolver::kApgd
                      ? "APGD" : "Jacobi",
                    state.size(), (unsigned long long)stats.contacts,
                    1e3 * seconds / steps, stats.residual,
                    !(fastest < 10.0) ? "; diverged" : "");
      }
    }
  }
}

/* Gauss-Seidel, one contact after another, runs on the CPU only */
TEST(DemNsc, GpuGaussSeidelThrows) {
  REQUIRE_DEVICE(device);
  std::vector<shift::core::dem::Template> templates;
  std::vector<shift::core::dem::Material> materials;
  shift::core::dem::Settings settings;
  shift::core::dem::State state;
  Box(2, templates, materials, settings, state);
  settings = Nsc(settings, shift::core::dem::NscSolver::kGaussSeidel);
  EXPECT_THROW(shift::core::dem::GpuSolver(*device, settings, materials,
                                           templates, state),
               shift::core::Exception);
}

/* the shape of grains at rest: the largest overlap of two spheres or of a
sphere and a wall (m), the mean speed (m/s), the height of the highest
sphere's top above the floor, and the farthest a sphere's center reaches
along x, of all but the farthest 1 % (m) */
struct Pile {
  double overlap = 0.0, speed = 0.0, height = 0.0, runout = 0.0;
};

static Pile Measure(const shift::core::dem::State& state,
                    const shift::core::dem::Settings& settings) {
  Pile p;
  const size_t n = state.size();
  std::vector<double> x(n);
  for(size_t a = 0; a < n; a++) {
    const auto& pa = state.position[a];
    for(size_t b = a + 1; b < n; b++) {
      const auto& pb = state.position[b];
      const double d = std::sqrt(std::pow(pa[0] - pb[0], 2) +
                                 std::pow(pa[1] - pb[1], 2) +
                                 std::pow(pa[2] - pb[2], 2));
      p.overlap = std::max(p.overlap, 2.0 * kRadius - d);
    }
    for(const auto& w : settings.walls) {
      const double h = w.normal[0] * pa[0] + w.normal[1] * pa[1] +
                       w.normal[2] * pa[2] - w.offset;
      p.overlap = std::max(p.overlap, kRadius - h);
    }
    const auto& v = state.velocity[a];
    p.speed += std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) / n;
    p.height = std::max(p.height, -pa[2] + kRadius);
    x[a] = pa[0];
  }
  std::sort(x.begin(), x.end());
  p.runout = x[std::min(n - 1, size_t(0.99 * n))];
  return p;
}

/* settings for the comparison: SMC, or NSC by APGD or Gauss-Seidel */
static shift::core::dem::Settings Method(shift::core::dem::Settings settings,
                                         int method) {
  if(method > 0) {
    settings = Nsc(settings,
                   method == 1 ? shift::core::dem::NscSolver::kApgd
                               : shift::core::dem::NscSolver::kGaussSeidel,
                   100);
  }
  return settings;
}

static const char* kMethods[] = {"SMC", "NSC, APGD", "NSC, Gauss-Seidel"};

/* run state with settings by method for duration, in the SMC's stable step
at speed or NSC's step of 0.5 ms; the wall time (s) and the steps */
static Pile RunPile(const std::vector<shift::core::dem::Template>& templates,
                const std::vector<shift::core::dem::Material>& materials,
                const shift::core::dem::Settings& base, int method,
                shift::core::dem::State& state, double duration, double speed,
                double& seconds, uint64_t& steps) {
  const shift::core::dem::Settings settings = Method(base, method);
  const float dt =
    method == 0 ? static_cast<float>(
                    shift::core::dem::StableStep(templates, materials, speed))
                : 5e-4f;
  shift::core::dem::CpuSolver solver(settings, materials, templates, state);
  const auto start = std::chrono::steady_clock::now();
  solver.Steps(dt, static_cast<long>(duration / dt + 0.5));
  seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          start)
              .count();
  steps = solver.GetStatistics().steps;
  solver.GetState(state);
  return Measure(state, settings);
}

/* smooth (SMC) against non-smooth (NSC) contact, on the CPU, in two tests
of beads of glass (radius 5 mm, friction 0.5, restitution 0.3): 1000 beads,
10 by 10 by 10 in a box of 11 cm, moved by up to 1 mm, settle for 0.5 s;
and a column 6 beads long, 4 wide and 24 high against a wall, between two
walls 4.2 cm apart, collapses along x for 1 s; each with rolling resistance
0.05 and without. Without it the two methods agree; with it they differ by
their rolling resistance, Chrono::GPU's moment once a contact has lasted a
collision (SMC) or a constraint on the relative rolling from the start
(NSC). A benchmark (meson benchmark dem_nsc), not run with the tests: build
it optimized */
TEST(DemSpeed, NscAgainstSmc) {
  const auto bead =
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0);
  const std::vector<shift::core::dem::Template> templates = {bead};
  const double s = 2.0 * kRadius + 1e-3;
  /* the bed */
  shift::core::dem::Settings bed;
  bed.walls = {Floor()};
  const double half = 0.055;
  for(int axis = 0; axis < 2; axis++) {
    for(int side = -1; side <= 1; side += 2) {
      shift::core::dem::Wall w;
      w.normal = {0.0, 0.0, 0.0};
      w.normal[axis] = -side;
      w.offset = -half;
      bed.walls.push_back(w);
    }
  }
  uint32_t seed = 4321u;
  auto random = [&seed]() {
    seed = seed * 1664525u + 1013904223u;
    return (seed >> 8) / 16777216.0 - 0.5;
  };
  shift::core::dem::State bed_start;
  for(int k = 0; k < 10; k++) {
    for(int j = 0; j < 10; j++) {
      for(int i = 0; i < 10; i++) {
        bed_start.AddGrain(0, {-half + (i + 0.5) * s + 2e-3 * random(),
                               -half + (j + 0.5) * s + 2e-3 * random(),
                               -(k + 0.5) * s});
      }
    }
  }
  /* the column: the wall behind it at x = 0, the walls beside it at y = 0
  and y = 4 s, the far wall at x = 0.5 */
  shift::core::dem::Settings column;
  column.walls = {Floor()};
  /* each wall's normal, out of its solid, and offset: the grains in
  x >= 0, x <= 0.5, y >= 0 and y <= 4 s */
  const double sides[4][4] = {{1, 0, 0, 0},
                              {-1, 0, 0, -0.5},
                              {0, 1, 0, 0},
                              {0, -1, 0, -4 * s}};
  for(const auto& side : sides) {
    shift::core::dem::Wall w;
    w.normal = {side[0], side[1], side[2]};
    w.offset = side[3];
    column.walls.push_back(w);
  }
  shift::core::dem::State column_start;
  for(int k = 0; k < 24; k++) {
    for(int j = 0; j < 4; j++) {
      for(int i = 0; i < 6; i++) {
        column_start.AddGrain(0, {(i + 0.5) * s + 1e-4 * random(),
                                  (j + 0.5) * s + 1e-4 * random(),
                                  -(k + 0.5) * s});
      }
    }
  }
  std::printf("%-18s %-8s %-7s %9s %8s %10s %9s %10s %11s\n", "", "test",
              "rolling", "time (s)", "steps", "height", "runout", "overlap",
              "speed");
  for(float rolling : {0.05f, 0.0f}) {
    const std::vector<shift::core::dem::Material> materials = {
      Glass(0.3f, 0.5f, rolling)};
    for(int method = 0; method < 3; method++) {
      double seconds;
      uint64_t steps;
      shift::core::dem::State state = bed_start;
      const Pile b = RunPile(templates, materials, bed, method, state, 0.5,
                             1.0, seconds, steps);
      std::printf("%-18s %-8s %-7.2f %9.2f %8llu %7.2f mm %9s %7.4f mm %6.2f "
                  "mm/s\n",
                  kMethods[method], "bed", rolling, seconds,
                  (unsigned long long)steps, 1e3 * b.height, "",
                  1e3 * b.overlap, 1e3 * b.speed);
      EXPECT_LT(b.height, 10 * s);
      EXPECT_LT(b.speed, 0.05);
      state = column_start;
      const Pile c = RunPile(templates, materials, column, method, state, 1.0,
                             2.0, seconds, steps);
      std::printf("%-18s %-8s %-7.2f %9.2f %8llu %7.2f mm %6.1f mm %7.4f mm "
                  "%6.2f mm/s\n",
                  kMethods[method], "column", rolling, seconds,
                  (unsigned long long)steps, 1e3 * c.height, 1e3 * c.runout,
                  1e3 * c.overlap, 1e3 * c.speed);
      /* it collapsed and spread */
      EXPECT_LT(c.height, 24 * s);
      EXPECT_GT(c.runout, 6 * s);
    }
  }
}




// ------------------------------------------------------------ convex hulls

/* a cube of side (m) as a hull of density */
static shift::core::dem::Template Cube(float side, uint32_t material = 0) {
  std::vector<std::array<float, 3>> points;
  for(int k = 0; k < 8; k++) {
    points.push_back({(k & 1 ? 0.5f : -0.5f) * side,
                      (k & 2 ? 0.5f : -0.5f) * side,
                      (k & 4 ? 0.5f : -0.5f) * side});
  }
  return shift::core::dem::Template::OfHull(points, kDensity, material);
}

/* the cube's hull has its 8 vertices and 6 planes, the mass and moments of
a cube, and the radius of the sphere of its volume */
TEST(DemHull, CubeOfHull) {
  const float side = 0.01f;
  const shift::core::dem::Template t = Cube(side);
  const double mass = kDensity * double(side) * side * side;
  EXPECT_EQ(t.hull.vertices.size(), 8u);
  EXPECT_EQ(t.hull.planes.size(), 6u);
  EXPECT_EQ(t.mesh_triangles.size(), 12u);
  EXPECT_NEAR(t.mass, mass, 1e-6 * mass);
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(t.inertia[k], mass * side * side / 6.0, 1e-5 * mass * side * side);
  }
  for(const auto& p : t.hull.planes) {
    EXPECT_NEAR(p[3], 0.5f * side, 1e-6f * side);
  }
  EXPECT_NEAR(t.hull.radius, std::cbrt(3.0 / (4.0 * kPi)) * side, 1e-6 * side);
  EXPECT_NEAR(t.hull.BoundingRadius(), 0.5 * std::sqrt(3.0) * side, 1e-6 * side);
}

/* a rock's hull: its center of mass at the origin, its inertia diagonal
(the products of the points' second moments vanish), and the clump of its
spheres inside it */
TEST(DemHull, RockAndItsClump) {
  const auto points = shift::core::dem::RockPoints(7, 0.005f);
  const shift::core::dem::Template rock =
    shift::core::dem::Template::OfHull(points, kDensity, 0);
  const shift::core::dem::Template clump =
    shift::core::dem::Template::ClumpOfHull(rock, 12, 0);
  EXPECT_GE(rock.hull.vertices.size(), 12u);
  EXPECT_LE(rock.hull.vertices.size(), shift::core::dem::kMaxHullVertices);
  /* the volume's centroid at the origin: the mesh's signed volumes */
  double volume = 0.0, first[3] = {0.0, 0.0, 0.0};
  for(const auto& tri : rock.mesh_triangles) {
    const auto& a = rock.mesh_vertices[tri[0]];
    const auto& b = rock.mesh_vertices[tri[1]];
    const auto& c = rock.mesh_vertices[tri[2]];
    const double v = (a[0] * (b[1] * c[2] - b[2] * c[1]) -
                      a[1] * (b[0] * c[2] - b[2] * c[0]) +
                      a[2] * (b[0] * c[1] - b[1] * c[0])) /
                     6.0;
    volume += v;
    for(int k = 0; k < 3; k++) {
      first[k] += v * (a[k] + b[k] + c[k]) / 4.0;
    }
  }
  EXPECT_GT(volume, 0.0);
  EXPECT_NEAR(rock.mass, kDensity * volume, 1e-4 * rock.mass);
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(first[k] / volume, 0.0, 1e-6);
  }
  EXPECT_EQ(clump.spheres.size(), 12u);
  EXPECT_EQ(clump.mass, rock.mass);
  for(const auto& s : clump.spheres) {
    for(const auto& p : rock.hull.planes) {
      EXPECT_LE(p[0] * s.center[0] + p[1] * s.center[1] + p[2] * s.center[2] +
                  s.radius,
                p[3] * (1.0f + 1e-5f));
    }
  }
}

/* a cube dropped flat onto the floor comes to rest on its face, not
rocking from corner to corner: its center at half its side above the floor
less the overlap, and its orientation as it was. With hmax-like steps of
StableStep */
TEST(DemHull, CubeRestsFlatOnFloor) {
  const float side = 0.01f;
  const std::vector<shift::core::dem::Template> templates = {Cube(side)};
  const std::vector<shift::core::dem::Material> materials = {Glass(0.3f)};
  shift::core::dem::Settings settings;
  settings.walls = {Floor()};
  shift::core::dem::State state;
  state.AddGrain(0, {0.0, 0.0, -0.5 * side - 0.002});
  shift::core::dem::CpuSolver solver(settings, materials, templates, state);
  const float dt =
    static_cast<float>(shift::core::dem::StableStep(templates, materials, 1.0));
  solver.Steps(dt, static_cast<long>(0.3 / dt));
  solver.GetState(state);
  const auto& v = state.velocity[0];
  const auto& q = state.orientation[0];
  const double speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  std::printf("the cube's center %.4g mm above the floor, at %.3g mm/s, "
              "turned %.3g rad\n",
              -1e3 * state.position[0][2], 1e3 * speed,
              2.0 * std::asin(std::min(1.0, std::sqrt(double(q[0] * q[0] +
                                                             q[1] * q[1] +
                                                             q[2] * q[2])))));
  EXPECT_NEAR(-state.position[0][2], 0.5 * side, 0.01 * side);
  EXPECT_LT(-state.position[0][2], 0.5 * side);
  EXPECT_LT(speed, 1e-4);
  EXPECT_LT(std::abs(q[0]) + std::abs(q[1]) + std::abs(q[2]), 1e-4f);
}

/* a cube resting on a cube on the floor, the top one a quarter side off
center (its center of mass still over the lower one), stays put: the
contact between two faces holds it as the floor does */
TEST(DemHull, CubeRestsOnCube) {
  const float side = 0.01f;
  const std::vector<shift::core::dem::Template> templates = {Cube(side)};
  const std::vector<shift::core::dem::Material> materials = {Glass(0.3f)};
  shift::core::dem::Settings settings;
  settings.walls = {Floor()};
  shift::core::dem::State state;
  state.AddGrain(0, {0.0, 0.0, -0.5 * side});
  state.AddGrain(0, {0.25 * side, 0.0, -1.5 * side - 1e-4});
  shift::core::dem::CpuSolver solver(settings, materials, templates, state);
  const float dt =
    static_cast<float>(shift::core::dem::StableStep(templates, materials, 1.0));
  solver.Steps(dt, static_cast<long>(0.3 / dt));
  solver.GetState(state);
  std::printf("the top cube's center at x = %.4g mm, %.4g mm above the "
              "floor\n",
              1e3 * state.position[1][0], -1e3 * state.position[1][2]);
  EXPECT_NEAR(state.position[1][0], 0.25 * side, 0.01 * side);
  EXPECT_NEAR(-state.position[1][2], 1.5 * side, 0.01 * side);
  const auto& q = state.orientation[1];
  EXPECT_LT(std::abs(q[0]) + std::abs(q[1]) + std::abs(q[2]), 1e-3f);
}

/* two spinning rocks, and a rock and a sphere, meet off center, with
friction: the momentum and the angular momentum are kept, the forces of a
pair being opposite and at the same point */
TEST(DemHull, CollisionKeepsMomentum) {
  std::vector<shift::core::dem::Template> templates = {
    shift::core::dem::Template::OfHull(
      shift::core::dem::RockPoints(3, kRadius), kDensity, 0),
    shift::core::dem::Template::OfHull(
      shift::core::dem::RockPoints(4, 1.4f * kRadius), kDensity, 1),
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 1)};
  const std::vector<shift::core::dem::Material> materials = {
    Glass(0.4f, 0.6f), Glass(0.7f, 0.3f)};
  shift::core::dem::State state;
  state.AddGrain(0, {-0.02, 0.003, 0.001});
  state.AddGrain(1, {0.02, -0.002, 0.0});
  state.AddGrain(2, {0.0, 0.03, 0.0});
  state.velocity[0] = {1.0f, 0.0f, 0.1f};
  state.velocity[1] = {-0.5f, 0.1f, 0.0f};
  state.velocity[2] = {0.0f, -1.0f, 0.0f};
  state.angular_velocity[0] = {10.0f, -40.0f, 25.0f};
  state.angular_velocity[1] = {-5.0f, 15.0f, 30.0f};
  shift::core::dem::CpuSolver solver(NoGravity(), materials, templates, state);
  double p0[3], l0[3], p1[3], l1[3];
  Momentum(state, templates, p0, l0);
  const float dt =
    static_cast<float>(shift::core::dem::StableStep(templates, materials, 2.0));
  solver.Steps(dt, static_cast<long>(0.05 / dt));
  solver.GetState(state);
  Momentum(state, templates, p1, l1);
  std::printf("velocities after: %.3f %.3f %.3f m/s along x, x, y\n",
              state.velocity[0][0], state.velocity[1][0],
              state.velocity[2][1]);
  /* they met and parted */
  EXPECT_LT(state.velocity[0][0], 0.9f);
  EXPECT_GT(state.velocity[2][1], -0.9f);
  const double scale_p = templates[0].mass * 1.0;
  const double scale_l = scale_p * 0.02;
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(p1[k], p0[k], 1e-5 * scale_p) << k;
    EXPECT_NEAR(l1[k], l0[k], 1e-4 * scale_l) << k;
  }
}

/* rocks of 6 shapes on a jittered lattice of n by n by n in a box of
walls */
static void RockBox(int n, std::vector<shift::core::dem::Template>& templates,
                    std::vector<shift::core::dem::Material>& materials,
                    shift::core::dem::Settings& settings,
                    shift::core::dem::State& state) {
  templates.clear();
  for(uint32_t k = 0; k < 6; k++) {
    templates.push_back(shift::core::dem::Template::OfHull(
      shift::core::dem::RockPoints(10 + k, kRadius), kDensity, 0));
  }
  materials = {Glass(0.3f), Glass(0.3f)};
  settings = shift::core::dem::Settings();
  settings.max_neighbors = 32;
  const double spacing = 3.2 * kRadius, half = 0.5 * n * spacing;
  settings.walls = {Floor()};
  for(int k = 0; k < 4; k++) {
    shift::core::dem::Wall w;
    w.normal = {k == 0 ? 1.0 : k == 1 ? -1.0 : 0.0, 0.0, 0.0};
    if(k >= 2) {
      w.normal = {0.0, k == 2 ? 1.0 : -1.0, 0.0};
    }
    w.offset = -half - 0.5 * kRadius;
    w.material = 1;
    settings.walls.push_back(w);
  }
  state = shift::core::dem::State();
  uint32_t seed = 1;
  for(int i = 0; i < n; i++) {
    for(int j = 0; j < n; j++) {
      for(int k = 0; k < n; k++) {
        seed = seed * 1103515245u + 12345u;
        state.AddGrain((seed >> 16) % 6,
                       {-half + (i + 0.5) * spacing,
                        -half + (j + 0.5) * spacing,
                        -(k + 0.5) * spacing - kRadius});
        const float a = 0.5f * ((seed >> 4) % 628) / 100.0f;
        const float c = std::cos(a), s = std::sin(a);
        state.orientation.back() = {s * 0.48f, s * 0.6f, s * 0.64f, c};
      }
    }
  }
}

/* rocks fall into a box and settle */
TEST(DemHull, RocksSettleInBox) {
  std::vector<shift::core::dem::Template> templates;
  std::vector<shift::core::dem::Material> materials;
  shift::core::dem::Settings settings;
  shift::core::dem::State state;
  RockBox(3, templates, materials, settings, state);
  shift::core::dem::CpuSolver solver(settings, materials, templates, state);
  const float dt =
    static_cast<float>(shift::core::dem::StableStep(templates, materials, 1.0));
  solver.Steps(dt, static_cast<long>(0.6 / dt));
  solver.GetState(state);
  const double half = 0.5 * 3 * 3.2 * kRadius;
  double mean = 0.0, lowest = -1.0, widest = 0.0;
  for(size_t g = 0; g < state.size(); g++) {
    const auto& v = state.velocity[g];
    mean += std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) / state.size();
    lowest = std::max(lowest, state.position[g][2]);
    widest = std::max(widest, std::max(std::abs(state.position[g][0]),
                                       std::abs(state.position[g][1])));
  }
  std::printf("mean speed %.4f m/s, lowest center %.4g mm above the floor, "
              "|x|, |y| to %.4g mm (walls at %.4g)\n",
              mean, -1e3 * lowest, 1e3 * widest, 1e3 * (half + 0.5 * kRadius));
  EXPECT_LT(mean, 0.005);
  EXPECT_LT(lowest, 0.0);
  EXPECT_GT(-lowest, 0.3 * kRadius);
  EXPECT_LT(widest, half + 0.5 * kRadius);
}

/* the GPU computes the hulls' contacts as the CPU does, and gives the same
bits on a second run */
TEST(DemHull, GpuMatchesCpu) {
  REQUIRE_DEVICE(device);
  std::vector<shift::core::dem::Template> templates;
  std::vector<shift::core::dem::Material> materials;
  shift::core::dem::Settings settings;
  shift::core::dem::State state;
  RockBox(4, templates, materials, settings, state);
  /* a few spheres among them */
  templates.push_back(
    shift::core::dem::Template::SingleSphere(kRadius, kDensity, 0));
  for(size_t g = 0; g < state.size(); g += 7) {
    state.template_index[g] = 6;
  }
  const float dt =
    static_cast<float>(shift::core::dem::StableStep(templates, materials, 1.0));
  shift::core::dem::CpuSolver cpu(settings, materials, templates, state);
  shift::core::dem::GpuSolver gpu(*device, settings, materials, templates,
                                  state);
  shift::core::dem::GpuSolver again(*device, settings, materials, templates,
                                    state);
  cpu.Steps(dt, 1500);
  gpu.Steps(dt, 1500);
  again.Steps(dt, 1500);
  shift::core::dem::State a, b, c;
  cpu.GetState(a);
  gpu.GetState(b);
  again.GetState(c);
  EXPECT_EQ(std::memcmp(b.position.data(), c.position.data(),
                        b.size() * sizeof(b.position[0])),
            0);
  EXPECT_EQ(std::memcmp(b.orientation.data(), c.orientation.data(),
                        b.size() * sizeof(b.orientation[0])),
            0);
  double moved = 0.0, difference = 0.0;
  for(size_t g = 0; g < state.size(); g++) {
    for(int k = 0; k < 3; k++) {
      moved = std::max(moved, std::abs(a.position[g][k] - state.position[g][k]));
      difference =
        std::max(difference, std::abs(a.position[g][k] - b.position[g][k]));
    }
  }
  std::printf("moved %.3g m, CPU and GPU differ by %.3g m\n", moved,
              difference);
  EXPECT_GT(moved, 1e-3);
  EXPECT_LT(difference, 1e-3 * moved);
}

/* a cube on the vertex where the plate's four triangles meet, across all
four, rests as on a floor, pushed once and not four times: at the height it
rests at on a wall, unturned; the body's impulse is the weight over the
time, and its moment about the body's origin, 0.02 m from the cube, the
weight's */
TEST(DemHull, CubeRestsOnBodyAsOnWall) {
  const float side = 0.01f;
  const std::vector<shift::core::dem::Template> templates = {Cube(side)};
  const std::vector<shift::core::dem::Material> materials = {Glass(0.3f)};
  const float dt =
    static_cast<float>(shift::core::dem::StableStep(templates, materials, 1.0));
  double heights[2];
  shift::core::dem::State state;
  std::vector<std::array<double, 6>> impulses;
  long n = 0;
  for(int on_body = 0; on_body < 2; on_body++) {
    shift::core::dem::Settings settings;
    std::vector<shift::core::dem::Body> bodies;
    if(on_body) {
      shift::core::dem::Body plate = Plate({-0.02f, 0.0f}, 0.03f);
      plate.pose.t = {0.02, 0.0, 0.0};
      bodies.push_back(plate);
    } else {
      settings.walls = {Floor()};
    }
    state = shift::core::dem::State();
    state.AddGrain(0, {0.0, 0.0, -0.5 * side - 0.002});
    shift::core::dem::CpuSolver solver(settings, materials, templates, state,
                                       bodies);
    solver.Steps(dt, static_cast<long>(0.3 / dt));
    solver.ClearImpulses();
    n = static_cast<long>(0.05 / dt);
    solver.Steps(dt, n);
    solver.GetState(state);
    solver.GetImpulses(impulses);
    heights[on_body] = -state.position[0][2];
  }
  const double weight = templates[0].mass * 9.81, duration = n * double(dt);
  const auto& q = state.orientation[0];
  std::printf("the cube's center %.6g mm above the plate, %.6g mm above a "
              "floor; impulse %.4g N s, weight over the time %.4g N s, "
              "moment %.4g N m s\n",
              1e3 * heights[1], 1e3 * heights[0], impulses[0][2],
              weight * duration, impulses[0][4]);
  const double overlap = 0.5 * side - heights[0];
  EXPECT_GT(overlap, 0.0);
  EXPECT_NEAR(heights[1], heights[0], 0.02 * overlap);
  EXPECT_LT(std::abs(q[0]) + std::abs(q[1]) + std::abs(q[2]), 1e-4f);
  ASSERT_EQ(impulses.size(), 1u);
  EXPECT_NEAR(impulses[0][2], weight * duration, 1e-3 * weight * duration);
  EXPECT_NEAR(impulses[0][0], 0.0, 1e-4 * weight * duration);
  EXPECT_NEAR(impulses[0][1], 0.0, 1e-4 * weight * duration);
  EXPECT_NEAR(impulses[0][4], 0.02 * weight * duration,
              2e-3 * 0.02 * weight * duration);
  EXPECT_NEAR(impulses[0][3], 0.0, 1e-3 * 0.02 * weight * duration);
}

/* a plate normal to x moving at speed v hits a cube at rest face on, with
gravity off: the cube leaves at about (1 + e) v without turning, and the
body's impulse is the cube's momentum, opposite */
TEST(DemHull, MovingBodyPushesCube) {
  const float side = 0.01f;
  const std::vector<shift::core::dem::Template> templates = {Cube(side)};
  const float restitution = 0.5f;
  const std::vector<shift::core::dem::Material> materials = {
    Glass(restitution)};
  const shift::core::dem::Settings settings = NoGravity();
  shift::core::dem::Body plate = Plate({0.0f, 0.0f}, 0.02f);
  plate.pose.R = {0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0};
  const double speed = 0.5, t1 = 0.1;
  plate.pose.t = {-0.5 * side - 1e-3, 0.0, 0.0};
  plate.pose.v = {speed, 0.0, 0.0};
  shift::core::sph::BodyPose end = plate.pose;
  end.t[0] += speed * t1;
  shift::core::dem::State state;
  state.AddGrain(0, {0.0, 0.0, 0.0});
  shift::core::dem::CpuSolver solver(settings, materials, templates, state,
                                     {plate});
  solver.SetBodyPaths({plate.pose}, {end}, 0.0, t1);
  solver.ClearImpulses();
  const float dt = static_cast<float>(
    shift::core::dem::StableStep(templates, materials, speed, 0.05));
  solver.Steps(dt, static_cast<long>(0.02 / dt));
  solver.GetState(state);
  std::vector<std::array<double, 6>> impulses;
  solver.GetImpulses(impulses);
  const double momentum = templates[0].mass * state.velocity[0][0];
  const auto& w = state.angular_velocity[0];
  std::printf("cube leaves at %.4f m/s, (1 + e) v = %.4f, turning at %.3g "
              "rad/s; impulse %.5g N s, momentum %.5g N s\n",
              state.velocity[0][0], (1.0 + restitution) * speed,
              std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]),
              impulses[0][0], momentum);
  EXPECT_NEAR(state.velocity[0][0], (1.0 + restitution) * speed, 0.05 * speed);
  EXPECT_NEAR(impulses[0][0], -momentum, 1e-4 * momentum);
  EXPECT_LT(std::abs(state.velocity[0][1]) + std::abs(state.velocity[0][2]),
            1e-5f);
  EXPECT_LT(std::abs(w[0]) + std::abs(w[1]) + std::abs(w[2]), 1e-2f);
}

/* the GPU does the CPU's arithmetic for rocks and a body: the tilted,
turning plate of BoxAndPlate comes down into the rocks of RockBox. Later a
rock's hit on the plate makes the runs part, as it makes two CPU runs part
whose rocks start 3e-7 m apart */
TEST(DemHull, GpuMatchesCpuWithBody) {
  REQUIRE_DEVICE(device);
  std::vector<shift::core::dem::Template> templates;
  std::vector<shift::core::dem::Material> materials;
  shift::core::dem::Settings settings;
  shift::core::dem::State state;
  shift::core::dem::Body plate;
  shift::core::sph::BodyPose end;
  const long steps = 400;
  float dt;
  BoxAndPlate(4, templates, materials, settings, state, plate, end, steps, dt);
  RockBox(4, templates, materials, settings, state);
  dt =
    static_cast<float>(shift::core::dem::StableStep(templates, materials, 1.0));
  {
    /* the plate's path over these steps */
    const double duration = steps * double(dt), w = 20.0, v = 1.0;
    end = plate.pose;
    end.t[2] += v * duration;
    const double cz = std::cos(w * duration), sz = std::sin(w * duration);
    for(int col = 0; col < 3; col++) {
      const double x = plate.pose.R[3 * col], y = plate.pose.R[3 * col + 1];
      end.R[3 * col] = cz * x - sz * y;
      end.R[3 * col + 1] = sz * x + cz * y;
    }
  }
  shift::core::dem::CpuSolver cpu(settings, materials, templates, state,
                                  {plate});
  shift::core::dem::GpuSolver gpu(*device, settings, materials, templates,
                                  state, {plate});
  for(shift::core::dem::Solver* solver :
      {static_cast<shift::core::dem::Solver*>(&cpu),
       static_cast<shift::core::dem::Solver*>(&gpu)}) {
    solver->SetBodyPaths({plate.pose}, {end}, 0.0, steps * double(dt));
    solver->ClearImpulses();
    solver->Steps(dt, steps);
  }
  shift::core::dem::State a, b;
  cpu.GetState(a);
  gpu.GetState(b);
  double moved = 0.0, difference = 0.0;
  for(size_t g = 0; g < state.size(); g++) {
    for(int k = 0; k < 3; k++) {
      moved = std::max(moved, std::abs(a.position[g][k] - state.position[g][k]));
      difference =
        std::max(difference, std::abs(a.position[g][k] - b.position[g][k]));
    }
  }
  std::vector<std::array<double, 6>> ia, ib;
  cpu.GetImpulses(ia);
  gpu.GetImpulses(ib);
  std::printf("moved %.3g m, CPU and GPU differ by %.3g m; impulse on the "
              "plate %.4g %.4g %.4g N s (GPU %.4g %.4g %.4g)\n",
              moved, difference, ia[0][0], ia[0][1], ia[0][2], ib[0][0],
              ib[0][1], ib[0][2]);
  EXPECT_GT(moved, 1e-3);
  EXPECT_LT(difference, 1e-3 * moved);
  /* the rocks push the plate back up */
  EXPECT_LT(ia[0][2], 0.0);
  double size = 0.0;
  for(int c = 0; c < 3; c++) {
    size = std::max(size, std::abs(ia[0][c]));
  }
  for(int c = 0; c < 3; c++) {
    EXPECT_NEAR(ia[0][c], ib[0][c], 1e-3 * size);
    EXPECT_NEAR(ia[0][3 + c], ib[0][3 + c], 1e-3 * size * 0.02);
  }
}

/* hulls take the smooth method */
TEST(DemHull, NscThrows) {
  const std::vector<shift::core::dem::Template> templates = {Cube(0.01f)};
  const std::vector<shift::core::dem::Material> materials = {Glass()};
  shift::core::dem::State state;
  state.AddGrain(0, {0.0, 0.0, -0.01});
  shift::core::dem::Settings settings;
  settings.method = shift::core::dem::ContactMethod::kNsc;
  EXPECT_THROW(
    shift::core::dem::CpuSolver(settings, materials, templates, state),
    shift::core::Exception);
}


/* the DEM in a model (force_module::Dem): a plate on a vertical slide drops
onto a bed of grains in a box of walls and comes to rest on it, carried by
the grains, and the GPU gives what the CPU does; a wedge, the convex hull of
its mesh, splits a bed; and, as benchmarks (DemSpeed), a rock thrown into
186,000 grains and a million grains falling into place. */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/dem/dem.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/dem.h"
#include "shift/core/gpu/device.h"

static const char* kLattice =
  R"([{"grain": "bead", "min": [-0.022, -0.022, -0.022],
       "max": [0.022, 0.022, 0], "spacing": 0.011, "jitter": 0.05,
       "seed": 7}])";

/* grains of radius 5 mm, by default (fill) 4 by 4 by 2 on a lattice of
11 mm in a box of 6 cm with its floor at z = 0 (z down); a plate of mass,
5 cm square and 1 cm thick, its bottom at z = -0.03, on a vertical slide;
run for time_span, in steps of at most hmax; extra adds parameters to the
DEM, such as "save"; the plate starts lift higher */
static void LoadPlateOnGrains(shift::core::Context& context,
                              const std::string& device, double mass,
                              double time_span, double hmax,
                              const std::string& fill = kLattice,
                              const std::string& extra = "",
                              double lift = 0.0) {
  const double plate_z = -0.035 - lift;
  const std::string path =
    testing::TempDir() + "/dem_plate_" + device + ".json";
  std::ostringstream model;
  model.precision(17);
  model << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": )" << time_span
        << R"(, "time_step": 0.01, "hmax": )" << hmax << R"(}]
  },
  "assemblies": [{"name": "a",
    "force_modules": [{"type": "dem", "name": "grains", "device": ")"
        << device << R"(",
      "materials": [{"name": "glass", "youngs_modulus": 5e7,
                     "poisson_ratio": 0.3, "restitution": 0.3,
                     "friction": 0.5, "rolling_resistance": 0.05}],
      "grains": [{"name": "bead", "radius": 0.005, "material": "glass",
                  "mass": 0.0013}],
      "fill": )" << fill << R"(,)" << extra << R"(
      "walls": [{"normal": [0, 0, -1], "offset": 0, "material": "glass"},
                {"normal": [1, 0, 0], "offset": -0.03, "material": "glass"},
                {"normal": [-1, 0, 0], "offset": -0.03, "material": "glass"},
                {"normal": [0, 1, 0], "offset": -0.03, "material": "glass"},
                {"normal": [0, -1, 0], "offset": -0.03, "material": "glass"}],
      "bodies": [{"body": "plate", "geometry": ["plate"],
                  "material": "glass"}]}],
    "bodies": [{"name": "plate", "center_of_mass": [0, 0, )" << plate_z
        << R"(],
                "collision_geometry": [{"name": "plate", "type": "box",
                  "half_extents": [0.025, 0.025, 0.005], "contact": false,
                  "pose": {"T": [0, 0, )" << plate_z << R"(]}}],
                "inertia": {"mass": )" << mass << R"(, "Ixx": 1e-4,
                            "Iyy": 1e-4, "Izz": 1e-4, "Ixy": 0, "Iyz": 0,
                            "Izx": 0}}],
    "joints": [{"name": "slide", "type": "prismatic",
                "parent": [["fixed"], "origin"],
                "child": [["plate"], "origin"], "axis": [0, 0, 1]}]
  }]
})";
  {
    std::ofstream file(path);
    file << model.str();
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

static shift::core::force_module::Dem* GetDem(shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto dem = dynamic_cast<shift::core::force_module::Dem*>(module)) {
      return dem;
    }
  }
  ADD_FAILURE() << "no DEM";
  return nullptr;
}

/* the plate falls onto the grains and settles: over the run the grains'
impulse on it is its weight's but for its momentum at the end; it ends at
rest on the top layer, which is 2 grains high, and the grains stay in the
box (on the CPU) */
static void PlateSettles(const std::string& device,
                         const std::string& extra = "") {
  const double mass = 0.1, time_span = 0.5;
  shift::core::Context context;
  /* the grains' load is a step behind: the integrator's steps no longer than
  a substep, or the plate rattles on the grains (at 1e-4 s, 0.1 to 0.2 m/s) */
  const double hmax = 4e-5;
  LoadPlateOnGrains(context, device, mass, time_span, hmax, kLattice, extra);
  const auto start = std::chrono::steady_clock::now();
  context.Simulate();
  const double seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
  shift::core::force_module::Dem* dem = GetDem(context);
  ASSERT_NE(dem, nullptr);
  /* a box: 12 triangles, divided to no longer than a cell */
  EXPECT_GE(dem->get_body_triangle_count(0), 12u);
  shift::core::body::Base* plate = nullptr;
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "plate") {
      plate = body;
    }
  }
  ASSERT_NE(plate, nullptr);
  const double z = context.get_body_origin_pose(plate).t.z;
  const ksl_screw_t velocity =
    context.GetBodyVelocity(context.GetBodyLoadIndex(plate));
  const ksl_coscrew_t& impulse = dem->get_body_impulse(0);
  const double expected = mass * 9.81 * dem->get_time() - mass * velocity.lin.z;
  shift::core::dem::State grains;
  dem->GetGrains(grains);
  double highest = 0.0, widest = 0.0;
  for(size_t g = 0; g < grains.size(); g++) {
    highest = std::min(highest, grains.position[g][2]);
    widest = std::max(widest, std::max(std::abs(grains.position[g][0]),
                                       std::abs(grains.position[g][1])));
  }
  /* the bottom of the plate (z down) */
  const double bottom = z - 0.03;
  const auto stats = dem->GetStatistics();
  std::printf("%s%s: %.2f s; %zu grains, %zu triangles, substep %.3g s, "
              "%llu steps; "
              "plate bottom at %.4f m, top grain at %.4f m; speed %.3g mm/s; "
              "impulse %.4g N s, of the weight %.4g N s\n",
              device.c_str(), extra.c_str(), seconds,
              dem->get_grain_count(),
              dem->get_body_triangle_count(0), dem->get_step(),
              (unsigned long long)stats.steps, bottom, highest,
              1e3 * velocity.lin.z, -impulse.lin.z, expected);
  EXPECT_GT(dem->get_time(), time_span - 2.0 * dem->get_step() - 0.01);
  EXPECT_NEAR(-impulse.lin.z, expected, 0.03 * expected);
  /* NSC: up to 2 %, the plate free in the grains' step, its slide holding
  it a step later */
  const double sideways = extra.empty() ? 0.01 : 0.02;
  EXPECT_LT(std::abs(impulse.lin.x), sideways * expected);
  EXPECT_LT(std::abs(impulse.lin.y), sideways * expected);
  EXPECT_LT(std::abs(velocity.lin.z), 0.02);
  /* on the top grains: their centers a radius below the plate, less the
  overlaps */
  EXPECT_NEAR(bottom, highest - 0.005, 0.002);
  /* two layers: the top grains below 2 diameters */
  EXPECT_GT(highest, -0.02 - 0.002);
  EXPECT_LT(widest, 0.03);
}

TEST(DemModel, PlateSettlesOnGrains) {
  PlateSettles("cpu");
}

/* the same with non-smooth contact: the plate rests on the grains, carried
by them, as with smooth contact */
TEST(DemModel, NscPlateSettlesOnGrains) {
  PlateSettles("cpu", R"("contact_method": "nsc",)");
}

/* the plate on the grains, its height and the grains' impulse on it after
time_span on device */
static void PlateOnGrains(const std::string& device, double time_span,
                          double& z, double& impulse) {
  shift::core::Context context;
  LoadPlateOnGrains(context, device, 0.1, time_span, 4e-5);
  context.Simulate();
  shift::core::force_module::Dem* dem = GetDem(context);
  ASSERT_NE(dem, nullptr);
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "plate") {
      z = context.get_body_origin_pose(body).t.z;
    }
  }
  impulse = dem->get_body_impulse(0).lin.z;
}

/* the GPU does the CPU's arithmetic in a model: after the plate has landed
on the grains, at 0.045 s, and bounced off them, it is where the CPU puts it,
with the same impulse. The runs are the same until the landing; in it, the
grains' many contacts make the last bits by which the GPU differs (where it
fuses a multiplication and an addition) grow to 0.5 % of the impulse */
TEST(DemModel, GpuMatchesCpu) {
  if(!shift::core::gpu::Device::is_built()) {
    GTEST_SKIP() << "built without Vulkan";
  }
  try {
    shift::core::Context context;
    context.GetGpuDevice();
  } catch(const shift::core::Exception& e) {
    if(std::getenv("SHIFT_GPU_REQUIRED")) {
      FAIL() << e.what();
    }
    GTEST_SKIP() << "no GPU device: " << e.what();
  }
  double z_cpu = 0.0, z_gpu = 0.0, impulse_cpu = 0.0, impulse_gpu = 0.0;
  PlateOnGrains("cpu", 0.06, z_cpu, impulse_cpu);
  PlateOnGrains("gpu", 0.06, z_gpu, impulse_gpu);
  std::printf("plate at z = %.6f m (GPU %.6f), impulse %.5g N s (GPU "
              "%.5g)\n",
              z_cpu, z_gpu, impulse_cpu, impulse_gpu);
  EXPECT_NEAR(z_gpu, z_cpu, 3e-5);
  EXPECT_NEAR(impulse_gpu, impulse_cpu, 0.01 * std::abs(impulse_cpu));
  EXPECT_LT(impulse_cpu, -0.05);
}

/* the grains a model saves at its end are read back as they were, and a
model filled from them starts from them exactly */
TEST(DemModel, SavesGrainsAndFillsFromThem) {
  const std::string file = testing::TempDir() + "/dem_saved_grains.csv";
  std::remove(file.c_str());
  shift::core::dem::State saved;
  {
    shift::core::Context context;
    LoadPlateOnGrains(context, "cpu", 0.1, 0.02, 4e-5, kLattice,
                      R"("save": ")" + file + R"(",)");
    context.Simulate();
    shift::core::force_module::Dem* dem = GetDem(context);
    ASSERT_NE(dem, nullptr);
    dem->GetGrains(saved);
    shift::core::dem::State read;
    shift::core::dem::ReadGrains(file, dem->get_grain_names(), read);
    ASSERT_EQ(read.size(), saved.size());
    EXPECT_EQ(read.template_index, saved.template_index);
    EXPECT_EQ(read.position, saved.position);
    EXPECT_EQ(read.orientation, saved.orientation);
    EXPECT_EQ(read.velocity, saved.velocity);
    EXPECT_EQ(read.angular_velocity, saved.angular_velocity);
    /* they moved: not the lattice */
    EXPECT_NE(saved.position, dem->get_initial_grains().position);
  }
  shift::core::Context context;
  LoadPlateOnGrains(context, "cpu", 0.1, 0.02, 4e-5,
                    R"([{"file": ")" + file + R"("}])");
  shift::core::force_module::Dem* dem = GetDem(context);
  ASSERT_NE(dem, nullptr);
  const auto& initial = dem->get_initial_grains();
  EXPECT_EQ(initial.position, saved.position);
  EXPECT_EQ(initial.orientation, saved.orientation);
  EXPECT_EQ(initial.velocity, saved.velocity);
  EXPECT_EQ(initial.angular_velocity, saved.angular_velocity);
  std::remove(file.c_str());
}

/* the largest distance a grain moves in a run of the model over time_span,
from its fill */
static double LargestMove(const std::string& fill, double time_span) {
  shift::core::Context context;
  LoadPlateOnGrains(context, "cpu", 0.1, time_span, 4e-5, fill, "", 5.0);
  context.Simulate();
  shift::core::force_module::Dem* dem = GetDem(context);
  shift::core::dem::State end;
  dem->GetGrains(end);
  const auto& start = dem->get_initial_grains();
  double largest = 0.0;
  for(size_t g = 0; g < end.size(); g++) {
    double d = 0.0;
    for(int k = 0; k < 3; k++) {
      d += std::pow(end.position[g][k] - start.position[g][k], 2);
    }
    largest = std::max(largest, std::sqrt(d));
  }
  return largest;
}

/* grains settled and saved stay where they are in a model filled from them,
while grains on the lattice fall into place; the plate starts 5 m up, so
that it does not reach them, and the grains settle on their own for 0.8 s */
TEST(DemModel, SettledGrainsStayAtRest) {
  const std::string file = testing::TempDir() + "/dem_settled_grains.csv";
  {
    shift::core::Context context;
    LoadPlateOnGrains(context, "cpu", 0.1, 0.8, 4e-5, kLattice,
                      R"("save": ")" + file + R"(",)", 5.0);
    context.Simulate();
  }
  const std::string settled = R"([{"file": ")" + file + R"("}])";
  const double lattice = LargestMove(kLattice, 0.03);
  const double rest = LargestMove(settled, 0.03);
  std::printf("in 0.03 s grains on the lattice move up to %.3g mm, settled "
              "ones %.3g mm\n",
              1e3 * lattice, 1e3 * rest);
  EXPECT_GT(lattice, 5e-4);
  EXPECT_LT(rest, 2e-5);
  std::remove(file.c_str());
}

/* a model of beads of glass, radius 5 mm, filled by fill (a JSON array) in
a box of steel walls half wide (m) with its floor at z = 0 (z down), and
the bodies, joints and DEM bodies given (JSON arrays), run for time_span in
steps of at most hmax on device; files are written beside the model, by
name and text, for its meshes, and removed with it once it is loaded; extra
adds parameters to the DEM, each followed by a comma */
static void LoadGrainsModel(
  shift::core::Context& context, const std::string& base,
  const std::string& device, double time_span, double hmax, double max_speed,
  double half, const std::string& fill, const std::string& bodies,
  const std::string& joints, const std::string& dem_bodies,
  const std::vector<std::pair<std::string, std::string>>& files,
  const std::string& extra = "",
  const std::string& grains = R"([{"name": "bead", "radius": 0.005,
                                   "material": "glass", "mass": 0.0013}])") {
  const std::string directory = testing::TempDir();
  const std::string path = directory + "/" + base + ".json";
  std::ostringstream model;
  model.precision(17);
  model << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": )" << time_span
        << R"(, "time_step": 0.01, "hmax": )" << hmax << R"(}]
  },
  "assemblies": [{"name": "a",
    "force_modules": [{"type": "dem", "name": "grains", "device": ")"
        << device << R"(", "max_speed": )" << max_speed << R"(,)" << extra
        << R"(
      "materials": [{"name": "glass", "youngs_modulus": 5e7,
                     "poisson_ratio": 0.3, "restitution": 0.3,
                     "friction": 0.5, "rolling_resistance": 0.05},
                    {"name": "steel", "youngs_modulus": 2e8,
                     "poisson_ratio": 0.3, "restitution": 0.3,
                     "friction": 0.3}],
      "grains": )" << grains << R"(,
      "fill": )" << fill << R"(,
      "walls": [{"normal": [0, 0, -1], "offset": 0, "material": "steel"},
                {"normal": [1, 0, 0], "offset": )" << -half
        << R"(, "material": "steel"},
                {"normal": [-1, 0, 0], "offset": )" << -half
        << R"(, "material": "steel"},
                {"normal": [0, 1, 0], "offset": )" << -half
        << R"(, "material": "steel"},
                {"normal": [0, -1, 0], "offset": )" << -half
        << R"(, "material": "steel"}],
      "bodies": )" << dem_bodies << R"(}],
    "bodies": )" << bodies << R"(,
    "joints": )" << joints << R"(
  }]
})";
  std::vector<std::string> written = {path};
  {
    std::ofstream file(path);
    file << model.str();
  }
  for(const auto& f : files) {
    written.push_back(directory + "/" + f.first);
    std::ofstream file(written.back());
    file << f.second;
  }
  auto remove = [&written]() {
    for(const auto& w : written) {
      std::remove(w.c_str());
    }
  };
  try {
    context.LoadJSON(path);
    remove();
  } catch(...) {
    remove();
    throw;
  }
}

/* the body of name in context */
static shift::core::body::Base* GetBody(shift::core::Context& context,
                                        const std::string& name) {
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == name) {
      return body;
    }
  }
  ADD_FAILURE() << "no body " << name;
  return nullptr;
}

/* a wedge, edge down: half wide kWedgeHalf in x, as long in y, kWedgeHeight
high, its edge along y at z = 0 in its body's coordinates */
static const double kWedgeHalf = 0.03;
static const double kWedgeHeight = 0.03;
static const char* kWedgeObj = R"(v -0.03 -0.03 -0.03
v 0.03 -0.03 -0.03
v 0 -0.03 0
v -0.03 0.03 -0.03
v 0.03 0.03 -0.03
v 0 0.03 0
f 1 2 3
f 4 6 5
f 1 4 5
f 1 5 2
f 2 5 6
f 2 6 3
f 3 6 4
f 3 4 1
)";

/* the distance from (x, z) to the segment from a to b */
static double SegmentDistance(double x, double z, double ax, double az,
                              double bx, double bz) {
  const double ex = bx - ax, ez = bz - az;
  const double s = std::clamp(((x - ax) * ex + (z - az) * ez) /
                                (ex * ex + ez * ez),
                              0.0, 1.0);
  return std::hypot(x - ax - s * ex, z - az - s * ez);
}

/* the depth of a sphere of radius r at p in the wedge whose edge is at
height z_edge: r less the distance from its center to the wedge, or r and
the distance from its center to the nearest face when the center is inside,
so that it is negative where the sphere is clear of it. The wedge is its
cross section in x and z, a triangle, swept along y, so the distance squared
outside it is the sum of the squares of the distances in the cross section
and in y; a distance to the planes of the faces alone would make the depth
of a bead under the edge too large by a third of its radius */
static double WedgeDepth(const std::array<double, 3>& p, double z_edge,
                         double r) {
  const double x = p[0], y = p[1], z = p[2] - z_edge;
  /* the triangle: the edge at the origin, the top corners at (+-w, -h) */
  const double w = kWedgeHalf, h = kWedgeHeight;
  const double n = std::hypot(h, w);
  /* the distances inside the faces: the slanted ones, the top, the ends */
  const double inside_section =
    std::min({-(h * x + w * z) / n, -(-h * x + w * z) / n, h + z});
  const double inside_y = kWedgeHalf - std::abs(y);
  if(inside_section >= 0.0 && inside_y >= 0.0) {
    return r + std::min(inside_section, inside_y);
  }
  const double section =
    inside_section >= 0.0
      ? 0.0
      : std::min({SegmentDistance(x, z, 0.0, 0.0, w, -h),
                  SegmentDistance(x, z, 0.0, 0.0, -w, -h),
                  SegmentDistance(x, z, -w, -h, w, -h)});
  const double along = std::max(-inside_y, 0.0);
  return r - std::hypot(section, along);
}

/* a wedge of 0.2 kg on a vertical slide, its collision geometry the convex
hull of its mesh ("convex_hull" without half extents), falls edge first into
a bed of beads and splits it: its edge sinks below the top of the bed, the
beads under it are pushed aside and to the floor, none is in the wedge but
by its overlap, and the beads' impulse on it is its weight's but for its
momentum at the end (on the CPU) */
static void WedgeSplitsGrains(const std::string& extra) {
  const double mass = 0.2, time_span = 0.25, z_start = -0.05;
  shift::core::Context context;
  std::ostringstream bodies, joints;
  bodies.precision(17);
  joints.precision(17);
  bodies << R"([{"name": "wedge", "center_of_mass": [0, 0, )"
         << -2.0 * kWedgeHeight / 3.0 << R"(],
    "geometry": [{"name": "wedge_mesh", "type": "mesh",
                  "filename": ["dem_wedge.obj"]}],
    "collision_geometry": [{"name": "wedge", "type": "convex_hull",
                            "contact": false}],
    "inertia": {"mass": )" << mass << R"(, "Ixx": 3e-5, "Iyy": 3e-5,
                "Izz": 6e-5, "Ixy": 0, "Iyz": 0, "Izx": 0}}])";
  /* the edge starts at z_start, placed by the slide */
  joints << R"([{"name": "slide", "type": "prismatic",
    "parent": [["fixed"], "origin"], "child": [["wedge"], "origin"],
    "axis": [0, 0, 1], "initial_displacement": )" << z_start << R"(}])";
  LoadGrainsModel(
    context, "dem_wedge", "cpu", time_span, 4e-5, 2.0, 0.04,
    R"([{"grain": "bead", "min": [-0.033, -0.033, -0.033],
         "max": [0.033, 0.033, 0], "spacing": 0.011, "jitter": 0.04,
         "seed": 3}])",
    bodies.str(), joints.str(),
    R"([{"body": "wedge", "geometry": ["wedge"], "material": "steel"}])",
    {{"dem_wedge.obj", kWedgeObj}}, extra);
  shift::core::force_module::Dem* dem = GetDem(context);
  ASSERT_NE(dem, nullptr);
  const shift::core::dem::State start = dem->get_initial_grains();
  const auto clock = std::chrono::steady_clock::now();
  context.Simulate();
  const double seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - clock)
      .count();
  /* the hull of the wedge: 8 triangles, divided to no longer than a cell */
  EXPECT_GE(dem->get_body_triangle_count(0), 8u);
  shift::core::body::Base* wedge = GetBody(context, "wedge");
  ASSERT_NE(wedge, nullptr);
  const double z_edge = context.get_body_origin_pose(wedge).t.z;
  const ksl_screw_t velocity =
    context.GetBodyVelocity(context.GetBodyLoadIndex(wedge));
  const double expected = mass * 9.81 * dem->get_time() - mass * velocity.lin.z;
  const double impulse = -dem->get_body_impulse(0).lin.z;
  shift::core::dem::State grains;
  dem->GetGrains(grains);
  /* the top of the bed at the start, and the deepest a bead is in the
  wedge */
  double top = 0.0, deepest = -1.0, under_start = 0.0, under_end = 0.0;
  int under = 0;
  for(size_t g = 0; g < grains.size(); g++) {
    const auto& p = grains.position[g];
    top = std::min(top, start.position[g][2] - 0.005);
    deepest = std::max(deepest, WedgeDepth(p, z_edge, 0.005));
    /* the beads of the top two layers that started under the edge */
    const auto& q = start.position[g];
    if(std::abs(q[0]) < 0.006 && std::abs(q[1]) < kWedgeHalf &&
       q[2] < -0.015) {
      under_start += std::abs(q[0]);
      under_end += std::abs(p[0]);
      under++;
    }
  }
  std::printf("%s %.2f s, %llu steps: %zu grains, %zu triangles; edge at "
              "%.4f m, the bed's top at the start at %.4f m; the deepest bead %.3g mm in the wedge; "
              "%d beads under the edge, at |x| %.4f m on average, from "
              "%.4f m; speed %.3g mm/s; impulse %.4g N s, of the weight "
              "%.4g N s\n",
              extra.c_str(), seconds,
              (unsigned long long)dem->GetStatistics().steps, grains.size(),
              dem->get_body_triangle_count(0), z_edge, top,
              1e3 * deepest, under, under_end / std::max(under, 1),
              under_start / std::max(under, 1), 1e3 * velocity.lin.z,
              impulse, expected);
  EXPECT_GT(dem->get_time(), time_span - 2.0 * dem->get_step() - 0.01);
  /* below the top of the bed by more than a radius (SMC 20 mm, NSC 23
  mm), above the floor */
  EXPECT_GT(z_edge, top + 0.005);
  EXPECT_LT(z_edge, 0.0);
  /* the beads under the edge pushed aside (from 5.5 mm to 16 mm) */
  ASSERT_GT(under, 10);
  EXPECT_GT(under_end, 2.0 * under_start);
  /* clear of the wedge but for overlaps of a fraction of a millimeter */
  EXPECT_LT(deepest, 1e-3);
  EXPECT_NEAR(impulse, expected, 0.03 * expected);
  /* slower than half its speed on reaching the bed, after falling from
  z_start to its top: the wedge does not come to rest, it sinks in steps as
  the beads under its edge give way, and at the end it may be in one (at up
  to 0.2 m/s, of 0.58 m/s); its momentum is in the impulse above */
  const double reaching = std::sqrt(2.0 * 9.81 * (top - z_start));
  EXPECT_LT(std::abs(velocity.lin.z), 0.5 * reaching);
}

TEST(DemModel, WedgeHullSplitsGrains) {
  WedgeSplitsGrains("");
}

/* the same with non-smooth contact */
TEST(DemModel, NscWedgeHullSplitsGrains) {
  WedgeSplitsGrains(R"("contact_method": "nsc",)");
}

/* whether the model's GPU device can be made: false, with why, if not */
static bool HasGpu(std::string& why) {
  if(!shift::core::gpu::Device::is_built()) {
    why = "built without Vulkan";
    return false;
  }
  try {
    shift::core::Context context;
    context.GetGpuDevice();
  } catch(const shift::core::Exception& e) {
    why = std::string("no GPU device: ") + e.what();
    return false;
  }
  return true;
}

#define REQUIRE_GPU()                                                         \
  {                                                                           \
    std::string why;                                                          \
    if(!HasGpu(why)) {                                                        \
      if(std::getenv("SHIFT_GPU_REQUIRED")) {                                 \
        FAIL() << why;                                                        \
      }                                                                       \
      GTEST_SKIP() << why;                                                    \
    }                                                                         \
  }

/* the plate of DemModel.NscPlateSettlesOnGrains on the GPU: the non-smooth
method there, with the plate taking part in its steps */
TEST(DemModel, GpuNscPlateSettlesOnGrains) {
  REQUIRE_GPU();
  PlateSettles("gpu", R"("contact_method": "nsc",)");
}

/* a rock: an icosahedron with its faces divided in four (80 triangles, at
most six to a vertex, so that no grain is near too many of them), stretched
to semi-axes of 0.2, 0.15 and 0.12 m and made lumpy by a few waves of its
radius (of up to 25 %), as an OBJ; its convex hull is the rock the grains
touch */
static std::string RockObj() {
  const double t = (1.0 + std::sqrt(5.0)) / 2.0;
  std::vector<std::array<double, 3>> v = {
    {-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
    {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
  std::vector<std::array<int, 3>> f = {
    {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
    {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
    {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
    {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
  /* each face in four, at the midpoints of its edges */
  std::vector<std::array<int, 3>> divided;
  std::vector<std::pair<std::pair<int, int>, int>> midpoints;
  auto midpoint = [&](int a, int b) {
    const std::pair<int, int> key = {std::min(a, b), std::max(a, b)};
    for(const auto& m : midpoints) {
      if(m.first == key) {
        return m.second;
      }
    }
    v.push_back({(v[a][0] + v[b][0]) / 2.0, (v[a][1] + v[b][1]) / 2.0,
                 (v[a][2] + v[b][2]) / 2.0});
    midpoints.push_back({key, int(v.size()) - 1});
    return int(v.size()) - 1;
  };
  for(const auto& face : f) {
    const int ab = midpoint(face[0], face[1]);
    const int bc = midpoint(face[1], face[2]);
    const int ca = midpoint(face[2], face[0]);
    divided.push_back({face[0], ab, ca});
    divided.push_back({face[1], bc, ab});
    divided.push_back({face[2], ca, bc});
    divided.push_back({ab, bc, ca});
  }
  std::ostringstream obj;
  obj.precision(9);
  for(const auto& p : v) {
    const double n = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    const double x = p[0] / n, y = p[1] / n, z = p[2] / n;
    const double r = 1.0 + 0.12 * std::sin(3.0 * x + 1.0) *
                             std::cos(2.0 * y + 0.3) +
                     0.08 * std::cos(4.0 * z + 0.5) +
                     0.05 * std::sin(5.0 * (x + y));
    obj << "v " << 0.2 * r * x << " " << 0.15 * r * y << " " << 0.12 * r * z
        << "\n";
  }
  for(const auto& face : divided) {
    obj << "f " << face[0] + 1 << " " << face[1] + 1 << " " << face[2] + 1
        << "\n";
  }
  return obj.str();
}

/* the time a model takes to simulate, and its substeps */
static void Report(const char* what, shift::core::force_module::Dem* dem,
                   double seconds) {
  const auto stats = dem->GetStatistics();
  std::printf("%s: %zu grains, %.4g s in %llu substeps of %.3g s, %llu "
              "searches: %.1f s, %.3g ms a substep, %.3g grain steps a "
              "second\n",
              what, dem->get_grain_count(), dem->get_time(),
              (unsigned long long)stats.steps, dem->get_step(),
              (unsigned long long)stats.searches, seconds,
              1e3 * seconds / double(std::max<uint64_t>(stats.steps, 1)),
              double(dem->get_grain_count()) * double(stats.steps) / seconds);
}

/* the grains are all where they can be: finite, in the box half wide and
above the floor but for overlaps */
static void ExpectInBox(const shift::core::dem::State& grains, double half) {
  size_t out = 0;
  for(size_t g = 0; g < grains.size(); g++) {
    const auto& p = grains.position[g];
    if(!(std::abs(p[0]) < half && std::abs(p[1]) < half && p[2] < 0.0)) {
      out++;
    }
  }
  EXPECT_EQ(out, 0u);
}

/* a rock of granite, 41 kg, its collision geometry the convex hull of a
lumpy ellipsoid's mesh, is thrown at 2 m/s, tumbling, into a bed of 186,000
beads in a box of a meter, 0.25 m deep, on the GPU: a benchmark (meson
benchmark dem_rock), not run with the tests. The rock sinks into the bed,
its center below the top of it, and the grains' impulse on it is its
weight's and the momentum it lost */
TEST(DemSpeed, RockInGrains) {
  REQUIRE_GPU();
  const double half = 0.5, time_span = 0.3;
  /* the mass and the moments of inertia of the ellipsoid, 2700 kg/m^3 */
  const double a = 0.2, b = 0.15, c = 0.12;
  const double mass = 2700.0 * 4.0 / 3.0 * 3.14159265358979323846 * a * b * c;
  const double z_start = -0.26 - 0.2;
  std::ostringstream bodies, joints;
  bodies.precision(17);
  joints.precision(17);
  bodies << R"([{"name": "rock", "center_of_mass": [0, 0, 0],
    "geometry": [{"name": "rock_mesh", "type": "mesh",
                  "filename": ["dem_rock.obj"]}],
    "collision_geometry": [{"name": "rock", "type": "convex_hull",
                            "contact": false}],
    "inertia": {"mass": )"
         << mass << R"(, "Ixx": )" << mass / 5.0 * (b * b + c * c)
         << R"(, "Iyy": )" << mass / 5.0 * (a * a + c * c) << R"(, "Izz": )"
         << mass / 5.0 * (a * a + b * b)
         << R"(, "Ixy": 0, "Iyz": 0, "Izx": 0}}])";
  /* tilted 0.3 rad about x */
  joints << R"([{"name": "free", "type": "float",
    "parent": [["fixed"], "origin"], "child": [["rock"], "origin"],
    "initial_displacement": {"T": [0, 0, )"
         << z_start << R"(], "R": [[1, 0, 0], [0, )" << std::cos(0.3) << ", "
         << -std::sin(0.3) << "], [0, " << std::sin(0.3) << ", "
         << std::cos(0.3) << R"(]]},
    "initial_velocity": [0, 0, 2, 1, 3, 0]}])";
  shift::core::Context context;
  LoadGrainsModel(
    context, "dem_rock", "gpu", time_span, 1e-4, 3.0, half,
    R"([{"grain": "bead", "min": [-0.495, -0.495, -0.253],
         "max": [0.495, 0.495, 0], "spacing": 0.011, "jitter": 0.04,
         "seed": 5}])",
    bodies.str(), joints.str(),
    R"([{"body": "rock", "geometry": ["rock"], "material": "steel"}])",
    {{"dem_rock.obj", RockObj()}},
    /* at a vertex of the hull a bead is near up to 26 of its triangles */
    R"("max_triangles": 32,)");
  shift::core::force_module::Dem* dem = GetDem(context);
  ASSERT_NE(dem, nullptr);
  const auto start = std::chrono::steady_clock::now();
  context.Simulate();
  const double seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
  Report("rock", dem, seconds);
  shift::core::body::Base* rock = GetBody(context, "rock");
  ASSERT_NE(rock, nullptr);
  const double z = context.get_body_origin_pose(rock).t.z;
  const double impulse = -dem->get_body_impulse(0).lin.z;
  const ksl_screw_t velocity =
    context.GetBodyVelocity(context.GetBodyLoadIndex(rock));
  /* the weight's impulse and the momentum lost, from 2 m/s along the
  rock's tilted z axis */
  const double expected =
    mass * 9.81 * dem->get_time() + mass * (2.0 * std::cos(0.3) - velocity.lin.z);
  shift::core::dem::State grains;
  dem->GetGrains(grains);
  double top = 0.0;
  for(size_t g = 0; g < grains.size(); g++) {
    if(std::hypot(grains.position[g][0], grains.position[g][1]) > 0.35) {
      top = std::min(top, grains.position[g][2] - 0.005);
    }
  }
  std::printf("%zu triangles; the rock's center at z = %.4f m, from %.4f "
              "m; the bed's top away from it at %.4f m; the grains' impulse "
              "on it %.4g N s, of its weight and momentum %.4g N s; speed "
              "%.3g m/s\n",
              dem->get_body_triangle_count(0), z, z_start, top, impulse,
              expected, velocity.lin.z);
  EXPECT_EQ(grains.size(), 186300u);
  ExpectInBox(grains, half);
  EXPECT_GT(dem->get_time(), time_span - 0.01);
  /* in the bed: its center below the top of it */
  EXPECT_GT(z, top);
  EXPECT_NEAR(impulse, expected, 0.03 * expected);
}

/* a million beads, 100 by 100 by 100 in a box of 1.1 m, fall into place
over 0.05 s on the GPU: a benchmark (meson benchmark dem_million), not run
with the tests. A small plate on a slide, away from the grains, is the
model's one body */
TEST(DemSpeed, MillionGrains) {
  REQUIRE_GPU();
  const double half = 0.56;
  shift::core::Context context;
  LoadGrainsModel(
    context, "dem_million", "gpu", 0.05, 1e-3, 2.0, half,
    R"([{"grain": "bead", "min": [-0.55, -0.55, -1.1],
         "max": [0.55, 0.55, 0], "spacing": 0.011, "jitter": 0.04,
         "seed": 9}])",
    R"([{"name": "plate", "center_of_mass": [0, 0, -2],
         "inertia": {"mass": 1, "Ixx": 1e-3, "Iyy": 1e-3, "Izz": 1e-3,
                     "Ixy": 0, "Iyz": 0, "Izx": 0}}])",
    R"([{"name": "slide", "type": "prismatic",
         "parent": [["fixed"], "origin"], "child": [["plate"], "origin"],
         "axis": [0, 0, 1]}])",
    "[]", {});
  shift::core::force_module::Dem* dem = GetDem(context);
  ASSERT_NE(dem, nullptr);
  ASSERT_EQ(dem->get_grain_count(), 1000000u);
  const shift::core::dem::State lattice = dem->get_initial_grains();
  const auto start = std::chrono::steady_clock::now();
  context.Simulate();
  const double seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
  Report("a million grains", dem, seconds);
  shift::core::dem::State grains;
  dem->GetGrains(grains);
  ASSERT_EQ(grains.size(), 1000000u);
  ExpectInBox(grains, half);
  /* the lattice leaves a millimeter between layers, 0.1 m in all, so the
  top grains fall freely over the run while the bottom ones rest on the
  floor */
  double fall = 0.0, top_before = 0.0, top_after = 0.0;
  for(size_t g = 0; g < grains.size(); g++) {
    fall = std::max(fall, grains.position[g][2] - lattice.position[g][2]);
    top_before = std::min(top_before, lattice.position[g][2]);
    top_after = std::min(top_after, grains.position[g][2]);
  }
  std::printf("the top grain from z = %.4f m to %.4f m; the largest fall "
              "%.4g mm\n",
              top_before, top_after, 1e3 * fall);
  EXPECT_GT(dem->get_time(), 0.05 - 0.01);
  const double t = dem->get_time();
  EXPECT_GT(top_after, top_before);
  EXPECT_NEAR(fall, 0.5 * 9.81 * t * t, 0.02 * 0.5 * 9.81 * t * t);
}

/* the mesh of a box of half extents about its center, its faces outward */
static std::string BoxObj(double hx, double hy, double hz) {
  std::ostringstream obj;
  obj.precision(9);
  for(int k = 0; k < 8; k++) {
    obj << "v " << (k & 1 ? hx : -hx) << " " << (k & 2 ? hy : -hy) << " "
        << (k & 4 ? hz : -hz) << "\n";
  }
  /* each face's corners counterclockwise about its outward normal */
  const int faces[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4},
                           {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
  for(const auto& f : faces) {
    obj << "f " << f[0] + 1 << " " << f[1] + 1 << " " << f[2] + 1 << "\n";
    obj << "f " << f[0] + 1 << " " << f[2] + 1 << " " << f[3] + 1 << "\n";
  }
  return obj.str();
}

/* the mesh of a template's hull */
static std::string HullObj(const shift::core::dem::Template& t) {
  std::ostringstream obj;
  obj.precision(9);
  for(const auto& v : t.mesh_vertices) {
    obj << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
  }
  for(const auto& tri : t.mesh_triangles) {
    obj << "f " << tri[0] + 1 << " " << tri[1] + 1 << " " << tri[2] + 1
        << "\n";
  }
  return obj.str();
}

/* 100,000 rocks of glass (2650 kg/m^3), convex hulls of 16 shapes of
RockPoints about 1 cm across, fall from a lattice of random orientations
into a box of 0.6 m and settle for 1.5 s; then a steel block of 20 kg, 16 by
16 by 10 cm, on a vertical slide, is dropped from 0.1 m above them and
makes a crater over 0.5 s, on the GPU: a benchmark (meson benchmark
dem_rocks), not run with the tests. The settled bed is at rest; the block
sinks into it, its bottom below the top of the bed around it, no rock
inside it, still creeping down at about 1% of the speed it hit at; and the
rocks' impulse on it is its weight's over the time less its momentum */
TEST(DemSpeed, BlockOnRocks) {
  REQUIRE_GPU();
  const int shapes = 16;
  const size_t count = 100000;
  const double half = 0.3;
  const std::string directory = testing::TempDir();
  const std::string lattice_file = directory + "/dem_rocks_lattice.csv";
  const std::string settled_file = directory + "/dem_rocks_settled.csv";
  std::vector<shift::core::dem::Template> rocks;
  std::vector<std::string> names;
  std::vector<std::pair<std::string, std::string>> files;
  std::ostringstream grains;
  grains.precision(9);
  grains << "[";
  float bounding = 0.0f;
  for(int k = 0; k < shapes; k++) {
    rocks.push_back(shift::core::dem::Template::OfHull(
      shift::core::dem::RockPoints(100 + k, 0.005f, 16, 0.3f), 2650.0f, 0));
    const shift::core::dem::Template& t = rocks.back();
    names.push_back("rock_" + std::to_string(k));
    files.push_back({"dem_" + names.back() + ".obj", HullObj(t)});
    bounding = std::max(bounding, t.hull.BoundingRadius());
    grains << (k ? ",\n" : "") << R"({"name": ")" << names.back()
           << R"(", "material": "glass", "hull": "dem_)" << names.back()
           << R"(.obj", "mass": )" << t.mass << R"(, "inertia": [)"
           << t.inertia[0] << ", " << t.inertia[1] << ", " << t.inertia[2]
           << "]}";
  }
  grains << "]";
  /* the lattice from the floor up, of random shapes and orientations */
  {
    const double spacing = 2.0 * bounding + 0.0005;
    const int across = static_cast<int>(2.0 * half / spacing);
    const double start = -0.5 * across * spacing + 0.5 * spacing;
    uint32_t seed = 1;
    auto random = [&seed]() {
      seed = seed * 1664525u + 1013904223u;
      return (seed >> 8) / 16777216.0f;
    };
    shift::core::dem::State lattice;
    for(size_t g = 0; g < count; g++) {
      const size_t layer = g / (across * across), cell = g % (across * across);
      lattice.AddGrain(static_cast<uint32_t>(random() * shapes) % shapes,
                       {start + (cell % across) * spacing,
                        start + (cell / across) * spacing,
                        -(layer + 0.5) * spacing - 0.001});
      /* a random axis and angle */
      const float z = 2.0f * random() - 1.0f, a = 6.2831853f * random();
      const float r = std::sqrt(1.0f - z * z), angle = 3.1415927f * random();
      const float s = std::sin(angle);
      lattice.orientation.back() = {s * r * std::cos(a), s * r * std::sin(a),
                                    s * z, std::cos(angle)};
    }
    shift::core::dem::WriteGrains(lattice_file, lattice, names);
  }
  /* the settling, a far plate on a hinge the model's one body */
  double settle_seconds = 0.0, mean = 0.0;
  {
    shift::core::Context context;
    LoadGrainsModel(
      context, "dem_rocks_settle", "gpu", 1.5, 1e-3, 5.0, half,
      R"([{"file": ")" + lattice_file + R"("}])",
      R"([{"name": "plate", "center_of_mass": [0, 0, -5],
           "inertia": {"mass": 1, "Ixx": 1e-3, "Iyy": 1e-3, "Izz": 1e-3,
                       "Ixy": 0, "Iyz": 0, "Izx": 0}}])",
      R"([{"name": "hold", "type": "revolute",
           "parent": [["fixed"], "origin"], "child": [["plate"], "origin"],
           "axis": [0, 0, 1]}])",
      "[]", files,
      R"("max_neighbors": 32, "save": ")" + settled_file + R"(",)",
      grains.str());
    shift::core::force_module::Dem* dem = GetDem(context);
    ASSERT_NE(dem, nullptr);
    ASSERT_EQ(dem->get_grain_count(), count);
    const auto start = std::chrono::steady_clock::now();
    context.Simulate();
    settle_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
        .count();
    Report("rocks settling", dem, settle_seconds);
    shift::core::dem::State settled;
    dem->GetGrains(settled);
    ExpectInBox(settled, half);
    for(size_t g = 0; g < settled.size(); g++) {
      const auto& v = settled.velocity[g];
      mean += std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) /
              double(settled.size());
    }
  }
  std::remove(lattice_file.c_str());
  /* the block over the bed: its center at the slide's displacement, its
  bottom 0.05 m below that */
  const double mass = 20.0, hx = 0.08, hz = 0.05;
  const double drop = -0.315;
  files.push_back({"dem_rocks_block.obj", BoxObj(hx, hx, hz)});
  std::ostringstream bodies, joints;
  bodies.precision(17);
  joints.precision(17);
  bodies << R"([{"name": "block", "center_of_mass": [0, 0, 0],
    "geometry": [{"name": "block_mesh", "type": "mesh",
                  "filename": ["dem_rocks_block.obj"]}],
    "inertia": {"mass": )"
         << mass << R"(, "Ixx": )" << mass / 3.0 * (hx * hx + hz * hz)
         << R"(, "Iyy": )" << mass / 3.0 * (hx * hx + hz * hz)
         << R"(, "Izz": )" << mass / 3.0 * (2.0 * hx * hx)
         << R"(, "Ixy": 0, "Iyz": 0, "Izx": 0}}])";
  joints << R"([{"name": "slide", "type": "prismatic",
    "parent": [["fixed"], "origin"], "child": [["block"], "origin"],
    "axis": [0, 0, 1], "initial_displacement": )"
         << drop << "}]";
  shift::core::Context context;
  LoadGrainsModel(
    context, "dem_rocks", "gpu", 0.5, 4e-5, 5.0, half,
    R"([{"file": ")" + settled_file + R"("}])", bodies.str(), joints.str(),
    R"([{"body": "block", "geometry": ["block_mesh"], "material": "steel"}])",
    files, R"("max_neighbors": 32,)", grains.str());
  shift::core::force_module::Dem* dem = GetDem(context);
  ASSERT_NE(dem, nullptr);
  const auto start = std::chrono::steady_clock::now();
  context.Simulate();
  const double seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
  std::remove(settled_file.c_str());
  Report("block on rocks", dem, seconds);
  shift::core::body::Base* block = GetBody(context, "block");
  ASSERT_NE(block, nullptr);
  const double z = context.get_body_origin_pose(block).t.z;
  const double speed =
    context.GetBodyVelocity(context.GetBodyLoadIndex(block)).lin.z;
  const double impulse = -dem->get_body_impulse(0).lin.z;
  /* the weight's impulse and the momentum it has */
  const double expected = mass * 9.81 * dem->get_time() - mass * speed;
  shift::core::dem::State end;
  dem->GetGrains(end);
  double top = 0.0;
  size_t inside = 0;
  for(size_t g = 0; g < end.size(); g++) {
    const auto& p = end.position[g];
    const double r = std::hypot(p[0], p[1]);
    if(r > 0.16 && r < 0.25) {
      top = std::min(top, p[2]);
    }
    if(std::abs(p[0]) < hx && std::abs(p[1]) < hx &&
       std::abs(p[2] - z) < hz) {
      inside++;
    }
  }
  std::printf("settled: mean speed %.3g mm/s (%.1f s); the block from z = "
              "%.4f m to %.4f m, its bottom %.4f m, the top rock around it "
              "%.4f m; %zu rocks inside it; speed %.3g m/s; the rocks' "
              "impulse on it %.4g N s, of its weight and momentum %.4g N s\n",
              1e3 * mean, settle_seconds, drop, z, z + hz, top, inside, speed,
              impulse, expected);
  ExpectInBox(end, half);
  EXPECT_LT(mean, 1e-3);
  EXPECT_GT(dem->get_time(), 0.5 - 0.01);
  /* in the bed: its bottom below the top of the bed around it */
  EXPECT_GT(z + hz, top);
  EXPECT_EQ(inside, 0u);
  EXPECT_LT(std::abs(speed), 0.02 * std::sqrt(2.0 * 9.81 * 0.1));
  EXPECT_NEAR(impulse, expected, 0.03 * expected);
}

/* grains of a convex hull from a mesh file, and a clump drawn as that
mesh: the hull is the cube's, the clump keeps its spheres and takes the
file's triangles to draw; cubes dropped in a row come to rest flat on the
floor */
TEST(DemModel, HullGrainsFromFile) {
  const std::string cube = R"(v -0.005 -0.005 -0.005
v 0.005 -0.005 -0.005
v 0.005 0.005 -0.005
v -0.005 0.005 -0.005
v -0.005 -0.005 0.005
v 0.005 -0.005 0.005
v 0.005 0.005 0.005
v -0.005 0.005 0.005
f 1 3 2
f 1 4 3
f 5 6 7
f 5 7 8
f 1 2 6
f 1 6 5
f 2 3 7
f 2 7 6
f 3 4 8
f 3 8 7
f 4 1 5
f 4 5 8
)";
  /* a cube of 1 cm of glass, 2500 kg/m^3 */
  const std::string inertia = "[4.1667e-08, 4.1667e-08, 4.1667e-08]";
  shift::core::Context context;
  LoadGrainsModel(
    context, "dem_hull_file", "cpu", 0.15, 1e-3, 1.0, 0.03,
    R"([{"grain": "cube", "min": [-0.03, -0.01, -0.028],
         "max": [0.03, 0.01, -0.008], "spacing": 0.02, "jitter": 0}])",
    R"([{"name": "plate", "center_of_mass": [0, 0, -2],
         "inertia": {"mass": 1, "Ixx": 1e-3, "Iyy": 1e-3, "Izz": 1e-3,
                     "Ixy": 0, "Iyz": 0, "Izx": 0}}])",
    R"([{"name": "slide", "type": "prismatic",
         "parent": [["fixed"], "origin"], "child": [["plate"], "origin"],
         "axis": [0, 0, 1]}])",
    "[]", {{"dem_hull_cube.obj", cube}}, "",
    R"([{"name": "cube", "material": "glass", "hull": "dem_hull_cube.obj",
         "mass": 0.0025, "inertia": )" +
      inertia + R"(},
        {"name": "clump", "mesh": "dem_hull_cube.obj", "mass": 0.0025,
         "inertia": )" +
      inertia + R"(, "spheres": [{"center": [0, 0, 0], "radius": 0.005,
                                  "material": "glass"}]}])");
  shift::core::force_module::Dem* dem = GetDem(context);
  ASSERT_NE(dem, nullptr);
  const auto& templates = dem->get_templates();
  ASSERT_EQ(templates.size(), 2u);
  EXPECT_EQ(templates[0].hull.vertices.size(), 8u);
  EXPECT_EQ(templates[0].hull.planes.size(), 6u);
  EXPECT_EQ(templates[0].mesh_triangles.size(), 12u);
  EXPECT_TRUE(templates[1].hull.vertices.empty());
  EXPECT_EQ(templates[1].spheres.size(), 1u);
  EXPECT_EQ(templates[1].mesh_triangles.size(), 12u);
  ASSERT_EQ(dem->get_grain_count(), 3u);
  context.Simulate();
  shift::core::dem::State grains;
  dem->GetGrains(grains);
  for(size_t g = 0; g < grains.size(); g++) {
    const auto& q = grains.orientation[g];
    std::printf("cube %zu: %.4g mm above the floor, turned by %.3g "
                "(q %.3g %.3g %.3g)\n",
                g, -1e3 * grains.position[g][2],
                std::abs(q[0]) + std::abs(q[1]) + std::abs(q[2]), q[0], q[1],
                q[2]);
    EXPECT_NEAR(-grains.position[g][2], 0.005, 5e-5);
    EXPECT_LT(std::abs(q[0]) + std::abs(q[1]) + std::abs(q[2]), 1e-3f);
  }
}

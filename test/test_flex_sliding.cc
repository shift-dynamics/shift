/* a prismatic joint sliding through a flexible body: a steel tube (outer
radius 20 mm, bore 10 mm, 60 mm long, an OBJ written here) swings on a
revolute joint about x at a frame inside its wall (tied to the nodes within
4 mm, clear of the bore), and a rigid rod slides along its bore on a
prismatic joint from a frame on the bore's axis. The rod (an OBJ written
here) bears on the bore with two lands, 8 mm long and 16 mm apart, a groove
between them. The bore is found from the frame and cut into stations, and
the lands found from the rod's surface; the rod moves with the mean motion
of the wall under its lands: its velocity and quadratic acceleration match
differences of its pose and velocity along a motion of every coordinate;
the energy is kept while a land slides out past the bore's end; and RNCA,
ABA and DCA give the same accelerations. */

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "ksl/SE3.h"
#include "ksl/screw.h"

#include "shift/config.h"
#include "shift/core/body/flex.h"
#include "shift/core/context.h"
#include "shift/core/integrator/base.h"
#include "shift/core/solver/base.h"

#if SHIFT_HAVE_FEA

static const int kSides = 24;
static const double kOuter = 0.02, kInner = 0.01, kLength = 0.06;
/* the rod's lands, along its axis from its frame, their radius and that of
the groove and ends */
static const double kLands[2][2] = {{-0.012, -0.004}, {0.004, 0.012}};
static const double kLand = 0.0095, kGroove = 0.008;

/* the tube's triangles, each with its own three vertices, normals out */
static void WriteTube(const std::string& path) {
  std::string vertices, faces;
  int count = 0;
  auto add = [&](std::array<double, 3> a, std::array<double, 3> b,
                 std::array<double, 3> c, std::array<double, 3> out) {
    const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const double v[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    const double n[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                         u[0] * v[1] - u[1] * v[0]};
    if(n[0] * out[0] + n[1] * out[1] + n[2] * out[2] < 0.0) {
      std::swap(b, c);
    }
    for(const auto& p : {a, b, c}) {
      vertices += "v " + std::to_string(p[0]) + " " + std::to_string(p[1]) +
                  " " + std::to_string(p[2]) + "\n";
    }
    faces += "f " + std::to_string(count + 1) + " " +
             std::to_string(count + 2) + " " + std::to_string(count + 3) +
             "\n";
    count += 3;
  };
  auto quad = [&](std::array<double, 3> a, std::array<double, 3> b,
                  std::array<double, 3> c, std::array<double, 3> d,
                  std::array<double, 3> out) {
    add(a, b, c, out);
    add(a, c, d, out);
  };
  auto at = [](double r, int i, double z) {
    const double a = 2.0 * M_PI * i / kSides;
    return std::array<double, 3>{r * std::cos(a), r * std::sin(a), z};
  };
  for(int i = 0; i < kSides; i++) {
    const double a = 2.0 * M_PI * (i + 0.5) / kSides;
    const std::array<double, 3> radial = {std::cos(a), std::sin(a), 0.0};
    const std::array<double, 3> inward = {-radial[0], -radial[1], 0.0};
    quad(at(kInner, i, kLength), at(kOuter, i, kLength),
         at(kOuter, i + 1, kLength), at(kInner, i + 1, kLength), {0, 0, 1});
    quad(at(kInner, i, 0), at(kOuter, i, 0), at(kOuter, i + 1, 0),
         at(kInner, i + 1, 0), {0, 0, -1});
    quad(at(kOuter, i, 0), at(kOuter, i + 1, 0), at(kOuter, i + 1, kLength),
         at(kOuter, i, kLength), radial);
    quad(at(kInner, i, 0), at(kInner, i + 1, 0), at(kInner, i + 1, kLength),
         at(kInner, i, kLength), inward);
  }
  std::ofstream file(path);
  file << vertices << faces;
}

/* the rod: rings along z from (z, r) and its two end caps */
static void WriteRod(const std::string& path) {
  const std::vector<std::array<double, 2>> rings = {
    {kLands[0][0], kGroove}, {kLands[0][0], kLand}, {kLands[0][1], kLand},
    {kLands[0][1], kGroove}, {kLands[1][0], kGroove}, {kLands[1][0], kLand},
    {kLands[1][1], kLand},   {kLands[1][1], kGroove}};
  std::ofstream file(path);
  for(const auto& ring : rings) {
    for(int i = 0; i < kSides; i++) {
      const double a = 2.0 * M_PI * i / kSides;
      file << "v " << ring[1] * std::cos(a) << " " << ring[1] * std::sin(a)
           << " " << ring[0] << "\n";
    }
  }
  const int ends = static_cast<int>(rings.size()) * kSides;
  file << "v 0 0 " << rings.front()[0] << "\nv 0 0 " << rings.back()[0]
       << "\n";
  for(int k = 0; k + 1 < static_cast<int>(rings.size()); k++) {
    for(int i = 0; i < kSides; i++) {
      const int a = k * kSides + i + 1, b = k * kSides + (i + 1) % kSides + 1;
      const int c = b + kSides, d = a + kSides;
      file << "f " << a << " " << b << " " << c << "\nf " << a << " " << c
           << " " << d << "\n";
    }
  }
  for(int i = 0; i < kSides; i++) {
    const int a = i + 1, b = (i + 1) % kSides + 1;
    const int last = (static_cast<int>(rings.size()) - 1) * kSides;
    file << "f " << ends + 1 << " " << b << " " << a << "\nf " << ends + 2
         << " " << last + a << " " << last + b << "\n";
  }
}

static std::string Model(const std::string& solver, const std::string& obj,
                         const std::string& rod, const std::string& cache) {
  return R"({"model_parameters": {"version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0, 0, 0], "solvers": [{"type": ")" +
         solver + R"("}],
    "integrators": [{"type": "ode113", "time_span": 0.004,
                     "time_step": 0.001, "relative_error": 1e-10,
                     "absolute_error": 1e-12}]},
    "assemblies": [{"name": "test", "bodies": [
      {"name": "barrel", "type": "modal", "mode_count": 2,
       "density": 7850, "youngs_modulus": 2.1e11, "poisson_ratio": 0.3,
       "fe_geometry": "solid", "element_size": 0.01,
       "attachments": [{"frame": "base", "radius": 0.004}],
       "fe_cache": ")" +
         cache + R"(",
       "frames": [{"name": "base", "T": [0.0165, 0, 0.006]},
                  {"name": "slide", "T": [0, 0, 0.01]}],
       "geometry": [{"name": "solid", "type": "mesh",
                     "filename": [")" +
         obj + R"("]}]},
      {"name": "rod", "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 0.5, "Ixx": 1e-4, "Iyy": 1e-4, "Izz": 2e-5,
                   "Ixy": 0, "Iyz": 0, "Izx": 0},
       "geometry": [{"name": "lands", "type": "mesh",
                     "filename": [")" +
         rod + R"("]}]}],
    "joints": [
      {"name": "hinge", "type": "revolute", "axis": [1, 0, 0],
       "initial_velocity": 2.0,
       "parent": [["fixed"], "origin"], "child": [["barrel"], "base"]},
      {"name": "slider", "type": "prismatic", "axis": [0, 0, 1],
       "initial_displacement": 0.037, "initial_velocity": 1.0,
       "parent": [["barrel"], "slide"], "child": [["rod"], "origin"]}]}]})";
}

/* the model loaded with \p solver; the tube's analysis is kept in a cache
that the first load writes and the others read */
static std::unique_ptr<shift::core::Context> Load(const std::string& solver) {
  const std::string dir = testing::TempDir();
  const std::string obj = dir + "/sliding_tube.obj";
  const std::string rod = dir + "/sliding_rod.obj";
  WriteTube(obj);
  WriteRod(rod);
  const std::string path = dir + "/sliding_" + solver + ".json";
  {
    std::ofstream file(path);
    file << Model(solver, obj, rod, dir + "/sliding_tube.fe.h5");
  }
  auto context = std::make_unique<shift::core::Context>();
  try {
    context->LoadJSON(path);
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
  std::remove(path.c_str());
  return context;
}

static shift::core::body::Base* Body(shift::core::Context& context,
                                     const std::string& name) {
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == name) {
      return body;
    }
  }
  return nullptr;
}

static ksl_screw_t SpatialVelocity(const ksl_SE3_t& d0, const ksl_SE3_t& d1,
                                   double dt) {
  ksl_SE3_t d10;
  ksl_product_ddinv(&d1, &d0, &d10);
  const ksl_mat3x3_t& w = d10.R;
  ksl_screw_t v;
  v.m3 = 0.5 * (w.m21 - w.m12) / dt;
  v.m4 = 0.5 * (w.m02 - w.m20) / dt;
  v.m5 = 0.5 * (w.m10 - w.m01) / dt;
  const double tx = 0.5 * (d0.t.x + d1.t.x);
  const double ty = 0.5 * (d0.t.y + d1.t.y);
  const double tz = 0.5 * (d0.t.z + d1.t.z);
  v.m0 = (d1.t.x - d0.t.x) / dt - (v.m4 * tz - v.m5 * ty);
  v.m1 = (d1.t.y - d0.t.y) / dt - (v.m5 * tx - v.m3 * tz);
  v.m2 = (d1.t.z - d0.t.z) / dt - (v.m3 * ty - v.m4 * tx);
  return v;
}

/* a deformed, moving state: the coordinates and rates of every joint and
elastic coordinate */
static void State(shift::core::Context& context, std::vector<double>* q,
                  std::vector<double>* dq) {
  q->assign(context.get_q().begin(),
            context.get_q().begin() + context.get_q_count());
  dq->assign(context.get_dq_count(), 0.0);
  auto barrel = Body(context, "barrel");
  const unsigned int e = barrel->get_elastic_q_index();
  for(unsigned int m = 0; m < barrel->get_elastic_count(); m++) {
    (*q)[e + m] = 1e-7 * std::sin(1.0 + 0.7 * m);
  }
  for(unsigned int k = 0; k < dq->size(); k++) {
    (*dq)[k] = 0.4 * std::cos(2.0 + 1.3 * k);
  }
  for(unsigned int m = 0; m < barrel->get_elastic_count(); m++) {
    (*dq)[barrel->get_elastic_dq_index() + m] = 1e-5 * std::cos(3.0 + m);
  }
}

TEST(FlexSliding, StationsAlongTheBore) {
  std::remove((testing::TempDir() + "/sliding_tube.fe.h5").c_str());
  auto context = Load("rnca");
  auto barrel = dynamic_cast<shift::core::body::Flex*>(Body(*context, "barrel"));
  ASSERT_NE(barrel, nullptr);
  EXPECT_FALSE(barrel->from_cache());
  /* the bore, 60 mm long, in as many stations as the mesh (10 mm) allows,
  as on so coarse a mesh more keep lowering the error: six, six static
  modes each (the hinge's frame holds the body) */
  EXPECT_EQ(barrel->get_static_count(), 6u * 6);
  ASSERT_EQ(barrel->get_node_count(), 2u);
  int sliding = 0;
  for(unsigned int k = 0; k < barrel->get_node_count(); k++) {
    sliding += barrel->is_sliding_node(k) ? 1 : 0;
    /* the rod's lands, from its surface (in single precision), and the
    bore's radius (the mean distance of its nodes from the axis, on a wall
    of 24 flats) and ends */
    if(const auto* bearing = barrel->get_sliding_bearing(k)) {
      ASSERT_EQ(bearing->intervals.size(), 2u);
      for(int i = 0; i < 2; i++) {
        EXPECT_NEAR(bearing->intervals[size_t(i)][0], kLands[i][0], 1e-7);
        EXPECT_NEAR(bearing->intervals[size_t(i)][1], kLands[i][1], 1e-7);
      }
      EXPECT_GT(bearing->radius, 0.95 * kInner);
      EXPECT_LT(bearing->radius, kInner);
      EXPECT_NEAR(bearing->low, -0.01, 1e-7);
      EXPECT_NEAR(bearing->high, kLength - 0.01, 1e-7);
    }
  }
  EXPECT_EQ(sliding, 1);
  const auto& slots = context->get_node_slots();
  int sliding_slots = 0;
  for(const auto& slot : slots) {
    sliding_slots += slot.slide_q >= 0 ? 1 : 0;
  }
  EXPECT_EQ(sliding_slots, 1);

  /* read again from the cache, the same */
  auto again = Load("rnca");
  auto cached = dynamic_cast<shift::core::body::Flex*>(Body(*again, "barrel"));
  EXPECT_TRUE(cached->from_cache());
  EXPECT_EQ(cached->get_static_count(), barrel->get_static_count());
  EXPECT_EQ(cached->get_stiffness(), barrel->get_stiffness());
}

/* undeformed, the rod is at s along the axis from the slide frame; deformed
and moving, its velocity and quadratic acceleration are the derivatives of
its pose and velocity */
TEST(FlexSliding, KinematicsMatchDifferences) {
  auto context = Load("rnca");
  auto rod = Body(*context, "rod");
  ASSERT_NE(rod, nullptr);
  const int frame = rod->get_reference_frame()->get_index();
  const int va = rod->get_va_index();

  std::vector<double> q0, dq;
  State(*context, &q0, &dq);
  {
    std::vector<double> q(context->get_q().begin(),
                          context->get_q().begin() + context->get_q_count());
    std::vector<double> rest(dq.size(), 0.0);
    context->UpdateDHVA(q.data(), rest.data());
    /* the hinge at 0: the slide frame at (0, 0, 0.01) less the base's
    offset, plus s = 0.037 along z */
    const ksl_SE3_t& d = context->get_dhva().d[frame];
    EXPECT_NEAR(d.t.x, -0.0165, 1e-12);
    EXPECT_NEAR(d.t.y, 0.0, 1e-12);
    EXPECT_NEAR(d.t.z, 0.01 - 0.006 + 0.037, 1e-12);
  }

  auto pose_at = [&](double t, ksl_screw_t* v, ksl_screw_t* aq) {
    std::vector<double> q(q0.size());
    for(size_t k = 0; k < q.size(); k++) {
      q[k] = q0[k] + dq[k] * t;
    }
    context->UpdateDHVA(q.data(), dq.data());
    const auto& dhva = context->get_dhva();
    if(v) {
      *v = dhva.v[va];
    }
    if(aq) {
      *aq = dhva.aq[va];
    }
    return dhva.d[frame];
  };
  ksl_screw_t v, aq;
  pose_at(0.0, &v, &aq);
  /* the rotation of a node is linear in eta, its rate taken as
  Upsilon^r eta_dot: they differ at second order in the deformation (3e-10
  here, 100 times more with eta and eta_dot ten times larger; the rod's
  rotation, from the translations of the wall over its lands, is some
  hundred times its stations' translations) */
  const double dt = 1e-6;
  const ksl_screw_t v_fd = SpatialVelocity(pose_at(-dt, nullptr, nullptr),
                                           pose_at(dt, nullptr, nullptr),
                                           2.0 * dt);
  for(int k = 0; k < 6; k++) {
    EXPECT_NEAR(v.at[k], v_fd.at[k], 1e-9) << "v " << k;
  }
  const double da = 1e-5;
  ksl_screw_t v_minus, v_plus;
  pose_at(-da, &v_minus, nullptr);
  pose_at(da, &v_plus, nullptr);
  for(int k = 0; k < 6; k++) {
    EXPECT_NEAR(aq.at[k], (v_plus.at[k] - v_minus.at[k]) / (2.0 * da), 1e-6)
      << "aq " << k;
  }
}

/* no gravity, damping or springs: the kinetic and elastic energies sum to a
constant as the rod slides and the tube swings */
TEST(FlexSliding, EnergyIsKept) {
  auto context = Load("rnca");
  auto barrel = dynamic_cast<shift::core::body::Flex*>(Body(*context, "barrel"));
  auto solver = (*context)["model_parameters"]["solvers"][0]
                  .get_data<shift::core::solver::Base>();
  const unsigned int n = barrel->get_elastic_count();
  const unsigned int e = barrel->get_elastic_q_index();
  auto energy = [&](const std::vector<double>& state) {
    std::vector<double> y(context->get_state_count() +
                            context->get_auxiliary_state_count(),
                          0.0);
    std::copy(state.begin(), state.end(), y.begin());
    std::vector<double> dy(y.size(), 0.0);
    EXPECT_FALSE(solver->Solve(0.0, y, dy));
    const auto& k = barrel->get_stiffness();
    double elastic = 0.0;
    for(unsigned int i = 0; i < n; i++) {
      for(unsigned int j = 0; j < n; j++) {
        elastic += 0.5 * state[e + i] * k[i * n + j] * state[e + j];
      }
    }
    return solver->KineticEnergy(context->get_dhva()) + elastic;
  };
  const std::vector<double> initial(
    context->get_q().begin(),
    context->get_q().begin() + context->get_state_count());
  std::vector<double> last;
  context->set_state_observer(
    [&last](double, const std::vector<double>& y) { last = y; });
  context->Simulate();
  ASSERT_GE(last.size(), context->get_state_count());
  last.resize(context->get_state_count());
  /* the rod moved along the bore, past the last station (at 0.04) */
  EXPECT_GT(last[context->get_q_count() - 1], 0.04);
  const double before = energy(initial);
  EXPECT_GT(before, 0.1);
  EXPECT_NEAR((energy(last) - before) / before, 0.0, 1e-6);
}

/* at a deformed, moving state, ABA and DCA give RNCA's accelerations */
TEST(FlexSliding, SolversAgree) {
  std::vector<std::vector<double>> results;
  for(const std::string name : {"rnca", "aba", "dca"}) {
    auto context = Load(name);
    std::vector<double> q, dq;
    State(*context, &q, &dq);
    std::vector<double> y(context->get_state_count() +
                            context->get_auxiliary_state_count(),
                          0.0);
    std::copy(q.begin(), q.end(), y.begin());
    std::copy(dq.begin(), dq.end(), y.begin() + long(q.size()));
    std::vector<double> dy(y.size(), 0.0);
    auto solver = (*context)["model_parameters"]["solvers"][0]
                    .get_data<shift::core::solver::Base>();
    context->get_active_integrators()[0]->Initialize();
    solver->Initialize();
    EXPECT_FALSE(solver->Solve(0.0, y, dy)) << name;
    results.push_back(dy);
  }
  double scale = 1.0;
  for(double value : results[0]) {
    scale = std::fmax(scale, std::fabs(value));
  }
  for(size_t s = 1; s < results.size(); s++) {
    ASSERT_EQ(results[s].size(), results[0].size());
    for(size_t i = 0; i < results[0].size(); i++) {
      EXPECT_NEAR(results[s][i], results[0][i], 1e-9 * scale)
        << (s == 1 ? "aba" : "dca") << " " << i;
    }
  }
  std::remove((testing::TempDir() + "/sliding_tube.fe.h5").c_str());
  std::remove((testing::TempDir() + "/sliding_tube.obj").c_str());
  std::remove((testing::TempDir() + "/sliding_rod.obj").c_str());
}

#endif

/* the weight of station i's translation along x in the child's at s, with
no bearing: the cardinal weights of the sliding spline */
static std::vector<double> Cardinal(size_t count, double first,
                                    double spacing, double s) {
  const std::vector<double> map = shift::core::body::Flex::SlidingMap(
    count, first, spacing, ksl_vec3(0.0, 0.0, 1.0), s);
  std::vector<double> w(count);
  for(size_t i = 0; i < count; i++) {
    w[i] = map[6 * i];
  }
  return w;
}

/* the cardinal weights of the sliding spline, with which the station
spacing is checked against the full model: one at its own station, zero at
the others; they sum to one and reproduce a line, inside and beyond the
ends; and they give the spline's value through any data */
TEST(FlexSliding, CardinalWeightsAreTheSplines) {
  const size_t count = 5;
  const double first = 0.01, spacing = 0.02;
  for(size_t k = 0; k < count; k++) {
    const std::vector<double> w =
      Cardinal(count, first, spacing, first + spacing * double(k));
    for(size_t i = 0; i < count; i++) {
      EXPECT_NEAR(w[i], i == k ? 1.0 : 0.0, 1e-14);
    }
  }
  const std::vector<double> y = {0.3, -0.1, 0.7, 0.2, 0.5};
  const std::vector<double> m =
    shift::core::body::Flex::SplineCurvatures(y, spacing);
  EXPECT_EQ(m.front(), 0.0);
  EXPECT_EQ(m.back(), 0.0);
  for(double s : {-0.01, 0.017, 0.04, 0.063, 0.1}) {
    const std::vector<double> w = Cardinal(count, first, spacing, s);
    double sum = 0.0, line = 0.0, value = 0.0;
    for(size_t i = 0; i < count; i++) {
      sum += w[i];
      line += w[i] * (2.0 - 3.0 * (first + spacing * double(i)));
      value += w[i] * y[i];
    }
    EXPECT_NEAR(sum, 1.0, 1e-14) << s;
    EXPECT_NEAR(line, 2.0 - 3.0 * s, 1e-14) << s;
    /* the natural spline between stations 1 and 2 at s = 0.04 (t = 0.5):
    the mean of the values less h^2 / 16 the mean of the curvatures */
    if(s == 0.04) {
      EXPECT_NEAR(value,
                  0.5 * (y[1] + y[2]) -
                    spacing * spacing / 16.0 * (m[1] + m[2]),
                  1e-14);
    }
  }
}

/* the child carried by the mean motion of the bore's wall under its
bearing (Flex::CarryShape), where the bearing lies within the bore,
overhangs either end, and spans it: stations moving rigidly carry the
child rigidly with them; for any motion of the stations, the child's is the
least-squares mean of the interpolated sections (integrals by Simpson's
rule on the point weights); and its derivatives in s are those of its
motion (central differences, to their truncation error) */
TEST(FlexSliding, BearingCarriesTheMeanMotion) {
  const size_t count = 6;
  const double first = 0.01, spacing = 0.02, radius = 0.008;
  const ksl_vec3_t axis = ksl_vec3(0.0, 0.0, 1.0);
  shift::core::body::SlidingBearing bearing;
  bearing.intervals = {{-0.012, -0.004}, {0.003, 0.015}};
  bearing.radius = radius;
  bearing.low = first - 0.5 * spacing;
  bearing.high = first + (double(count) - 0.5) * spacing;
  EXPECT_NEAR(shift::core::body::Flex::BearingCenter(bearing),
              (0.5 * (0.004 * 0.004 - 0.012 * 0.012) +
               0.5 * (0.015 * 0.015 - 0.003 * 0.003)) /
                0.020,
              1e-15);
  shift::core::body::SlidingBearing spanning = bearing;
  spanning.intervals = {{-0.2, 0.2}};
  /* a motion of the stations: their translations and rotations */
  std::vector<double> motion(6 * count);
  for(size_t k = 0; k < motion.size(); k++) {
    motion[k] = std::sin(1.0 + 0.9 * double(k));
  }
  const size_t n = 6 * count;
  auto apply = [&](const std::vector<double>& map,
                   const std::vector<double>& u) {
    std::array<double, 6> out = {};
    for(size_t r = 0; r < 6; r++) {
      for(size_t k = 0; k < n; k++) {
        out[r] += map[r * n + k] * u[k];
      }
    }
    return out;
  };
  for(const auto* b : {&bearing, &spanning}) {
    const double center = shift::core::body::Flex::BearingCenter(*b);
    for(double s : {0.043, 0.0057, 0.111, 0.0}) {
      const std::vector<double> map = shift::core::body::Flex::SlidingMap(
        count, first, spacing, axis, s, *b);
      /* rigid: u_i = t + w x (x_i a), theta_i = w */
      const double t[3] = {0.3, -0.2, 0.5}, w[3] = {0.7, -0.4, 0.25};
      std::vector<double> rigid(n);
      for(size_t i = 0; i < count; i++) {
        const double x = first + spacing * double(i);
        rigid[6 * i + 0] = t[0] + w[1] * x;
        rigid[6 * i + 1] = t[1] - w[0] * x;
        rigid[6 * i + 2] = t[2];
        for(int c = 0; c < 3; c++) {
          rigid[6 * i + 3 + size_t(c)] = w[c];
        }
      }
      const double x_c = s + center;
      const std::array<double, 6> carried = apply(map, rigid);
      const double expected[6] = {t[0] + w[1] * x_c, t[1] - w[0] * x_c, t[2],
                                  w[0], w[1], w[2]};
      for(int r = 0; r < 6; r++) {
        EXPECT_NEAR(carried[size_t(r)], expected[r], 1e-12) << s << " " << r;
      }
      /* the least-squares mean of the sections */
      double l = 0.0, g = 0.0, h = 0.0;
      double u[3] = {}, xu[3] = {}, th[3] = {};
      for(const auto& interval : b->intervals) {
        const double low = std::max(s + interval[0], b->low);
        const double high = std::min(s + interval[1], b->high);
        if(!(high > low)) {
          continue;
        }
        const int panels = 2000;
        const double step = (high - low) / panels;
        for(int p = 0; p <= panels; p++) {
          const double x = low + step * p;
          const double weight =
            step / 3.0 * (p == 0 || p == panels ? 1.0 : p % 2 ? 4.0 : 2.0);
          const std::array<double, 6> at = apply(
            shift::core::body::Flex::SlidingMap(count, first, spacing, axis,
                                                x),
            motion);
          for(int c = 0; c < 3; c++) {
            u[c] += weight * at[size_t(c)];
            xu[c] += weight * x * at[size_t(c)];
            th[c] += weight * at[3 + size_t(c)];
          }
        }
        l += high - low;
        g += 0.5 * (high * high - low * low);
        h += (high * high * high - low * low * low) / 3.0;
      }
      const double middle = g / l;
      const double d = h - g * middle + 0.5 * radius * radius * l;
      /* across the axis z: (a x (int x u - xbar int u) + r^2/2 int theta) */
      const double rotation[3] = {
        (-(xu[1] - middle * u[1]) + 0.5 * radius * radius * th[0]) / d,
        ((xu[0] - middle * u[0]) + 0.5 * radius * radius * th[1]) / d,
        th[2] / l};
      const double translation[3] = {
        u[0] / l + (x_c - middle) * rotation[1],
        u[1] / l - (x_c - middle) * rotation[0], u[2] / l};
      const std::array<double, 6> mean = apply(map, motion);
      for(int c = 0; c < 3; c++) {
        EXPECT_NEAR(mean[size_t(c)], translation[c], 1e-9) << s << " " << c;
        EXPECT_NEAR(mean[3 + size_t(c)], rotation[c], 1e-7) << s << " " << c;
      }
      /* the derivatives */
      const double ds = 2e-6;
      auto at = [&](double x, int derivative) {
        return apply(shift::core::body::Flex::SlidingMap(
                       count, first, spacing, axis, x, *b, derivative),
                     motion);
      };
      const std::array<double, 6> d1 = at(s, 1), d2 = at(s, 2);
      const std::array<double, 6> plus = at(s + ds, 0),
                                  minus = at(s - ds, 0);
      const std::array<double, 6> plus_1 = at(s + ds, 1),
                                  minus_1 = at(s - ds, 1);
      double size_1 = 1.0, size_2 = 1.0;
      for(int r = 0; r < 6; r++) {
        size_1 = std::max(size_1, std::abs(d1[size_t(r)]));
        size_2 = std::max(size_2, std::abs(d2[size_t(r)]));
      }
      for(size_t r = 0; r < 6; r++) {
        EXPECT_NEAR(d1[r], (plus[r] - minus[r]) / (2.0 * ds), 1e-6 * size_1)
          << s << " " << r;
        EXPECT_NEAR(d2[r], (plus_1[r] - minus_1[r]) / (2.0 * ds),
                    1e-6 * size_2)
          << s << " " << r;
      }
    }
  }
}

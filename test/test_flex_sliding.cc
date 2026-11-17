/* a prismatic joint sliding through a flexible body: a steel tube (outer
radius 20 mm, bore 10 mm, 60 mm long, an OBJ written here) swings on a
revolute joint about x at a frame inside its wall (tied to the nodes within
4 mm, clear of the bore), and a rigid rod slides
along its bore on a prismatic joint from a frame on the bore's axis. The
bore is found from the frame, cut into stations a bearing length apart, and
the rod's attachment slides between them: its velocity and quadratic
acceleration match differences of its pose and velocity along a motion of
every coordinate; the energy is kept while it slides past a station into
the spline's linear continuation; and RNCA, ABA and DCA give the same
accelerations. */

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

static std::string Model(const std::string& solver, const std::string& obj,
                         const std::string& cache) {
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
                   "Ixy": 0, "Iyz": 0, "Izx": 0}}],
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
  WriteTube(obj);
  const std::string path = dir + "/sliding_" + solver + ".json";
  {
    std::ofstream file(path);
    file << Model(solver, obj, dir + "/sliding_tube.fe.h5");
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
    (*q)[e + m] = 2e-6 * std::sin(1.0 + 0.7 * m);
  }
  for(unsigned int k = 0; k < dq->size(); k++) {
    (*dq)[k] = 0.4 * std::cos(2.0 + 1.3 * k);
  }
  for(unsigned int m = 0; m < barrel->get_elastic_count(); m++) {
    (*dq)[barrel->get_elastic_dq_index() + m] = 2e-4 * std::cos(3.0 + m);
  }
}

TEST(FlexSliding, StationsAlongTheBore) {
  std::remove((testing::TempDir() + "/sliding_tube.fe.h5").c_str());
  auto context = Load("rnca");
  auto barrel = dynamic_cast<shift::core::body::Flex*>(Body(*context, "barrel"));
  ASSERT_NE(barrel, nullptr);
  EXPECT_FALSE(barrel->from_cache());
  /* the bore, 60 mm long, in lengths of its diameter: three stations, six
  static modes each (the hinge's frame holds the body) */
  EXPECT_EQ(barrel->get_static_count(), 6u * 3);
  ASSERT_EQ(barrel->get_node_count(), 2u);
  int sliding = 0;
  for(unsigned int k = 0; k < barrel->get_node_count(); k++) {
    sliding += barrel->is_sliding_node(k) ? 1 : 0;
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
  Upsilon^r eta_dot: they differ at second order in the deformation (1e-10
  here, 1e-8 with eta and eta_dot ten times larger) */
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
}

#endif

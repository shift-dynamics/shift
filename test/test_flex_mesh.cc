/* flexible bodies built from closed triangle meshes (an OBJ file as
fe_geometry), written with each triangle's corners its own, as exporters
split them: a steel ring plate (outer radius 40 mm, bore 10 mm, 10 mm
thick, 32 sides) hangs on a revolute joint at its centre, whose bore is
found from the triangles alone; a bar's joint frame lies inside the
material, where only flat faces are near, and is a node tied to the elements
around it. */

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/config.h"
#include "shift/core/body/flex.h"
#include "shift/core/context.h"

#if SHIFT_HAVE_FEA

/* an OBJ of the triangles, each with three vertices of its own, turned so
that its normal has a positive part along outward */
class ObjWriter {
public:
  void Add(std::array<double, 3> a, std::array<double, 3> b,
           std::array<double, 3> c, std::array<double, 3> outward) {
    const std::array<double, 3> u = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const std::array<double, 3> v = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    const std::array<double, 3> n = {u[1] * v[2] - u[2] * v[1],
                                     u[2] * v[0] - u[0] * v[2],
                                     u[0] * v[1] - u[1] * v[0]};
    if(n[0] * outward[0] + n[1] * outward[1] + n[2] * outward[2] < 0.0) {
      std::swap(b, c);
    }
    for(const auto& p : {a, b, c}) {
      text_ += "v " + std::to_string(p[0]) + " " + std::to_string(p[1]) +
               " " + std::to_string(p[2]) + "\n";
    }
    faces_ += "f " + std::to_string(count_ + 1) + " " +
              std::to_string(count_ + 2) + " " + std::to_string(count_ + 3) +
              "\n";
    count_ += 3;
  }
  void Quad(std::array<double, 3> a, std::array<double, 3> b,
            std::array<double, 3> c, std::array<double, 3> d,
            std::array<double, 3> outward) {
    Add(a, b, c, outward);
    Add(a, c, d, outward);
  }
  void Write(const std::string& path) const {
    std::ofstream file(path);
    file << text_ << faces_;
  }

private:
  std::string text_, faces_;
  int count_ = 0;
};

static const int kSides = 32;
static const double kOuter = 0.04, kInner = 0.01, kThickness = 0.01;

static void WriteRing(const std::string& path) {
  ObjWriter obj;
  auto at = [](double r, int i, double z) {
    const double a = 2.0 * M_PI * i / kSides;
    return std::array<double, 3>{r * std::cos(a), r * std::sin(a), z};
  };
  for(int i = 0; i < kSides; i++) {
    const double a = 2.0 * M_PI * (i + 0.5) / kSides;
    const std::array<double, 3> radial = {std::cos(a), std::sin(a), 0.0};
    const std::array<double, 3> inward = {-radial[0], -radial[1], 0.0};
    const double h = kThickness;
    obj.Quad(at(kInner, i, h), at(kOuter, i, h), at(kOuter, i + 1, h),
             at(kInner, i + 1, h), {0, 0, 1});
    obj.Quad(at(kInner, i, 0), at(kOuter, i, 0), at(kOuter, i + 1, 0),
             at(kInner, i + 1, 0), {0, 0, -1});
    obj.Quad(at(kOuter, i, 0), at(kOuter, i + 1, 0), at(kOuter, i + 1, h),
             at(kOuter, i, h), radial);
    obj.Quad(at(kInner, i, 0), at(kInner, i + 1, 0), at(kInner, i + 1, h),
             at(kInner, i, h), inward);
  }
  obj.Write(path);
}

/* the box [0, 0.1] x [-0.01, 0.01] x [0, 0.02] */
static void WriteBar(const std::string& path) {
  ObjWriter obj;
  const double x[2] = {0.0, 0.1}, y[2] = {-0.01, 0.01}, z[2] = {0.0, 0.02};
  auto p = [&](int i, int j, int k) {
    return std::array<double, 3>{x[i], y[j], z[k]};
  };
  obj.Quad(p(0, 0, 0), p(0, 1, 0), p(0, 1, 1), p(0, 0, 1), {-1, 0, 0});
  obj.Quad(p(1, 0, 0), p(1, 1, 0), p(1, 1, 1), p(1, 0, 1), {1, 0, 0});
  obj.Quad(p(0, 0, 0), p(1, 0, 0), p(1, 0, 1), p(0, 0, 1), {0, -1, 0});
  obj.Quad(p(0, 1, 0), p(1, 1, 0), p(1, 1, 1), p(0, 1, 1), {0, 1, 0});
  obj.Quad(p(0, 0, 0), p(1, 0, 0), p(1, 1, 0), p(0, 1, 0), {0, 0, -1});
  obj.Quad(p(0, 0, 1), p(1, 0, 1), p(1, 1, 1), p(0, 1, 1), {0, 0, 1});
  obj.Write(path);
}

/* a model of the body on a revolute joint about z at its frame "pin" */
static void Load(shift::core::Context& context, const std::string& name,
                 const std::string& obj, const std::string& pin,
                 const std::string& body_keys) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 9.81, 0], "solvers": [{"type": "rnca"}],
      "integrators": [{"type": "ode113", "time_span": 0.002,
                       "time_step": 0.001, "hmax": 1e-4,
                       "relative_error": 1e-8, "absolute_error": 1e-10}]},
      "assemblies": [{"name": "test", "bodies": [{"name": "part",
        "density": 7850, "youngs_modulus": 2.1e11, "poisson_ratio": 0.3,
        "fe_geometry": "solid", "element_size": 0.01,
        "fe_cache": ")"
         << name << R"(.fe.h5",
        "frames": [{"name": "pin", "T": )"
         << pin << R"(}],
        "geometry": [{"name": "solid", "type": "mesh",
                      "filename": [")"
         << obj << R"("]}],
        )" << body_keys
         << R"(}],
      "joints": [{"name": "hinge", "type": "revolute",
                  "parent": [["fixed"], "origin"],
                  "child": [["part"], "pin"], "axis": [0, 0, 1]}]}]})";
  }
  std::remove((testing::TempDir() + "/" + name + ".fe.h5").c_str());
  try {
    context.LoadJSON(path);
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
  std::remove(path.c_str());
  std::remove((testing::TempDir() + "/" + name + ".fe.h5").c_str());
}

static shift::core::body::Flex* Part(shift::core::Context& context) {
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "part") {
      return dynamic_cast<shift::core::body::Flex*>(body);
    }
  }
  return nullptr;
}

/* the ring's bore, made finer than element_size where it curves: every tied
node within the sag of a chord of a sixteenth of the circle of the bore's
radius; the mass of the 32-sided ring; a brief swing */
TEST(FlexMesh, RingOnItsBore) {
  const std::string obj = testing::TempDir() + "/ring.obj";
  WriteRing(obj);
  shift::core::Context context;
  Load(context, "ring", obj, "[0, 0, 0.005]",
       R"("type": "modal", "mode_count": 1)");
  std::remove(obj.c_str());
  shift::core::body::Flex* ring = Part(context);
  ASSERT_NE(ring, nullptr);
  EXPECT_EQ(ring->get_elastic_count(), 1u);

  const double polygon = 0.5 * kSides * std::sin(2.0 * M_PI / kSides);
  const double mass = 7850.0 * kThickness * polygon *
                      (kOuter * kOuter - kInner * kInner);
  EXPECT_NEAR(ring->get_inertia_wrt_reference().m / mass, 1.0, 1e-2);

  const auto& tied = ring->get_attachment_nodes();
  ASSERT_EQ(tied.size(), 1u);
  EXPECT_GT(tied[0].size(), 2u * 16);
  const auto& positions = ring->get_positions();
  for(int node : tied[0]) {
    const double r = std::hypot(positions[size_t(node)].x,
                                positions[size_t(node)].y);
    EXPECT_GT(r, kInner * std::cos(M_PI / 16));
    EXPECT_LT(r, kInner * 1.001);
  }

  std::vector<double> last;
  context.set_state_observer(
    [&last](double, const std::vector<double>& y) { last = y; });
  context.Simulate();
  ASSERT_FALSE(last.empty());
  for(double y : last) {
    EXPECT_TRUE(std::isfinite(y));
  }
}

/* the bar's pin frame, inside the material: a node, tied with the elements
around it, none of them farther than an element or two */
TEST(FlexMesh, BarOnAFrameInsideIt) {
  const std::string obj = testing::TempDir() + "/bar.obj";
  WriteBar(obj);
  shift::core::Context context;
  Load(context, "bar", obj, "[0.0137, 0.0011, 0.0093]",
       R"("type": "modal", "mode_count": 2)");
  std::remove(obj.c_str());
  shift::core::body::Flex* bar = Part(context);
  ASSERT_NE(bar, nullptr);
  const auto& positions = bar->get_positions();
  const auto& tied = bar->get_attachment_nodes();
  ASSERT_EQ(tied.size(), 1u);
  bool at_frame = false;
  for(int node : tied[0]) {
    const ksl_vec3_t& p = positions[size_t(node)];
    at_frame = at_frame || (p.x == 0.0137 && p.y == 0.0011 && p.z == 0.0093);
    EXPECT_LT(std::hypot(p.x - 0.0137, p.y - 0.0011, p.z - 0.0093), 0.025);
  }
  EXPECT_TRUE(at_frame);
  EXPECT_NEAR(bar->get_inertia_wrt_reference().m, 7850.0 * 0.1 * 0.02 * 0.02,
              1e-6);
}

#endif

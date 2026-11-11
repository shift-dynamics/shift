#include <gtest/gtest.h>
#include <json/json.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/soil.h"
#include "shift/core/force_module/track_soil.h"
#include "shift/core/force_module/tire/bekker_wong.h"

using shift::core::force_module::LookupSoil;
using shift::core::force_module::Soil;
using shift::core::force_module::SoilProperties;
using shift::core::force_module::TrackSoil;
using shift::core::force_module::tire::BekkerWong;
using shift::core::force_module::tire::Contact;

/* a wheel of radius 0.5 sunk by depth, its hub moving forward at v_x, its
rim turning at v_r about the hub, and slipping sideways at v_y: the contact
point, on the terrain, is 0.5 - depth from the hub */
static Contact WheelContact(double depth, double v_x, double v_r,
                            double v_y = 0.0, double radius = 0.5) {
  Contact c;
  c.in_contact = true;
  c.depth = depth;
  c.vx = v_x;
  c.contact_velocity_w = {{v_x - v_r * (radius - depth) / radius, v_y, 0.0}};
  c.hub_velocity_w = {{v_x, v_y, 0.0}};
  c.angular_velocity_w = {{0.0, -v_r / radius, 0.0}};
  return c;
}

/* the same with the axle tilted by the camber gamma out of the terrain
plane (positive when its +y end is lower), the wheel rolling straight along
the heading: the hub moves at v_x and the rim at v_r */
static Contact CamberedContact(double depth, double gamma, double v_x,
                               double v_r, double radius = 0.5) {
  Contact c = WheelContact(depth, v_x, v_r, 0.0, radius);
  c.spin_axis_w = {{0.0, std::cos(gamma), std::sin(gamma)}};
  c.angular_velocity_w = {{0.0, 0.0, 0.0}};
  for(int i = 0; i < 3; i++) {
    c.angular_velocity_w.at[i] = -v_r / radius * c.spin_axis_w.at[i];
  }
  return c;
}

static ksl_coscrew_t Evaluate(BekkerWong& wheel, const Contact& contact) {
  ksl_coscrew_t f;
  wheel.Evaluate(contact, nullptr, nullptr, &f);
  return f;
}

TEST(Soil, Table) {
  SoilProperties s;
  ASSERT_TRUE(LookupSoil("lete_sand", s));
  EXPECT_EQ(s.n, 0.79);
  EXPECT_DOUBLE_EQ(s.k_c, 102e3);
  EXPECT_DOUBLE_EQ(s.k_phi, 5301e3);
  EXPECT_DOUBLE_EQ(s.cohesion, 1.3e3);
  EXPECT_DOUBLE_EQ(s.friction_angle, 31.1 * M_PI / 180.0);
  EXPECT_EQ(s.shear_modulus, 0.01); // the default: not in the table
  ASSERT_TRUE(LookupSoil("tri1_lunar_simulant", s));
  EXPECT_EQ(s.shear_modulus, 0.0103);
  EXPECT_FALSE(LookupSoil("moon_cheese", s));
  // Rankine: Kp = tan^2(45 + phi/2), 3 for 30 degrees
  s.friction_angle = M_PI / 6.0;
  s.cohesion = 0.0;
  s.unit_weight = 16000.0;
  EXPECT_NEAR(s.PassiveCoefficient(), 3.0, 1e-12);
  EXPECT_NEAR(s.PassiveForce(0.1), 0.5 * 16000.0 * 0.01 * 3.0, 1e-9);
  EXPECT_NEAR(s.ActiveCoefficient(), 1.0 / 3.0, 1e-12);
  EXPECT_NEAR(s.AtRestCoefficient(), 0.5, 1e-12);
}

TEST(Soil, CoulombEarthPressure) {
  // the thrust along its line, tabulated for phi 30, delta 15 degrees, is
  // Kp 4.98 and Ka 0.301: the coefficients are its part normal to the face
  SoilProperties s;
  s.friction_angle = M_PI / 6.0;
  s.wall_friction = M_PI / 12.0;
  EXPECT_NEAR(s.PassiveCoefficient() / std::cos(s.wall_friction), 4.98, 5e-3);
  EXPECT_NEAR(s.ActiveCoefficient() / std::cos(s.wall_friction), 0.301, 5e-4);
  EXPECT_GT(s.PassiveCoefficient(), 3.0);
  EXPECT_LT(s.ActiveCoefficient(), 1.0 / 3.0);
  s.passive_coefficient = 3.5;
  EXPECT_EQ(s.PassiveCoefficient(), 3.5);
  EXPECT_NEAR(s.PassiveForce(0.1), 0.5 * 16000.0 * 0.01 * 3.5, 1e-9);
}

static SoilProperties ResolveSoil(const std::string& json) {
  Json::Value value;
  std::istringstream stream(json);
  std::string errors;
  EXPECT_TRUE(Json::parseFromStream(Json::CharReaderBuilder(), stream, &value,
                                    &errors));
  Soil soil;
  soil.ReadJSON(value);
  return soil.Resolve("test");
}

TEST(Soil, EarthPressureKeys) {
  const SoilProperties s =
    ResolveSoil(R"({"name": "lete_sand", "wall_friction": 0.2})");
  EXPECT_EQ(s.wall_friction, 0.2);
  EXPECT_EQ(
    ResolveSoil(R"({"name": "lete_sand", "passive_coefficient": 4.0})")
      .PassiveCoefficient(),
    4.0);
  // above the friction angle, a coefficient below 1, and 45 degrees on both
  // sides, where Coulomb's passive pressure is infinite
  EXPECT_THROW(ResolveSoil(R"({"name": "lete_sand", "wall_friction": 0.6})"),
               shift::core::Exception);
  EXPECT_THROW(
    ResolveSoil(R"({"name": "lete_sand", "passive_coefficient": 0.5})"),
    shift::core::Exception);
  EXPECT_THROW(ResolveSoil(R"({"name": "lete_sand", "friction_angle": 0.8,
                               "wall_friction": 0.8})"),
               shift::core::Exception);
  EXPECT_NO_THROW(ResolveSoil(R"({"name": "lete_sand", "friction_angle": 0.8,
    "wall_friction": 0.8, "passive_coefficient": 8.0})"));
}

/* a frictionless, cohesionless soil (no shear) with the greatest stress at
the bottom (c1 = c2 = 0) and no rebound: Bekker's rigid wheel */
static void Bekker(BekkerWong& p, double radius, double width) {
  p.radius = radius;
  p.width = width;
  LookupSoil("lete_sand", p.soil);
  p.soil.cohesion = 0.0;
  p.soil.friction_angle = 0.0;
  p.c1 = p.c2 = 0.0;
  p.stations = 64;
}

TEST(BekkerWong, CompactionResistance) {
  // the horizontal force of the normal stress is Bekker's compaction
  // resistance exactly, b k z^(n+1) / (n+1), as dz = r sin(theta) dtheta
  BekkerWong p;
  Bekker(p, 0.5, 0.3);
  p.Prepare();
  BekkerWong& wheel = p;
  const double k = p.soil.k_c / p.width + p.soil.k_phi;
  const double n = p.soil.n;
  for(double z : {0.002, 0.02, 0.1}) {
    SCOPED_TRACE(z);
    const ksl_coscrew_t f = Evaluate(wheel, WheelContact(z, 1.0, 1.0));
    const double resistance = p.width * k * std::pow(z, n + 1.0) / (n + 1.0);
    EXPECT_NEAR(f.lin.x, -resistance, 1e-4 * resistance);
    EXPECT_EQ(f.lin.y, 0.0);
    // the normal stress passes through the hub, r - z above the contact
    // point: its moment there is that of the force at the hub
    EXPECT_NEAR(f.ang.y, (z - p.radius) * f.lin.x, 1e-9 * resistance);
    EXPECT_NEAR(f.ang.x, 0.0, 1e-12);
    EXPECT_NEAR(f.ang.z, 0.0, 1e-12);
  }
}

TEST(BekkerWong, BekkerSinkage) {
  // for small sinkage, the rim is the parabola z(x) = z - x^2 / (2 r) and
  // the load b k sqrt(2 r) z^(n + 1/2) integral_0^1 (1 - u^2)^n du, the
  // integral sqrt(pi) / 2 Gamma(n + 1) / Gamma(n + 3/2); Bekker's (3 - n)
  // / 3 approximates it (0.737 for 0.711 here)
  BekkerWong p;
  Bekker(p, 0.5, 0.3);
  p.Prepare();
  BekkerWong& wheel = p;
  const double k = p.soil.k_c / p.width + p.soil.k_phi;
  const double n = p.soil.n;
  const double z = 1e-3;
  const ksl_coscrew_t f = Evaluate(wheel, WheelContact(z, 1.0, 1.0));
  const double shape =
    0.5 * std::sqrt(M_PI) * std::tgamma(n + 1.0) / std::tgamma(n + 1.5);
  const double load = shape * p.width * k * std::sqrt(2.0 * p.radius) *
                      std::pow(z, n + 0.5);
  EXPECT_NEAR(-f.lin.z, load, 2e-3 * load);
}

TEST(BekkerWong, PublishedAngles) {
  // the small plain rigid wheel of Kumar and Jayalekshmi (2019), 160 mm
  // by 32 mm, at its measured sinkage of 2.66 mm on TRI-1: their entry
  // angle 14.82 deg and Wong-Reece angle of greatest stress 5.93 deg (c1
  // 0.4, no slip); the large one, 210 mm by 50 mm at 3.0 mm: 13.73 deg
  for(const auto& [d, b, z, entry] :
      {std::tuple<double, double, double, double>{0.160, 0.032, 2.66e-3,
                                                  14.82},
       std::tuple<double, double, double, double>{0.210, 0.050, 3.0e-3,
                                                  13.73}}) {
    BekkerWong p;
    p.radius = 0.5 * d;
    p.width = b;
    ASSERT_TRUE(LookupSoil("tri1_lunar_simulant", p.soil));
    p.Prepare();
    BekkerWong& wheel = p;
    Evaluate(wheel, WheelContact(z, 0.0, 0.0, 0.0, p.radius));
    EXPECT_NEAR(wheel.get_entry_angle() * 180.0 / M_PI, entry, 0.005);
    EXPECT_NEAR(wheel.get_max_stress_angle(), 0.4 * wheel.get_entry_angle(),
                1e-15);
    EXPECT_EQ(wheel.get_exit_angle(), 0.0);
  }
}

TEST(BekkerWong, DrawbarPullGrowsWithSlip) {
  // LETE sand, a wheel of 0.5 m by 0.3 m sunk 30 mm: pulled back at no
  // slip (the compaction resistance less a little shear), pulling with
  // more slip, and never beyond the soil's shear strength over the rim;
  // with theta_m fixed (c2 = 0), as at a fixed sinkage its moving back
  // with the slip lowers the pull at high slip
  BekkerWong p;
  p.radius = 0.5;
  p.width = 0.3;
  p.c2 = 0.0;
  ASSERT_TRUE(LookupSoil("lete_sand", p.soil));
  p.Prepare();
  BekkerWong& wheel = p;
  const double z = 0.03;
  double last = -HUGE_VAL;
  double strength = 0.0;
  for(double s : {0.0, 0.05, 0.1, 0.2, 0.4, 0.6, 0.8}) {
    SCOPED_TRACE(s);
    const ksl_coscrew_t f = Evaluate(wheel, WheelContact(z, 2.0, 2.0 / (1.0 - s)));
    EXPECT_NEAR(wheel.get_slip(), s, 1e-4);
    EXPECT_GT(f.lin.x, last);
    last = f.lin.x;
    EXPECT_LT(f.lin.z, 0.0);
    // the shear strength, c + sigma tan(phi), on the rim's horizontal
    // projection
    const double theta_1 = wheel.get_entry_angle();
    strength = 0.0;
    for(int i = 0; i < 2000; i++) {
      const double theta =
        wheel.get_exit_angle() +
        (i + 0.5) / 2000.0 * (theta_1 - wheel.get_exit_angle());
      strength += (p.soil.cohesion +
                   wheel.NormalStress(theta) * std::tan(p.soil.friction_angle)) *
                  std::cos(theta) * p.radius * p.width *
                  (theta_1 - wheel.get_exit_angle()) / 2000.0;
    }
    EXPECT_LT(f.lin.x, strength);
    // the soil resists the driving torque
    EXPECT_GT(wheel.get_shear_torque(), 0.0);
  }
  EXPECT_GT(last, 0.0);
  {
    SCOPED_TRACE("no slip");
    const ksl_coscrew_t f = Evaluate(wheel, WheelContact(z, 2.0, 2.0));
    EXPECT_LT(f.lin.x, 0.0);
  }
}

TEST(BekkerWong, BrakingAndSideSlip) {
  BekkerWong p;
  p.radius = 0.5;
  p.width = 0.3;
  ASSERT_TRUE(LookupSoil("lete_sand", p.soil));
  p.Prepare();
  BekkerWong& wheel = p;
  // a locked wheel is pushed back by the shear, harder than a rolling one
  const double rolling = Evaluate(wheel, WheelContact(0.03, 2.0, 2.0)).lin.x;
  const ksl_coscrew_t locked = Evaluate(wheel, WheelContact(0.03, 2.0, 0.0));
  EXPECT_LT(locked.lin.x, rolling);
  EXPECT_LT(wheel.get_slip(), -0.99);
  // sliding to the right, the soil pushes it left
  EXPECT_LT(Evaluate(wheel, WheelContact(0.03, 2.0, 2.0, 0.2)).lin.y, 0.0);
  EXPECT_GT(Evaluate(wheel, WheelContact(0.03, 2.0, 2.0, -0.2)).lin.y, 0.0);
}

TEST(BekkerWong, Standstill) {
  // at rest the shear is zero, and it grows from zero with the slip: no
  // division by a zero speed
  BekkerWong p;
  p.radius = 0.5;
  p.width = 0.3;
  ASSERT_TRUE(LookupSoil("lete_sand", p.soil));
  p.Prepare();
  BekkerWong& wheel = p;
  const ksl_coscrew_t rest = Evaluate(wheel, WheelContact(0.03, 0.0, 0.0));
  EXPECT_EQ(wheel.get_shear_torque(), 0.0);
  EXPECT_LT(rest.lin.z, 0.0);
  const double compaction = rest.lin.x;
  double last = 0.0;
  for(double v : {1e-9, 1e-7, 1e-5}) {
    const ksl_coscrew_t f = Evaluate(wheel, WheelContact(0.03, 0.0, v));
    EXPECT_GT(f.lin.x - compaction, last);
    last = f.lin.x - compaction;
    EXPECT_NEAR(f.lin.z, rest.lin.z, 1e-3 * std::fabs(rest.lin.z));
  }
  EXPECT_LT(last, 1e-2 * std::fabs(rest.lin.z));
  // and out of the soil there is no force
  const ksl_coscrew_t clear = Evaluate(wheel, WheelContact(0.0, 1.0, 1.0));
  EXPECT_EQ(clear.lin.x, 0.0);
  EXPECT_EQ(clear.lin.z, 0.0);
}

/* ------------------------------------------------------------------------ */
/* the soil under a track shoe */

/* a shoe of 0.2 m by 0.5 m with 20 mm grousers, flat at depth z on LETE
sand (z down, the ground at 0). With a grid, the ranges x and y of a terrain
of type scm of LETE sand at the ground, a box of the shoe for soil_contact,
which lists the shoe if soil_contact; and track_soil has its own soil only if
own_soil. */
static std::string ShoeModel(double z, const std::string& grid = "",
                             bool soil_contact = true, bool own_soil = true) {
  std::ostringstream s;
  s.precision(17);
  s << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0, 0, 9.81], "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": 0.0, "time_step": 0.01}]},)";
  if(!grid.empty()) {
    s << R"(
 "frames": [{"name": "terrain", "T": [0, 0, 0]}],
 "terrain": [{"type": "scm", "name": "sand", "parent": [["fixed"], "terrain"],
   )" << grid << R"(, "spacing": 0.05,
   "soil": {"name": "lete_sand", "unit_weight": 16000}}],)";
  }
  s << R"(
 "assemblies": [{"name": "a",
  "bodies": [{"name": "shoe", "center_of_mass": [0, 0, 0],)"
    << (grid.empty() ? "" : R"(
    "collision_geometry": [{"name": "plate", "type": "box", "contact": false,
      "half_extents": [0.1, 0.25, 0.01], "pose": {"T": [0, 0, -0.01]}}],)")
    << R"(
    "inertia": {"mass": 30, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0,
                "Iyz": 0, "Izx": 0}}],
  "joints": [{"name": "j", "type": "float", "parent": [["fixed"], "origin"],
    "child": [["shoe"], "origin"],
    "initial_displacement": {"T": [0, 0, )"
    << z << R"(], "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}}],
  "force_modules": [)"
    << (!grid.empty() && soil_contact
          ? R"({"type": "soil_contact", "name": "soil_contact",
                "bodies": ["shoe"]}, )"
          : "")
    << R"({"type": "track_soil", "name": "soil",)"
    << (own_soil ? R"(
    "soil": {"name": "lete_sand", "unit_weight": 16000},)"
                 : "")
    << R"(
    "shoes": ["shoe"], "shoe_center": [0, 0, 0], "shoe_length": 0.2,
    "shoe_width": 0.5, "grouser_height": 0.02}]}]})";
  return s.str();
}

class TrackSoilTest : public testing::Test {
protected:
  void Load(double z, const std::string& grid = "", bool own_soil = true) {
    const std::string path = testing::TempDir() + "/track_shoe.json";
    {
      std::ofstream file(path);
      file << ShoeModel(z, grid, true, own_soil);
    }
    context_.LoadJSON(path);
    std::remove(path.c_str());
    context_.UpdateDHVA();
    for(auto module : context_.get_active_force_modules()) {
      if(auto m = dynamic_cast<TrackSoil*>(module)) {
        soil_ = m;
      }
    }
    ASSERT_NE(soil_, nullptr);
    y_.assign(soil_->get_shoe_state_count(), 0.0);
  }

  /* the full state vector, with the module's states y_ */
  std::vector<double> State() {
    std::vector<double> y(context_.get_q().begin(), context_.get_q().end());
    y.resize(y.size() + context_.get_auxiliary_state_count(), 0.0);
    const size_t offset = soil_->get_state_offset();
    EXPECT_LE(offset + y_.size(), y.size());
    std::copy(y_.begin(), y_.end(), y.begin() + offset);
    return y;
  }

  /* sinkage, normal force, traction and side force for the states y_ */
  std::vector<double> Outputs() {
    soil_->UpdateOutputs(0.0, State().data());
    std::vector<double> v;
    soil_->AppendOutputs(v);
    return v;
  }

  /* the rates of the module's states */
  std::vector<double> Rates() {
    const std::vector<double> y = State();
    std::vector<double> dy(y.size(), 0.0);
    context_.SetForces(0.0, y, dy);
    return std::vector<double>(dy.begin() + soil_->get_state_offset(),
                               dy.begin() + soil_->get_state_offset() +
                                 y_.size());
  }

  shift::core::Context context_;
  TrackSoil* soil_ = nullptr;
  std::vector<double> y_;
};

TEST_F(TrackSoilTest, LoadingUnloadingAndPlasticity) {
  Load(0.03);
  const SoilProperties& s = soil_->get_soil();
  const double k = s.k_c / 0.2 + s.k_phi; // b, the smaller side
  // loading: Bekker's pressure under the whole shoe, at its six stations
  std::vector<double> out = Outputs();
  ASSERT_EQ(out.size(), 4u);
  EXPECT_NEAR(out[0], 0.03, 1e-12);
  const double load = k * std::pow(0.03, s.n) * 0.2 * 0.5;
  EXPECT_NEAR(out[1], load, 1e-9 * load);
  EXPECT_EQ(out[2], 0.0);
  EXPECT_EQ(out[3], 0.0);
  // after a deeper rut (z_p = 0.05), on the stiff line back: the soil
  // springs back by z_p / 10, so at z = 0.03 it is clear
  const double reached = k * std::pow(0.05, s.n);
  EXPECT_DOUBLE_EQ(soil_->Pressure(0.05, 0.05), reached);
  EXPECT_NEAR(soil_->Pressure(0.048, 0.05), reached * (1.0 - 10.0 * 0.04),
              1e-9 * reached);
  EXPECT_EQ(soil_->Pressure(0.044, 0.05), 0.0);
  for(int n = 0; n < 6; n++) {
    y_[n] = 0.05;
  }
  EXPECT_EQ(Outputs()[1], 0.0);
  // and deeper than the rut, loading again: Bekker's
  EXPECT_DOUBLE_EQ(soil_->Pressure(0.06, 0.05), k * std::pow(0.06, s.n));
  // the plastic sinkage follows the sinkage from below, and stays when
  // the shoe rises
  y_.assign(y_.size(), 0.0);
  std::vector<double> dy = Rates();
  for(int n = 0; n < 6; n++) {
    EXPECT_NEAR(dy[n], 0.03 / 0.01, 1e-9);
    y_[n] = 0.05;
  }
  dy = Rates();
  for(int n = 0; n < 6; n++) {
    EXPECT_EQ(dy[n], 0.0);
  }
}

TEST_F(TrackSoilTest, TractionAndSideForce) {
  Load(0.03);
  const SoilProperties& s = soil_->get_soil();
  const double k = s.k_c / 0.2 + s.k_phi;
  const double p = k * std::pow(0.03, s.n);
  const double c = s.cohesion * -std::expm1(-0.03 / 1e-3);
  const double shear = (c + p * std::tan(s.friction_angle)) * 0.2 * 0.5;
  const double kp = std::pow(std::tan(M_PI / 4 + s.friction_angle / 2), 2);
  auto passive = [&](double h) {
    return 0.5 * 16000.0 * h * h * kp + 2.0 * s.cohesion * h * std::sqrt(kp);
  };
  const double j = 0.02;
  const double mobilized = 1.0 - std::exp(-j / s.shear_modulus);
  // slipped back along the track: the soil pushes the shoe forward, by
  // its shear and the earth pressure on the leading grouser (20 mm deep)
  y_[6] = -j;
  std::vector<double> out = Outputs();
  const double traction = mobilized * (shear + 0.5 * passive(0.02));
  EXPECT_NEAR(out[2], traction, 1e-9 * traction);
  EXPECT_NEAR(out[3], 0.0, 1e-12);
  // slipped sideways (+y): pushed back by the shear and the earth
  // pressure on the side of the shoe, 30 mm deep over its 0.2 m
  y_[6] = 0.0;
  y_[7] = j;
  out = Outputs();
  const double side = mobilized * (shear + 0.2 * passive(0.03));
  EXPECT_NEAR(out[3], -side, 1e-9 * side);
  EXPECT_NEAR(out[2], 0.0, 1e-12);
  // at rest on the soil the displacement holds; lifted, it relaxes
  std::vector<double> dy = Rates();
  // w = 1 - exp(-z / contact_depth) = 1 - exp(-30): nearly no relaxation
  // (1 - w is rounded, as w is near 1)
  EXPECT_NEAR(dy[7], -std::exp(-30.0) * j / 0.01, 1e-3 * std::exp(-30.0));
}

TEST_F(TrackSoilTest, LiftedShoe) {
  Load(-0.01); // 10 mm above the ground
  y_[6] = 0.02;
  std::vector<double> out = Outputs();
  EXPECT_EQ(out[0], 0.0);
  EXPECT_EQ(out[1], 0.0);
  EXPECT_EQ(out[2], 0.0);
  std::vector<double> dy = Rates();
  EXPECT_NEAR(dy[6], -0.02 / 0.01, 1e-12);
}

TEST_F(TrackSoilTest, AShoeOffTheSoilForgetsItsRut) {
  Load(-0.01);
  y_[0] = 0.05;
  // 10 mm above the ground, ten contact depths: all of z_p / rut_time
  EXPECT_NEAR(Rates()[0], -0.05 / 0.5, 1e-5);
}

TEST_F(TrackSoilTest, AShoeOnTheSoilKeepsItsRut) {
  Load(0.03);
  y_[0] = 0.05;
  EXPECT_EQ(Rates()[0], 0.0);
}

/* over the grid of a terrain of type scm, soil_contact carries the shoe:
track_soil adds only the earth pressure on the leading grouser and the side,
with the terrain's soil and the sinkage below its undisturbed surface, and
its stations forget their plastic sinkage */
TEST_F(TrackSoilTest, OnDeformableSoilOnlyTheEarthPressure) {
  Load(0.03, R"("x": [-1, 1], "y": [-1, 1])", false);
  const SoilProperties s =
    ResolveSoil(R"({"name": "lete_sand", "unit_weight": 16000})");
  const double kp = std::pow(std::tan(M_PI / 4 + s.friction_angle / 2), 2);
  auto passive = [&](double h) {
    return 0.5 * 16000.0 * h * h * kp + 2.0 * s.cohesion * h * std::sqrt(kp);
  };
  const double j = 0.02;
  const double mobilized = 1.0 - std::exp(-j / s.shear_modulus);
  y_[6] = -j;
  std::vector<double> out = Outputs();
  EXPECT_NEAR(out[0], 0.03, 1e-12);
  EXPECT_EQ(out[1], 0.0); // no pressure of track_soil's own
  const double traction = mobilized * 0.5 * passive(0.02);
  EXPECT_NEAR(out[2], traction, 1e-9 * traction);
  y_[6] = 0.0;
  y_[7] = j;
  out = Outputs();
  const double side = mobilized * 0.2 * passive(0.03);
  EXPECT_NEAR(out[3], -side, 1e-9 * side);
  y_[0] = 0.05;
  EXPECT_NEAR(Rates()[0], -0.05 / 0.5, 1e-12);
}

/* beyond the grid, the shoe takes all of track_soil's forces on the ground
plane, with its own soil or, without one, the terrain's */
TEST_F(TrackSoilTest, BeyondTheGridAllOfTheForces) {
  for(bool own_soil : {true, false}) {
    shift::core::Context context;
    const std::string path = testing::TempDir() + "/track_shoe_beyond.json";
    {
      std::ofstream file(path);
      file << ShoeModel(0.03, R"("x": [3, 4], "y": [-1, 1])", true, own_soil);
    }
    context.LoadJSON(path);
    std::remove(path.c_str());
    context.UpdateDHVA();
    TrackSoil* soil = nullptr;
    for(auto module : context.get_active_force_modules()) {
      if(auto m = dynamic_cast<TrackSoil*>(module)) {
        soil = m;
      }
    }
    ASSERT_NE(soil, nullptr);
    std::vector<double> y(context.get_q().begin(), context.get_q().end());
    y.resize(y.size() + context.get_auxiliary_state_count(), 0.0);
    soil->UpdateOutputs(0.0, y.data());
    std::vector<double> out;
    soil->AppendOutputs(out);
    const SoilProperties s =
    ResolveSoil(R"({"name": "lete_sand", "unit_weight": 16000})");
    const double load =
      (s.k_c / 0.2 + s.k_phi) * std::pow(0.03, s.n) * 0.2 * 0.5;
    EXPECT_NEAR(out[1], load, 1e-9 * load) << own_soil;
  }
}

TEST(TrackSoil, NeedsSoilContactOnDeformableSoil) {
  const std::string path = testing::TempDir() + "/track_shoe_checks.json";
  auto error = [&](const std::string& model) -> std::string {
    {
      std::ofstream file(path);
      file << model;
    }
    shift::core::Context context;
    std::string message;
    try {
      context.LoadJSON(path);
    } catch(const std::exception& e) {
      message = e.what();
    }
    std::remove(path.c_str());
    return message;
  };
  EXPECT_NE(error(ShoeModel(0.03, R"("x": [-1, 1], "y": [-1, 1])", false))
              .find("must be listed by a soil_contact force module"),
            std::string::npos);
  EXPECT_NE(error(ShoeModel(0.03, "", true, false)).find("needs its soil"),
            std::string::npos);
}

/* ------------------------------------------------------------------------ */
/* camber and the bulldozing of the sides */

TEST(BekkerWong, CamberedCompactionResistance) {
  // the force along the heading of the normal stress is, with the axle
  // tilted by gamma, -k / ((n + 1) cos(gamma)) integral(H(y)^(n+1) dy), the
  // sinkage H(y) = depth + y sin(gamma) over the part of the width in the
  // soil: integrating the stress along the radius gives dz / cos(gamma)
  BekkerWong p;
  Bekker(p, 0.5, 0.3);
  p.Prepare();
  BekkerWong& wheel = p;
  const double k = p.soil.k_c / p.width + p.soil.k_phi;
  const double n = p.soil.n;
  for(double gamma : {0.0, 0.1, -0.1, 0.3}) {
    for(double depth : {0.03, 0.0}) {
      SCOPED_TRACE(gamma);
      SCOPED_TRACE(depth);
      const double sin_g = std::sin(gamma);
      // the width in the soil
      double lo = -0.5 * p.width, hi = 0.5 * p.width;
      if(sin_g > 0.0) {
        lo = std::max(lo, -depth / sin_g);
      } else if(sin_g < 0.0) {
        hi = std::min(hi, -depth / sin_g);
      }
      if(!(hi > lo)) {
        continue;
      }
      double integral;
      if(std::fabs(sin_g) < 1e-12) {
        integral = p.width * std::pow(depth, n + 1.0);
      } else {
        auto h = [&](double y) { return std::pow(depth + y * sin_g, n + 2.0); };
        integral = (h(hi) - h(lo)) / ((n + 2.0) * sin_g);
      }
      const double resistance =
        k * integral / ((n + 1.0) * std::cos(gamma));
      const ksl_coscrew_t f =
        Evaluate(wheel, CamberedContact(depth, gamma, 1.0, 1.0));
      EXPECT_NEAR(f.lin.x, -resistance, 2e-3 * resistance);
    }
  }
}

TEST(BekkerWong, CamberThrust) {
  // without shear the stress is along the radius, in the wheel's plane:
  // the lateral force is the load times tan(gamma), towards +y when the +y
  // end of the axle is lower: the top of the wheel leans that way, and the
  // soil pushes the wheel up along its plane
  BekkerWong p;
  Bekker(p, 0.5, 0.3);
  p.Prepare();
  BekkerWong& wheel = p;
  for(double gamma : {0.05, 0.2, -0.2}) {
    SCOPED_TRACE(gamma);
    const ksl_coscrew_t f =
      Evaluate(wheel, CamberedContact(0.03, gamma, 1.0, 1.0));
    ASSERT_LT(f.lin.z, 0.0);
    EXPECT_NEAR(f.lin.y / f.lin.z, -std::tan(gamma), 1e-9);
    EXPECT_NEAR(wheel.get_camber(), gamma, 1e-12);
  }
  // on its edge: the middle of the rim is above the terrain (negative
  // depth) but the low edge of the tread is in it
  EXPECT_LT(Evaluate(wheel, CamberedContact(-0.01, 0.3, 1.0, 1.0)).lin.z, 0.0);
  // not in the soil at all: no force
  EXPECT_EQ(Evaluate(wheel, CamberedContact(-0.2, 0.3, 1.0, 1.0)).lin.z, 0.0);
}

TEST(BekkerWong, CamberMirror) {
  // mirrored in y: the same drawbar pull, load and torque, the lateral
  // force opposite
  BekkerWong p;
  p.radius = 0.5;
  p.width = 0.3;
  ASSERT_TRUE(LookupSoil("lete_sand", p.soil));
  p.Prepare();
  BekkerWong& wheel = p;
  const ksl_coscrew_t a = Evaluate(wheel, CamberedContact(0.03, 0.15, 2.0, 2.5));
  const double torque = wheel.get_shear_torque();
  const ksl_coscrew_t b = Evaluate(wheel, CamberedContact(0.03, -0.15, 2.0, 2.5));
  EXPECT_NEAR(a.lin.x, b.lin.x, 1e-9 * std::fabs(a.lin.x));
  EXPECT_NEAR(a.lin.z, b.lin.z, 1e-9 * std::fabs(a.lin.z));
  EXPECT_NEAR(a.lin.y, -b.lin.y, 1e-9 * std::fabs(a.lin.y));
  EXPECT_NEAR(torque, wheel.get_shear_torque(), 1e-9 * torque);
  EXPECT_GT(std::fabs(a.lin.y), 0.0);
}

TEST(BekkerWong, BulldozingOfTheSide) {
  // a vertical wheel pushed sideways fast: the side facing the push meets
  // the soil's passive pressure over the circular segment the terrain cuts
  // off it, 2 c sqrt(Kp) A + unit_weight Kp integral(h dA), against the
  // push; the other side is not touched. The segment's first moment about
  // the wheel's center is chord^3 / 12, chord = 2 sqrt(2 r H - H^2), so
  // the depth h below the terrain integrates to chord^3 / 12 - (r - H) A
  BekkerWong p;
  p.radius = 0.5;
  p.width = 0.3;
  ASSERT_TRUE(LookupSoil("lete_sand", p.soil));
  p.soil.unit_weight = 16000.0;
  p.stations = 8;
  const double depth = 0.04;
  const double kp = p.soil.PassiveCoefficient();
  const double chord = 2.0 * std::sqrt(2.0 * p.radius * depth - depth * depth);
  const double theta = std::acos(1.0 - depth / p.radius);
  const double area = p.radius * p.radius * theta -
                      (p.radius - depth) * 0.5 * chord;
  const double expected = 2.0 * p.soil.cohesion * std::sqrt(kp) * area +
                          p.soil.unit_weight * kp *
                            (chord * chord * chord / 12.0 -
                             (p.radius - depth) * area);
  p.Prepare();
  BekkerWong& wheel = p;
  Evaluate(wheel, WheelContact(depth, 0.0, 0.0, 1.0));
  EXPECT_NEAR(wheel.get_bulldozing_force(), -expected, 2e-3 * expected);
  Evaluate(wheel, WheelContact(depth, 0.0, 0.0, -1.0));
  EXPECT_NEAR(wheel.get_bulldozing_force(), expected, 2e-3 * expected);
  // at rest, or sliding slower than the speed that mobilizes it: less, and
  // none at rest
  Evaluate(wheel, WheelContact(depth, 0.0, 0.0, 0.0));
  EXPECT_EQ(wheel.get_bulldozing_force(), 0.0);
  Evaluate(wheel, WheelContact(depth, 0.0, 0.0, 0.5 * p.slip_speed));
  EXPECT_LT(wheel.get_bulldozing_force(), 0.0);
  EXPECT_GT(wheel.get_bulldozing_force(), -0.6 * expected);
  // switched off
  p.bulldozing = false;
  p.Prepare();
  Evaluate(p, WheelContact(depth, 0.0, 0.0, 1.0));
  EXPECT_EQ(p.get_bulldozing_force(), 0.0);
}

TEST(BekkerWong, PublishedWheelsAtTheirMeasuredSinkage) {
  // Kumar and Jayalekshmi (2019) measured 2.66 mm of sinkage under 52.2 N
  // (the 160 mm by 32 mm wheel) and 3.0 mm under 67.4 N (210 mm by 50 mm)
  // on TRI-1. With the soil's plate-test moduli the model carries about a
  // quarter to a third of those loads at those sinkages, and so sinks
  // 3 to 4 times deeper under the weights (9.5 to 11 mm, 8.6 to 10 mm). It
  // is Bekker's law and not the integration: the stress at the measured
  // sinkage is the paper's (24 to 29 kPa, of its 32 kPa), and for the
  // stress greatest at the bottom (c1 = 0) the load is the closed form
  // (sqrt(pi) / 2) Gamma(n + 1) / Gamma(n + 3/2) b (k_c / b + k_phi)
  // sqrt(2 r) z^(n + 1/2) (the paper's own figure for the small wheel, 37.8
  // N, is no more than 52.2 N either, and larger than its stresses allow:
  // 32 kPa over the contact area is at most 21 N)
  for(const auto& [d, b, z] :
      {std::tuple<double, double, double>{0.160, 0.032, 2.66e-3},
       std::tuple<double, double, double>{0.210, 0.050, 3.0e-3}}) {
    SCOPED_TRACE(d);
    BekkerWong p;
    p.radius = 0.5 * d;
    p.width = b;
    ASSERT_TRUE(LookupSoil("tri1_lunar_simulant", p.soil));
    p.c1 = p.c2 = 0.0;
    p.Prepare();
    BekkerWong& wheel = p;
    const ksl_coscrew_t f =
      Evaluate(wheel, WheelContact(z, 0.0, 0.0, 0.0, p.radius));
    const double n = p.soil.n;
    const double shape =
      0.5 * std::sqrt(M_PI) * std::tgamma(n + 1.0) / std::tgamma(n + 1.5);
    const double load = shape * b * (p.soil.k_c / b + p.soil.k_phi) *
                        std::sqrt(2.0 * p.radius) * std::pow(z, n + 0.5);
    // (the closed form has the rim as a parabola: z / r is 3%)
    EXPECT_NEAR(-f.lin.z, load, 1.5e-2 * load);
    EXPECT_LT(-f.lin.z, 0.5 * (d < 0.18 ? 52.2 : 67.4));
    // the stress at the entry angle's neighbour, at theta_m for c1 = 0.4
    p.c1 = 0.4;
    p.Prepare();
    Evaluate(p, WheelContact(z, 0.0, 0.0, 0.0, p.radius));
    const double peak = p.NormalStress(p.get_max_stress_angle());
    EXPECT_GT(peak, 20e3);
    EXPECT_LT(peak, 40e3);
  }
}

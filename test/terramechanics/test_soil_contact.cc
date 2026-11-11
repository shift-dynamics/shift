/* the force of deformable SCM soil on bodies (force_module::SoilContact): a
plate settles to the sinkage of Bekker's law and leaves a rut that has sprung
back; the soil unloads and reloads along the stiffer line of terrain::Scm */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <H5Cpp.h>
#include <gtest/gtest.h>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/joint/base.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/soil.h"
#include "shift/core/force_module/soil_contact.h"
#include "shift/core/terrain/scm.h"

static const double kMass = 2000.0;
static const double kKc = 2.0e4, kKphi = 1.0e6, kN = 1.1;

/* a square plate, 0.5 m wide and 0.1 m thick, on a vertical prismatic joint
with its bottom on the surface of an SCM grid of spacing 0.1; if sliding, the
joint is on a carriage that slides along x */
static std::string PlateModel(const std::string& terrain,
                              const std::string& force_modules,
                              const std::string& geometry, double time_span,
                              bool sliding = false) {
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
        << R"(, "time_step": 0.01, "relative_error": 1e-8,
                     "absolute_error": 1e-9}]
  },
  "frames": [{"name": "terrain", "T": [0, 0, 0]}],
  "terrain": [)" << terrain << R"(],
  "assemblies": [{"name": "a",
    "force_modules": [)" << force_modules << R"(],
    "bodies": [{"name": "plate", "collision_geometry": [)" << geometry << R"(],
                "center_of_mass": [0, 0, 0],
                "inertia": {"mass": )" << kMass << R"(, "Ixx": 40, "Iyy": 40,
                            "Izz": 80, "Ixy": 0, "Iyz": 0, "Izx": 0}})"
        << (sliding ? R"(,
               {"name": "carriage", "center_of_mass": [0, 0, 0],
                "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1,
                            "Ixy": 0, "Iyz": 0, "Izx": 0}})"
                    : "")
        << R"(],
    "joints": [)"
        << (sliding ? R"({"name": "x", "type": "prismatic",
                "parent": [["fixed"], "origin"],
                "child": [["carriage"], "origin"], "axis": [1, 0, 0]},)"
                    : "")
        << R"({"name": "slide", "type": "prismatic",
                "parent": [[")"
        << (sliding ? "carriage" : "fixed") << R"("], "origin"],
                "child": [["plate"], "origin"], "axis": [0, 0, 1],
                "initial_displacement": -0.05}]
  }]
})";
  return model.str();
}

static std::string Grid(const std::string& type,
                        const std::string& extra = "",
                        const std::string& soil_extra = "") {
  std::ostringstream grid;
  grid.precision(17);
  grid << R"({"type": ")" << type
       << R"(", "name": "site", "parent": [["fixed"], "terrain"],)" << extra
       << R"(
      "x": [-1, 1], "y": [-0.5, 0.5], "spacing": 0.1, "damping": 5,
      "soil": {"sinkage_exponent": )"
       << kN << R"(, "cohesive_modulus": )" << kKc
       << R"(, "frictional_modulus": )" << kKphi
       << R"(, "cohesion": 1000, "friction_angle": 0.5,
               "shear_modulus": 0.01)" << soil_extra << R"(}})";
  return grid.str();
}

static const char* kSoilContact =
  R"({"type": "soil_contact", "name": "soil", "bodies": ["plate"]})";
static const char* kPlate =
  R"({"name": "plate", "type": "box", "half_extents": [0.25, 0.25, 0.05]})";

static void Load(shift::core::Context& context, const std::string& name,
                 const std::string& model) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file << model;
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

static shift::core::terrain::Scm* GetScm(shift::core::Context& context) {
  if(context.get_active_terrain().size() != 1u) {
    ADD_FAILURE() << "no terrain";
    return nullptr;
  }
  return dynamic_cast<shift::core::terrain::Scm*>(
    context.get_active_terrain()[0]);
}

/* the plate covers the 5 x 5 nodes from -0.2 to 0.2: a patch of area 25 h^2
and perimeter 20 h, so b = 2 A / P = 0.25 m. At rest the pressure of Bekker's
law on the 25 nodes carries the plate's weight; it sank monotonically
(overdamped), so each node's deepest sinkage is that sinkage and its level the
rut sprung back. The nodes around the plate are undisturbed. */
TEST(SoilContact, PlateSinkage) {
  shift::core::Context context;
  Load(context, "soil_plate", PlateModel(Grid("scm"), kSoilContact, kPlate, 4.0));
  context.Simulate();
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);

  const double h = 0.1, b = 0.25;
  const double p = kMass * 9.81 / (25.0 * h * h);
  const double s = std::pow(p / (kKc / b + kKphi), 1.0 / kN);

  shift::core::force_module::SoilContact* soil = nullptr;
  for(auto module : context.get_active_force_modules()) {
    if(auto found = dynamic_cast<shift::core::force_module::SoilContact*>(module)) {
      soil = found;
    }
  }
  ASSERT_NE(soil, nullptr);
  EXPECT_EQ(soil->get_contact_nodes(), 25.0);
  EXPECT_NEAR(soil->get_sinkage(), s, 1e-4 * s);
  EXPECT_NEAR(soil->get_normal_force(), kMass * 9.81, 1e-3 * kMass * 9.81);

  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "plate") {
      EXPECT_NEAR(context.get_body_origin_pose(body).t.z + 0.05, s, 1e-4 * s);
    }
  }
  EXPECT_EQ(scm->get_changed_node_count(), 25u);
  for(long j = 3; j <= 7; j++) {
    for(long i = 8; i <= 12; i++) {
      const auto* node = scm->FindNode(i, j);
      ASSERT_NE(node, nullptr) << i << " " << j;
      EXPECT_NEAR(node->plastic_sinkage, s, 1e-4 * s);
      EXPECT_DOUBLE_EQ(node->level, scm->ReboundLevel(node->plastic_sinkage));
      EXPECT_TRUE(node->in_contact);
      EXPECT_NEAR(node->shear[0], 0.0, 1e-12);
      EXPECT_NEAR(node->shear[1], 0.0, 1e-12);
    }
  }
  EXPECT_EQ(scm->FindNode(7, 5), nullptr);
  EXPECT_EQ(scm->FindNode(10, 2), nullptr);
}

/* pushed along x by a force that grows slowly to half the shear strength of
the soil under it, the plate slides until the shear stress of Janosi and
Hanamoto's curve carries the push: j = -K ln(1 - F / (A tau_max)), with
tau_max = c + p tan(phi) at the pressure that carries its weight. The shear
displacement grows from the accepted one, so the nodes hold it. The shear
stress is a function of j alone, a spring without loss, so a viscous damper on
the slide, which takes nothing at rest, settles the plate. */
TEST(SoilContact, ShearDisplacement) {
  const double h = 0.1, area = 25.0 * h * h, cohesion = 1000.0, phi = 0.5,
               k = 0.01;
  const double p = kMass * 9.81 / area;
  const double push = 0.5 * area * (cohesion + p * std::tan(phi));
  std::ostringstream load;
  load.precision(17);
  load << kSoilContact << R"(, {"type": "applied_load", "name": "push",
    "body": "plate", "coordinates": "fixed", "force": [)"
       << push << R"(, 0, 0], "scale_curve": [0, 0, 1, 0, 3, 1]},
    {"type": "joint_friction", "name": "damper", "joint": "x", "friction": 0,
     "viscous_friction": 1e5})";
  shift::core::Context context;
  Load(context, "soil_shear",
       PlateModel(Grid("scm"), load.str(), kPlate, 5.0, true));
  context.Simulate();
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  const double j = -k * std::log(0.5);
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "plate") {
      EXPECT_NEAR(context.get_body_origin_pose(body).t.x, j, 1e-4 * j);
    }
  }
  const auto* node = scm->FindNode(10, 5);
  ASSERT_NE(node, nullptr);
  EXPECT_NEAR(node->shear[0], j, 1e-4 * j);
  EXPECT_NEAR(node->shear[1], 0.0, 1e-9);
}

/* below the deepest sinkage z_p the soil unloads along a line of
unloading_ratio times the secant stiffness, to zero at the rebound level, and
reloads along the same line until it meets Bekker's curve again at z_p */
TEST(SoilContact, UnloadReload) {
  shift::core::Context context;
  Load(context, "soil_unload",
       PlateModel(Grid("scm"), kSoilContact, kPlate, 0.0));
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  ASSERT_EQ(scm->get_unloading_ratio(), 10.0);
  shift::core::force_module::SoilProperties soil;
  soil.n = kN;
  soil.k_c = kKc;
  soil.k_phi = kKphi;
  const double b = 0.25, z_p = 0.08;
  const double load = soil.Pressure(b, z_p);
  const double k_u = 10.0 * load / z_p;

  EXPECT_DOUBLE_EQ(scm->Pressure(soil, b, z_p, z_p), load);
  EXPECT_DOUBLE_EQ(scm->Pressure(soil, b, 0.05, 0.0), soil.Pressure(b, 0.05));
  EXPECT_NEAR(scm->Pressure(soil, b, 0.078, z_p), load - k_u * 0.002,
              1e-9 * load);
  EXPECT_NEAR(scm->Pressure(soil, b, scm->ReboundLevel(z_p), z_p), 0.0,
              1e-9 * load);
  EXPECT_EQ(scm->Pressure(soil, b, 0.05, z_p), 0.0);
  /* past z_p it follows Bekker's curve, deeper than before */
  EXPECT_DOUBLE_EQ(scm->Pressure(soil, b, 0.09, z_p), soil.Pressure(b, 0.09));
  /* never above the curve, near the rebound level where the line is
  steeper than the curve */
  for(double s = scm->ReboundLevel(z_p); s <= z_p; s += 0.001) {
    EXPECT_LE(scm->Pressure(soil, b, s, z_p), soil.Pressure(b, s) * (1 + 1e-12))
      << s;
  }
}

/* the sum of the levels of all nodes: the volume of soil taken from the
grid, over the area of a node */
static double LevelSum(const shift::core::terrain::Scm& scm) {
  double sum = 0.0;
  for(long j = 0; j < scm.get_node_count_y(); j++) {
    for(long i = 0; i < scm.get_node_count_x(); i++) {
      const auto* node = scm.FindNode(i, j);
      sum += node ? node->level : 0.0;
    }
  }
  return sum;
}

/* the steepest slope of the levels between neighbors along x and y that no
body is on */
static double SteepestSlope(const shift::core::terrain::Scm& scm) {
  double steepest = 0.0;
  auto free_level = [&](long i, long j, double& level) {
    const auto* node = scm.FindNode(i, j);
    if(node && node->in_contact) {
      return false;
    }
    level = node ? node->level : 0.0;
    return true;
  };
  for(long j = 0; j < scm.get_node_count_y(); j++) {
    for(long i = 0; i < scm.get_node_count_x(); i++) {
      double a, b;
      if(!free_level(i, j, a)) {
        continue;
      }
      if(i + 1 < scm.get_node_count_x() && free_level(i + 1, j, b)) {
        steepest = std::max(steepest, std::abs(a - b) / scm.get_spacing());
      }
      if(j + 1 < scm.get_node_count_y() && free_level(i, j + 1, b)) {
        steepest = std::max(steepest, std::abs(a - b) / scm.get_spacing());
      }
    }
  }
  return steepest;
}

static const char* kBulldozing =
  R"( "bulldozing": {"enable": true, "erosion_angle": 0.3},)";

/* with bulldozing the plate pushes the soil it presses down to the rim of its
rut, the nodes around it: the volume of soil is conserved, the rim is raised,
and the slopes of the soil off the plate are eroded to the angle of repose.
The plate settles as without bulldozing, since piles carry nothing. */
TEST(Bulldozing, PlateRaisesTheRimOfItsRut) {
  shift::core::Context context;
  Load(context, "soil_bulldozing",
       PlateModel(Grid("scm", kBulldozing), kSoilContact, kPlate, 4.0));
  context.Simulate();
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  ASSERT_TRUE(scm->get_bulldozing().is_enabled());

  const double h = 0.1, b = 0.25;
  const double p = kMass * 9.81 / (25.0 * h * h);
  const double s = std::pow(p / (kKc / b + kKphi), 1.0 / kN);
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "plate") {
      EXPECT_NEAR(context.get_body_origin_pose(body).t.z + 0.05, s, 1e-4 * s);
    }
  }
  /* the rut, and the soil around it */
  double rut = 0.0;
  for(long j = 3; j <= 7; j++) {
    for(long i = 8; i <= 12; i++) {
      const auto* node = scm->FindNode(i, j);
      ASSERT_NE(node, nullptr);
      EXPECT_TRUE(node->in_contact);
      EXPECT_DOUBLE_EQ(node->level, scm->ReboundLevel(node->plastic_sinkage));
      rut += node->level;
    }
  }
  EXPECT_GT(rut, 0.0);
  EXPECT_NEAR(LevelSum(*scm), 0.0, 1e-12 * rut);
  for(long i = 7; i <= 13; i++) {
    for(long j : {2L, 8L}) {
      const auto* node = scm->FindNode(i, j);
      ASSERT_NE(node, nullptr) << i << " " << j;
      EXPECT_LT(node->level, 0.0) << i << " " << j;
    }
  }
  EXPECT_LE(SteepestSlope(*scm), std::tan(0.3) + 2e-6);
}

/* the deepest sinkage of the nodes under the plate, after it was set on
the soil without damping: falling in, it sinks about twice as deep as
Bekker's law gives for its weight, and the soil keeps that; settled over the
first 3 s (settle_time) with the default settle_damping, it sinks to
Bekker's depth */
static double DeepestSinkage(const std::string& name,
                             const std::string& settling) {
  std::string grid = Grid("scm", settling);
  const std::string damping = R"("damping": 5,)";
  grid.replace(grid.find(damping), damping.size(), R"("damping": 0,)");
  shift::core::Context context;
  Load(context, name, PlateModel(grid, kSoilContact, kPlate, 4.0));
  context.Simulate();
  auto* scm = GetScm(context);
  if(!scm) {
    return 0.0;
  }
  double deepest = 0.0;
  for(long j = 3; j <= 7; j++) {
    for(long i = 8; i <= 12; i++) {
      const auto* node = scm->FindNode(i, j);
      deepest = std::max(deepest, node ? node->plastic_sinkage : 0.0);
    }
  }
  return deepest;
}

TEST(SoilContact, SettlingSinksToBekkersDepth) {
  const double h = 0.1, b = 0.25;
  const double p = kMass * 9.81 / (25.0 * h * h);
  const double s = std::pow(p / (kKc / b + kKphi), 1.0 / kN);
  const double fallen = DeepestSinkage("soil_unsettled", "");
  const double settled =
    DeepestSinkage("soil_settled", R"( "settle_time": 3.0,)");
  EXPECT_GT(fallen, 1.9 * s);
  EXPECT_NEAR(settled, s, 0.01 * s);
}

/* the damping of the settling fades from damping + settle_damping at the
start to damping at settle_time, half way at half of it */
TEST(SoilContact, SettleDampingFades) {
  shift::core::Context context;
  Load(context, "soil_settle_damping",
       PlateModel(Grid("scm", R"( "settle_time": 0.4, "settle_damping": 30,)"),
                  kSoilContact, kPlate, 0.0));
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  EXPECT_DOUBLE_EQ(scm->DampingAt(0.0), 35.0);
  EXPECT_NEAR(scm->DampingAt(0.2), 20.0, 1e-12);
  EXPECT_DOUBLE_EQ(scm->DampingAt(0.4), 5.0);
  EXPECT_DOUBLE_EQ(scm->DampingAt(1.0), 5.0);
  EXPECT_LT(scm->DampingAt(0.1), 35.0);
  EXPECT_GT(scm->DampingAt(0.1), 20.0);
}

/* the soil under the plate compacts by up to 0.4 of a layer as deep as the
patch is wide, 0.25 m: 0.1 m, more than the plate's rut, about 0.083 m. All
of it compacts, so no rim rises, and each node keeps its compaction, its
level */
TEST(Bulldozing, ShallowPrintCompactsWithoutARim) {
  shift::core::Context context;
  Load(context, "soil_compaction",
       PlateModel(Grid("scm", kBulldozing,
                       R"(, "unit_weight": 15000, "max_unit_weight": 25000)"),
                  kSoilContact, kPlate, 4.0));
  context.Simulate();
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  double rut = 0.0;
  for(long j = 3; j <= 7; j++) {
    for(long i = 8; i <= 12; i++) {
      const auto* node = scm->FindNode(i, j);
      ASSERT_NE(node, nullptr);
      EXPECT_GT(node->level, 0.05);
      EXPECT_LT(node->level, 0.1);
      EXPECT_NEAR(node->compaction, node->level, 1e-12);
      rut += node->level;
    }
  }
  EXPECT_NEAR(LevelSum(*scm), rut, 1e-12 * rut);
  for(long j = 0; j < scm->get_node_count_y(); j++) {
    for(long i = 0; i < scm->get_node_count_x(); i++) {
      const auto* node = scm->FindNode(i, j);
      EXPECT_FALSE(node && node->level < 0.0) << i << " " << j;
    }
  }
}

/* the layer that compacts given as 0.1 m deep: each node under the plate
compacts by 0.4 of it, 0.04 m, less than its rut, and the rest of the soil
the plate pressed down is pushed to the rim */
TEST(Bulldozing, DeepPrintPushesAsideWhatDoesNotCompact) {
  shift::core::Context context;
  Load(context, "soil_compaction_depth",
       PlateModel(Grid("scm",
                       R"( "bulldozing": {"enable": true, "erosion_angle": 0.3,
                           "compaction_depth": 0.1},)",
                       R"(, "unit_weight": 15000, "max_unit_weight": 25000)"),
                  kSoilContact, kPlate, 4.0));
  context.Simulate();
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  const double capacity = 0.4 * 0.1;
  double displaced = 0.0;
  for(long j = 3; j <= 7; j++) {
    for(long i = 8; i <= 12; i++) {
      const auto* node = scm->FindNode(i, j);
      ASSERT_NE(node, nullptr);
      ASSERT_GT(node->level, capacity);
      EXPECT_NEAR(node->compaction, capacity, 1e-12);
      displaced += node->level - capacity;
    }
  }
  double pile = 0.0;
  for(long j = 0; j < scm->get_node_count_y(); j++) {
    for(long i = 0; i < scm->get_node_count_x(); i++) {
      const auto* node = scm->FindNode(i, j);
      if(node && node->level < 0.0) {
        pile -= node->level;
      }
    }
  }
  EXPECT_NEAR(pile, displaced, 1e-9 * displaced);
  EXPECT_NEAR(LevelSum(*scm), 25.0 * capacity, 1e-9);
  EXPECT_LE(SteepestSlope(*scm), std::tan(0.3) + 2e-6);
}

/* a max_unit_weight less than the unit weight is refused */
TEST(Bulldozing, ChecksTheMaxUnitWeight) {
  shift::core::Context context;
  EXPECT_THROW(
    Load(context, "soil_compaction_check",
         PlateModel(Grid("scm", kBulldozing,
                         R"(, "unit_weight": 15000, "max_unit_weight": 14000)"),
                    kSoilContact, kPlate, 0.1)),
    shift::core::Exception);
}

/* a plate of two halves with a crack of one node between them, as the shoes
of a track: the crack is inside the plate's footprint, so the soil the halves
press down goes to the rim around the whole plate, not into the crack, which
stays at most as high as the rim (soil may erode into its ends) */
TEST(Bulldozing, CrackBetweenPlatesTakesNoSoil) {
  shift::core::Context context;
  const std::string halves =
    R"({"name": "left", "type": "box", "half_extents": [0.1, 0.25, 0.05],
        "pose": {"T": [-0.15, 0, 0]}},
       {"name": "right", "type": "box", "half_extents": [0.1, 0.25, 0.05],
        "pose": {"T": [0.15, 0, 0]}})";
  Load(context, "soil_crack",
       PlateModel(Grid("scm", kBulldozing), kSoilContact, halves, 2.0));
  context.Simulate();
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  /* the halves are on nodes 8, 9 and 11, 12 along x; the crack is node 10 */
  for(long j = 3; j <= 7; j++) {
    for(long i : {8L, 9L, 11L, 12L}) {
      const auto* node = scm->FindNode(i, j);
      ASSERT_NE(node, nullptr);
      EXPECT_TRUE(node->in_contact) << i << " " << j;
    }
    const auto* crack = scm->FindNode(10, j);
    EXPECT_TRUE(!crack || !crack->in_contact);
  }
  double rim = 0.0; // the highest of the rim, the most negative level
  for(long i = 7; i <= 13; i++) {
    for(long j : {2L, 8L}) {
      const auto* node = scm->FindNode(i, j);
      ASSERT_NE(node, nullptr) << i << " " << j;
      rim = std::min(rim, node->level);
    }
  }
  EXPECT_LT(rim, 0.0);
  for(long j = 3; j <= 7; j++) {
    const auto* crack = scm->FindNode(10, j);
    const double level = crack ? crack->level : 0.0;
    EXPECT_GE(level, rim) << j;
    if(j >= 4 && j <= 6) {
      EXPECT_GE(level, 0.0) << j; // the middle of the crack: none at all
    }
  }
}

/* the halves 0.3 m apart, three nodes of crack between them: closed by a
closing_distance of 0.4 m, so the crack takes no soil, and left open by the
default of 0.05 m, so that it takes the soil of its rims */
TEST(Bulldozing, WiderCrackClosedByClosingDistance) {
  const std::string halves =
    R"({"name": "left", "type": "box", "half_extents": [0.1, 0.25, 0.05],
        "pose": {"T": [-0.25, 0, 0]}},
       {"name": "right", "type": "box", "half_extents": [0.1, 0.25, 0.05],
        "pose": {"T": [0.25, 0, 0]}})";
  double middle[2];
  for(int k = 0; k < 2; k++) {
    shift::core::Context context;
    const std::string bulldozing =
      k == 0 ? R"( "bulldozing": {"enable": true, "erosion_angle": 0.3,
                                  "closing_distance": 0.4},)"
             : kBulldozing;
    Load(context, "soil_wide_crack",
         PlateModel(Grid("scm", bulldozing), kSoilContact, halves, 1.0));
    context.Simulate();
    auto* scm = GetScm(context);
    ASSERT_NE(scm, nullptr);
    /* the halves are on nodes 7, 8 and 12, 13 along x; the crack is 9 to
    11 */
    const auto* node = scm->FindNode(10, 5);
    middle[k] = node ? node->level : 0.0;
  }
  EXPECT_GE(middle[0], 0.0);
  EXPECT_LT(middle[1], 0.0);
}

/* a pile set on the grid next to a node a body is on is eroded down to the
angle of repose by the erosion after each accepted step, and keeps its
volume */
TEST(Bulldozing, ErodesAPileToTheAngleOfRepose) {
  shift::core::Context context;
  Load(context, "soil_erosion",
       PlateModel(Grid("scm", R"( "bulldozing": {"enable": true,
         "erosion_angle": 0.5, "erosion_iterations": 2, "interval": 0},)"),
                  kSoilContact, kPlate, 0.0));
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  const long nx = scm->get_node_count_x();
  std::vector<float> levels(
    static_cast<size_t>(nx * scm->get_node_count_y()), 0.0f);
  levels[static_cast<size_t>(5 * nx + 10)] = -0.3f; // a spike 0.3 m high
  scm->SetLevels(levels);
  const double volume = LevelSum(*scm);
  ASSERT_GT(SteepestSlope(*scm), 1.0);
  /* a body on the node next to it for a step, pressing nothing: the erosion
  near it runs after the step, and after each step after it, the body gone,
  until the slopes settle */
  scm->ClearTrials();
  scm->Trial(9, 5).in_contact = true;
  scm->AcceptStep();
  long revision = scm->get_revision();
  int steps = 0;
  for(; steps < 1000; steps++) {
    scm->ClearTrials();
    scm->AcceptStep();
    if(scm->get_revision() == revision) {
      break; // settled
    }
    revision = scm->get_revision();
  }
  EXPECT_LT(steps, 1000);
  EXPECT_NEAR(LevelSum(*scm), volume, 1e-12 * std::abs(volume));
  EXPECT_LE(SteepestSlope(*scm), std::tan(0.5) + 2e-6);
  /* the pile spread around the spike, also over the node the body was on */
  EXPECT_GT(scm->FindNode(10, 5)->level, -0.3);
  EXPECT_LT(scm->FindNode(12, 5)->level, 0.0);
}

/* a plate set into a pile cuts it down to its own depth and the soil is
pushed on: the plate settles to the sinkage of Bekker's law below the
undisturbed surface, as without the pile, and the volume of soil is that of
the pile */
TEST(Bulldozing, PlateCutsThroughAPile) {
  shift::core::Context context;
  Load(context, "soil_pile",
       PlateModel(Grid("scm", kBulldozing), kSoilContact, kPlate, 4.0));
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  const long nx = scm->get_node_count_x();
  std::vector<float> levels(
    static_cast<size_t>(nx * scm->get_node_count_y()), 0.0f);
  for(long j = 3; j <= 7; j++) {
    for(long i = 8; i <= 12; i++) {
      levels[static_cast<size_t>(j * nx + i)] = -0.02f;
    }
  }
  scm->SetLevels(levels);
  const double volume = LevelSum(*scm);
  context.Simulate();

  const double h = 0.1, b = 0.25;
  const double p = kMass * 9.81 / (25.0 * h * h);
  const double s = std::pow(p / (kKc / b + kKphi), 1.0 / kN);
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "plate") {
      EXPECT_NEAR(context.get_body_origin_pose(body).t.z + 0.05, s, 1e-3 * s);
    }
  }
  EXPECT_NEAR(LevelSum(*scm), volume, 1e-12 * std::abs(volume));
  for(long j = 3; j <= 7; j++) {
    for(long i = 8; i <= 12; i++) {
      EXPECT_GT(scm->FindNode(i, j)->level, 0.0) << i << " " << j;
    }
  }
}

/* without bulldozing nothing is pushed aside */
TEST(Bulldozing, OffByDefault) {
  shift::core::Context context;
  Load(context, "soil_no_bulldozing",
       PlateModel(Grid("scm"), kSoilContact, kPlate, 1.0));
  context.Simulate();
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  EXPECT_FALSE(scm->get_bulldozing().is_enabled());
  EXPECT_EQ(scm->FindNode(7, 5), nullptr);
  EXPECT_GT(LevelSum(*scm), 0.0);
}

/* the message of the exception loading model throws, empty if none */
static std::string LoadError(const std::string& name, const std::string& model) {
  shift::core::Context context;
  try {
    Load(context, name, model);
  } catch(const std::exception& e) {
    return e.what();
  }
  return "";
}

TEST(SoilContact, Checks) {
  EXPECT_NE(LoadError("soil_no_terrain",
                      PlateModel("", kSoilContact, kPlate, 0.0))
              .find("needs a terrain of type \"scm\""),
            std::string::npos);
  EXPECT_NE(LoadError("soil_no_geometry",
                      PlateModel(Grid("scm"), kSoilContact, "", 0.0))
              .find("body plate has no collision geometry"),
            std::string::npos);
  EXPECT_NE(LoadError("soil_no_body",
                      PlateModel(Grid("scm"),
                                 R"({"type": "soil_contact", "name": "soil",
                                     "bodies": ["wheel"]})",
                                 kPlate, 0.0))
              .find("body wheel not found"),
            std::string::npos);
}

/* ------------------------------------------------------------------------ */
/* a track shoe with its grousers on the exact faces of its STEP file */

/* a link of the excavator track (examples/cad/track_link.step: link plates,
bushing, pin and a shoe 0.18 m by 0.6 m with three grousers 26 mm high) on a
vertical prismatic joint, its grousers' tips (z = 0.092 of the link) on the
surface at q = 0, over dry sand on a grid of 5 mm, its nodes half a spacing off
the shoe's edges. The link presses on the soil with its CAD geometry, or with
a box of the shoe and its grousers, their tips the box's bottom. */
static std::string ShoeOnSand(bool cad) {
  std::ostringstream model;
  model << R"({"model_parameters": {"version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0, 0, 9.81],
    "include_directories": [")"
        << SHIFT_EXAMPLES_DIR << R"(/cad"],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": 0.0, "time_step": 0.01}]},
  "frames": [{"name": "terrain", "T": [0, 0, 0]}],
  "terrain": [{"type": "scm", "name": "sand", "parent": [["fixed"], "terrain"],
    "x": [-0.0525, 0.2475], "y": [-0.3525, 0.3525], "spacing": 0.005,
    "max_sinkage": 0.2, "soil": {"name": "dry_sand"}}],
  "assemblies": [{"name": "a",
    "force_modules": [{"type": "soil_contact", "name": "soil",
                       "bodies": ["link"]}],
    "bodies": [{"name": "link",
      "geometry": [{"type": "cad", "filename": "track_link.step"}],
      "collision_geometry": [)"
        << (cad ? R"({"name": "link", "type": "cad", "contact": false})"
                : R"({"name": "link", "type": "box", "contact": false,
                      "half_extents": [0.09, 0.3, 0.021],
                      "pose": {"T": [0.095, 0.0, 0.071]}})")
        << R"(],
      "center_of_mass": [0.0974, 0.0, 0.0359],
      "inertia": {"mass": 38.06, "Ixx": 0.75, "Iyy": 0.15, "Izz": 0.8,
                  "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [{"name": "slide", "type": "prismatic",
                "parent": [["fixed"], "origin"],
                "child": [["link"], "origin"], "axis": [0, 0, 1],
                "initial_displacement": -0.092}]}]})";
  return model.str();
}

struct Shoe {
  explicit Shoe(bool cad) {
    ::Load(context_, cad ? "shoe_cad" : "shoe_box", ShoeOnSand(cad));
    for(auto module : context_.get_active_force_modules()) {
      if(auto found =
           dynamic_cast<shift::core::force_module::SoilContact*>(module)) {
        soil_ = found;
      }
    }
    EXPECT_NE(soil_, nullptr);
    slide_ = context_["assemblies"]["a"]["joints"]["slide"]
               .get_data<shift::core::joint::Base>();
  }

  /* the normal force of fresh soil with the grousers' tips at depth s */
  double NormalForce(double s) {
    context_.get_q()[slide_->get_q_index()] = s - 0.092;
    context_.UpdateDHVA();
    soil_->UpdateOutputs(0.0, nullptr);
    return soil_->get_normal_force();
  }

  /* the depth of the tips at which the soil carries load */
  double Sinkage(double load) {
    double low = 0.0, high = 0.15;
    for(int k = 0; k < 30; k++) {
      const double s = 0.5 * (low + high);
      (NormalForce(s) < load ? low : high) = s;
    }
    return 0.5 * (low + high);
  }

  shift::core::Context context_;
  shift::core::force_module::SoilContact* soil_ = nullptr;
  shift::core::joint::Base* slide_ = nullptr;
};

/* the box covers 36 x 120 nodes: a patch of b = 2 A / P =
36 * 120 * h / (36 + 120), on which the force is Bekker's law over the nodes'
area */
TEST(ShoeOnSand, BoxFollowsBekker) {
  Shoe shoe(false);
  ASSERT_NE(shoe.soil_, nullptr);
  shift::core::force_module::SoilProperties sand;
  ASSERT_TRUE(shift::core::force_module::LookupSoil("dry_sand", sand));
  const double h = 0.005, nodes = 36.0 * 120.0;
  const double b = 36.0 * 120.0 * h / (36.0 + 120.0);
  for(double s : {0.01, 0.03, 0.06}) {
    const double expected = nodes * h * h * sand.Pressure(b, s);
    EXPECT_NEAR(shoe.NormalForce(s), expected, 1e-9 * expected) << s;
    EXPECT_EQ(shoe.soil_->get_contact_nodes(), nodes);
  }
}

/* on the exact faces of the STEP file, only the grousers' tips touch at
first, so at a depth less than their height the shoe carries less than the
box; carrying a load, it sinks deeper, by less than the grousers' height, as
the plate between them comes onto the soil */
TEST(ShoeOnSand, CadGrousersSinkDeeperThanAFlatShoe) {
  const double load = 3000.0, grouser = 0.026;
  Shoe box(false), cad(true);
  ASSERT_NE(box.soil_, nullptr);
  ASSERT_NE(cad.soil_, nullptr);
  const double box_at_10mm = box.NormalForce(0.01);
  const double cad_at_10mm = cad.NormalForce(0.01);
  const double cad_nodes_at_10mm = cad.soil_->get_contact_nodes();
  const double box_sinkage = box.Sinkage(load);
  const double cad_sinkage = cad.Sinkage(load);
  std::printf("at 10 mm: box %.0f N, cad %.0f N on %.0f nodes; carrying "
              "%.0f N: box %.2f mm, cad %.2f mm\n",
              box_at_10mm, cad_at_10mm, cad_nodes_at_10mm, load,
              1000.0 * box_sinkage, 1000.0 * cad_sinkage);
  EXPECT_LT(cad_at_10mm, 0.5 * box_at_10mm);
  EXPECT_GT(cad_sinkage, box_sinkage);
  EXPECT_LT(cad_sinkage, box_sinkage + grouser);
}

/* the changes of the levels at each output time (Scm::get_output_changes),
applied in order to levels that start at 0, give the levels of the terrain
at the end; SetLevels restarts the terrain from levels, each rut taken to
have sprung back */
TEST(SoilContact, OutputChanges) {
  shift::core::Context context;
  Load(context, "soil_output",
       PlateModel(Grid("scm"), kSoilContact, kPlate, 1.0));
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  const long nodes = scm->get_node_count_x() * scm->get_node_count_y();
  std::vector<float> levels(size_t(nodes), 0.0f);
  int outputs = 0, changes = 0;
  context.set_state_observer([&](double, const std::vector<double>&) {
    int32_t previous = -1;
    for(const auto& change : scm->get_output_changes()) {
      EXPECT_GT(change.index, previous); // increasing, each node once
      previous = change.index;
      levels[size_t(change.index)] = change.level;
      changes++;
    }
    outputs++;
  });
  context.Simulate();
  EXPECT_GT(outputs, 10);
  EXPECT_GE(changes, 25);
  std::vector<float> end;
  scm->GetLevels(end);
  EXPECT_EQ(levels, end);

  const long i = 10, j = 5, k = j * scm->get_node_count_x() + i;
  ASSERT_GT(end[size_t(k)], 0.0f);
  scm->SetLevels(end);
  const auto* node = scm->FindNode(i, j);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->level, double(end[size_t(k)]));
  EXPECT_NEAR(scm->ReboundLevel(node->plastic_sinkage), node->level,
              1e-12);
  EXPECT_FALSE(node->in_contact);
  EXPECT_EQ(scm->get_changed_node_count(), 25u);
  EXPECT_TRUE(scm->get_output_changes().empty());
  std::vector<float> zero(size_t(nodes), 0.0f);
  scm->SetLevels(zero);
  EXPECT_EQ(scm->get_changed_node_count(), 0u);
}

/* the HDF5 logger writes the changes of the levels at each output time
under /terrain/<name>: applied in order to a grid of zeros, those of all the
rows give the levels at the end */
TEST(SoilContact, Hdf5LogsLevelChanges) {
  const std::string h5 = testing::TempDir() + "/soil_levels.h5";
  std::string model = PlateModel(Grid("scm"), kSoilContact, kPlate, 1.0);
  const std::string integrators = R"("integrators")";
  model.replace(model.find(integrators), 0,
                R"("loggers": [{"type": "hdf5", "logfile": ")" + h5 +
                  R"("}], )");
  shift::core::Context context;
  Load(context, "soil_hdf5", model);
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  context.Simulate();
  std::vector<float> end;
  scm->GetLevels(end);
  const long nx = scm->get_node_count_x(), ny = scm->get_node_count_y();

  H5::H5File file(h5, H5F_ACC_RDONLY);
  auto length = [](const H5::DataSet& dataset) {
    hsize_t n = 0;
    dataset.getSpace().getSimpleExtentDims(&n);
    return n;
  };
  const hsize_t rows = length(file.openDataSet("/time"));
  H5::Group group = file.openGroup("/terrain/site");
  int64_t counts[2];
  group.openAttribute("node_count").read(H5::PredType::NATIVE_INT64, counts);
  EXPECT_EQ(counts[0], nx);
  EXPECT_EQ(counts[1], ny);
  double spacing = 0.0, x0 = 0.0;
  group.openAttribute("spacing").read(H5::PredType::NATIVE_DOUBLE, &spacing);
  group.openAttribute("x0").read(H5::PredType::NATIVE_DOUBLE, &x0);
  EXPECT_EQ(spacing, 0.1);
  EXPECT_EQ(x0, -1.0);
  std::vector<double> undisturbed(size_t(nx * ny), 1.0);
  group.openDataSet("undisturbed_z")
    .read(undisturbed.data(), H5::PredType::NATIVE_DOUBLE);
  EXPECT_EQ(undisturbed, std::vector<double>(size_t(nx * ny), 0.0));

  H5::DataSet row_start = group.openDataSet("row_start");
  H5::DataSet index = group.openDataSet("index");
  H5::DataSet level = group.openDataSet("level");
  ASSERT_EQ(length(row_start), rows);
  ASSERT_GT(rows, 10u);
  std::vector<int64_t> starts(rows);
  row_start.read(starts.data(), H5::PredType::NATIVE_INT64);
  const hsize_t changes = length(index);
  ASSERT_EQ(length(level), changes);
  ASSERT_GE(changes, 25u);
  std::vector<int32_t> indices(changes);
  std::vector<float> levels(changes);
  index.read(indices.data(), H5::PredType::NATIVE_INT32);
  level.read(levels.data(), H5::PredType::NATIVE_FLOAT);
  EXPECT_EQ(starts[0], 0);
  EXPECT_EQ(starts[1], 0); // the plate starts on the surface
  std::vector<float> replayed(size_t(nx * ny), 0.0f);
  for(hsize_t r = 0; r < rows; r++) {
    const int64_t last = r + 1 < rows ? starts[r + 1] : int64_t(changes);
    ASSERT_LE(starts[r], last);
    for(int64_t k = starts[r]; k < last; k++) {
      replayed[size_t(indices[size_t(k)])] = levels[size_t(k)];
    }
  }
  EXPECT_EQ(replayed, end);
  file.close();
  std::remove(h5.c_str());
}

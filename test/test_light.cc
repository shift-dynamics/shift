/* Lights (shift::core::light): their frames, spot defaults, and the IES
LM-63 reader */

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/frame.h"
#include "shift/core/light/ies.h"
#include "shift/core/light/spot.h"

static constexpr double kDegree = M_PI / 180.0;

static std::string WriteFile(const std::string& name,
                             const std::string& contents) {
  const std::string path = testing::TempDir() + "/" + name;
  std::ofstream file(path);
  file << contents;
  return path;
}

/* type C, symmetric in each quadrant (horizontal angles 0 and 90),
vertical angles 0, 45, 90, candela multiplier 2 and tilt data to skip; the
values are separated by blanks, commas and line breaks */
static const char* kQuadrantIes = R"(IESNA:LM-63-2002
[TEST] quadrant symmetric
[MANUFAC] none
TILT=INCLUDE
1
3
0 45 90
1.0 0.9 0.8
1 1000 2.0 3 2 1 2 0.5 0.5 0.1
1.0 1.0 50
0, 45, 90
0 90
100 50 0
100 20 0
)";

TEST(IesLight, ReadsTypeCWithQuadrantSymmetry) {
  shift::core::light::IES ies("test");
  ies.Read(WriteFile("quadrant.ies", kQuadrantIes));
  EXPECT_DOUBLE_EQ(ies.get_peak_candela(), 200.0);
  // nadir: the peak, in every horizontal direction
  EXPECT_DOUBLE_EQ(ies.Distribution(0.0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(ies.Distribution(0.0, 1.0), 1.0);
  // the two given planes, and their mirror images in the other quadrants
  EXPECT_NEAR(ies.Distribution(45 * kDegree, 0.0), 0.5, 1e-12);
  EXPECT_NEAR(ies.Distribution(45 * kDegree, 90 * kDegree), 0.2, 1e-12);
  EXPECT_NEAR(ies.Distribution(45 * kDegree, 180 * kDegree), 0.5, 1e-12);
  EXPECT_NEAR(ies.Distribution(45 * kDegree, 270 * kDegree), 0.2, 1e-12);
  EXPECT_NEAR(ies.Distribution(45 * kDegree, -90 * kDegree), 0.2, 1e-12);
  // linear between the angles of the file
  EXPECT_NEAR(ies.Distribution(45 * kDegree, 45 * kDegree), 0.35, 1e-12);
  EXPECT_NEAR(ies.Distribution(22.5 * kDegree, 0.0), 0.75, 1e-12);
  EXPECT_NEAR(ies.Distribution(45 * kDegree, 135 * kDegree), 0.35, 1e-12);
  // no light above the vertical angles of the file
  EXPECT_DOUBLE_EQ(ies.Distribution(120 * kDegree, 0.0), 0.0);
}

TEST(IesLight, ReadsRotationallySymmetricAndBilateralFiles) {
  shift::core::light::IES round("round");
  round.Read(WriteFile("round.ies", "IESNA91\nTILT=NONE\n"
                                    "1 -1 1 3 1 1 2 0 0 0\n1 1 10\n"
                                    "0 90 180\n0\n10 40 20\n"));
  EXPECT_DOUBLE_EQ(round.get_peak_candela(), 40.0);
  EXPECT_NEAR(round.Distribution(90 * kDegree, 2.0), 1.0, 1e-12);
  EXPECT_NEAR(round.Distribution(135 * kDegree, 5.0), 0.75, 1e-12);

  // symmetric about the 0-180 plane
  shift::core::light::IES bilateral("bilateral");
  bilateral.Read(WriteFile("bilateral.ies", "TILT=NONE\n"
                                            "1 -1 1 2 3 1 2 0 0 0\n1 1 10\n"
                                            "0 90\n0 90 180\n"
                                            "4 4\n4 2\n4 1\n"));
  EXPECT_NEAR(bilateral.Distribution(90 * kDegree, 90 * kDegree), 0.5, 1e-12);
  EXPECT_NEAR(bilateral.Distribution(90 * kDegree, 270 * kDegree), 0.5,
              1e-12);
  EXPECT_NEAR(bilateral.Distribution(90 * kDegree, 225 * kDegree), 0.375,
              1e-12);
}

TEST(IesLight, RejectsFilesItCannotRead) {
  shift::core::light::IES ies("bad");
  // no TILT line
  EXPECT_THROW(ies.Read(WriteFile("no_tilt.ies", "IESNA:LM-63-2002\n1 2 3\n")),
               shift::core::Exception);
  // type B photometry
  EXPECT_THROW(ies.Read(WriteFile("type_b.ies", "TILT=NONE\n"
                                                "1 -1 1 2 1 2 2 0 0 0\n1 1 10\n"
                                                "0 90\n0\n1 1\n")),
               shift::core::Exception);
  // ends early
  EXPECT_THROW(ies.Read(WriteFile("short.ies", "TILT=NONE\n"
                                               "1 -1 1 2 1 1 2 0 0 0\n1 1 10\n"
                                               "0 90\n0\n1\n")),
               shift::core::Exception);
}

/* lights on a frame of the fixed body and of a moving body, and one whose
frame does not exist */
TEST(Light, ConnectsToTheFrameOfItsBody) {
  WriteFile("lamp.ies", kQuadrantIes);
  const std::string path = testing::TempDir() + "/lights.json";
  std::ostringstream model;
  model << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "include_directories": [")" << testing::TempDir() << R"("],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "rk4", "time_span": 0.01, "time_step": 0.01}]
  },
  "frames": [{"name": "lamp", "T": [1, 2, -3]}],
  "lights": [
    {"name": "bulb", "type": "point", "parent": ["fixed", "lamp"],
     "Kd": [0.5, 0.6, 0.7], "intensity": 2, "attenuation": [0, 0, 1]},
    {"name": "beam", "type": "spot", "parent": ["fixed", "lamp"],
     "cone_angle": 0.4},
    {"name": "lost", "type": "point", "parent": ["fixed", "nowhere"]}
  ],
  "assemblies": [{"name": "arm",
    "bodies": [{"name": "arm", "geometry": [], "center_of_mass": [0, 0, 0],
                "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1,
                            "Ixy": 0, "Iyz": 0, "Izx": 0},
                "frames": [{"name": "tip", "T": [0, 0, 2]}],
                "lights": [{"name": "head", "type": "ies",
                            "parent": ["arm", "tip"],
                            "filename": "lamp.ies"}]}],
    "joints": [{"name": "A", "type": "revolute",
                "parent": [["fixed"], "origin"], "child": [["arm"], "origin"],
                "axis": [1, 0, 0]}]
  }]
})";
  {
    std::ofstream file(path);
    file << model.str();
  }
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());

  const auto& lights = context.get_active_lights();
  ASSERT_EQ(lights.size(), 4u);
  int found = 0;
  for(auto light : lights) {
    if(light->get_name() == "bulb") {
      found++;
      ASSERT_NE(light->get_frame(), nullptr);
      EXPECT_EQ(light->get_frame()->get_name(), "lamp");
      EXPECT_EQ(light->get_parent_body(), context.get_fixed_body());
      EXPECT_DOUBLE_EQ(light->get_frame()->get_d_wrt_origin()->t.z, -3.0);
      EXPECT_DOUBLE_EQ(light->get_Kd().y, 0.6);
      EXPECT_DOUBLE_EQ(light->get_intensity(), 2.0);
      EXPECT_DOUBLE_EQ(light->get_attenuation().z, 1.0);
    } else if(light->get_name() == "beam") {
      found++;
      auto spot = dynamic_cast<const shift::core::light::Spot*>(light);
      ASSERT_NE(spot, nullptr);
      EXPECT_DOUBLE_EQ(spot->get_cone_angle(), 0.4);
      EXPECT_DOUBLE_EQ(spot->get_inner_cone_angle(), 0.8 * 0.4);
    } else if(light->get_name() == "lost") {
      found++;
      EXPECT_EQ(light->get_frame(), nullptr);
    } else if(light->get_name() == "head") {
      found++;
      ASSERT_NE(light->get_frame(), nullptr);
      EXPECT_EQ(light->get_frame()->get_name(), "tip");
      ASSERT_NE(light->get_parent_body(), nullptr);
      EXPECT_EQ(light->get_parent_body()->get_name(), "arm");
      auto ies = dynamic_cast<const shift::core::light::IES*>(light);
      ASSERT_NE(ies, nullptr);
      EXPECT_DOUBLE_EQ(ies->get_peak_candela(), 200.0);
    }
  }
  EXPECT_EQ(found, 4);
}

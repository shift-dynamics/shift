/* Hydraulic cylinder force module (force_module::HydraulicCylinder): a held
load, the speed of an open valve, and the relief valve */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <H5Cpp.h>
#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/hydraulic_cylinder.h"
#include "shift/core/force_module/table.h"

using shift::core::force_module::HydraulicCylinder;
using shift::core::force_module::Table;

TEST(Table, PassesThroughItsPointsWithoutOvershoot) {
  const Table table({0.0, 1.0, 2.0, 3.0, 4.0}, {0.0, 0.0, 1.0, 1.0, 3.0},
                    Table::Extrapolation::kHold);
  const std::vector<double> x = {0.0, 1.0, 2.0, 3.0, 4.0};
  const std::vector<double> y = {0.0, 0.0, 1.0, 1.0, 3.0};
  for(size_t k = 0; k < x.size(); k++) {
    EXPECT_NEAR(table.Evaluate(x[k]), y[k], 1e-14);
  }
  /* level where the data are level, rising where they rise */
  double previous = table.Evaluate(0.0);
  for(double t = 0.0; t <= 4.0; t += 0.01) {
    const double value = table.Evaluate(t);
    EXPECT_GE(value, previous - 1e-14) << t;
    if(t > 0.0 && t < 1.0) {
      EXPECT_NEAR(value, 0.0, 1e-14) << t;
    }
    if(t > 2.0 && t < 3.0) {
      EXPECT_NEAR(value, 1.0, 1e-14) << t;
    }
    previous = value;
  }
  EXPECT_EQ(table.Evaluate(-5.0), 0.0);
  EXPECT_EQ(table.Evaluate(9.0), 3.0);
}

TEST(Table, SlopeIsTheDerivativeAndContinuous) {
  const Table table({0.0, 0.5, 2.0, 2.5}, {1.0, 2.0, 2.5, 5.0},
                    Table::Extrapolation::kLinear);
  const double h = 1e-6;
  for(double x = -0.5; x <= 3.0; x += 0.0625) {
    double slope;
    table.Evaluate(x, &slope);
    const double rate =
      (table.Evaluate(x + h) - table.Evaluate(x - h)) / (2.0 * h);
    /* central differences across a point, where the curvature changes,
    are good to the order of h */
    EXPECT_NEAR(slope, rate, 1e-5 * (1.0 + std::fabs(slope))) << x;
  }
  /* continuing with the end slopes */
  double end_slope;
  table.Evaluate(2.5, &end_slope);
  EXPECT_NEAR(table.Evaluate(3.0), 5.0 + 0.5 * end_slope, 1e-12);
}

TEST(Table, RejectsUnorderedPoints) {
  EXPECT_THROW(Table::FromPairs({0, 1, 0, 2}, Table::Extrapolation::kHold,
                                "test"),
               shift::core::Exception);
  EXPECT_THROW(Table::FromPairs({0, 1, 1}, Table::Extrapolation::kHold,
                                "test"),
               shift::core::Exception);
}

static const char* kLoadInertia =
  R"("center_of_mass": [0, 0, 0], "inertia": {"mass": 500, "Ixx": 10,
     "Iyy": 10, "Izz": 10, "Ixy": 0, "Iyz": 0, "Izx": 0})";

/* a body on a prismatic joint along axis, driven by a cylinder whose other
parameters are given in cylinder */
static void LoadCylinder(shift::core::Context& context, const std::string& name,
                         const std::string& axis, const std::string& cylinder,
                         double time_span,
                         const std::string& loggers = "",
                         const std::string& extra_modules = "") {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "loggers": [)" << loggers << R"(],
    "integrators": [{"type": "ode113", "time_span": )"
         << time_span << R"(, "time_step": 0.01,
                     "relative_error": 1e-7, "absolute_error": 1e-9}]
  },
  "assemblies": [{"name": "test",
    "bodies": [{"name": "load", )"
         << kLoadInertia << R"(}],
    "joints": [{"name": "slide", "type": "prismatic",
                "parent": [["fixed"], "origin"],
                "child": [["load"], "origin"], "axis": )"
         << axis << R"(}],
    "force_modules": [{"type": "hydraulic_cylinder", "name": "cylinder",
      "joint": "slide", "stroke": 0.5, "bore_diameter": 0.05,
      "rod_diameter": 0.03, )"
         << cylinder << R"(})" << extra_modules << R"(]
  }]
})";
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

static HydraulicCylinder* FindCylinder(shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto cylinder = dynamic_cast<HydraulicCylinder*>(module)) {
      return cylinder;
    }
  }
  return nullptr;
}

/* the final state of a simulation */
static std::vector<double> Simulate(shift::core::Context& context) {
  std::vector<double> state;
  context.set_state_observer(
    [&state](double, const std::vector<double>& y) { state = y; });
  context.Simulate();
  return state;
}

/* a load hanging on a cylinder pointing up, its valve closed: the load sinks
until the oil it compresses carries its weight */
TEST(HydraulicCylinder, ClosedValveHoldsTheLoad) {
  shift::core::Context context;
  /* the joint coordinate grows downward (z down), so the cylinder extends
  as it shrinks; mid stroke at q = 0 */
  LoadCylinder(context, "cylinder_hold", "[0, 0, 1]",
               R"("direction": -1, "retracted_position": 0.25,
                  "supply_pressure": 10e6, "relief_pressure": 20e6,
                  "rated_flow": 5e-5, "friction": 7e4)",
               3.0);
  auto cylinder = FindCylinder(context);
  ASSERT_NE(cylinder, nullptr);
  const std::vector<double> y = Simulate(context);
  ASSERT_FALSE(y.empty());
  const double weight = 500.0 * 9.81;
  EXPECT_NEAR(cylinder->get_force(), weight, 1e-3 * weight);
  EXPECT_NEAR(y[context.get_q_count()], 0.0, 1e-5); /* at rest */
  /* the head side carries the weight; the rod side stays near the tank
  pressure, topped up by its makeup valve as it grows (and a little under
  it, so the head carries a little less than the weight over its area) */
  EXPECT_NEAR(cylinder->get_head_pressure(),
              weight / cylinder->get_head_area(),
              1e-2 * weight / cylinder->get_head_area());
  EXPECT_LT(std::fabs(cylinder->get_rod_pressure()), 0.05e6);
  /* the load sank by about weight / (hydraulic stiffness) */
  const double sink = y[0];
  EXPECT_GT(sink, 0.0);
  EXPECT_LT(sink, 1e-3);
}

static const char* kHanging =
  R"("direction": -1, "retracted_position": 0.25,
     "supply_pressure": 10e6, "relief_pressure": 20e6,
     "rated_flow": 5e-5, "friction": 7e4, "hold": true)";

/* started with the pressures that hold it, the hanging load does not move:
the head side carries the weight, the rod side is at the tank pressure */
TEST(HydraulicCylinder, HoldStartsWithThePressuresThatHoldTheLoad) {
  shift::core::Context context;
  LoadCylinder(context, "cylinder_start_held", "[0, 0, 1]", kHanging, 1.0);
  auto cylinder = FindCylinder(context);
  ASSERT_NE(cylinder, nullptr);
  const std::vector<double> y = Simulate(context);
  ASSERT_FALSE(y.empty());
  const double weight = 500.0 * 9.81;
  EXPECT_NEAR(y[0], 0.0, 1e-9);
  EXPECT_NEAR(y[context.get_q_count()], 0.0, 1e-9);
  EXPECT_NEAR(cylinder->get_force(), weight, 1e-6 * weight);
  EXPECT_NEAR(cylinder->get_head_pressure(),
              weight / cylinder->get_head_area(), 1.0);
  EXPECT_NEAR(cylinder->get_rod_pressure(), 0.0, 1e-3);
}

/* two cylinders side by side on the same joint share the load equally */
TEST(HydraulicCylinder, HoldSharesALoadEqually) {
  shift::core::Context context;
  LoadCylinder(context, "cylinder_pair_held", "[0, 0, 1]", kHanging, 0.5, "",
               std::string(R"(, {"type": "hydraulic_cylinder",
                  "name": "second", "joint": "slide", "stroke": 0.5,
                  "bore_diameter": 0.05, "rod_diameter": 0.03, )") +
                 kHanging + "}");
  std::vector<HydraulicCylinder*> cylinders;
  for(auto module : context.get_active_force_modules()) {
    if(auto cylinder = dynamic_cast<HydraulicCylinder*>(module)) {
      cylinders.push_back(cylinder);
    }
  }
  ASSERT_EQ(cylinders.size(), 2u);
  const std::vector<double> y = Simulate(context);
  ASSERT_FALSE(y.empty());
  const double weight = 500.0 * 9.81;
  EXPECT_NEAR(y[context.get_q_count()], 0.0, 1e-9);
  EXPECT_NEAR(cylinders[0]->get_force(), 0.5 * weight, 1e-6 * weight);
  EXPECT_NEAR(cylinders[1]->get_force(), 0.5 * weight, 1e-6 * weight);
}

TEST(HydraulicCylinder, HoldOrInitialPressuresNotBoth) {
  shift::core::Context context;
  EXPECT_THROW(LoadCylinder(context, "cylinder_hold_and_pressures",
                            "[0, 0, 1]",
                            std::string(kHanging) +
                              R"(, "initial_pressures": [1e6, 0])",
                            0.1),
               shift::core::Exception);
}

/* the valve fully open on a horizontal cylinder: it extends at the speed at
which the flows through the two open edges give pressures that balance the
seal friction */
TEST(HydraulicCylinder, OpenValveExtendsAtTheOrificeSpeed) {
  const double supply = 10e6, rated_flow = 4.5e-5, rated_drop = 5e5;
  const double friction = 1e4;
  shift::core::Context context;
  std::ostringstream parameters;
  parameters << R"("retracted_position": -0.1, "supply_pressure": )" << supply
             << R"(, "relief_pressure": 20e6, "rated_flow": )" << rated_flow
             << R"(, "rated_pressure_drop": )" << rated_drop
             << R"(, "friction": )" << friction << R"(, "command": 1)";
  LoadCylinder(context, "cylinder_speed", "[1, 0, 0]", parameters.str(), 0.5);
  auto cylinder = FindCylinder(context);
  ASSERT_NE(cylinder, nullptr);
  const std::vector<double> y = Simulate(context);
  ASSERT_FALSE(y.empty());
  /* steady speed v: head pressure supply - (A_h v / Q)^2 dp, rod pressure
  (A_r v / Q)^2 dp, and p_h A_h - p_r A_r = c v; solved by bisection */
  const double area_head = cylinder->get_head_area();
  const double area_rod = cylinder->get_rod_area();
  auto residual = [&](double v) {
    const double head =
      supply - std::pow(area_head * v / rated_flow, 2) * rated_drop;
    const double rod = std::pow(area_rod * v / rated_flow, 2) * rated_drop;
    return head * area_head - rod * area_rod - friction * v;
  };
  double low = 0.0, high = 1.0;
  for(int k = 0; k < 100; k++) {
    const double mid = 0.5 * (low + high);
    (residual(mid) > 0.0 ? low : high) = mid;
  }
  const double speed = 0.5 * (low + high);
  ASSERT_GT(speed, 0.05);
  EXPECT_NEAR(y[context.get_q_count()], speed, 5e-3 * speed);
  EXPECT_NEAR(cylinder->get_head_flow(), area_head * speed, 5e-3 * area_head *
                                                               speed);
  EXPECT_NEAR(cylinder->get_rod_flow(), -area_rod * speed,
              5e-3 * area_rod * speed);
  EXPECT_NEAR(cylinder->get_opening(), 1.0, 0.0);
}

/* fully extended against its stop, the valve open: the head pressure rises
past the relief pressure until the relief valve passes the valve's flow */
TEST(HydraulicCylinder, ReliefValveLimitsThePressure) {
  const double supply = 10e6, relief = 8e6;
  shift::core::Context context;
  std::ostringstream parameters;
  parameters << R"("retracted_position": -0.5, "supply_pressure": )" << supply
             << R"(, "relief_pressure": )" << relief
             << R"(, "rated_flow": 4.5e-5, "friction": 1e4,
                   "end_stop_stiffness": 1e8, "end_stop_damping": 10,
                   "command": 1)";
  LoadCylinder(context, "cylinder_relief", "[1, 0, 0]", parameters.str(), 1.0);
  auto cylinder = FindCylinder(context);
  ASSERT_NE(cylinder, nullptr);
  const std::vector<double> y = Simulate(context);
  ASSERT_FALSE(y.empty());
  /* the valve's flow into the head side, Q sqrt((p_s - p) / dp), equals
  the relief valve's flow out, g (p - p_r - w / 2) with the default gain
  g = Q / (0.1 p_r) and band w = 0.02 p_r; solved by bisection */
  const double rated_flow = 4.5e-5, rated_drop = 5e5;
  const double gain = rated_flow / (0.1 * relief), band = 0.02 * relief;
  auto residual = [&](double p) {
    return rated_flow * std::sqrt((supply - p) / rated_drop) -
           gain * (p - relief - 0.5 * band);
  };
  double low = relief + band, high = supply;
  for(int k = 0; k < 100; k++) {
    const double mid = 0.5 * (low + high);
    (residual(mid) > 0.0 ? low : high) = mid;
  }
  const double expected = 0.5 * (low + high);
  const double head = cylinder->get_head_pressure();
  EXPECT_GT(head, relief);
  EXPECT_LT(head, supply);
  EXPECT_NEAR(head, expected, 1e-3 * expected);
  /* pressed against the stop, which carries the cylinder's force */
  EXPECT_GT(cylinder->get_extension(), 0.5);
  EXPECT_LT(cylinder->get_extension(), 0.5 + 1e-3);
  EXPECT_NEAR(y[context.get_q_count()], 0.0, 1e-5);
}

/* the CSV and HDF5 loggers record the cylinder's outputs of each logged
state: the last row holds the outputs of the final state */
TEST(HydraulicCylinder, LoggersRecordTheOutputs) {
  const std::string csv = testing::TempDir() + "/cylinder_outputs.csv";
  const std::string h5 = testing::TempDir() + "/cylinder_outputs.h5";
  shift::core::Context context;
  LoadCylinder(context, "cylinder_outputs", "[0, 0, 1]",
               R"("direction": -1, "retracted_position": 0.25,
                  "supply_pressure": 10e6, "relief_pressure": 20e6,
                  "rated_flow": 5e-5, "friction": 7e4,
                  "command": 0.5)",
               0.3,
               R"({"type": "csv", "csv_logfile": ")" + csv +
                 R"(", "precision": 12},
                  {"type": "hdf5", "logfile": ")" + h5 + R"("})");
  auto cylinder = FindCylinder(context);
  ASSERT_NE(cylinder, nullptr);
  Simulate(context);
  const std::vector<std::string> names = {
    "head_pressure", "rod_pressure", "force",   "extension",
    "opening",       "head_flow",    "rod_flow"};
  const std::vector<double> expected = {
    cylinder->get_head_pressure(), cylinder->get_rod_pressure(),
    cylinder->get_force(),         cylinder->get_extension(),
    cylinder->get_opening(),       cylinder->get_head_flow(),
    cylinder->get_rod_flow()};
  EXPECT_GT(cylinder->get_head_pressure(), 1e6);

  /* CSV: the columns cylinder_<output>, last row */
  std::vector<std::string> header, last;
  {
    std::ifstream file(csv);
    std::string line, row;
    std::getline(file, line);
    while(std::getline(file, row)) {
      if(!row.empty()) {
        last.clear();
        std::stringstream cells(row);
        for(std::string cell; std::getline(cells, cell, ',');) {
          last.push_back(cell);
        }
      }
    }
    std::stringstream cells(line);
    for(std::string cell; std::getline(cells, cell, ',');) {
      header.push_back(cell);
    }
  }
  for(size_t k = 0; k < names.size(); k++) {
    const auto it =
      std::find(header.begin(), header.end(), "cylinder_" + names[k]);
    ASSERT_NE(it, header.end()) << names[k];
    const size_t column = it - header.begin();
    ASSERT_LT(column, last.size());
    EXPECT_NEAR(std::stod(last[column]), expected[k],
                1e-10 * (1.0 + std::fabs(expected[k])))
      << names[k];
  }

  /* HDF5: /force_modules/cylinder/<output>, as many rows as /time */
  {
    H5::H5File file(h5, H5F_ACC_RDONLY);
    hsize_t rows = 0;
    file.openDataSet("/time").getSpace().getSimpleExtentDims(&rows);
    ASSERT_GT(rows, 2u);
    for(size_t k = 0; k < names.size(); k++) {
      H5::DataSet dataset =
        file.openDataSet("/force_modules/cylinder/" + names[k]);
      hsize_t count = 0;
      dataset.getSpace().getSimpleExtentDims(&count);
      ASSERT_EQ(count, rows) << names[k];
      std::vector<double> values(count);
      dataset.read(values.data(), H5::PredType::NATIVE_DOUBLE);
      EXPECT_EQ(values.back(), expected[k]) << names[k];
    }
    hsize_t dims[2] = {0, 0};
    file.openDataSet("/states/q").getSpace().getSimpleExtentDims(dims);
    EXPECT_EQ(dims[0], rows);
    EXPECT_EQ(dims[1], context.get_q_count());
  }
  std::remove(csv.c_str());
  std::remove(h5.c_str());
}

TEST(HydraulicCylinder, RejectsAJointThatIsNotPrismatic) {
  shift::core::Context context;
  const std::string path = testing::TempDir() + "/cylinder_revolute.json";
  {
    std::ofstream file(path);
    file << R"({
  "model_parameters": {"version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0, 0, 9.81], "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "rk4", "time_span": 0.01, "time_step": 0.01}]},
  "assemblies": [{"name": "test",
    "bodies": [{"name": "load", )"
         << kLoadInertia << R"(}],
    "joints": [{"name": "hinge", "type": "revolute",
                "parent": [["fixed"], "origin"],
                "child": [["load"], "origin"], "axis": [0, 0, 1]}],
    "force_modules": [{"type": "hydraulic_cylinder", "name": "cylinder",
      "joint": "hinge", "stroke": 0.5, "bore_diameter": 0.05,
      "rod_diameter": 0.03, "retracted_position": 0,
      "supply_pressure": 1e7, "relief_pressure": 2e7, "rated_flow": 1e-4}]
  }]
})";
  }
  EXPECT_ANY_THROW(context.LoadJSON(path));
  std::remove(path.c_str());
}

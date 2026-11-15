/* the orientation of a frame may be given by any of the keys R, rotation,
quaternion, q, Q and euler_angles; each gives the same rotation */
#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/frame.h"

#include "ksl/mat3x3.h"
#include "ksl/quaternion.h"

#include <gtest/gtest.h>
#include <json/json.h>

#include <filesystem>
#include <fstream>
#include <string>

static Json::Value Numbers(const double* values, int count) {
  Json::Value array(Json::arrayValue);
  for(int i = 0; i < count; i++) {
    array.append(values[i]);
  }
  return array;
}

static Json::Value Frame(const std::string& name, const std::string& key,
                         const Json::Value& orientation) {
  Json::Value frame;
  frame["name"] = name;
  frame[key] = orientation;
  return frame;
}

TEST(FrameOrientation, EveryKeyGivesTheSameRotation) {
  // R = Rz(0.3) Ry(0.2) Rx(0.1)
  ksl_vec3_t zyx = ksl_vec3(0.3, 0.2, 0.1);
  ksl_mat3x3_t expected;
  ksl_mat3x3_setFromEulerAngles(&expected, KSL_AXIS_ZYX, &zyx);
  // the same rotation about x, y, z: R = Rx(a) Ry(b) Rz(c)
  ksl_vec3_t xyz;
  ksl_mat3x3_getEulerAnglesWithReference(&expected, KSL_AXIS_XYZ, &xyz, NULL);

  Json::Value matrix(Json::arrayValue);
  for(int row = 0; row < 3; row++) {
    Json::Value json_row(Json::arrayValue);
    for(int column = 0; column < 3; column++) {
      json_row.append(expected.at[column][row]);
    }
    matrix.append(json_row);
  }
  ksl_quaternion_t quaternion;
  ksl_mat3x3_toQuaternion(&expected, &quaternion);
  Json::Value euler_zyx;
  euler_zyx["angles"] = Numbers(zyx.at, 3);
  euler_zyx["axes"].append("z");
  euler_zyx["axes"].append("y");
  euler_zyx["axes"].append("x");
  Json::Value euler_210 = euler_zyx;
  euler_210["axes"] = Json::Value(Json::arrayValue);
  for(int axis : {2, 1, 0}) {
    euler_210["axes"].append(axis);
  }
  Json::Value euler_default; // x, y, z
  euler_default["angles"] = Numbers(xyz.at, 3);

  const std::string path =
    std::string(SHIFT_EXAMPLES_DIR) + "/force_modules/gear_mesh.json";
  Json::Value model;
  std::ifstream(path) >> model;
  model["model_parameters"].removeMember("loggers"); // no log files
  Json::Value& frames = model["frames"];
  frames.append(Frame("R", "R", matrix));
  frames.append(Frame("rotation", "rotation", matrix));
  frames.append(Frame("quaternion", "quaternion", Numbers(quaternion.at, 4)));
  frames.append(Frame("q", "q", Numbers(quaternion.at, 4)));
  frames.append(Frame("Q", "Q", Numbers(quaternion.at, 4)));
  frames.append(Frame("euler_zyx", "euler_angles", euler_zyx));
  frames.append(Frame("euler_210", "euler_angles", euler_210));
  frames.append(Frame("euler_default", "euler_angles", euler_default));

  shift::core::Context context;
  context.LoadJSON(model, path);
  int checked = 0;
  for(auto& frame : context.get_frames()) {
    const std::string& name = frame->get_name();
    if(name == "origin" || name == "axis_2") {
      continue;
    }
    const ksl_mat3x3_t& R = frame->get_d_wrt_origin()->R;
    for(int i = 0; i < 9; i++) {
      EXPECT_NEAR(R.as_array[i], expected.as_array[i], 1e-12) << name;
    }
    checked++;
  }
  EXPECT_EQ(checked, 8);
}

/* The "scale" option of mesh geometry (geometry::Mesh): vertices are scaled
about the mesh origin, and normals stay perpendicular to the scaled
surface */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/geometry/mesh.h"

/* one triangle in the plane x + y = 1, with its normal, and a model of a
body that draws it with the given scale; returns the model file */
static std::string WriteScaledTriangle(const std::string& name,
                                       const std::string& scale) {
  const std::string directory = testing::TempDir();
  const std::string obj = directory + "/" + name + ".obj";
  {
    std::ofstream file(obj);
    file << "v 1 0 0\nv 0 1 0\nv 0 1 1\n"
            "vn 0.70710678 0.70710678 0\n"
            "f 1//1 2//1 3//1\n";
  }
  const std::string path = directory + "/" + name + ".json";
  {
    std::ofstream file(path);
    file << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "rk4", "time_span": 0.01, "time_step": 0.01}]
  },
  "assemblies": [{"name": "test",
    "bodies": [{"name": "part", "center_of_mass": [0, 0, 0],
      "inertia": {"mass": 1, "Ixx": 0.01, "Iyy": 0.01, "Izz": 0.01,
                  "Ixy": 0, "Iyz": 0, "Izx": 0},
      "geometry": [{"type": "mesh", "filename": [")"
         << name << R"(.obj"])"
         << scale << R"(}]}],
    "joints": [{"name": "float", "type": "float",
      "parent": [["fixed"], "origin"], "child": [["part"], "origin"]}]
  }]
})";
  }
  return path;
}

/* the mesh of the body of WriteScaledTriangle, loaded */
static const shift::core::geometry::Mesh* LoadScaledTriangle(
  shift::core::Context& context, const std::string& name,
  const std::string& scale) {
  const std::string path = WriteScaledTriangle(name, scale);
  try {
    context.LoadJSON(path);
  } catch(const shift::core::Exception& e) {
    ADD_FAILURE() << e.what();
  }
  std::remove(path.c_str());
  std::remove((testing::TempDir() + "/" + name + ".obj").c_str());
  auto body = context.FindActiveBody("part");
  if(!body || body->get_geometry().empty()) {
    return nullptr;
  }
  return dynamic_cast<const shift::core::geometry::Mesh*>(
    body->get_geometry().front().get());
}

TEST(MeshScale, ScalesVerticesAndNormals) {
  shift::core::Context context;
  auto mesh =
    LoadScaledTriangle(context, "mesh_scale_xyz", R"(, "scale": [2, 1, 0.5])");
  ASSERT_NE(mesh, nullptr);
  ASSERT_EQ(mesh->get_mesh_data().size(), 1u);
  const auto& data = mesh->get_mesh_data()[0];
  ASSERT_EQ(data.vertices.size(), 9u);
  ASSERT_EQ(data.normals.size(), 9u);

  // the scaled vertices: (2, 0, 0), (0, 1, 0), (0, 1, 0.5), in any order
  double sum[3] = {0, 0, 0};
  for(int j = 0; j < 3; j++) {
    for(int k = 0; k < 3; k++) {
      sum[k] += data.vertices[3 * j + k];
    }
  }
  EXPECT_NEAR(sum[0], 2.0, 1e-6);
  EXPECT_NEAR(sum[1], 2.0, 1e-6);
  EXPECT_NEAR(sum[2], 0.5, 1e-6);

  // the scaled plane is x / 2 + y = 1: its normal is (1, 2, 0) / sqrt(5)
  for(int j = 0; j < 3; j++) {
    EXPECT_NEAR(data.normals[3 * j + 0], 1.0 / std::sqrt(5.0), 1e-6);
    EXPECT_NEAR(data.normals[3 * j + 1], 2.0 / std::sqrt(5.0), 1e-6);
    EXPECT_NEAR(data.normals[3 * j + 2], 0.0, 1e-6);
  }
}

TEST(MeshScale, OneFactorScalesUniformly) {
  shift::core::Context context;
  auto mesh = LoadScaledTriangle(context, "mesh_scale_one", R"(, "scale": [3])");
  ASSERT_NE(mesh, nullptr);
  const auto& data = mesh->get_mesh_data().at(0);
  double largest = 0.0;
  for(float v : data.vertices) {
    largest = std::max(largest, double(v));
  }
  EXPECT_NEAR(largest, 3.0, 1e-6);
  EXPECT_NEAR(data.normals[0], 1.0 / std::sqrt(2.0), 1e-6);
}

TEST(MeshScale, RejectsInvalidFactors) {
  for(const char* scale : {R"(, "scale": [1, 2])", R"(, "scale": [1, 0, 1])"}) {
    shift::core::Context context;
    const std::string path = WriteScaledTriangle("mesh_scale_bad", scale);
    std::string message;
    try {
      context.LoadJSON(path);
    } catch(const std::exception& e) {
      message = e.what();
    }
    EXPECT_NE(message.find("Mesh scale"), std::string::npos)
      << scale << ": " << message;
    std::remove(path.c_str());
    std::remove((testing::TempDir() + "/mesh_scale_bad.obj").c_str());
  }
}

TEST(MeshScale, NegativeFactorMirrorsTheMesh) {
  // the triangle mirrored across the y-z plane: vertices (-1, 0, 0),
  // (0, 1, 0), (0, 1, 1) and the normal (-1, 1, 0) / sqrt(2); the winding is
  // reversed so the face normal from the vertices still agrees with it
  shift::core::Context context;
  auto mesh =
    LoadScaledTriangle(context, "mesh_scale_mirror", R"(, "scale": [-1, 1, 1])");
  ASSERT_NE(mesh, nullptr);
  const auto& data = mesh->get_mesh_data().at(0);
  ASSERT_EQ(data.vertices.size(), 9u);
  double sum[3] = {0, 0, 0};
  for(int j = 0; j < 3; j++) {
    for(int k = 0; k < 3; k++) {
      sum[k] += data.vertices[3 * j + k];
    }
  }
  EXPECT_NEAR(sum[0], -1.0, 1e-6);
  EXPECT_NEAR(sum[1], 2.0, 1e-6);
  EXPECT_NEAR(sum[2], 1.0, 1e-6);
  for(int j = 0; j < 3; j++) {
    EXPECT_NEAR(data.normals[3 * j + 0], -1.0 / std::sqrt(2.0), 1e-6);
    EXPECT_NEAR(data.normals[3 * j + 1], 1.0 / std::sqrt(2.0), 1e-6);
    EXPECT_NEAR(data.normals[3 * j + 2], 0.0, 1e-6);
  }
  // the face normal from the winding of the vertices is along the normal
  ASSERT_EQ(data.faces.size(), 3u);
  double p[3][3];
  for(int j = 0; j < 3; j++) {
    for(int k = 0; k < 3; k++) {
      p[j][k] = data.vertices[3 * data.faces[j] + k];
    }
  }
  const double u[3] = {p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]};
  const double v[3] = {p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]};
  const double face[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                          u[0] * v[1] - u[1] * v[0]};
  EXPECT_GT(face[0] * data.normals[0] + face[1] * data.normals[1] +
              face[2] * data.normals[2],
            0.0);
}

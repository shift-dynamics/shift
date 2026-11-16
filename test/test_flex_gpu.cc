/* the compute shaders that deform a flexible body's mesh (FlexGpu), on an
OpenGL device without a window: the steel plate of examples/cad/
plate_hole.step on a pin, a modal body deformed 1000 times as large as its
initial elastic coordinates. The vertex buffer they write is DeformFlexMesh's
surface; the ray tracer's triangles are its triangles in the BVH's order,
their motion the change since they were last written, and the BVH's boxes
those of the triangles under them. A frame without a new deformation zeroes
the motion. Skipped where there is no OpenGL 4.3 device. */

#include <gtest/gtest.h>

#include <QGuiApplication>
#include <QOffscreenSurface>
#include <QSurfaceFormat>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/gui/flex_gpu.h"
#include "shift/gui/simulation.h"

#if SHIFT_HAVE_FEA && SHIFT_HAVE_STEP

static std::string WriteGpuPlate() {
  const std::string path = testing::TempDir() + "/flex_gpu.json";
  std::ofstream file(path);
  file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 9.81, 0], "solvers": [{"type": "rnca"}],
      "include_directories": [")"
       << SHIFT_EXAMPLES_DIR << R"(/cad"],
      "integrators": [{"type": "ode113", "time_span": 0.001,
                       "time_step": 0.001, "hmax": 1e-4}]},
      "assemblies": [{"name": "test", "bodies": [{"name": "plate",
        "type": "modal", "mode_count": 2,
        "initial_elastic_coordinates": [1e-4, 5e-5],
        "density": 7850, "youngs_modulus": 2.1e11, "poisson_ratio": 0.3,
        "fe_geometry": "solid", "element_size": 0.02,
        "frames": [{"name": "pin", "T": [0, 0, 0.01]}],
        "geometry": [{"name": "solid", "type": "cad",
                      "filename": "plate_hole.step"}]}],
      "joints": [{"name": "hinge", "type": "revolute",
                  "parent": [["fixed"], "origin"],
                  "child": [["plate"], "pin"], "axis": [0, 0, 1]}]}]})";
  return path;
}

/* the offscreen platform needs no display */
static void EnsureGuiApplication() {
  static int argc = 1;
  static char name[] = "shift-test";
  static char* argv[] = {name, nullptr};
  if(!QGuiApplication::instance()) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    new QGuiApplication(argc, argv);
  }
}

/* a buffer's bytes as values of T */
template <class T>
static std::vector<T> Values(const QRhiReadbackResult& result) {
  std::vector<T> values(size_t(result.data.size()) / sizeof(T));
  std::memcpy(values.data(), result.data.constData(),
              values.size() * sizeof(T));
  return values;
}

TEST(FlexGpu, DeformsTheSurfaceAndRefitsTheRayTracersBvh) {
  EnsureGuiApplication();
  QSurfaceFormat format;
  format.setVersion(4, 3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  std::unique_ptr<QOffscreenSurface> surface(
    QRhiGles2InitParams::newFallbackSurface(format));
  QRhiGles2InitParams params;
  params.format = format;
  params.fallbackSurface = surface.get();
  std::unique_ptr<QRhi> rhi(QRhi::create(QRhi::OpenGLES2, &params));
  if(!rhi || !rhi->isFeatureSupported(QRhi::Compute) ||
     !rhi->isFeatureSupported(QRhi::ReadBackNonUniformBuffer)) {
    GTEST_SKIP() << "no OpenGL 4.3 device with compute shaders";
  }

  const std::string path = WriteGpuPlate();
  shift::gui::Simulation simulation;
  const std::string cache = testing::TempDir() + "/flex_gpu.test__plate.fe.h5";
  std::remove(cache.c_str());
  simulation.Load(path);
  std::remove(path.c_str());
  std::remove(cache.c_str());
  const shift::gui::Scene& scene = simulation.get_scene();
  int index = -1;
  for(size_t m = 0; m < scene.meshes.size(); m++) {
    if(scene.meshes[m].deformable) {
      index = static_cast<int>(m);
    }
  }
  ASSERT_GE(index, 0);
  const shift::gui::SceneMesh& mesh = scene.meshes[size_t(index)];
  simulation.set_deformation_scale(1000.0);
  std::vector<shift::gui::FlexUpdate> updates;
  simulation.TakeFlexUpdates(updates, true);
  ASSERT_EQ(updates.size(), 1u);
  const shift::gui::FlexUpdate& update = updates[0];
  std::vector<shift::gui::Vertex> expected;
  shift::gui::DeformFlexMesh(mesh, update, expected);

  /* the ray tracer's triangles of the undeformed mesh and their BVH */
  std::vector<shift::gui::RtTriangle> triangles(mesh.indices.size() / 3);
  for(size_t t = 0; t < triangles.size(); t++) {
    float* positions[3] = {triangles[t].v0, triangles[t].v1, triangles[t].v2};
    for(int j = 0; j < 3; j++) {
      const shift::gui::Vertex& v = mesh.vertices[mesh.indices[3 * t + j]];
      std::copy(v.position, v.position + 3, positions[j]);
    }
  }
  /* the BVH sorts the triangles, and the tags with them: the mesh's
  triangle at each place */
  std::vector<int32_t> order(triangles.size());
  for(size_t t = 0; t < order.size(); t++) {
    order[t] = static_cast<int32_t>(t);
  }
  const std::vector<shift::gui::BvhNode> nodes =
    shift::gui::BuildBvh(triangles, &order);
  ASSERT_FALSE(nodes.empty());
  const std::vector<shift::gui::RtTriangle>& sorted = triangles;
  shift::gui::RtFlexMesh rt;
  rt.mesh = index;
  rt.node_count = static_cast<int>(nodes.size());
  rt.order = order;

  /* the buffers the viewport makes */
  auto make = [&](QRhiBuffer::UsageFlags usage, size_t bytes) {
    QRhiBuffer* buffer =
      rhi->newBuffer(QRhiBuffer::Static, usage, quint32(bytes));
    buffer->create();
    return std::unique_ptr<QRhiBuffer>(buffer);
  };
  auto vertices = make(QRhiBuffer::VertexBuffer | QRhiBuffer::StorageBuffer,
                       mesh.vertices.size() * sizeof(shift::gui::Vertex));
  auto indices = make(QRhiBuffer::IndexBuffer | QRhiBuffer::StorageBuffer,
                      mesh.indices.size() * sizeof(uint32_t));
  auto triangle_buffer =
    make(QRhiBuffer::StorageBuffer,
         sorted.size() * sizeof(shift::gui::RtTriangle));
  auto node_buffer = make(QRhiBuffer::StorageBuffer,
                          nodes.size() * sizeof(shift::gui::BvhNode));
  const size_t motion_floats = 12 * sorted.size();
  auto motion_buffer =
    make(QRhiBuffer::StorageBuffer, motion_floats * sizeof(float));
  const std::vector<float> zero(motion_floats, 0.f);
  /* boxes the refit must replace */
  std::vector<shift::gui::BvhNode> stale_nodes = nodes;
  for(auto& node : stale_nodes) {
    std::fill(node.box_min, node.box_min + 3, 0.f);
    std::fill(node.box_max, node.box_max + 3, 0.f);
  }

  shift::gui::FlexGpu flex(rhi.get());
  QRhiReadbackResult read_vertices, read_triangles, read_nodes, read_motion;
  QRhiCommandBuffer* cb = nullptr;
  ASSERT_EQ(rhi->beginOffscreenFrame(&cb), QRhi::FrameOpSuccess);
  QRhiResourceUpdateBatch* batch = rhi->nextResourceUpdateBatch();
  batch->uploadStaticBuffer(vertices.get(), mesh.vertices.data());
  batch->uploadStaticBuffer(indices.get(), mesh.indices.data());
  batch->uploadStaticBuffer(triangle_buffer.get(), sorted.data());
  batch->uploadStaticBuffer(node_buffer.get(), stale_nodes.data());
  batch->uploadStaticBuffer(motion_buffer.get(), zero.data());
  const int id = flex.Add(mesh, vertices.get(), indices.get(), batch);
  flex.AddRayTracing(id, rt, nodes, triangle_buffer.get(), node_buffer.get(),
                     motion_buffer.get(), batch);
  EXPECT_TRUE(flex.is_ray_traced(id));
  flex.Update(id, mesh, update, batch);
  batch = flex.Deform(cb, batch, true);
  batch->readBackBuffer(vertices.get(), 0, vertices->size(), &read_vertices);
  batch->readBackBuffer(triangle_buffer.get(), 0, triangle_buffer->size(),
                        &read_triangles);
  batch->readBackBuffer(node_buffer.get(), 0, node_buffer->size(),
                        &read_nodes);
  batch->readBackBuffer(motion_buffer.get(), 0, motion_buffer->size(),
                        &read_motion);
  cb->resourceUpdate(batch);
  ASSERT_EQ(rhi->endOffscreenFrame(), QRhi::FrameOpSuccess);

  /* the surface drawn: DeformFlexMesh's, to float rounding */
  const auto drawn = Values<shift::gui::Vertex>(read_vertices);
  ASSERT_EQ(drawn.size(), expected.size());
  double largest_move = 0.0;
  for(size_t v = 0; v < drawn.size(); v++) {
    for(int c = 0; c < 3; c++) {
      EXPECT_NEAR(drawn[v].position[c], expected[v].position[c], 1e-6);
      EXPECT_NEAR(drawn[v].normal[c], expected[v].normal[c], 1e-4);
      largest_move = std::max(largest_move,
                              double(std::fabs(expected[v].position[c] -
                                               mesh.vertices[v].position[c])));
    }
  }
  EXPECT_GT(largest_move, 1e-3); /* a deformation the test sees */

  /* the ray tracer's triangles, their motion and the boxes */
  const auto written = Values<shift::gui::RtTriangle>(read_triangles);
  const auto motion = Values<float>(read_motion);
  ASSERT_EQ(written.size(), sorted.size());
  ASSERT_GE(motion.size(), motion_floats);
  for(size_t k = 0; k < written.size(); k++) {
    const float* positions[3] = {written[k].v0, written[k].v1, written[k].v2};
    const float* before[3] = {sorted[k].v0, sorted[k].v1, sorted[k].v2};
    for(int j = 0; j < 3; j++) {
      const shift::gui::Vertex& v =
        expected[mesh.indices[3 * size_t(rt.order[k]) + size_t(j)]];
      for(int c = 0; c < 3; c++) {
        EXPECT_NEAR(positions[j][c], v.position[c], 1e-6);
        EXPECT_NEAR(motion[12 * k + 4 * j + c], v.position[c] - before[j][c],
                    1e-6);
      }
    }
  }
  const auto fitted = Values<shift::gui::BvhNode>(read_nodes);
  ASSERT_EQ(fitted.size(), nodes.size());
  for(size_t i = 0; i < fitted.size(); i++) {
    /* the triangles under the node, from the leaves under it */
    float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    std::vector<size_t> stack = {i};
    while(!stack.empty()) {
      const size_t n = stack.back();
      stack.pop_back();
      if(fitted[n].triangle_count < 0) {
        stack.push_back(n + 1);
        stack.push_back(size_t(fitted[n].right_or_first));
        continue;
      }
      for(int t = 0; t < fitted[n].triangle_count; t++) {
        const auto& triangle = written[size_t(fitted[n].right_or_first + t)];
        for(const float* p : {triangle.v0, triangle.v1, triangle.v2}) {
          for(int c = 0; c < 3; c++) {
            lo[c] = std::min(lo[c], p[c]);
            hi[c] = std::max(hi[c], p[c]);
          }
        }
      }
    }
    for(int c = 0; c < 3; c++) {
      EXPECT_EQ(fitted[i].box_min[c], lo[c]) << "node " << i;
      EXPECT_EQ(fitted[i].box_max[c], hi[c]) << "node " << i;
    }
    EXPECT_EQ(fitted[i].right_or_first, nodes[i].right_or_first);
    EXPECT_EQ(fitted[i].triangle_count, nodes[i].triangle_count);
  }

  /* the instance's box holds the deformed mesh */
  ksl_vec3f_t box_min, box_max;
  flex.get_bounds(id, box_min, box_max);
  for(const auto& v : expected) {
    for(int c = 0; c < 3; c++) {
      EXPECT_LE(box_min.at[c], v.position[c]);
      EXPECT_GE(box_max.at[c], v.position[c]);
    }
  }

  /* a frame without a new deformation: no motion */
  QRhiReadbackResult read_still;
  ASSERT_EQ(rhi->beginOffscreenFrame(&cb), QRhi::FrameOpSuccess);
  batch = flex.Deform(cb, rhi->nextResourceUpdateBatch(), true);
  batch->readBackBuffer(motion_buffer.get(), 0, motion_buffer->size(),
                        &read_still);
  cb->resourceUpdate(batch);
  ASSERT_EQ(rhi->endOffscreenFrame(), QRhi::FrameOpSuccess);
  const auto still = Values<float>(read_still);
  ASSERT_GE(still.size(), motion_floats);
  EXPECT_EQ(*std::max_element(still.begin(), still.begin() + motion_floats),
            0.f);
  EXPECT_EQ(*std::min_element(still.begin(), still.begin() + motion_floats),
            0.f);
}

#endif

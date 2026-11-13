#include "shift/gui/viewport.h"

#include "shift/core/context.h"
#include "shift/core/terrain/base.h"
#include "shift/core/terrain/basic.h"

#include "shift/gui/environment.h"
#include "shift/gui/simulation.h"

#include <QExposeEvent>
#include <QFile>
#include <QMouseEvent>
#include <QPlatformSurfaceEvent>
#include <QVulkanInstance>
#include <QWheelEvent>
#include <rhi/qrhi.h>
#include <rhi/qshader.h>

#include <QFloat16>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <vector>


static constexpr int kRtMaxFrames = 1024; //!< accumulated samples per pixel
static constexpr int kAtrousIterations = 5;  //!< steps 1, 2, 4, 8, 16
// maximum SVGF history: short while bodies move (less ghosting), longer when
// only the camera moves (reprojection is exact for a static scene)
static constexpr float kHistoryMoving = 12.f;
static constexpr float kHistoryCamera = 32.f;
static constexpr float kHistoryStatic = 64.f;
// a static view blends from the denoised image into the exact progressive
// mean between these numbers of accumulated frames
static constexpr float kBlendStart = 4.f;
static constexpr float kBlendEnd = 256.f;

// uniform and storage buffer layouts, keep in sync with the shaders

struct FrameUniforms {
  float view_projection[16];
  float inverse_view_projection[16];
  float camera_position[4];
  float sun_direction[4];
  float sun_color[4];
  float sky_zenith[4];
  float sky_horizon[4];
  float floor_params[4];
  float floor_frame[16]; // column major: axes u, v, normal, origin
};

/*! the soil surface over SPH particles, see fluid_common.glsl */
struct FluidUniforms {
  float view_projection[16];
  float inverse_view_projection[16];
  float camera_position[4]; // w: 1 if clip space depth runs 0 to 1
  float camera_forward[4];
  float particles[4];     // radius, pixels per unit length at unit depth
  float filter_params[4]; // blur radius, its range in depth beyond a slant,
                          // its largest radius in pixels
};

/*! the surface over SPH particles: the blur's radius and its range in
 * depth beyond that of a slant, in radii of the particles drawn; the
 * strides of its levels in pixels, an even number of them so that the last
 * writes fluid_depth_[0]: doubling to the largest, then fine again, which
 * takes out what the sparse samples of the coarse levels leave between
 * neighbouring pixels (the normals are found from them); and its largest
 * radius (fluid_blur.frag) */
static constexpr float kFluidBlurRadius = 4.0f;
static constexpr float kFluidBlurRange = 2.0f;
static constexpr int kFluidBlurStrides[] = {1, 2, 4, 8, 16, 32, 2, 1};
static constexpr int kFluidBlurLevels =
  int(sizeof(kFluidBlurStrides) / sizeof(kFluidBlurStrides[0]));
static constexpr float kFluidBlurMaxPixels = 64.f;
static_assert(kFluidBlurLevels % 2 == 0, "the last level writes [0]");

/*! per terrain, see terrain_common.glsl */
struct TerrainUniforms {
  float pose[16];
  float extent[4];       // x, y extent (m), patches along x, y
  float tessellation[4]; // pixels per unit angle, pixels per triangle edge,
                         // maximum level
  float scale[4];        // spacing of the control points (m)
  int32_t samples[4];    // control points along x, y; 1: bicubic
};
static_assert(sizeof(TerrainUniforms) == 128);

/* a terrain patch covers this many control point cells a side; the
tessellator divides it into up to kTerrainMaximumLevel segments a side */
static constexpr int kTerrainPatchCells = 4;
static constexpr float kTerrainMaximumLevel = 64.f;
static constexpr float kTerrainPixelsPerEdge = 6.f;

/*! CAD tessellation and edges, see cad_uniforms.glsl */
struct CadUniforms {
  float tessellation[4]; // pixels per unit angle, pixels per triangle edge,
                         // maximum level, largest distance from the surface
                         // (pixels)
  float edges[4];        // half width of edge lines (pixels), darkening,
                         // widening of the trims (pixels)
};
static_assert(sizeof(CadUniforms) == 32);
static constexpr float kCadPixelsPerEdge = 12.f;
static constexpr float kCadMaximumLevel = 64.f;
static constexpr float kCadSagittaPixels = 0.2f;
static constexpr float kCadEdgeHalfWidth = 0.5f;
static constexpr float kCadEdgeDarkening = 0.6f;
static constexpr float kCadTrimWidening = 0.5f;

/*! the lights of the model, see lights.glsl */
struct LightData {
  float position[4];    // xyz: world position, w: type (SceneLight::Type)
  float direction[4];   // xyz: direction it shines, w: cos of cone angle
  float axis_x[4];      // xyz: x axis of its frame, w: cos of inner angle
  float diffuse[4];     // rgb: pi Kd intensity, w: IES profile or -1
  float specular[4];    // rgb: pi Ks intensity
  float attenuation[4]; // xyz: c, l, q
};
static constexpr int kMaxLights = 16;
struct LightUniforms {
  int32_t count[4]; // x: lights, y: IES profiles
  float ambient[4]; // rgb: sum of Ka
  LightData lights[kMaxLights];
};
static_assert(sizeof(LightUniforms) == 32 + kMaxLights * 96);

/*! per mesh, at a dynamic offset */
struct MaterialUniforms {
  float albedo[4];   // w = normal map scale
  float material[4]; // x = metallic, y = roughness, z = triplanar (2: hex-tiled
                     // too),
                     // w = 1 / texture scale
  int32_t layers[4]; // base color, metallic roughness, normal; -1: none;
                     // w = holes used
  float variation[4]; // x = 1: vary the tiles of triplanar textures
  int32_t surface_layers[4]; // base color and normal of surfaces 0 and 1
  float surface[2][4]; // x = 1 / texture scale, y = normal scale, z = noise
                       // level, w = 1 / patch size
  float emission[4]; // rgb: factor of the base color added to the shading
  float holes[4][4]; // of a terrain, in its uv: x0, y0, x1, y1 (SceneMesh)
};

/*! per instance vertex attributes: the body origin pose, column major */
struct InstanceAttributes {
  float model[16];
};
static_assert(sizeof(InstanceAttributes) == 64);

/*! a vertex of the arrows of a coordinate frame, see axes.vert */
struct AxesVertex {
  float position[3];
  float normal[3];
  float color[3];
};

/*! per frame or joint axis drawn, see axes.vert */
struct AxesInstance {
  float model[16];
  float tint[4];
};
static_assert(sizeof(AxesInstance) == 80);

/* the arrows of a coordinate frame, one unit long: along each axis a
cylinder and a cone at its end, x red, y green and z blue; arrow_indices of
the indices are theirs. Then the arrow of a joint axis, white, from -1 to 1
along z, pointing to +z: a cylinder of radius 1, scaled to the thickness of
the line, and a cone three times as wide over the last 12 % of its length. */
static void MakeAxesMesh(std::vector<AxesVertex>& vertices,
                         std::vector<uint32_t>& indices,
                         uint32_t& arrow_indices) {
  constexpr int kSegments = 24;
  constexpr float kShaftRadius = 0.025f;
  constexpr float kShaftLength = 0.78f;
  constexpr float kHeadRadius = 0.07f;
  const float kColors[3][3] = {
    {0.92f, 0.16f, 0.14f}, {0.18f, 0.78f, 0.18f}, {0.20f, 0.38f, 0.98f}};
  vertices.clear();
  indices.clear();
  for(int axis = 0; axis < 3; axis++) {
    QVector3D e, u, v;
    e[axis] = 1.f;
    u[(axis + 1) % 3] = 1.f;
    v[(axis + 2) % 3] = 1.f;
    const float* color = kColors[axis];
    auto add = [&](const QVector3D& p, const QVector3D& n) {
      AxesVertex vertex;
      for(int i = 0; i < 3; i++) {
        vertex.position[i] = p[i];
        vertex.normal[i] = n[i];
        vertex.color[i] = color[i];
      }
      vertices.push_back(vertex);
      return static_cast<uint32_t>(vertices.size() - 1);
    };
    // the slope of the cone's side
    const float head_length = 1.f - kShaftLength;
    const float slope = std::hypot(head_length, kHeadRadius);
    const QVector3D tip = e;
    const QVector3D base = kShaftLength * e;
    const uint32_t base_center = add(base, -e);
    for(int k = 0; k < kSegments; k++) {
      const float a0 = 2.f * float(M_PI) * float(k) / kSegments;
      const float a1 = 2.f * float(M_PI) * float(k + 1) / kSegments;
      const QVector3D r0 = std::cos(a0) * u + std::sin(a0) * v;
      const QVector3D r1 = std::cos(a1) * u + std::sin(a1) * v;
      // shaft
      const uint32_t s0 = add(kShaftRadius * r0, r0);
      const uint32_t s1 = add(kShaftRadius * r1, r1);
      const uint32_t s2 = add(base + kShaftRadius * r1, r1);
      const uint32_t s3 = add(base + kShaftRadius * r0, r0);
      indices.insert(indices.end(), {s0, s1, s2, s0, s2, s3});
      // the base of the head
      const uint32_t b0 = add(base + kHeadRadius * r0, -e);
      const uint32_t b1 = add(base + kHeadRadius * r1, -e);
      indices.insert(indices.end(), {base_center, b1, b0});
      // the side of the head
      const QVector3D n0 = (head_length * r0 + kHeadRadius * e) / slope;
      const QVector3D n1 = (head_length * r1 + kHeadRadius * e) / slope;
      const QVector3D nm = (n0 + n1).normalized();
      const uint32_t h0 = add(base + kHeadRadius * r0, n0);
      const uint32_t h1 = add(base + kHeadRadius * r1, n1);
      const uint32_t h2 = add(tip, nm);
      indices.insert(indices.end(), {h0, h1, h2});
    }
  }
  arrow_indices = static_cast<uint32_t>(indices.size());

  constexpr int kLineSegments = 16;
  constexpr float kLineRadius = 1.f;
  constexpr float kLineHeadRadius = 3.f;
  constexpr float kLineHeadLength = 0.24f; // of the length 2
  auto add = [&](const QVector3D& p, const QVector3D& n) {
    AxesVertex vertex;
    for(int i = 0; i < 3; i++) {
      vertex.position[i] = p[i];
      vertex.normal[i] = n[i];
      vertex.color[i] = 1.f;
    }
    vertices.push_back(vertex);
    return static_cast<uint32_t>(vertices.size() - 1);
  };
  const QVector3D z(0.f, 0.f, 1.f);
  const QVector3D head_base = (1.f - kLineHeadLength) * z;
  const float head_slope = std::hypot(kLineHeadLength, kLineHeadRadius);
  const uint32_t tail_center = add(-z, -z);
  const uint32_t head_center = add(head_base, -z);
  for(int k = 0; k < kLineSegments; k++) {
    const float a0 = 2.f * float(M_PI) * float(k) / kLineSegments;
    const float a1 = 2.f * float(M_PI) * float(k + 1) / kLineSegments;
    const QVector3D r0(std::cos(a0), std::sin(a0), 0.f);
    const QVector3D r1(std::cos(a1), std::sin(a1), 0.f);
    // shaft, closed at the tail
    const uint32_t s0 = add(kLineRadius * r0 - z, r0);
    const uint32_t s1 = add(kLineRadius * r1 - z, r1);
    const uint32_t s2 = add(head_base + kLineRadius * r1, r1);
    const uint32_t s3 = add(head_base + kLineRadius * r0, r0);
    indices.insert(indices.end(), {s0, s1, s2, s0, s2, s3});
    const uint32_t t0 = add(kLineRadius * r0 - z, -z);
    const uint32_t t1 = add(kLineRadius * r1 - z, -z);
    indices.insert(indices.end(), {tail_center, t1, t0});
    // the base of the head
    const uint32_t b0 = add(head_base + kLineHeadRadius * r0, -z);
    const uint32_t b1 = add(head_base + kLineHeadRadius * r1, -z);
    indices.insert(indices.end(), {head_center, b1, b0});
    // the side of the head
    const QVector3D n0 = (kLineHeadLength * r0 + kLineHeadRadius * z) / head_slope;
    const QVector3D n1 = (kLineHeadLength * r1 + kLineHeadRadius * z) / head_slope;
    const uint32_t h0 = add(head_base + kLineHeadRadius * r0, n0);
    const uint32_t h1 = add(head_base + kLineHeadRadius * r1, n1);
    const uint32_t h2 = add(z, (n0 + n1).normalized());
    indices.insert(indices.end(), {h0, h1, h2});
  }
}

struct RtUniforms {
  float inverse_view[16];
  float inverse_projection[16];
  float camera_position[4];
  float sun_direction[4];
  float sun_color[4];
  float sky_zenith[4];
  float sky_horizon[4];
  float floor_params[4];
  float texture_params[4]; // x = pixel spread angle, y = texture size
  int32_t control[4];
  float floor_frame[16]; // as in FrameUniforms
};

struct RtInstance {
  float body_to_world[16];
  float world_to_body[16];
  float box_min[4];
  float box_max[4];
  float albedo[4];
  float material[4]; // as in MaterialUniforms
  int32_t offsets[4]; // x = root node, y..w = texture layers as in
                      // MaterialUniforms
  float previous_body_to_world[16]; // pose in the previous frame
  float optical[4]; // x = opacity, yzw = glow (CopyEmission)
  float variation[4]; // as in MaterialUniforms
  int32_t surface_layers[4];
  float surface[2][4];
};
static_assert(sizeof(RtInstance) == 352);
static_assert(sizeof(MaterialUniforms) == 192);

struct DenoiseUniforms {
  float previous_view_projection[16];
  float params[4]; // x = pixel spread angle, y = maximum history length,
                   // z = history valid, w = a-trous step
  float blend[4];  // x = progressive frames, y, z = blend start, end
  int32_t control[4]; // x = instance count
  float view_projection[16]; // current frame
};
static_assert(sizeof(DenoiseUniforms) == 176);

static const float kSunColor[4] = {1.f, 0.96f, 0.9f, 0.f};
static const float kSunIntensity = 3.f;
static const float kNoSun[4] = {0.f, 0.f, 0.f, 0.f};
static const float kSkyZenith[4] = {0.22f, 0.36f, 0.62f, 0.f};
static const float kSkyHorizon[4] = {0.62f, 0.68f, 0.74f, 0.f};

static QShader LoadShader(const char* name) {
  QFile file(QStringLiteral(":/shaders/") + QLatin1String(name));
  if(!file.open(QIODevice::ReadOnly)) {
    qFatal("Cannot open shader resource %s", name);
  }
  QShader shader = QShader::fromSerialized(file.readAll());
  if(!shader.isValid()) {
    qFatal("Invalid shader resource %s", name);
  }
  return shader;
}

/*! Vulkan instance shared by all viewports, created on first use */
static QVulkanInstance* GetVulkanInstance() {
  static QVulkanInstance* instance = [] {
    auto* created = new QVulkanInstance;
    created->setApiVersion(created->supportedApiVersion());
    created->setExtensions(QRhiVulkanInitParams::preferredInstanceExtensions());
    // SHIFT_VULKAN_VALIDATION=1 enables the Khronos validation layer; its
    // messages are logged as warnings
    if(qEnvironmentVariableIntValue("SHIFT_VULKAN_VALIDATION")) {
      created->setLayers({"VK_LAYER_KHRONOS_validation"});
    }
    if(!created->create()) {
      qWarning("Failed to create a Vulkan instance: %d",
               int(created->errorCode()));
    }
    return created;
  }();
  return instance;
}

/*! RGBA16F texture contents from RGBA float values */
static QByteArray ToHalf(const std::vector<float>& values) {
  QByteArray data(int(values.size() * sizeof(qfloat16)), Qt::Uninitialized);
  qFloatToFloat16(reinterpret_cast<qfloat16*>(data.data()), values.data(),
                  qsizetype(values.size()));
  return data;
}

static void CopyMatrix(const QMatrix4x4& m, float* out) {
  std::memcpy(out, m.constData(), 16 * sizeof(float));
}

static void CopyVector(const QVector3D& v, float w, float* out) {
  out[0] = v.x();
  out[1] = v.y();
  out[2] = v.z();
  out[3] = w;
}

static void CopyArray(const float* in, float* out) {
  std::memcpy(out, in, 4 * sizeof(float));
}

/*! material parameters and texture layers of a mesh, as in MaterialUniforms;
 * writes material[4] and the three texture layers layers[0..2] (base
 * color, metallic roughness, normal); RtInstance passes offsets + 1, so a
 * fourth layer would overwrite the next field */
static void CopyMaterial(const shift::gui::SceneMesh& mesh, float* material,
                  int32_t* layers) {
  material[0] = mesh.metallic;
  material[1] = mesh.roughness;
  material[2] = mesh.hex_tiling ? 2.f : mesh.triplanar ? 1.f : 0.f;
  material[3] = 1.f / mesh.texture_scale;
  layers[0] = mesh.base_color_layer;
  layers[1] = mesh.metallic_roughness_layer;
  layers[2] = mesh.normal_layer;
}

/*! the variation of the textures of a mesh and its surfaces, as in
 * MaterialUniforms */
static void CopySurfaces(const shift::gui::SceneMesh& mesh, float* variation,
                         int32_t* surface_layers, float surface[2][4]) {
  variation[0] = mesh.texture_variation ? 1.f : 0.f;
  variation[1] = variation[2] = variation[3] = 0.f;
  for(int i = 0; i < 2; i++) {
    surface_layers[2 * i] = surface_layers[2 * i + 1] = -1;
    surface[i][0] = surface[i][1] = surface[i][3] = 1.f;
    surface[i][2] = 2.f;
    if(i < static_cast<int>(mesh.surfaces.size())) {
      const auto& s = mesh.surfaces[i];
      surface_layers[2 * i] = s.base_color_layer;
      surface_layers[2 * i + 1] = s.normal_layer;
      surface[i][0] = 1.f / s.texture_scale;
      surface[i][1] = s.normal_scale;
      surface[i][2] = s.level;
      surface[i][3] = 1.f / s.patch_size;
    }
  }
}

/*! the glow of a mesh at the time shown, as rgb: the diffuse color times
 * the intensity of the light it glows with (SceneMesh::glow_light), or 0 */
static void CopyEmission(const shift::gui::Scene& scene,
                         const shift::gui::SceneMesh& mesh,
                         const shift::gui::Poses& poses, float* emission) {
  emission[0] = emission[1] = emission[2] = 0.f;
  if(mesh.glow_light < 0 ||
     mesh.glow_light >= static_cast<int>(scene.lights.size())) {
    return;
  }
  const auto& light = scene.lights[mesh.glow_light];
  const float intensity = mesh.glow_light < int(poses.lights.size())
                            ? poses.lights[mesh.glow_light]
                            : light.intensity;
  for(int k = 0; k < 3; k++) {
    emission[k] = light.diffuse[k] * intensity;
  }
}

/*! world space box of a body space box under a pose */
static void TransformBox(const QMatrix4x4& pose, const QVector3D& box_min,
                  const QVector3D& box_max, float* out_min, float* out_max) {
  for(int k = 0; k < 3; k++) {
    out_min[k] = 1e30f;
    out_max[k] = -1e30f;
  }
  for(int corner = 0; corner < 8; corner++) {
    const QVector3D p = pose.map(
      QVector3D((corner & 1) ? box_max.x() : box_min.x(),
                (corner & 2) ? box_max.y() : box_min.y(),
                (corner & 4) ? box_max.z() : box_min.z()));
    for(int k = 0; k < 3; k++) {
      out_min[k] = std::min(out_min[k], p[k]);
      out_max[k] = std::max(out_max[k], p[k]);
    }
  }
  out_min[3] = out_max[3] = 0.f;
}


/*! the lights of a scene in the given body poses; returns their number */
static int FillLights(const shift::gui::Scene& scene,
                      const shift::gui::Poses& poses, LightUniforms& out) {
  out = LightUniforms{};
  int count = 0;
  QVector3D ambient;
  for(size_t i = 0; i < scene.lights.size(); i++) {
    const auto& light = scene.lights[i];
    if(count == kMaxLights) {
      break;
    }
    // as bright as at the time shown (a controller may switch it)
    const float intensity =
      i < poses.lights.size() ? poses.lights[i] : light.intensity;
    if(light.body_index >= int(poses.bodies.size())) {
      continue;
    }
    const QMatrix4x4 frame = poses.bodies[light.body_index] * light.frame;
    LightData& d = out.lights[count++];
    const QVector3D shine = -frame.column(2).toVector3D().normalized();
    const float cos_outer = std::cos(light.cone_angle);
    const float cos_inner =
      std::max(std::cos(light.inner_cone_angle), cos_outer + 1e-4f);
    CopyVector(frame.column(3).toVector3D(), float(light.type), d.position);
    CopyVector(shine, cos_outer, d.direction);
    CopyVector(frame.column(0).toVector3D().normalized(), cos_inner,
               d.axis_x);
    // pi: a white matte surface facing the light shows its color Kd
    CopyVector(float(M_PI) * intensity * light.diffuse,
               float(light.ies_profile), d.diffuse);
    CopyVector(float(M_PI) * intensity * light.specular, 0.f, d.specular);
    CopyVector(light.attenuation, 0.f, d.attenuation);
    ambient += light.ambient;
  }
  out.count[0] = count;
  out.count[1] = int(scene.ies_profiles.size());
  CopyVector(ambient, 0.f, out.ambient);
  return count;
}

shift::gui::Viewport::Viewport(QWindow* parent)
  : QWindow(parent) {
  setSurfaceType(QSurface::VulkanSurface);
  setVulkanInstance(GetVulkanInstance());
  sun_direction_ = QVector3D(0.45f, 0.3f, -0.84f).normalized();
}

shift::gui::Viewport::~Viewport() {
  ReleaseRhi();
}

void shift::gui::Viewport::set_simulation(Simulation* simulation,
                                          bool frame_scene) {
  set_follow_body(-1);
  simulation_ = simulation;
  poses_ = Poses();
  cad_triangles_.clear();
  cad_triangle_faces_.clear();
  cad_triangles_dirty_ = simulation_ != nullptr;
  if(simulation_) {
    simulation_->TakePoses(poses_);
  }
  scene_dirty_ = true;
  if(frame_scene) {
    FrameScene();
  }
}

void shift::gui::Viewport::set_render_mode(RenderMode mode) {
  render_mode_ = mode;
  rt_frame_ = 0;
  rt_history_valid_ = false;
  update();
}

void shift::gui::Viewport::set_smooth_terrain(bool smooth) {
  smooth_terrain_ = smooth;
  update();
}

void shift::gui::Viewport::set_particle_surface(bool surface) {
  particle_surface_ = surface;
  update();
}

void shift::gui::Viewport::set_floor_enabled(bool enabled) {
  floor_enabled_ = enabled;
  rt_frame_ = 0;
  rt_history_valid_ = false;
  update();
}

void shift::gui::Viewport::set_frame_axes(std::vector<FrameAxes> axes) {
  frame_axes_ = std::move(axes);
  update();
}

void shift::gui::Viewport::set_frame_axes_scale(float scale) {
  frame_axes_scale_ = scale;
  update();
}

void shift::gui::Viewport::set_joint_axes(std::vector<JointAxis> axes) {
  joint_axes_ = std::move(axes);
  update();
}

void shift::gui::Viewport::set_joint_color(const QVector3D& color) {
  joint_color_ = color;
  update();
}

void shift::gui::Viewport::set_joint_thickness(float thickness) {
  joint_thickness_ = thickness;
  update();
}

void shift::gui::Viewport::set_hidden_bodies(std::vector<bool> hidden) {
  hidden_bodies_ = std::move(hidden);
  // the ray tracer's instances hold the bodies' boxes
  rt_instances_dirty_ = true;
  rt_frame_ = 0;
  update();
}

void shift::gui::Viewport::FrameScene() {
  if(simulation_ && !poses_.bodies.empty()) {
    QVector3D box_min, box_max;
    SceneBounds(simulation_->get_scene(), poses_, box_min, box_max);
    camera_.Frame(box_min, box_max);
    UpdateEnvironment();
  }
  rt_frame_ = 0;
  update();
}

void shift::gui::Viewport::FrameSphere(const QVector3D& center, float radius) {
  const QVector3D reach(radius, radius, radius);
  camera_.Frame(center - reach, center + reach);
  UpdateEnvironment();
  rt_frame_ = 0;
  update();
}

void shift::gui::Viewport::set_view(float azimuth, float elevation) {
  camera_.set_direction(azimuth, elevation);
  FrameScene();
}

void shift::gui::Viewport::UpdatePoses() {
  if(simulation_ && simulation_->TakePoses(poses_)) {
    rt_instances_dirty_ = true;
    LowerFloorBelowModel();
    if(follow_body_ >= 0 &&
       follow_body_ < static_cast<int>(poses_.bodies.size())) {
      camera_.set_follow(FollowFrame(follow_body_, follow_mode_) *
                         follow_offset_);
    }
  }
}

QMatrix4x4 shift::gui::Viewport::FollowFrame(int body, FollowMode mode) const {
  const QMatrix4x4& pose = poses_.bodies[body];
  if(mode == FollowMode::kBody) {
    return pose;
  }
  // the height of the ground under the body; without a terrain the ground is
  // flat at z = 0, as it is for the tires
  float ground_z = 0.f;
  const ksl_vec3_t point = {{pose(0, 3), pose(1, 3), pose(2, 3)}};
  for(const auto* terrain : simulation_->get_context()->get_active_terrain()) {
    ksl_vec3_t surface, normal;
    // the worker may be changing a deformable terrain: the undisturbed
    // surface, which never changes, is safe to read from this thread
    const auto* basic =
      dynamic_cast<const shift::core::terrain::Basic*>(terrain);
    if(basic ? basic->QueryUndisturbed(point, surface, normal)
             : terrain->Query(point, surface, normal)) {
      ground_z = static_cast<float>(surface.z);
      break;
    }
  }
  return Camera::GroundFrame(pose, ground_z);
}

void shift::gui::Viewport::set_follow_body(int body, FollowMode mode) {
  if(body < 0 || body >= static_cast<int>(poses_.bodies.size())) {
    follow_body_ = -1;
    camera_.ClearFollow();
  } else {
    // the camera does not move: changing bodies keeps the current view
    follow_body_ = body;
    follow_mode_ = mode;
    follow_offset_ = FollowFrame(body, mode).inverted() * camera_.get_follow();
  }
  rt_frame_ = 0;
  update();
}

void shift::gui::Viewport::set_floor_below_model(float z) {
  PlaceFloor(QVector3D(0.f, 0.f, z), QVector3D(0.f, 0.f, -1.f));
  floor_below_model_ = true;
  LowerFloorBelowModel();
}

void shift::gui::Viewport::LowerFloorBelowModel() {
  if(!floor_below_model_ || !simulation_ || poses_.bodies.empty()) {
    return;
  }
  QVector3D box_min, box_max;
  SceneBounds(simulation_->get_scene(), poses_, box_min, box_max);
  // +z is down: the lowest point has the largest z; leave a small gap so
  // the floor does not touch the model
  const float gap = 0.01f * (box_max - box_min).length();
  const float level = box_max.z() + gap;
  if(level > floor_origin_.z()) {
    PlaceFloor(QVector3D(0.5f * (box_min.x() + box_max.x()),
                        0.5f * (box_min.y() + box_max.y()), level),
               QVector3D(0.f, 0.f, -1.f));
  }
}

void shift::gui::Viewport::set_floor(const QVector3D& point,
                                     const QVector3D& normal) {
  floor_below_model_ = false;
  PlaceFloor(point, normal);
}

void shift::gui::Viewport::PlaceFloor(const QVector3D& point,
                                      const QVector3D& normal) {
  floor_origin_ = point;
  floor_normal_ = normal.normalized();
  // in-plane axes: the x axis projected onto the plane, so a level floor
  // keeps its grid along x and y
  QVector3D x(1.f, 0.f, 0.f);
  if(std::fabs(QVector3D::dotProduct(x, floor_normal_)) > 0.9f) {
    x = QVector3D(0.f, 1.f, 0.f);
  }
  floor_u_ =
    (x - QVector3D::dotProduct(x, floor_normal_) * floor_normal_).normalized();
  floor_v_ = QVector3D::crossProduct(floor_normal_, floor_u_);
  rt_frame_ = 0;
  rt_history_valid_ = false;
  update();
}

void shift::gui::Viewport::CopyFloorFrame(float out[16]) const {
  const QVector3D* columns[4] = {&floor_u_, &floor_v_, &floor_normal_,
                                 &floor_origin_};
  for(int c = 0; c < 4; c++) {
    out[4 * c + 0] = columns[c]->x();
    out[4 * c + 1] = columns[c]->y();
    out[4 * c + 2] = columns[c]->z();
    out[4 * c + 3] = c == 3 ? 1.f : 0.f;
  }
}

void shift::gui::Viewport::UpdateEnvironment() {
  // size the floor grid to the model in its current pose; the floor itself
  // is the floor plane of the fixed body (see set_floor)
  QVector3D box_min, box_max;
  SceneBounds(simulation_->get_scene(), poses_, box_min, box_max);
  const float size = std::max((box_max - box_min).length(), 1e-3f);
  floor_spacing_ = std::pow(10.f, std::floor(std::log10(size / 4.f)));
  floor_extent_ = 100.f * size;
}

int shift::gui::Viewport::TakeFrameCount() {
  const int count = frame_count_;
  frame_count_ = 0;
  return count;
}

void shift::gui::Viewport::InitializeRhi() {
  QRhiVulkanInitParams params;
  params.inst = vulkanInstance();
  params.window = this;
  rhi_.reset(QRhi::create(QRhi::Vulkan, &params));
  if(!rhi_) {
    qWarning("Failed to create a Vulkan QRhi");
    return;
  }
  swapchain_.reset(rhi_->newSwapChain());
  depth_stencil_.reset(rhi_->newRenderBuffer(QRhiRenderBuffer::DepthStencil,
                                             QSize(), 1,
                                             QRhiRenderBuffer::UsedWithSwapChainOnly));
  swapchain_->setWindow(this);
  swapchain_->setDepthStencil(depth_stencil_.get());
  swapchain_render_pass_.reset(
    swapchain_->newCompatibleRenderPassDescriptor());
  swapchain_->setRenderPassDescriptor(swapchain_render_pass_.get());
}

void shift::gui::Viewport::ReleaseRhi() {
  offscreen_target_.reset();
  offscreen_render_pass_.reset();
  offscreen_depth_.reset();
  offscreen_texture_.reset();
  ReleaseAll();
  device_resources_ready_ = false;
  swapchain_ready_ = false;
  swapchain_render_pass_.reset();
  swapchain_.reset();
  depth_stencil_.reset();
  rhi_.reset();
}

QRhiRenderTarget* shift::gui::Viewport::get_render_target() const {
  if(offscreen_target_) {
    return offscreen_target_.get();
  }
  return swapchain_->currentFrameRenderTarget();
}

bool shift::gui::Viewport::BeginOffscreen(const QSize& size, QString& error) {
  EndOffscreen();
  if(!rhi_) {
    error = "the viewport has no graphics device yet";
    return false;
  }
  const int maximum = rhi_->resourceLimit(QRhi::TextureSizeMax);
  if(size.width() <= 0 || size.height() <= 0 || size.width() > maximum ||
     size.height() > maximum) {
    error = QString("the graphics device renders images of 1 to %1 pixels "
                    "a side, not %2 x %3")
              .arg(maximum)
              .arg(size.width())
              .arg(size.height());
    return false;
  }
  offscreen_texture_.reset(rhi_->newTexture(
    QRhiTexture::RGBA8, size, 1,
    QRhiTexture::RenderTarget | QRhiTexture::UsedAsTransferSource));
  offscreen_depth_.reset(
    rhi_->newRenderBuffer(QRhiRenderBuffer::DepthStencil, size, 1));
  if(!offscreen_texture_->create() || !offscreen_depth_->create()) {
    error = "could not allocate the image";
    EndOffscreen();
    return false;
  }
  QRhiTextureRenderTargetDescription description(
    QRhiColorAttachment(offscreen_texture_.get()));
  description.setDepthStencilBuffer(offscreen_depth_.get());
  offscreen_target_.reset(rhi_->newTextureRenderTarget(description));
  offscreen_render_pass_.reset(
    offscreen_target_->newCompatibleRenderPassDescriptor());
  offscreen_target_->setRenderPassDescriptor(offscreen_render_pass_.get());
  if(!offscreen_target_->create()) {
    error = "could not create the render target";
    EndOffscreen();
    return false;
  }
  return true;
}

bool shift::gui::Viewport::RenderOffscreen(int samples,
                                           std::vector<uint8_t>& rgba) {
  if(!rhi_ || !offscreen_target_) {
    return false;
  }
  if(cad_triangles_dirty_) {
    TriangulateCad();
  }
  if(render_mode_ != RenderMode::kRayTraced || !ray_tracing_supported_) {
    samples = 1;
  }
  QRhiReadbackResult readback;
  for(int s = 0; s < std::max(samples, 1); s++) {
    QRhiCommandBuffer* cb = nullptr;
    if(rhi_->beginOffscreenFrame(&cb) != QRhi::FrameOpSuccess) {
      return false;
    }
    initialize();
    render(cb);
    if(s == std::max(samples, 1) - 1) {
      QRhiResourceUpdateBatch* batch = rhi_->nextResourceUpdateBatch();
      batch->readBackTexture(QRhiReadbackDescription(offscreen_texture_.get()),
                             &readback);
      cb->resourceUpdate(batch);
    }
    // waits for the GPU, so the readback is complete after the last frame
    if(rhi_->endOffscreenFrame() != QRhi::FrameOpSuccess) {
      return false;
    }
  }
  const int width = readback.pixelSize.width();
  const int height = readback.pixelSize.height();
  const size_t row = size_t(width) * 4;
  if(width <= 0 || size_t(readback.data.size()) < row * height) {
    return false;
  }
  rgba.resize(row * height);
  const bool flip = rhi_->isYUpInFramebuffer();
  for(int y = 0; y < height; y++) {
    const int source = flip ? height - 1 - y : y;
    std::memcpy(rgba.data() + row * y, readback.data.constData() + row * source,
                row);
  }
  return true;
}

void shift::gui::Viewport::EndOffscreen() {
  if(!offscreen_target_ && !offscreen_texture_) {
    return;
  }
  offscreen_target_.reset();
  offscreen_render_pass_.reset();
  offscreen_depth_.reset();
  offscreen_texture_.reset();
  // the pipelines were made for the offscreen render pass
  pipeline_render_pass_ = nullptr;
  rt_history_valid_ = false;
  rt_frame_ = 0;
  requestUpdate();
}

void shift::gui::Viewport::exposeEvent(QExposeEvent*) {
  if(isExposed() && !rhi_) {
    InitializeRhi();
  }
  if(isExposed() && rhi_) {
    RenderFrame();
  }
}

bool shift::gui::Viewport::event(QEvent* event) {
  switch(event->type()) {
    case QEvent::UpdateRequest:
      if(isExposed() && rhi_) {
        RenderFrame();
      }
      break;
    case QEvent::PlatformSurface:
      // the swap chain must go before the native surface does
      if(static_cast<QPlatformSurfaceEvent*>(event)->surfaceEventType() ==
         QPlatformSurfaceEvent::SurfaceAboutToBeDestroyed) {
        ReleaseRhi();
      }
      break;
    default:
      break;
  }
  return QWindow::event(event);
}

void shift::gui::Viewport::RenderFrame() {
  if(offscreen_target_) {
    return; // the window waits until offscreen rendering ends
  }
  if(!swapchain_ready_ ||
     swapchain_->currentPixelSize() != swapchain_->surfacePixelSize()) {
    swapchain_ready_ = swapchain_->createOrResize();
    if(!swapchain_ready_) {
      return; // zero size, e.g. minimized
    }
  }

  if(cad_triangles_dirty_) {
    TriangulateCad(); // in a frame of its own, before the scene's upload
  }
  const QRhi::FrameOpResult result = rhi_->beginFrame(swapchain_.get());
  if(result == QRhi::FrameOpSwapChainOutOfDate) {
    swapchain_ready_ = false;
    requestUpdate();
    return;
  }
  if(result != QRhi::FrameOpSuccess) {
    return;
  }
  initialize();
  render(swapchain_->currentFrameCommandBuffer());
  rhi_->endFrame(swapchain_.get());
}

void shift::gui::Viewport::initialize() {
  if(!device_resources_ready_) {
    CreateDeviceResources();
    device_resources_ready_ = true;
    scene_dirty_ = simulation_ != nullptr;
  }
  if(output_size_ != get_render_target()->pixelSize()) {
    CreateSizeDependentResources();
  }
  if(pipeline_render_pass_ != get_render_target()->renderPassDescriptor()) {
    CreatePipelines();
  }
}

void shift::gui::Viewport::ReleaseAll() {
  axes_pipeline_.reset();
  axes_instances_.reset();
  axes_indices_.reset();
  axes_vertices_.reset();
  axes_mesh_uploaded_ = false;
  axes_instance_count_ = 0;
  cad_pipeline_.reset();
  gpu_cads_.clear();
  cad_ubo_.reset();
  terrain_pipeline_.reset();
  gpu_terrains_.clear();
  fluid_pipeline_.reset();
  fluid_sphere_pipeline_.reset();
  fluid_bindings_.reset();
  fluid_sphere_bindings_.reset();
  for(int k = 0; k < 2; k++) {
    fluid_blur_pipeline_[k].reset();
    fluid_blur_bindings_[k].reset();
    fluid_blur_target_[k].reset();
  }
  fluid_blur_render_pass_.reset();
  fluid_sphere_target_.reset();
  fluid_sphere_render_pass_.reset();
  fluid_depth_stencil_.reset();
  fluid_depth_[0].reset();
  fluid_depth_[1].reset();
  fluid_level_ubo_.reset();
  fluid_ubo_.reset();
  particle_pipeline_.reset();
  gpu_particles_.clear();
  ies_texture_.reset();
  lights_ubo_.reset();
  texture_sampler_.reset();
  texture_array_.reset();
  linear_sampler_.reset();
  environment_brdf_.reset();
  environment_irradiance_.reset();
  environment_specular_.reset();
  blit_pipeline_.reset();
  blit_bindings_.reset();
  sampler_.reset();
  svgf_taa_pipeline_.reset();
  svgf_composite_pipeline_.reset();
  svgf_atrous_pipeline_.reset();
  svgf_variance_pipeline_.reset();
  svgf_temporal_pipeline_.reset();
  rt_pipeline_.reset();
  for(int p = 0; p < 2; p++) {
    svgf_taa_bindings_[p].reset();
    svgf_composite_bindings_[p].reset();
    rt_taa_[p].reset();
    svgf_atrous_bindings_[p][0].reset();
    svgf_atrous_bindings_[p][1].reset();
    svgf_variance_bindings_[p].reset();
    svgf_temporal_bindings_[p].reset();
    rt_bindings_[p].reset();
    rt_filter_[p].reset();
    rt_history_albedo_[p].reset();
    rt_history_moments_[p].reset();
    rt_history_illumination_[p].reset();
    rt_normal_[p].reset();
    rt_position_[p].reset();
  }
  rt_color_.reset();
  rt_direct_.reset();
  rt_albedo_.reset();
  rt_illumination_.reset();
  svgf_ubo_.reset();
  rt_accumulation_.reset();
  rt_output_.reset();
  rt_instance_node_buffer_.reset();
  rt_instance_buffer_.reset();
  rt_node_buffer_.reset();
  rt_cad_trim_index_buffer_.reset();
  rt_cad_segment_buffer_.reset();
  rt_cad_face_buffer_.reset();
  rt_triangle_face_buffer_.reset();
  rt_triangle_buffer_.reset();
  rt_ubo_.reset();
  floor_pipeline_.reset();
  sky_pipeline_.reset();
  mesh_pipeline_.reset();
  raster_bindings_.reset();
  instance_buffer_.reset();
  material_ubo_.reset();
  frame_ubo_.reset();
  gpu_meshes_.clear();
  pipeline_render_pass_ = nullptr;
  output_size_ = QSize();
  scene_dirty_ = simulation_ != nullptr;
  rt_history_valid_ = false;
}

void shift::gui::Viewport::CreateDeviceResources() {
  material_stride_ = rhi_->ubufAligned(sizeof(MaterialUniforms));

  frame_ubo_.reset(rhi_->newBuffer(QRhiBuffer::Dynamic,
                                   QRhiBuffer::UniformBuffer,
                                   sizeof(FrameUniforms)));
  frame_ubo_->create();
  material_ubo_.reset(rhi_->newBuffer(
    QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, material_stride_));
  material_ubo_->create();
  lights_ubo_.reset(rhi_->newBuffer(QRhiBuffer::Dynamic,
                                    QRhiBuffer::UniformBuffer,
                                    sizeof(LightUniforms)));
  lights_ubo_->create();
  fluid_ubo_.reset(rhi_->newBuffer(QRhiBuffer::Dynamic,
                                   QRhiBuffer::UniformBuffer,
                                   sizeof(FluidUniforms)));
  fluid_ubo_->create();
  fluid_level_stride_ = rhi_->ubufAligned(4 * sizeof(float));
  fluid_level_ubo_.reset(rhi_->newBuffer(
    QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer,
    fluid_level_stride_ * quint32(kFluidBlurLevels)));
  fluid_level_ubo_->create();
  // placeholder until a scene with IES lights is loaded
  ies_texture_.reset(rhi_->newTexture(QRhiTexture::R16F, QSize(1, 1)));
  ies_texture_->create();

  raster_bindings_.reset(rhi_->newShaderResourceBindings());

  // the arrows of the coordinate frames, uploaded with the first frame
  {
    std::vector<AxesVertex> vertices;
    std::vector<uint32_t> indices;
    MakeAxesMesh(vertices, indices, axes_arrow_index_count_);
    axes_vertices_.reset(rhi_->newBuffer(
      QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer,
      quint32(vertices.size() * sizeof(AxesVertex))));
    axes_vertices_->create();
    axes_indices_.reset(rhi_->newBuffer(
      QRhiBuffer::Immutable, QRhiBuffer::IndexBuffer,
      quint32(indices.size() * sizeof(uint32_t))));
    axes_indices_->create();
    axes_index_count_ = static_cast<uint32_t>(indices.size());
    axes_mesh_uploaded_ = false;
  }

  using Tables = EnvironmentTables;
  auto make_table = [&](int width, int height) {
    std::unique_ptr<QRhiTexture> texture(
      rhi_->newTexture(QRhiTexture::RGBA16F, QSize(width, height)));
    texture->create();
    return texture;
  };
  environment_specular_ = make_table(Tables::kZCount, Tables::kRoughnessCount);
  environment_irradiance_ = make_table(Tables::kZCount, 1);
  environment_brdf_ = make_table(Tables::kBrdfCount, Tables::kBrdfCount);
  linear_sampler_.reset(rhi_->newSampler(
    QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::None,
    QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge));
  linear_sampler_->create();
  environment_dirty_ = true;

  texture_sampler_.reset(rhi_->newSampler(
    QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::Linear,
    QRhiSampler::Repeat, QRhiSampler::Repeat));
  texture_sampler_->create();
  CreateTextureArray(1, 1); // placeholder until a scene is loaded

  sampler_.reset(rhi_->newSampler(QRhiSampler::Nearest, QRhiSampler::Nearest,
                                  QRhiSampler::None, QRhiSampler::ClampToEdge,
                                  QRhiSampler::ClampToEdge));
  sampler_->create();
  blit_bindings_.reset(rhi_->newShaderResourceBindings());

  tessellation_supported_ = rhi_->isFeatureSupported(QRhi::Tessellation);
  ray_tracing_supported_ = rhi_->isFeatureSupported(QRhi::Compute);
  if(ray_tracing_supported_) {
    rt_ubo_.reset(rhi_->newBuffer(QRhiBuffer::Dynamic,
                                  QRhiBuffer::UniformBuffer,
                                  sizeof(RtUniforms)));
    rt_ubo_->create();
    // placeholders so the bindings are valid before a scene is loaded
    rt_triangle_buffer_.reset(rhi_->newBuffer(
      QRhiBuffer::Immutable, QRhiBuffer::StorageBuffer, sizeof(RtTriangle)));
    rt_triangle_buffer_->create();
    for(auto* buffer : {&rt_triangle_face_buffer_, &rt_cad_face_buffer_,
                        &rt_cad_segment_buffer_, &rt_cad_trim_index_buffer_}) {
      buffer->reset(rhi_->newBuffer(QRhiBuffer::Immutable,
                                    QRhiBuffer::StorageBuffer, 16));
      (*buffer)->create();
    }
    rt_node_buffer_.reset(rhi_->newBuffer(
      QRhiBuffer::Immutable, QRhiBuffer::StorageBuffer, sizeof(BvhNode)));
    rt_node_buffer_->create();
    rt_instance_buffer_.reset(rhi_->newBuffer(
      QRhiBuffer::Static, QRhiBuffer::StorageBuffer, sizeof(RtInstance)));
    rt_instance_buffer_->create();
    rt_instance_node_buffer_.reset(rhi_->newBuffer(
      QRhiBuffer::Static, QRhiBuffer::StorageBuffer, sizeof(BvhNode)));
    rt_instance_node_buffer_->create();
    svgf_ubo_stride_ = rhi_->ubufAligned(sizeof(DenoiseUniforms));
    // slot 0: temporal, variance and composite; 1..: a-trous iterations
    svgf_ubo_.reset(rhi_->newBuffer(
      QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer,
      svgf_ubo_stride_ * quint32(1 + kAtrousIterations)));
    svgf_ubo_->create();
  }
}

void shift::gui::Viewport::CreateSizeDependentResources() {
  output_size_ = get_render_target()->pixelSize();
  rt_frame_ = 0;
  rt_history_valid_ = false;
  CreateFluidTargets();
  // bindings reference the textures; rebuild them along with the pipelines
  pipeline_render_pass_ = nullptr;
  if(!ray_tracing_supported_) {
    return;
  }
  auto make_image = [&](QRhiTexture::Format format) {
    std::unique_ptr<QRhiTexture> texture(rhi_->newTexture(
      format, output_size_, 1, QRhiTexture::UsedWithLoadStore));
    texture->create();
    return texture;
  };
  rt_output_ = make_image(QRhiTexture::RGBA8);
  rt_accumulation_ = make_image(QRhiTexture::RGBA32F);
  rt_illumination_ = make_image(QRhiTexture::RGBA16F);
  rt_albedo_ = make_image(QRhiTexture::RGBA16F);
  rt_direct_ = make_image(QRhiTexture::RGBA16F);
  rt_color_ = make_image(QRhiTexture::RGBA16F);
  for(int p = 0; p < 2; p++) {
    rt_position_[p] = make_image(QRhiTexture::RGBA32F);
    rt_normal_[p] = make_image(QRhiTexture::RGBA16F);
    rt_history_illumination_[p] = make_image(QRhiTexture::RGBA16F);
    rt_history_moments_[p] = make_image(QRhiTexture::RGBA32F);
    rt_history_albedo_[p] = make_image(QRhiTexture::RGBA16F);
    rt_filter_[p] = make_image(QRhiTexture::RGBA16F);
    rt_taa_[p] = make_image(QRhiTexture::RGBA16F);
  }
  // bindings reference the textures; rebuild them along with the pipelines
  pipeline_render_pass_ = nullptr;
}

void shift::gui::Viewport::CreateFluidTargets() {
  const QSize size(std::max(output_size_.width(), 1),
                   std::max(output_size_.height(), 1));
  for(int k = 0; k < 2; k++) {
    fluid_depth_[k].reset(rhi_->newTexture(QRhiTexture::R32F, size, 1,
                                           QRhiTexture::RenderTarget));
    fluid_depth_[k]->create();
  }
  fluid_depth_stencil_.reset(
    rhi_->newRenderBuffer(QRhiRenderBuffer::DepthStencil, size));
  fluid_depth_stencil_->create();

  QRhiTextureRenderTargetDescription spheres{
    QRhiColorAttachment(fluid_depth_[0].get())};
  spheres.setDepthStencilBuffer(fluid_depth_stencil_.get());
  fluid_sphere_target_.reset(rhi_->newTextureRenderTarget(spheres));
  fluid_sphere_render_pass_.reset(
    fluid_sphere_target_->newCompatibleRenderPassDescriptor());
  fluid_sphere_target_->setRenderPassDescriptor(
    fluid_sphere_render_pass_.get());
  fluid_sphere_target_->create();

  // [k] renders into fluid_depth_[k]; the targets are compatible
  fluid_blur_render_pass_.reset();
  for(int k = 0; k < 2; k++) {
    fluid_blur_target_[k].reset(rhi_->newTextureRenderTarget(
      QRhiTextureRenderTargetDescription(
        QRhiColorAttachment(fluid_depth_[k].get()))));
    if(!fluid_blur_render_pass_) {
      fluid_blur_render_pass_.reset(
        fluid_blur_target_[k]->newCompatibleRenderPassDescriptor());
    }
    fluid_blur_target_[k]->setRenderPassDescriptor(
      fluid_blur_render_pass_.get());
    fluid_blur_target_[k]->create();
  }
}

void shift::gui::Viewport::CreatePipelines() {
  using Binding = QRhiShaderResourceBinding;
  const auto vertex_fragment = Binding::VertexStage | Binding::FragmentStage;

  raster_bindings_->setBindings(
    {Binding::uniformBuffer(0, vertex_fragment, frame_ubo_.get()),
     Binding::uniformBufferWithDynamicOffset(
       1, vertex_fragment, material_ubo_.get(), sizeof(MaterialUniforms)),
     Binding::sampledTexture(2, Binding::FragmentStage,
                             environment_specular_.get(),
                             linear_sampler_.get()),
     Binding::sampledTexture(3, Binding::FragmentStage,
                             environment_irradiance_.get(),
                             linear_sampler_.get()),
     Binding::sampledTexture(4, Binding::FragmentStage,
                             environment_brdf_.get(), linear_sampler_.get()),
     Binding::sampledTexture(5, Binding::FragmentStage, texture_array_.get(),
                             texture_sampler_.get()),
     Binding::uniformBuffer(8, Binding::FragmentStage, lights_ubo_.get()),
     Binding::sampledTexture(9, Binding::FragmentStage, ies_texture_.get(),
                             linear_sampler_.get())});
  raster_bindings_->create();

  pipeline_render_pass_ = get_render_target()->renderPassDescriptor();

  auto make_pipeline = [&](const char* vertex, const char* fragment,
                           QRhiShaderResourceBindings* bindings,
                           bool depth) {
    auto* pipeline = rhi_->newGraphicsPipeline();
    pipeline->setShaderStages(
      {{QRhiShaderStage::Vertex, LoadShader(vertex)},
       {QRhiShaderStage::Fragment, LoadShader(fragment)}});
    pipeline->setShaderResourceBindings(bindings);
    pipeline->setRenderPassDescriptor(pipeline_render_pass_);
    pipeline->setTopology(QRhiGraphicsPipeline::Triangles);
    pipeline->setCullMode(QRhiGraphicsPipeline::None);
    pipeline->setDepthTest(depth);
    pipeline->setDepthWrite(depth);
    pipeline->setDepthOp(QRhiGraphicsPipeline::LessOrEqual);
    return pipeline;
  };

  mesh_pipeline_.reset(make_pipeline("mesh.vert.qsb", "mesh.frag.qsb",
                                     raster_bindings_.get(), true));
  // binding 0: mesh vertices; binding 1: the pose of each instance, as the
  // four columns of a matrix
  QRhiVertexInputLayout layout;
  layout.setBindings(
    {QRhiVertexInputBinding(sizeof(Vertex)),
     QRhiVertexInputBinding(sizeof(InstanceAttributes),
                            QRhiVertexInputBinding::PerInstance)});
  layout.setAttributes(
    {QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3,
                              offsetof(Vertex, position)),
     QRhiVertexInputAttribute(0, 1, QRhiVertexInputAttribute::Float3,
                              offsetof(Vertex, normal)),
     QRhiVertexInputAttribute(0, 2, QRhiVertexInputAttribute::Float2,
                              offsetof(Vertex, uv)),
     QRhiVertexInputAttribute(1, 3, QRhiVertexInputAttribute::Float4, 0),
     QRhiVertexInputAttribute(1, 4, QRhiVertexInputAttribute::Float4, 16),
     QRhiVertexInputAttribute(1, 5, QRhiVertexInputAttribute::Float4, 32),
     QRhiVertexInputAttribute(1, 6, QRhiVertexInputAttribute::Float4, 48)});
  mesh_pipeline_->setVertexInputLayout(layout);
  mesh_pipeline_->create();

  // particles: the meshes' vertices, and the centre of each instance
  particle_pipeline_.reset(make_pipeline("particle.vert.qsb", "mesh.frag.qsb",
                                         raster_bindings_.get(), true));
  QRhiVertexInputLayout particle_layout;
  particle_layout.setBindings(
    {QRhiVertexInputBinding(sizeof(Vertex)),
     QRhiVertexInputBinding(4 * sizeof(float),
                            QRhiVertexInputBinding::PerInstance)});
  particle_layout.setAttributes(
    {QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3,
                              offsetof(Vertex, position)),
     QRhiVertexInputAttribute(0, 1, QRhiVertexInputAttribute::Float3,
                              offsetof(Vertex, normal)),
     QRhiVertexInputAttribute(0, 2, QRhiVertexInputAttribute::Float2,
                              offsetof(Vertex, uv)),
     QRhiVertexInputAttribute(1, 3, QRhiVertexInputAttribute::Float4, 0)});
  particle_pipeline_->setVertexInputLayout(particle_layout);
  particle_pipeline_->create();

  // their surface: the spheres' depth, the blur (from [0] into [1] and
  // back), and the shading of the surface in the target
  fluid_sphere_bindings_.reset(rhi_->newShaderResourceBindings());
  fluid_sphere_bindings_->setBindings(
    {Binding::uniformBuffer(0, vertex_fragment, fluid_ubo_.get())});
  fluid_sphere_bindings_->create();
  fluid_sphere_pipeline_.reset(rhi_->newGraphicsPipeline());
  fluid_sphere_pipeline_->setShaderStages(
    {{QRhiShaderStage::Vertex, LoadShader("fluid_depth.vert.qsb")},
     {QRhiShaderStage::Fragment, LoadShader("fluid_depth.frag.qsb")}});
  fluid_sphere_pipeline_->setShaderResourceBindings(
    fluid_sphere_bindings_.get());
  fluid_sphere_pipeline_->setRenderPassDescriptor(
    fluid_sphere_render_pass_.get());
  fluid_sphere_pipeline_->setCullMode(QRhiGraphicsPipeline::None);
  fluid_sphere_pipeline_->setDepthTest(true);
  fluid_sphere_pipeline_->setDepthWrite(true);
  fluid_sphere_pipeline_->setDepthOp(QRhiGraphicsPipeline::LessOrEqual);
  QRhiVertexInputLayout center_layout;
  center_layout.setBindings({QRhiVertexInputBinding(
    4 * sizeof(float), QRhiVertexInputBinding::PerInstance)});
  center_layout.setAttributes(
    {QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float4, 0)});
  fluid_sphere_pipeline_->setVertexInputLayout(center_layout);
  fluid_sphere_pipeline_->create();

  for(int k = 0; k < 2; k++) {
    fluid_blur_bindings_[k].reset(rhi_->newShaderResourceBindings());
    fluid_blur_bindings_[k]->setBindings(
      {Binding::sampledTexture(0, Binding::FragmentStage,
                               fluid_depth_[k].get(), sampler_.get()),
       Binding::uniformBuffer(1, Binding::FragmentStage, fluid_ubo_.get()),
       Binding::uniformBufferWithDynamicOffset(
         2, Binding::FragmentStage, fluid_level_ubo_.get(),
         4 * sizeof(float))});
    fluid_blur_bindings_[k]->create();
    fluid_blur_pipeline_[k].reset(rhi_->newGraphicsPipeline());
    fluid_blur_pipeline_[k]->setShaderStages(
      {{QRhiShaderStage::Vertex, LoadShader("fullscreen.vert.qsb")},
       {QRhiShaderStage::Fragment, LoadShader("fluid_blur.frag.qsb")}});
    fluid_blur_pipeline_[k]->setShaderResourceBindings(
      fluid_blur_bindings_[k].get());
    fluid_blur_pipeline_[k]->setRenderPassDescriptor(
      fluid_blur_render_pass_.get());
    fluid_blur_pipeline_[k]->setCullMode(QRhiGraphicsPipeline::None);
    fluid_blur_pipeline_[k]->create();
  }

  fluid_bindings_.reset(rhi_->newShaderResourceBindings());
  fluid_bindings_->setBindings(
    {Binding::uniformBuffer(0, vertex_fragment, frame_ubo_.get()),
     Binding::uniformBufferWithDynamicOffset(
       1, vertex_fragment, material_ubo_.get(), sizeof(MaterialUniforms)),
     Binding::sampledTexture(2, Binding::FragmentStage,
                             environment_specular_.get(),
                             linear_sampler_.get()),
     Binding::sampledTexture(3, Binding::FragmentStage,
                             environment_irradiance_.get(),
                             linear_sampler_.get()),
     Binding::sampledTexture(4, Binding::FragmentStage,
                             environment_brdf_.get(), linear_sampler_.get()),
     Binding::sampledTexture(5, Binding::FragmentStage, texture_array_.get(),
                             texture_sampler_.get()),
     Binding::uniformBuffer(8, Binding::FragmentStage, lights_ubo_.get()),
     Binding::sampledTexture(9, Binding::FragmentStage, ies_texture_.get(),
                             linear_sampler_.get()),
     Binding::sampledTexture(10, Binding::FragmentStage,
                             fluid_depth_[0].get(), sampler_.get()),
     Binding::uniformBuffer(11, Binding::FragmentStage, fluid_ubo_.get())});
  fluid_bindings_->create();
  fluid_pipeline_.reset(make_pipeline("fullscreen.vert.qsb", "fluid.frag.qsb",
                                      fluid_bindings_.get(), true));
  fluid_pipeline_->create();

  sky_pipeline_.reset(make_pipeline("fullscreen.vert.qsb", "sky.frag.qsb",
                                    raster_bindings_.get(), false));
  sky_pipeline_->create();

  floor_pipeline_.reset(make_pipeline("floor.vert.qsb", "floor.frag.qsb",
                                      raster_bindings_.get(), true));
  floor_pipeline_->create();

  // the arrows of the coordinate frames: binding 0 their vertices, binding
  // 1 the pose of each frame, as the meshes
  axes_pipeline_.reset(make_pipeline("axes.vert.qsb", "axes.frag.qsb",
                                     raster_bindings_.get(), true));
  QRhiVertexInputLayout axes_layout;
  axes_layout.setBindings(
    {QRhiVertexInputBinding(sizeof(AxesVertex)),
     QRhiVertexInputBinding(sizeof(AxesInstance),
                            QRhiVertexInputBinding::PerInstance)});
  axes_layout.setAttributes(
    {QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3,
                              offsetof(AxesVertex, position)),
     QRhiVertexInputAttribute(0, 1, QRhiVertexInputAttribute::Float3,
                              offsetof(AxesVertex, normal)),
     QRhiVertexInputAttribute(0, 2, QRhiVertexInputAttribute::Float3,
                              offsetof(AxesVertex, color)),
     QRhiVertexInputAttribute(1, 3, QRhiVertexInputAttribute::Float4, 0),
     QRhiVertexInputAttribute(1, 4, QRhiVertexInputAttribute::Float4, 16),
     QRhiVertexInputAttribute(1, 5, QRhiVertexInputAttribute::Float4, 32),
     QRhiVertexInputAttribute(1, 6, QRhiVertexInputAttribute::Float4, 48),
     QRhiVertexInputAttribute(1, 7, QRhiVertexInputAttribute::Float4,
                              offsetof(AxesInstance, tint))});
  axes_pipeline_->setVertexInputLayout(axes_layout);
  axes_pipeline_->create();

  // terrains: the bindings of the meshes, with the terrain's uniforms and
  // control points for the tessellation stages
  terrain_pipeline_.reset();
  if(tessellation_supported_ && !gpu_terrains_.empty()) {
    const auto tessellation =
      Binding::TessellationControlStage | Binding::TessellationEvaluationStage;
    for(auto& gpu : gpu_terrains_) {
      gpu.bindings.reset(rhi_->newShaderResourceBindings());
      gpu.bindings->setBindings(
        {Binding::uniformBuffer(0, vertex_fragment | tessellation,
                                frame_ubo_.get()),
         Binding::uniformBufferWithDynamicOffset(
           1, vertex_fragment, material_ubo_.get(), sizeof(MaterialUniforms)),
         Binding::sampledTexture(2, Binding::FragmentStage,
                                 environment_specular_.get(),
                                 linear_sampler_.get()),
         Binding::sampledTexture(3, Binding::FragmentStage,
                                 environment_irradiance_.get(),
                                 linear_sampler_.get()),
         Binding::sampledTexture(4, Binding::FragmentStage,
                                 environment_brdf_.get(),
                                 linear_sampler_.get()),
         Binding::sampledTexture(5, Binding::FragmentStage,
                                 texture_array_.get(), texture_sampler_.get()),
         Binding::uniformBuffer(6, Binding::VertexStage | tessellation,
                                gpu.uniforms.get()),
         Binding::sampledTexture(7, tessellation, gpu.control_points.get(),
                                 sampler_.get()),
         Binding::uniformBuffer(8, Binding::FragmentStage, lights_ubo_.get()),
         Binding::sampledTexture(9, Binding::FragmentStage,
                                 ies_texture_.get(), linear_sampler_.get())});
      gpu.bindings->create();
    }
    terrain_pipeline_.reset(rhi_->newGraphicsPipeline());
    terrain_pipeline_->setShaderStages(
      {{QRhiShaderStage::Vertex, LoadShader("terrain.vert.qsb")},
       {QRhiShaderStage::TessellationControl, LoadShader("terrain.tesc.qsb")},
       {QRhiShaderStage::TessellationEvaluation,
        LoadShader("terrain.tese.qsb")},
       {QRhiShaderStage::Fragment, LoadShader("mesh.frag.qsb")}});
    terrain_pipeline_->setShaderResourceBindings(
      gpu_terrains_[0].bindings.get());
    terrain_pipeline_->setRenderPassDescriptor(pipeline_render_pass_);
    terrain_pipeline_->setTopology(QRhiGraphicsPipeline::Patches);
    terrain_pipeline_->setPatchControlPointCount(4);
    terrain_pipeline_->setCullMode(QRhiGraphicsPipeline::None);
    terrain_pipeline_->setDepthTest(true);
    terrain_pipeline_->setDepthWrite(true);
    terrain_pipeline_->setDepthOp(QRhiGraphicsPipeline::LessOrEqual);
    terrain_pipeline_->setVertexInputLayout(QRhiVertexInputLayout());
    if(!terrain_pipeline_->create()) {
      qWarning("Cannot create the terrain tessellation pipeline; terrains are "
               "drawn as meshes");
      terrain_pipeline_.reset();
    }
  }

  // CAD geometry: the bindings of the meshes, with the records of its faces
  // for the tessellation stages and its trims for the fragment stage
  cad_pipeline_.reset();
  if(tessellation_supported_ && !gpu_cads_.empty()) {
    const auto tessellation =
      Binding::TessellationControlStage | Binding::TessellationEvaluationStage;
    const auto all = vertex_fragment | tessellation;
    for(auto& gpu : gpu_cads_) {
      gpu.bindings.reset(rhi_->newShaderResourceBindings());
      gpu.bindings->setBindings(
        {Binding::uniformBuffer(0, all, frame_ubo_.get()),
         Binding::uniformBufferWithDynamicOffset(
           1, vertex_fragment, material_ubo_.get(), sizeof(MaterialUniforms)),
         Binding::sampledTexture(2, Binding::FragmentStage,
                                 environment_specular_.get(),
                                 linear_sampler_.get()),
         Binding::sampledTexture(3, Binding::FragmentStage,
                                 environment_irradiance_.get(),
                                 linear_sampler_.get()),
         Binding::sampledTexture(4, Binding::FragmentStage,
                                 environment_brdf_.get(),
                                 linear_sampler_.get()),
         Binding::sampledTexture(5, Binding::FragmentStage,
                                 texture_array_.get(), texture_sampler_.get()),
         Binding::uniformBuffer(6, Binding::FragmentStage | tessellation,
                                cad_ubo_.get()),
         Binding::uniformBuffer(8, Binding::FragmentStage, lights_ubo_.get()),
         Binding::sampledTexture(9, Binding::FragmentStage,
                                 ies_texture_.get(), linear_sampler_.get()),
         Binding::bufferLoad(10, all, gpu.faces.get()),
         Binding::bufferLoad(11, all, gpu.curves.get()),
         Binding::bufferLoad(12, all, gpu.patches.get()),
         Binding::bufferLoad(13, all, gpu.points.get()),
         Binding::bufferLoad(14, all, gpu.knots.get()),
         Binding::bufferLoad(15, Binding::FragmentStage, gpu.segments.get()),
         Binding::bufferLoad(16, Binding::FragmentStage,
                             gpu.trim_index.get())});
      gpu.bindings->create();
    }
    cad_pipeline_.reset(rhi_->newGraphicsPipeline());
    cad_pipeline_->setShaderStages(
      {{QRhiShaderStage::Vertex, LoadShader("cad.vert.qsb")},
       {QRhiShaderStage::TessellationControl, LoadShader("cad.tesc.qsb")},
       {QRhiShaderStage::TessellationEvaluation, LoadShader("cad.tese.qsb")},
       {QRhiShaderStage::Fragment, LoadShader("cad.frag.qsb")}});
    cad_pipeline_->setShaderResourceBindings(gpu_cads_[0].bindings.get());
    cad_pipeline_->setRenderPassDescriptor(pipeline_render_pass_);
    cad_pipeline_->setTopology(QRhiGraphicsPipeline::Patches);
    cad_pipeline_->setPatchControlPointCount(4);
    cad_pipeline_->setCullMode(QRhiGraphicsPipeline::None);
    cad_pipeline_->setDepthTest(true);
    cad_pipeline_->setDepthWrite(true);
    cad_pipeline_->setDepthOp(QRhiGraphicsPipeline::LessOrEqual);
    // the pose of each body drawn, per instance, as in mesh.vert
    QRhiVertexInputLayout layout;
    layout.setBindings({QRhiVertexInputBinding(
      sizeof(InstanceAttributes), QRhiVertexInputBinding::PerInstance)});
    layout.setAttributes(
      {QRhiVertexInputAttribute(0, 3, QRhiVertexInputAttribute::Float4, 0),
       QRhiVertexInputAttribute(0, 4, QRhiVertexInputAttribute::Float4, 16),
       QRhiVertexInputAttribute(0, 5, QRhiVertexInputAttribute::Float4, 32),
       QRhiVertexInputAttribute(0, 6, QRhiVertexInputAttribute::Float4, 48)});
    cad_pipeline_->setVertexInputLayout(layout);
    if(!cad_pipeline_->create()) {
      qWarning("Cannot create the CAD tessellation pipeline; CAD geometry is "
               "not drawn");
      cad_pipeline_.reset();
    }
  }

  if(!ray_tracing_supported_) {
    return;
  }

  const auto compute = Binding::ComputeStage;
  auto make_bindings = [&](std::initializer_list<Binding> bindings) {
    std::unique_ptr<QRhiShaderResourceBindings> srb(
      rhi_->newShaderResourceBindings());
    srb->setBindings(bindings);
    srb->create();
    return srb;
  };
  auto load = [&](int binding, const std::unique_ptr<QRhiTexture>& texture) {
    return Binding::imageLoad(binding, compute, texture.get(), 0);
  };
  auto store = [&](int binding, const std::unique_ptr<QRhiTexture>& texture) {
    return Binding::imageStore(binding, compute, texture.get(), 0);
  };
  // slot 0 of the denoiser uniforms
  const Binding denoise_uniforms = Binding::uniformBuffer(
    0, compute, svgf_ubo_.get(), 0, sizeof(DenoiseUniforms));
  const Binding instances =
    Binding::bufferLoad(1, compute, rt_instance_buffer_.get());

  // parity p writes set p of the G-buffer and the history, reading set q
  for(int p = 0; p < 2; p++) {
    const int q = 1 - p;
    rt_bindings_[p] = make_bindings(
      {store(0, rt_illumination_),
       Binding::uniformBuffer(1, compute, rt_ubo_.get()),
       Binding::bufferLoad(2, compute, rt_triangle_buffer_.get()),
       Binding::bufferLoad(3, compute, rt_node_buffer_.get()),
       Binding::bufferLoad(4, compute, rt_instance_buffer_.get()),
       Binding::imageLoadStore(5, compute, rt_accumulation_.get(), 0),
       Binding::sampledTexture(6, compute, texture_array_.get(),
                               texture_sampler_.get()),
       store(7, rt_position_[p]), store(8, rt_normal_[p]),
       store(9, rt_albedo_), store(10, rt_direct_),
       Binding::bufferLoad(11, compute, rt_instance_node_buffer_.get()),
       Binding::uniformBuffer(12, compute, lights_ubo_.get()),
       Binding::sampledTexture(13, compute, ies_texture_.get(),
                               linear_sampler_.get()),
       Binding::bufferLoad(14, compute, rt_triangle_face_buffer_.get()),
       Binding::bufferLoad(15, compute, rt_cad_face_buffer_.get()),
       Binding::bufferLoad(16, compute, rt_cad_segment_buffer_.get()),
       Binding::bufferLoad(17, compute, rt_cad_trim_index_buffer_.get())});
    svgf_temporal_bindings_[p] = make_bindings(
      {denoise_uniforms, instances, load(2, rt_illumination_),
       load(3, rt_albedo_), load(4, rt_position_[p]), load(5, rt_normal_[p]),
       load(6, rt_position_[q]), load(7, rt_normal_[q]),
       load(8, rt_history_illumination_[q]), load(9, rt_history_moments_[q]),
       load(10, rt_history_albedo_[q]), store(11, rt_history_illumination_[p]),
       store(12, rt_history_moments_[p]), store(13, rt_history_albedo_[p])});
    svgf_variance_bindings_[p] = make_bindings(
      {denoise_uniforms, load(1, rt_position_[p]), load(2, rt_normal_[p]),
       load(3, rt_history_illumination_[p]), load(4, rt_history_moments_[p]),
       store(5, rt_filter_[0])});
    for(int d = 0; d < 2; d++) {
      svgf_atrous_bindings_[p][d] = make_bindings(
        {Binding::uniformBufferWithDynamicOffset(0, compute, svgf_ubo_.get(),
                                                 sizeof(DenoiseUniforms)),
         load(1, rt_position_[p]), load(2, rt_normal_[p]),
         load(3, rt_filter_[d]), store(4, rt_filter_[1 - d]),
         // the first iteration's output becomes the history illumination
         Binding::imageLoadStore(5, compute,
                                 rt_history_illumination_[p].get(), 0)});
    }
    // an odd number of iterations ends in rt_filter_[1]
    static_assert(kAtrousIterations % 2 == 1);
    svgf_composite_bindings_[p] = make_bindings(
      {denoise_uniforms, load(1, rt_position_[p]), load(2, rt_illumination_),
       load(3, rt_history_albedo_[p]), load(4, rt_filter_[1]),
       store(6, rt_color_), load(7, rt_direct_)});
    svgf_taa_bindings_[p] = make_bindings(
      {denoise_uniforms, instances, load(2, rt_position_[p]),
       load(3, rt_color_), load(4, rt_taa_[q]), store(5, rt_taa_[p]),
       store(6, rt_output_), load(7, rt_accumulation_),
       load(8, rt_position_[q]), load(9, rt_normal_[p]),
       load(10, rt_normal_[q])});
  }

  rt_pipeline_.reset(NewComputePipeline("raytrace.comp.qsb",
                                        rt_bindings_[0].get()));
  svgf_temporal_pipeline_.reset(NewComputePipeline(
    "svgf_temporal.comp.qsb", svgf_temporal_bindings_[0].get()));
  svgf_variance_pipeline_.reset(NewComputePipeline(
    "svgf_variance.comp.qsb", svgf_variance_bindings_[0].get()));
  svgf_atrous_pipeline_.reset(NewComputePipeline(
    "svgf_atrous.comp.qsb", svgf_atrous_bindings_[0][0].get()));
  svgf_composite_pipeline_.reset(NewComputePipeline(
    "svgf_composite.comp.qsb", svgf_composite_bindings_[0].get()));
  svgf_taa_pipeline_.reset(
    NewComputePipeline("svgf_taa.comp.qsb", svgf_taa_bindings_[0].get()));

  blit_bindings_->setBindings({Binding::sampledTexture(
    0, Binding::FragmentStage, rt_output_.get(), sampler_.get())});
  blit_bindings_->create();
  blit_pipeline_.reset(make_pipeline("fullscreen.vert.qsb", "blit.frag.qsb",
                                     blit_bindings_.get(), false));
  blit_pipeline_->create();
}

QRhiComputePipeline* shift::gui::Viewport::NewComputePipeline(
  const char* shader, QRhiShaderResourceBindings* bindings) {
  QRhiComputePipeline* pipeline = rhi_->newComputePipeline();
  pipeline->setShaderStage({QRhiShaderStage::Compute, LoadShader(shader)});
  pipeline->setShaderResourceBindings(bindings);
  pipeline->create();
  return pipeline;
}

void shift::gui::Viewport::CreateTextureArray(int layer_count, int size) {
  QRhiTexture::Flags flags;
  if(size > 1) {
    flags |= QRhiTexture::MipMapped | QRhiTexture::UsedWithGenerateMips;
  }
  texture_array_.reset(rhi_->newTextureArray(
    QRhiTexture::RGBA8, layer_count, QSize(size, size), 1, flags));
  texture_array_->create();
}

void shift::gui::Viewport::UploadEnvironment(QRhiResourceUpdateBatch* batch) {
  Sky sky;
  sky.zenith = QVector3D(kSkyZenith[0], kSkyZenith[1], kSkyZenith[2]);
  sky.horizon = QVector3D(kSkyHorizon[0], kSkyHorizon[1], kSkyHorizon[2]);
  const EnvironmentTables tables = ComputeEnvironmentTables(sky);
  auto upload = [&](QRhiTexture* texture, const std::vector<float>& values) {
    batch->uploadTexture(texture,
                         QRhiTextureUploadDescription(
                           {0, 0, QRhiTextureSubresourceUploadDescription(
                                    ToHalf(values))}));
  };
  upload(environment_specular_.get(), tables.specular);
  upload(environment_irradiance_.get(), tables.irradiance);
  upload(environment_brdf_.get(), tables.brdf);
  environment_dirty_ = false;
}

/* the instances of ray tracing and picking: the meshes' instances, then one
per body of each SceneCad, with mesh_index meshes.size() + its index */
static std::vector<shift::gui::SceneInstance>
RtInstanceList(const shift::gui::Scene& scene) {
  std::vector<shift::gui::SceneInstance> list = scene.instances;
  for(size_t c = 0; c < scene.cads.size(); c++) {
    for(int body : scene.cads[c].bodies) {
      list.push_back({static_cast<int>(scene.meshes.size() + c), body});
    }
  }
  return list;
}

/* one BVH per mesh in body coordinates, shared by its instances and
concatenated with absolute node and triangle indices, then one per SceneCad
from its triangles (cad_faces: their faces, made global with the face_base
of trims); roots holds the root node of each, faces the face of each
triangle (-1 for a mesh's) */
static void BuildMeshBvhs(
  const shift::gui::Scene& scene,
  const std::vector<std::vector<shift::gui::RtTriangle>>& cad_triangles,
  const std::vector<std::vector<int32_t>>& cad_faces,
  const shift::gui::CadTrimTables& trims,
  std::vector<shift::gui::RtTriangle>& all_triangles,
  std::vector<int32_t>& all_faces, std::vector<shift::gui::BvhNode>& all_nodes,
  std::vector<int>& roots,
  std::vector<shift::gui::RtDeformableTerrain>* deformables = nullptr) {
  int last_node_count = 0;
  auto append = [&](std::vector<shift::gui::RtTriangle>& triangles,
                    std::vector<int32_t>& faces) {
    std::vector<shift::gui::BvhNode> nodes =
      shift::gui::BuildBvh(triangles, &faces);
    if(nodes.empty()) {
      // a leaf without triangles that no ray enters, so the root is its own
      shift::gui::BvhNode empty{};
      for(int k = 0; k < 3; k++) {
        empty.box_min[k] = 1e30f;
        empty.box_max[k] = -1e30f;
      }
      nodes.push_back(empty);
    }
    const int node_offset = static_cast<int>(all_nodes.size());
    const int triangle_offset = static_cast<int>(all_triangles.size());
    for(auto& node : nodes) {
      node.right_or_first +=
        node.triangle_count < 0 ? node_offset : triangle_offset;
    }
    roots.push_back(node_offset);
    last_node_count = static_cast<int>(nodes.size());
    all_nodes.insert(all_nodes.end(), nodes.begin(), nodes.end());
    all_triangles.insert(all_triangles.end(), triangles.begin(),
                         triangles.end());
    all_faces.insert(all_faces.end(), faces.begin(), faces.end());
  };
  if(deformables) {
    deformables->clear();
  }
  for(size_t m = 0; m < scene.meshes.size(); m++) {
    const auto& mesh = scene.meshes[m];
    std::vector<shift::gui::RtTriangle> triangles(mesh.indices.size() / 3);
    for(size_t i = 0; i < triangles.size(); i++) {
      float* positions[3] = {triangles[i].v0, triangles[i].v1, triangles[i].v2};
      float* normals[3] = {triangles[i].n0, triangles[i].n1, triangles[i].n2};
      for(int j = 0; j < 3; j++) {
        const shift::gui::Vertex& v = mesh.vertices[mesh.indices[3 * i + j]];
        std::copy(v.position, v.position + 3, positions[j]);
        std::copy(v.normal, v.normal + 3, normals[j]);
        // texture coordinates in the spare components
        positions[j][3] = v.uv[0];
        normals[j][3] = v.uv[1];
      }
    }
    const bool deformable =
      deformables && mesh.terrain >= 0 &&
      mesh.terrain < static_cast<int>(scene.terrains.size()) &&
      scene.terrains[mesh.terrain].deformable;
    std::vector<int32_t> faces(triangles.size(), -1);
    if(deformable) {
      // the BVH reorders the tags with the triangles: where each went
      for(size_t i = 0; i < faces.size(); i++) {
        faces[i] = static_cast<int32_t>(i);
      }
    }
    const int first_triangle = static_cast<int>(all_triangles.size());
    append(triangles, faces);
    if(deformable) {
      shift::gui::RtDeformableTerrain d;
      d.mesh = static_cast<int>(m);
      d.terrain = mesh.terrain;
      d.first_triangle = first_triangle;
      d.first_node = roots.back();
      d.node_count = last_node_count;
      d.position.assign(faces.size(), 0);
      for(size_t k = 0; k < faces.size(); k++) {
        d.position[size_t(faces[k])] = static_cast<int32_t>(k);
        all_faces[size_t(first_triangle) + k] = -1;
      }
      deformables->push_back(std::move(d));
    }
  }
  for(size_t c = 0; c < scene.cads.size(); c++) {
    std::vector<shift::gui::RtTriangle> triangles;
    std::vector<int32_t> faces;
    if(c < cad_triangles.size() && c < cad_faces.size()) {
      triangles = cad_triangles[c];
      faces = cad_faces[c];
      for(int32_t& f : faces) {
        f = 2 * ((f >> 1) + trims.face_base[c]) + (f & 1);
      }
    }
    append(triangles, faces);
  }
}

/* distance along the ray o + t d to the box, if it is hit before t_max */
static bool RayHitsBox(const QVector3D& o, const QVector3D& inverse_d,
                       const float box_min[3], const float box_max[3],
                       float t_max) {
  float t_near = 0.f, t_far = t_max;
  for(int k = 0; k < 3; k++) {
    float t0 = (box_min[k] - o[k]) * inverse_d[k];
    float t1 = (box_max[k] - o[k]) * inverse_d[k];
    if(t0 > t1) {
      std::swap(t0, t1);
    }
    t_near = std::max(t_near, t0);
    t_far = std::min(t_far, t1);
  }
  return t_near <= t_far;
}

/* Moller-Trumbore: the distance t along the ray o + t d to the triangle, if
it is hit at 0 < t < t_max */
static bool RayHitsTriangle(const QVector3D& o, const QVector3D& d,
                            const shift::gui::RtTriangle& triangle, float t_max,
                            float& t, float& b1_out, float& b2_out) {
  const QVector3D v0(triangle.v0[0], triangle.v0[1], triangle.v0[2]);
  const QVector3D e1 =
    QVector3D(triangle.v1[0], triangle.v1[1], triangle.v1[2]) - v0;
  const QVector3D e2 =
    QVector3D(triangle.v2[0], triangle.v2[1], triangle.v2[2]) - v0;
  const QVector3D p = QVector3D::crossProduct(d, e2);
  const float det = QVector3D::dotProduct(e1, p);
  if(std::fabs(det) < 1e-12f) {
    return false;
  }
  const float inverse_det = 1.f / det;
  const QVector3D s = o - v0;
  const float b1 = QVector3D::dotProduct(s, p) * inverse_det;
  if(b1 < 0.f || b1 > 1.f) {
    return false;
  }
  const QVector3D q = QVector3D::crossProduct(s, e1);
  const float b2 = QVector3D::dotProduct(d, q) * inverse_det;
  if(b2 < 0.f || b1 + b2 > 1.f) {
    return false;
  }
  const float hit = QVector3D::dotProduct(e2, q) * inverse_det;
  if(hit <= 0.f || hit >= t_max) {
    return false;
  }
  t = hit;
  b1_out = b1;
  b2_out = b2;
  return true;
}

void shift::gui::Viewport::UploadParticleUpdates(
  QRhiResourceUpdateBatch* batch, bool all) {
  simulation_->TakeParticleUpdates(particle_updates_, all);
  for(const auto& update : particle_updates_) {
    for(auto& gpu : gpu_particles_) {
      if(gpu.particles_index != update.particles) {
        continue;
      }
      const quint32 bytes = quint32(update.centers.size() * sizeof(float));
      gpu.count = quint32(update.centers.size() / 4);
      if(bytes == 0) {
        continue;
      }
      if(!gpu.centers || gpu.centers->size() < bytes) {
        gpu.centers.reset(rhi_->newBuffer(QRhiBuffer::Static,
                                          QRhiBuffer::VertexBuffer, bytes));
        gpu.centers->create();
      }
      batch->uploadStaticBuffer(gpu.centers.get(), 0, bytes,
                                update.centers.data());
    }
  }
}

void shift::gui::Viewport::UploadTerrainUpdates(
  QRhiResourceUpdateBatch* batch) {
  simulation_->TakeTerrainUpdates(terrain_updates_);
  if(rt_terrain_smooth_ != smooth_terrain_) {
    // the ray tracer's terrains all over, on the other surface
    rt_terrain_smooth_ = smooth_terrain_;
    for(const auto& terrain : rt_terrains_) {
      const SceneTerrain& t = simulation_->get_scene().terrains[terrain.terrain];
      TerrainUpdate all;
      all.terrain = terrain.terrain;
      all.width = t.width;
      all.height = t.height;
      terrain_updates_.push_back(std::move(all)); // z unused: no texture
    }
  }
  for(const auto& update : terrain_updates_) {
    for(auto& gpu : gpu_terrains_) {
      if(gpu.terrain_index != update.terrain || update.z.empty()) {
        continue;
      }
      QRhiTextureSubresourceUploadDescription points(
        update.z.data(), quint32(update.z.size() * sizeof(float)));
      points.setDestinationTopLeft(QPoint(update.x, update.y));
      points.setSourceSize(QSize(update.width, update.height));
      batch->uploadTexture(gpu.control_points.get(),
                           QRhiTextureUploadEntry(0, 0, points));
    }
    // the ray tracer's triangles: the cells around the control points
    // changed, whose vertex normals change too
    if(!ray_tracing_supported_ || !rt_triangle_buffer_ ||
       !rt_node_buffer_) {
      continue;
    }
    for(const auto& terrain : rt_terrains_) {
      if(terrain.terrain != update.terrain) {
        continue;
      }
      int lo = INT32_MAX, hi = -1;
      ShapeRtTerrain(terrain, update.x - 2, update.y - 2,
                     update.x + update.width, update.y + update.height,
                     pick_triangles_, pick_nodes_, lo, hi);
      if(hi < lo) {
        continue;
      }
      const size_t first = size_t(terrain.first_triangle + lo);
      batch->uploadStaticBuffer(
        rt_triangle_buffer_.get(), quint32(first * sizeof(RtTriangle)),
        quint32(size_t(hi - lo + 1) * sizeof(RtTriangle)),
        pick_triangles_.data() + first);
      batch->uploadStaticBuffer(
        rt_node_buffer_.get(),
        quint32(size_t(terrain.first_node) * sizeof(BvhNode)),
        quint32(size_t(terrain.node_count) * sizeof(BvhNode)),
        pick_nodes_.data() + terrain.first_node);
      rt_instances_dirty_ = true; // the terrain's box
      rt_frame_ = 0;
    }
  }
}

void shift::gui::Viewport::ShapeRtTerrain(
  const RtDeformableTerrain& d, int i0, int j0, int i1, int j1,
  std::vector<RtTriangle>& triangles, std::vector<BvhNode>& nodes, int& lo,
  int& hi) const {
  const SceneTerrain& t = simulation_->get_scene().terrains[d.terrain];
  const int width = t.width, height = t.height;
  const int s = std::max(t.mesh_subdivisions, 1);
  const int squares_x = s * (width - 1);
  if(width < 2 || height < 2 ||
     d.position.size() != size_t(2 * squares_x * s * (height - 1))) {
    return;
  }
  i0 = std::max(i0, 0);
  j0 = std::max(j0, 0);
  i1 = std::min(i1, width - 2);
  j1 = std::min(j1, height - 2);
  if(i1 < i0 || j1 < j0) {
    return;
  }
  const bool smooth = smooth_terrain_;
  const QMatrix3x3 rotation = t.pose.normalMatrix();
  // control points beyond the map: those of its edge, as the shaders take
  // them (terrain_common.glsl, ControlZ without the B-spline)
  auto z = [&](int i, int j) {
    return t.z[size_t(std::clamp(j, 0, height - 1)) * size_t(width) +
               size_t(std::clamp(i, 0, width - 1))];
  };
  // the surface at control point coordinates (i + a, j + b) of cell (i, j),
  // a and b in [0, 1]: z and its slopes per control point spacing
  auto surface = [&](int i, int j, float a, float b, float& z_a, float& z_b) {
    if(!smooth) {
      const float z00 = z(i, j), z10 = z(i + 1, j);
      const float z01 = z(i, j + 1), z11 = z(i + 1, j + 1);
      z_a = (z10 - z00) * (1.f - b) + (z11 - z01) * b;
      z_b = (z01 - z00) * (1.f - a) + (z11 - z10) * a;
      return (z00 * (1.f - a) + z10 * a) * (1.f - b) +
             (z01 * (1.f - a) + z11 * a) * b;
    }
    // Catmull-Rom, as CatmullRomBasis in terrain_common.glsl
    auto basis = [](float x, float w[4], float dw[4]) {
      const float x2 = x * x, x3 = x2 * x;
      w[0] = 0.5f * (-x + 2.f * x2 - x3);
      w[1] = 0.5f * (2.f - 5.f * x2 + 3.f * x3);
      w[2] = 0.5f * (x + 4.f * x2 - 3.f * x3);
      w[3] = 0.5f * (-x2 + x3);
      dw[0] = 0.5f * (-1.f + 4.f * x - 3.f * x2);
      dw[1] = 0.5f * (-10.f * x + 9.f * x2);
      dw[2] = 0.5f * (1.f + 8.f * x - 9.f * x2);
      dw[3] = 0.5f * (-2.f * x + 3.f * x2);
    };
    float wa[4], dwa[4], wb[4], dwb[4];
    basis(a, wa, dwa);
    basis(b, wb, dwb);
    float value = 0.f;
    z_a = z_b = 0.f;
    for(int q = 0; q < 4; q++) {
      for(int p = 0; p < 4; p++) {
        const float c = z(i - 1 + p, j - 1 + q);
        value += wa[p] * wb[q] * c;
        z_a += dwa[p] * wb[q] * c;
        z_b += wa[p] * dwb[q] * c;
      }
    }
    return value;
  };
  // the vertex at control point coordinates (i + a, j + b)
  auto vertex = [&](int i, int j, float a, float b, float* position,
                    float* normal) {
    float z_a, z_b;
    const float h = surface(i, j, a, b, z_a, z_b);
    const QVector3D world = t.pose.map(QVector3D(
      t.x_scale * (float(i) + a), t.y_scale * (float(j) + b), h));
    float z_x = z_a / t.x_scale, z_y = z_b / t.y_scale;
    if(!smooth) {
      // facets would show each cell: the normals of the central differences
      // of the control points, interpolated
      auto central = [&](int ci, int cj, float& nx, float& ny) {
        const int il = std::max(ci - 1, 0), ir = std::min(ci + 1, width - 1);
        const int jl = std::max(cj - 1, 0), jr = std::min(cj + 1, height - 1);
        nx = (z(ir, cj) - z(il, cj)) / (t.x_scale * float(ir - il));
        ny = (z(ci, jr) - z(ci, jl)) / (t.y_scale * float(jr - jl));
      };
      float x00, y00, x10, y10, x01, y01, x11, y11;
      central(i, j, x00, y00);
      central(std::min(i + 1, width - 1), j, x10, y10);
      central(i, std::min(j + 1, height - 1), x01, y01);
      central(std::min(i + 1, width - 1), std::min(j + 1, height - 1), x11,
              y11);
      z_x = (x00 * (1.f - a) + x10 * a) * (1.f - b) +
            (x01 * (1.f - a) + x11 * a) * b;
      z_y = (y00 * (1.f - a) + y10 * a) * (1.f - b) +
            (y01 * (1.f - a) + y11 * a) * b;
    }
    const float length = std::sqrt(z_x * z_x + z_y * z_y + 1.f);
    const float n[3] = {z_x / length, z_y / length, -1.f / length};
    for(int c = 0; c < 3; c++) {
      position[c] = world[c];
      normal[c] = rotation(c, 0) * n[0] + rotation(c, 1) * n[1] +
                  rotation(c, 2) * n[2];
    }
  };
  const float step = 1.f / float(s);
  for(int j = j0; j <= j1; j++) {
    for(int i = i0; i <= i1; i++) {
      for(int v = 0; v < s; v++) {
        for(int u = 0; u < s; u++) {
          const int square = (j * s + v) * squares_x + i * s + u;
          // a, b, e and a, e, c as AddTerrain (simulation.cc) made them, in
          // fractions of the cell
          const float a0 = step * float(u), a1 = step * float(u + 1);
          const float b0 = step * float(v), b1 = step * float(v + 1);
          const float corners[2][3][2] = {{{a0, b0}, {a1, b0}, {a1, b1}},
                                          {{a0, b0}, {a1, b1}, {a0, b1}}};
          for(int half = 0; half < 2; half++) {
            const int k = d.position[size_t(2 * square + half)];
            RtTriangle& triangle = triangles[size_t(d.first_triangle + k)];
            float* positions[3] = {triangle.v0, triangle.v1, triangle.v2};
            float* normals[3] = {triangle.n0, triangle.n1, triangle.n2};
            for(int c = 0; c < 3; c++) {
              vertex(i, j, corners[half][c][0], corners[half][c][1],
                     positions[c], normals[c]);
            }
            lo = std::min(lo, k);
            hi = std::max(hi, k);
          }
        }
      }
    }
  }
  // refit: in depth first order a node's children come after it
  for(int n = d.first_node + d.node_count - 1; n >= d.first_node; n--) {
    BvhNode& node = nodes[size_t(n)];
    float box_min[3] = {1e30f, 1e30f, 1e30f};
    float box_max[3] = {-1e30f, -1e30f, -1e30f};
    auto grow = [&](const float* p) {
      for(int a = 0; a < 3; a++) {
        box_min[a] = std::min(box_min[a], p[a]);
        box_max[a] = std::max(box_max[a], p[a]);
      }
    };
    if(node.triangle_count >= 0) {
      for(int k = 0; k < node.triangle_count; k++) {
        const RtTriangle& triangle =
          triangles[size_t(node.right_or_first + k)];
        grow(triangle.v0);
        grow(triangle.v1);
        grow(triangle.v2);
      }
    } else {
      for(const BvhNode* child :
          {&nodes[size_t(n + 1)], &nodes[size_t(node.right_or_first)]}) {
        grow(child->box_min);
        grow(child->box_max);
      }
    }
    if(node.triangle_count != 0 || n != d.first_node) {
      std::copy(box_min, box_min + 3, node.box_min);
      std::copy(box_max, box_max + 3, node.box_max);
    }
  }
}

void shift::gui::Viewport::UploadScene(QRhiResourceUpdateBatch* batch) {
  scene_dirty_ = false;
  rt_terrains_.clear();
  gpu_meshes_.clear();
  rt_triangles_.clear();
  rt_triangle_faces_.clear();
  rt_nodes_.clear();
  rt_roots_.clear();
  rt_instance_list_.clear();
  pick_triangles_.clear();
  pick_triangle_faces_.clear();
  pick_nodes_.clear();
  pick_roots_.clear();
  pick_cad_trims_ = CadTrimTables();
  pick_built_ = false;
  if(!simulation_) {
    return;
  }
  const Scene& scene = simulation_->get_scene();

  for(const auto& mesh : scene.meshes) {
    GpuMesh gpu;
    const quint32 vertex_bytes = quint32(mesh.vertices.size() * sizeof(Vertex));
    const quint32 index_bytes = quint32(mesh.indices.size() * sizeof(uint32_t));
    gpu.vertices.reset(rhi_->newBuffer(QRhiBuffer::Immutable,
                                       QRhiBuffer::VertexBuffer, vertex_bytes));
    gpu.vertices->create();
    gpu.indices.reset(rhi_->newBuffer(QRhiBuffer::Immutable,
                                      QRhiBuffer::IndexBuffer, index_bytes));
    gpu.indices->create();
    batch->uploadStaticBuffer(gpu.vertices.get(), mesh.vertices.data());
    batch->uploadStaticBuffer(gpu.indices.get(), mesh.indices.data());
    gpu.index_count = quint32(mesh.indices.size());
    gpu_meshes_.push_back(std::move(gpu));
  }
  // the instances of a mesh are contiguous (Scene::instances)
  for(size_t i = 0; i < scene.instances.size(); i++) {
    GpuMesh& gpu = gpu_meshes_[scene.instances[i].mesh_index];
    if(gpu.instance_count == 0) {
      gpu.first_instance = quint32(i);
    }
    gpu.instance_count++;
  }

  // the particles of the SPH soils, uploaded with the updates
  gpu_particles_.clear();
  gpu_particles_reset_ = true;
  for(size_t p = 0; p < scene.particles.size(); p++) {
    GpuParticles gpu;
    gpu.particles_index = static_cast<int>(p);
    gpu_particles_.push_back(std::move(gpu));
  }

  // terrain control points, read exactly by the tessellation shaders
  gpu_terrains_.clear();
  if(tessellation_supported_) {
    for(size_t t = 0; t < scene.terrains.size(); t++) {
      const SceneTerrain& terrain = scene.terrains[t];
      GpuTerrain gpu;
      gpu.terrain_index = static_cast<int>(t);
      gpu.control_points.reset(rhi_->newTexture(
        QRhiTexture::R32F, QSize(terrain.width, terrain.height)));
      gpu.control_points->create();
      QRhiTextureSubresourceUploadDescription points(
        terrain.z.data(), quint32(terrain.z.size() * sizeof(float)));
      batch->uploadTexture(gpu.control_points.get(),
                           QRhiTextureUploadEntry(0, 0, points));
      gpu.uniforms.reset(rhi_->newBuffer(QRhiBuffer::Dynamic,
                                         QRhiBuffer::UniformBuffer,
                                         sizeof(TerrainUniforms)));
      gpu.uniforms->create();
      gpu.patches_x = std::max(1, (terrain.width - 1 + kTerrainPatchCells - 1) /
                                    kTerrainPatchCells);
      gpu.patches_y = std::max(
        1, (terrain.height - 1 + kTerrainPatchCells - 1) / kTerrainPatchCells);
      gpu_terrains_.push_back(std::move(gpu));
    }
  }

  // CAD geometry, evaluated by the tessellation shaders
  gpu_cads_.clear();
  if(tessellation_supported_) {
    auto storage = [&](const void* data, size_t bytes) {
      // a buffer may not be empty: at least one record of 16 bytes
      std::unique_ptr<QRhiBuffer> b(
        rhi_->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::StorageBuffer,
                        quint32(std::max<size_t>(bytes, 16))));
      b->create();
      if(bytes > 0) {
        batch->uploadStaticBuffer(b.get(), 0, quint32(bytes), data);
      }
      return b;
    };
    for(size_t c = 0; c < scene.cads.size(); c++) {
      const SceneCad& cad = scene.cads[c];
      if(cad.patches.empty() || cad.bodies.empty()) {
        continue;
      }
      GpuCad gpu;
      gpu.cad_index = static_cast<int>(c);
      gpu.faces = storage(cad.faces.data(), cad.faces.size() * sizeof(CadGpuFace));
      gpu.curves =
        storage(cad.curves.data(), cad.curves.size() * sizeof(CadGpuCurve));
      gpu.patches =
        storage(cad.patches.data(), cad.patches.size() * sizeof(CadGpuPatch));
      gpu.points = storage(cad.points.data(), cad.points.size() * sizeof(float));
      gpu.knots = storage(cad.knots.data(), cad.knots.size() * sizeof(float));
      gpu.segments =
        storage(cad.segments.data(), cad.segments.size() * sizeof(float));
      gpu.trim_index = storage(cad.trim_index.data(),
                               cad.trim_index.size() * sizeof(int32_t));
      gpu.instances.reset(rhi_->newBuffer(
        QRhiBuffer::Dynamic, QRhiBuffer::VertexBuffer,
        quint32(cad.bodies.size() * sizeof(InstanceAttributes))));
      gpu.instances->create();
      gpu_cads_.push_back(std::move(gpu));
    }
    cad_ubo_.reset(rhi_->newBuffer(QRhiBuffer::Dynamic,
                                   QRhiBuffer::UniformBuffer,
                                   sizeof(CadUniforms)));
    cad_ubo_->create();
  }

  // texture layers; without any, a placeholder layer keeps the bindings valid
  if(scene.texture_layers.empty()) {
    CreateTextureArray(1, 1);
  } else {
    CreateTextureArray(static_cast<int>(scene.texture_layers.size()),
                       Scene::kTextureSize);
    QVector<QRhiTextureUploadEntry> entries;
    for(size_t i = 0; i < scene.texture_layers.size(); i++) {
      entries.append(QRhiTextureUploadEntry(
        int(i), 0,
        QRhiTextureSubresourceUploadDescription(scene.texture_layers[i])));
    }
    QRhiTextureUploadDescription description;
    description.setEntries(entries.cbegin(), entries.cend());
    batch->uploadTexture(texture_array_.get(), description);
    batch->generateMips(texture_array_.get());
  }

  // IES distributions, one block of rows each
  if(!scene.ies_profiles.empty()) {
    std::vector<float> values;
    for(const auto& profile : scene.ies_profiles) {
      values.insert(values.end(), profile.begin(), profile.end());
    }
    ies_texture_.reset(rhi_->newTexture(
      QRhiTexture::R16F,
      QSize(Scene::kIesVertical,
            Scene::kIesHorizontal * int(scene.ies_profiles.size()))));
    ies_texture_->create();
    batch->uploadTexture(ies_texture_.get(),
                         QRhiTextureUploadDescription(
                           {0, 0, QRhiTextureSubresourceUploadDescription(
                                    ToHalf(values))}));
  }

  // materials of the meshes, then of the CAD geometry
  const quint32 mesh_count = quint32(
    std::max<size_t>(scene.meshes.size() + scene.cads.size(), 1));
  material_ubo_.reset(rhi_->newBuffer(QRhiBuffer::Dynamic,
                                      QRhiBuffer::UniformBuffer,
                                      material_stride_ * mesh_count));
  material_ubo_->create();
  const quint32 instance_count =
    quint32(std::max<size_t>(scene.instances.size(), 1));
  instance_buffer_.reset(
    rhi_->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::VertexBuffer,
                    instance_count * sizeof(InstanceAttributes)));
  instance_buffer_->create();

  rt_instance_list_ = RtInstanceList(scene);
  if(ray_tracing_supported_) {
    CadTrimTables trims = MergeCadTrims(scene.cads);
    BuildMeshBvhs(scene, cad_triangles_, cad_triangle_faces_, trims,
                  rt_triangles_, rt_triangle_faces_, rt_nodes_, rt_roots_,
                  &rt_terrains_);
    // the deformable terrains as shown, not as the meshes were made
    rt_terrain_smooth_ = smooth_terrain_;
    for(const auto& terrain : rt_terrains_) {
      int lo = 0, hi = -1;
      ShapeRtTerrain(terrain, 0, 0, INT32_MAX, INT32_MAX, rt_triangles_,
                     rt_nodes_, lo, hi);
    }
    if(rt_triangles_.empty()) {
      rt_triangles_.resize(1);
      rt_triangle_faces_.assign(1, -1);
      rt_nodes_.resize(1);
    }
    // the faces of the CAD triangles and their trims; storage buffers may
    // not be empty
    auto storage = [&](std::unique_ptr<QRhiBuffer>& buffer, const void* data,
                       size_t bytes) {
      buffer.reset(rhi_->newBuffer(QRhiBuffer::Immutable,
                                   QRhiBuffer::StorageBuffer,
                                   quint32(std::max<size_t>(bytes, 16))));
      buffer->create();
      if(bytes > 0) {
        batch->uploadStaticBuffer(buffer.get(), 0, quint32(bytes), data);
      }
    };
    storage(rt_triangle_face_buffer_, rt_triangle_faces_.data(),
            rt_triangle_faces_.size() * sizeof(int32_t));
    storage(rt_cad_face_buffer_, trims.faces.data(),
            trims.faces.size() * sizeof(CadGpuFace));
    storage(rt_cad_segment_buffer_, trims.segments.data(),
            trims.segments.size() * sizeof(float));
    storage(rt_cad_trim_index_buffer_, trims.trim_index.data(),
            trims.trim_index.size() * sizeof(int32_t));

    // static: the deformable terrains' ranges are uploaded again as they
    // change
    rt_triangle_buffer_.reset(rhi_->newBuffer(
      QRhiBuffer::Static, QRhiBuffer::StorageBuffer,
      quint32(rt_triangles_.size() * sizeof(RtTriangle))));
    rt_triangle_buffer_->create();
    rt_node_buffer_.reset(
      rhi_->newBuffer(QRhiBuffer::Static, QRhiBuffer::StorageBuffer,
                      quint32(rt_nodes_.size() * sizeof(BvhNode))));
    rt_node_buffer_->create();
    const quint32 rt_instance_count =
      quint32(std::max<size_t>(rt_instance_list_.size(), 1));
    rt_instance_buffer_.reset(
      rhi_->newBuffer(QRhiBuffer::Static, QRhiBuffer::StorageBuffer,
                      quint32(rt_instance_count * sizeof(RtInstance))));
    rt_instance_buffer_->create();
    // one leaf per instance: 2 n - 1 nodes
    rt_instance_node_buffer_.reset(rhi_->newBuffer(
      QRhiBuffer::Static, QRhiBuffer::StorageBuffer,
      quint32((2 * rt_instance_count - 1) * sizeof(BvhNode))));
    rt_instance_node_buffer_->create();
    batch->uploadStaticBuffer(rt_triangle_buffer_.get(), rt_triangles_.data());
    batch->uploadStaticBuffer(rt_node_buffer_.get(), rt_nodes_.data());
    // the GPU copies are all that ray tracing needs from here on; picking
    // keeps the CPU copies
    pick_triangles_ = std::move(rt_triangles_);
    pick_triangle_faces_ = std::move(rt_triangle_faces_);
    pick_nodes_ = std::move(rt_nodes_);
    pick_roots_ = rt_roots_;
    pick_cad_trims_ = std::move(trims);
    pick_built_ = true;
    rt_triangles_ = std::vector<RtTriangle>();
    rt_triangle_faces_ = std::vector<int32_t>();
    rt_nodes_ = std::vector<BvhNode>();
  }

  CreatePipelines();
  rt_instances_dirty_ = true;
  rt_frame_ = 0;
  rt_history_valid_ = false;
  rt_previous_poses_.clear();
}

void shift::gui::Viewport::render(QRhiCommandBuffer* cb) {
  frame_count_++;
  UpdatePoses();

  QRhiResourceUpdateBatch* batch = rhi_->nextResourceUpdateBatch();
  if(environment_dirty_) {
    UploadEnvironment(batch);
  }
  if(scene_dirty_) {
    UploadScene(batch);
    // the scene holds the terrains as shown
    if(simulation_) {
      simulation_->TakeTerrainUpdates(terrain_updates_);
    }
  } else if(simulation_) {
    UploadTerrainUpdates(batch);
  }
  if(simulation_) {
    // all of them after the scene was uploaded again
    UploadParticleUpdates(batch, gpu_particles_reset_);
    gpu_particles_reset_ = false;
  }

  if(render_mode_ == RenderMode::kRayTraced && ray_tracing_supported_) {
    RenderRayTraced(cb, batch);
  } else {
    RenderRaster(cb, batch);
  }

  // the workspace requests frames when poses change; keep rendering while
  // ray traced samples are accumulating. Instances are uploaded by the ray
  // traced path only, so they stay dirty while rendering raster.
  const bool accumulating = render_mode_ == RenderMode::kRayTraced &&
                            ray_tracing_supported_ && rt_frame_ < kRtMaxFrames;
  if(accumulating) {
    update();
  }
}

bool shift::gui::Viewport::UpdateLights(QRhiResourceUpdateBatch* batch) {
  LightUniforms lights{};
  int count = 0;
  if(simulation_ && !poses_.bodies.empty()) {
    count = FillLights(simulation_->get_scene(), poses_, lights);
  }
  batch->updateDynamicBuffer(lights_ubo_.get(), 0, sizeof(lights), &lights);
  return count > 0;
}

void shift::gui::Viewport::RenderRaster(QRhiCommandBuffer* cb,
                                        QRhiResourceUpdateBatch* batch) {
  const QSize size = get_render_target()->pixelSize();
  const float aspect = float(size.width()) / float(std::max(size.height(), 1));
  const QMatrix4x4 view_projection = rhi_->clipSpaceCorrMatrix() *
                                     camera_.get_projection_matrix(aspect) *
                                     camera_.get_view_matrix();

  // the lights of the model replace the sun
  const bool sun = !UpdateLights(batch);
  FrameUniforms frame{};
  CopyMatrix(view_projection, frame.view_projection);
  CopyMatrix(view_projection.inverted(), frame.inverse_view_projection);
  CopyVector(camera_.get_position(), 1.f, frame.camera_position);
  CopyVector(sun_direction_, sun ? kSunIntensity : 0.f, frame.sun_direction);
  CopyArray(sun ? kSunColor : kNoSun, frame.sun_color);
  CopyArray(kSkyZenith, frame.sky_zenith);
  CopyArray(kSkyHorizon, frame.sky_horizon);
  const float floor_params[4] = {0.f, floor_spacing_,
                                 floor_enabled_ ? 1.f : 0.f, floor_extent_};
  CopyArray(floor_params, frame.floor_params);
  CopyFloorFrame(frame.floor_frame);
  batch->updateDynamicBuffer(frame_ubo_.get(), 0, sizeof(frame), &frame);

  const bool have_poses = simulation_ && !poses_.bodies.empty();
  if(have_poses) {
    const Scene& scene = simulation_->get_scene();
    for(size_t i = 0; i < scene.meshes.size(); i++) {
      const SceneMesh& mesh = scene.meshes[i];
      MaterialUniforms material{};
      CopyVector(mesh.albedo, mesh.normal_scale, material.albedo);
      CopyMaterial(mesh, material.material, material.layers);
      CopySurfaces(mesh, material.variation, material.surface_layers,
                   material.surface);
      CopyEmission(scene, mesh, poses_, material.emission);
      const int holes = std::min<int>(int(mesh.holes.size()), 4);
      material.layers[3] = holes;
      for(int k = 0; k < holes; k++) {
        CopyVector(mesh.holes[k].toVector3D(), mesh.holes[k].w(),
                   material.holes[k]);
      }
      batch->updateDynamicBuffer(material_ubo_.get(),
                                 quint32(i) * material_stride_,
                                 sizeof(material), &material);
    }
    std::vector<InstanceAttributes> instances(scene.instances.size());
    for(size_t i = 0; i < instances.size(); i++) {
      // a hidden body's meshes collapse to a point, and draw nothing
      const int body = scene.instances[i].body_index;
      if(is_body_shown(body)) {
        CopyMatrix(poses_.bodies[body], instances[i].model);
      } else {
        std::fill(instances[i].model, instances[i].model + 16, 0.f);
      }
    }
    if(!instances.empty()) {
      batch->updateDynamicBuffer(
        instance_buffer_.get(), 0,
        quint32(instances.size() * sizeof(InstanceAttributes)),
        instances.data());
    }
  }

  // CAD geometry: materials, the poses of its bodies, tessellation density
  const bool draw_cad = cad_pipeline_ && have_poses;
  if(draw_cad) {
    const Scene& scene = simulation_->get_scene();
    for(const auto& gpu : gpu_cads_) {
      const SceneCad& cad = scene.cads[gpu.cad_index];
      MaterialUniforms material{};
      CopyVector(cad.albedo, 1.f, material.albedo);
      material.material[0] = cad.metallic;
      material.material[1] = cad.roughness;
      material.material[3] = 1.f;
      material.layers[0] = material.layers[1] = material.layers[2] = -1;
      material.layers[3] = 0; // no holes
      CopySurfaces(SceneMesh(), material.variation, material.surface_layers,
                   material.surface);
      batch->updateDynamicBuffer(
        material_ubo_.get(),
        quint32(scene.meshes.size() + gpu.cad_index) * material_stride_,
        sizeof(material), &material);
      std::vector<InstanceAttributes> instances(cad.bodies.size());
      for(size_t i = 0; i < instances.size(); i++) {
        QMatrix4x4 pose = poses_.bodies[cad.bodies[i]];
        pose.scale(is_body_shown(cad.bodies[i]) ? cad.scale : 0.f);
        CopyMatrix(pose, instances[i].model);
      }
      batch->updateDynamicBuffer(
        gpu.instances.get(), 0,
        quint32(instances.size() * sizeof(InstanceAttributes)),
        instances.data());
    }
    const QMatrix4x4 projection = camera_.get_projection_matrix(aspect);
    CadUniforms u{};
    u.tessellation[0] =
      0.5f * float(size.height()) * std::fabs(projection(1, 1));
    u.tessellation[1] = kCadPixelsPerEdge;
    u.tessellation[2] = kCadMaximumLevel;
    u.tessellation[3] = kCadSagittaPixels;
    u.edges[0] = kCadEdgeHalfWidth;
    u.edges[1] = kCadEdgeDarkening;
    u.edges[2] = kCadTrimWidening;
    batch->updateDynamicBuffer(cad_ubo_.get(), 0, sizeof(u), &u);
  }

  // terrains: pose, size and the tessellation density for this view
  const bool tessellate = terrain_pipeline_ && have_poses;
  if(tessellate) {
    const Scene& scene = simulation_->get_scene();
    const QMatrix4x4 projection = camera_.get_projection_matrix(aspect);
    // pixels per unit angle at the center of the view
    const float pixels_per_angle =
      0.5f * float(size.height()) * std::fabs(projection(1, 1));
    for(const auto& gpu : gpu_terrains_) {
      const SceneTerrain& terrain = scene.terrains[gpu.terrain_index];
      TerrainUniforms u{};
      CopyMatrix(terrain.pose, u.pose);
      u.extent[0] = terrain.x_scale * float(terrain.width - 1);
      u.extent[1] = terrain.y_scale * float(terrain.height - 1);
      u.extent[2] = float(gpu.patches_x);
      u.extent[3] = float(gpu.patches_y);
      u.tessellation[0] = pixels_per_angle;
      u.tessellation[1] = kTerrainPixelsPerEdge;
      u.tessellation[2] = kTerrainMaximumLevel;
      u.scale[0] = terrain.x_scale;
      u.scale[1] = terrain.y_scale;
      u.samples[0] = terrain.width;
      u.samples[1] = terrain.height;
      u.samples[2] = terrain.bicubic                            ? 1
                     : terrain.deformable && smooth_terrain_ ? 2
                                                             : 0;
      batch->updateDynamicBuffer(gpu.uniforms.get(), 0, sizeof(u), &u);
    }
  }

  batch = UploadFrameAxes(batch, view_projection, false);
  // the surface over the particles: its depth before the render pass
  const bool particle_surface =
    have_poses && particle_surface_ && fluid_pipeline_ &&
    RenderFluidDepth(cb, batch, view_projection, aspect);

  cb->beginPass(get_render_target(), QColor(0, 0, 0), {1.0f, 0}, batch);
  const QRhiViewport viewport(0, 0, float(size.width()), float(size.height()));
  const QRhiCommandBuffer::DynamicOffset zero_offset(1, 0);

  cb->setGraphicsPipeline(sky_pipeline_.get());
  cb->setViewport(viewport);
  cb->setShaderResources(raster_bindings_.get(), 1, &zero_offset);
  cb->draw(6);

  if(have_poses) {
    cb->setGraphicsPipeline(mesh_pipeline_.get());
    cb->setViewport(viewport);
    // one instanced draw per mesh
    const Scene& scene = simulation_->get_scene();
    for(size_t i = 0; i < gpu_meshes_.size(); i++) {
      const GpuMesh& gpu = gpu_meshes_[i];
      if(gpu.instance_count == 0 ||
         (tessellate && scene.meshes[i].terrain >= 0)) {
        continue; // terrains are tessellated below
      }
      const QRhiCommandBuffer::DynamicOffset offset(
        1, quint32(i) * material_stride_);
      cb->setShaderResources(raster_bindings_.get(), 1, &offset);
      const QRhiCommandBuffer::VertexInput inputs[2] = {
        {gpu.vertices.get(), 0},
        {instance_buffer_.get(),
         gpu.first_instance * quint32(sizeof(InstanceAttributes))}};
      cb->setVertexInput(0, 2, inputs, gpu.indices.get(), 0,
                         QRhiCommandBuffer::IndexUInt32);
      cb->drawIndexed(gpu.index_count, gpu.instance_count);
    }
  }

  if(particle_surface) {
    // with the material of the first particles' sphere
    const Scene& scene = simulation_->get_scene();
    const int mesh_index = scene.particles[0].mesh_index;
    const QRhiCommandBuffer::DynamicOffset offset(
      1, quint32(mesh_index) * material_stride_);
    cb->setGraphicsPipeline(fluid_pipeline_.get());
    cb->setViewport(viewport);
    cb->setShaderResources(fluid_bindings_.get(), 1, &offset);
    cb->draw(6);
  } else if(have_poses && particle_pipeline_) {
    const Scene& scene = simulation_->get_scene();
    bool bound = false;
    for(const auto& gpu : gpu_particles_) {
      if(gpu.count == 0 || !gpu.centers) {
        continue;
      }
      if(!bound) {
        cb->setGraphicsPipeline(particle_pipeline_.get());
        cb->setViewport(viewport);
        bound = true;
      }
      const int mesh_index = scene.particles[gpu.particles_index].mesh_index;
      const GpuMesh& mesh = gpu_meshes_[size_t(mesh_index)];
      const QRhiCommandBuffer::DynamicOffset offset(
        1, quint32(mesh_index) * material_stride_);
      cb->setShaderResources(raster_bindings_.get(), 1, &offset);
      const QRhiCommandBuffer::VertexInput inputs[2] = {
        {mesh.vertices.get(), 0}, {gpu.centers.get(), 0}};
      cb->setVertexInput(0, 2, inputs, mesh.indices.get(), 0,
                         QRhiCommandBuffer::IndexUInt32);
      cb->drawIndexed(mesh.index_count, gpu.count);
    }
  }

  if(tessellate) {
    const Scene& scene = simulation_->get_scene();
    cb->setGraphicsPipeline(terrain_pipeline_.get());
    cb->setViewport(viewport);
    for(const auto& gpu : gpu_terrains_) {
      const int mesh_index = scene.terrains[gpu.terrain_index].mesh_index;
      bool shown = false; // as the instances of its mesh
      for(const SceneInstance& instance : scene.instances) {
        shown = shown || (instance.mesh_index == mesh_index &&
                          is_body_shown(instance.body_index));
      }
      if(!shown) {
        continue;
      }
      const QRhiCommandBuffer::DynamicOffset offset(1, quint32(mesh_index) *
                                                         material_stride_);
      cb->setShaderResources(gpu.bindings.get(), 1, &offset);
      cb->draw(4, quint32(gpu.patches_x * gpu.patches_y));
    }
  }

  if(draw_cad) {
    const Scene& scene = simulation_->get_scene();
    cb->setGraphicsPipeline(cad_pipeline_.get());
    cb->setViewport(viewport);
    for(const auto& gpu : gpu_cads_) {
      const SceneCad& cad = scene.cads[gpu.cad_index];
      const QRhiCommandBuffer::DynamicOffset offset(
        1, quint32(scene.meshes.size() + gpu.cad_index) * material_stride_);
      cb->setShaderResources(gpu.bindings.get(), 1, &offset);
      const QRhiCommandBuffer::VertexInput input(gpu.instances.get(), 0);
      cb->setVertexInput(0, 1, &input);
      cb->draw(quint32(4 * cad.patches.size()), quint32(cad.bodies.size()));
    }
  }

  if(floor_enabled_ && have_poses) {
    cb->setGraphicsPipeline(floor_pipeline_.get());
    cb->setViewport(viewport);
    cb->setShaderResources(raster_bindings_.get(), 1, &zero_offset);
    cb->draw(6);
  }
  DrawFrameAxes(cb, viewport);
  cb->endPass();
}

bool shift::gui::Viewport::RenderFluidDepth(
  QRhiCommandBuffer* cb, QRhiResourceUpdateBatch*& batch,
  const QMatrix4x4& view_projection, float aspect) {
  const Scene& scene = simulation_->get_scene();
  float radius = 0.f;
  bool any = false;
  for(const auto& gpu : gpu_particles_) {
    if(gpu.count > 0 && gpu.centers) {
      any = true;
      radius = std::max(radius, scene.particles[gpu.particles_index].radius);
    }
  }
  if(!any || !fluid_depth_[0] ||
     fluid_depth_[0]->pixelSize() != output_size_) {
    return false;
  }
  const QSize size = output_size_;
  const QMatrix4x4 projection = camera_.get_projection_matrix(aspect);
  FluidUniforms u{};
  CopyMatrix(view_projection, u.view_projection);
  CopyMatrix(view_projection.inverted(), u.inverse_view_projection);
  CopyVector(camera_.get_position(), rhi_->isClipDepthZeroToOne() ? 1.f : 0.f,
             u.camera_position);
  CopyVector(camera_.get_forward().normalized(), 0.f, u.camera_forward);
  u.particles[0] = radius;
  u.particles[1] = 0.5f * float(size.height()) * std::fabs(projection(1, 1));
  u.filter_params[0] = kFluidBlurRadius * radius;
  u.filter_params[1] = kFluidBlurRange * radius;
  u.filter_params[2] = kFluidBlurMaxPixels;
  batch->updateDynamicBuffer(fluid_ubo_.get(), 0, sizeof(u), &u);
  for(int k = 0; k < kFluidBlurLevels; k++) {
    const float level[4] = {float(kFluidBlurStrides[k]), 0.f, 0.f, 0.f};
    batch->updateDynamicBuffer(fluid_level_ubo_.get(),
                               quint32(k) * fluid_level_stride_,
                               sizeof(level), level);
  }

  const QRhiViewport viewport(0, 0, float(size.width()), float(size.height()));
  cb->beginPass(fluid_sphere_target_.get(), QColor(0, 0, 0, 0), {1.0f, 0},
                batch);
  batch = nullptr; // submitted with the first pass
  cb->setGraphicsPipeline(fluid_sphere_pipeline_.get());
  cb->setViewport(viewport);
  cb->setShaderResources(fluid_sphere_bindings_.get());
  for(const auto& gpu : gpu_particles_) {
    if(gpu.count == 0 || !gpu.centers) {
      continue;
    }
    const QRhiCommandBuffer::VertexInput input(gpu.centers.get(), 0);
    cb->setVertexInput(0, 1, &input);
    cb->draw(6, gpu.count);
  }
  cb->endPass();

  for(int level = 0; level < kFluidBlurLevels; level++) {
    // from [0] into [1], and back
    const int k = level % 2;
    const QRhiCommandBuffer::DynamicOffset offset(
      2, quint32(level) * fluid_level_stride_);
    cb->beginPass(fluid_blur_target_[1 - k].get(), QColor(0, 0, 0, 0),
                  {1.0f, 0});
    cb->setGraphicsPipeline(fluid_blur_pipeline_[k].get());
    cb->setViewport(viewport);
    cb->setShaderResources(fluid_blur_bindings_[k].get(), 1, &offset);
    cb->draw(6);
    cb->endPass();
  }
  return true;
}

bool shift::gui::Viewport::UploadInstances(QRhiResourceUpdateBatch* batch,
                                           int instance_count) {
  const Scene& scene = simulation_->get_scene();
  // without history (new scene, first frame) the previous pose is the
  // current one
  if(!rt_history_valid_ ||
     static_cast<int>(rt_previous_poses_.size()) != instance_count) {
    rt_previous_poses_.clear();
  }
  bool moved = false;
  std::vector<RtInstance> instances(std::max(instance_count, 1));
  const int mesh_count = static_cast<int>(scene.meshes.size());
  for(int i = 0; i < instance_count; i++) {
    const SceneInstance& instance = rt_instance_list_[i];
    const QMatrix4x4& pose = poses_.bodies[instance.body_index];
    const QMatrix4x4& previous =
      rt_previous_poses_.empty() ? pose : rt_previous_poses_[i];
    moved = moved || previous != pose;
    CopyMatrix(pose, instances[i].body_to_world);
    CopyMatrix(pose.inverted(), instances[i].world_to_body);
    CopyMatrix(previous, instances[i].previous_body_to_world);
    if(instance.mesh_index < mesh_count) {
      const SceneMesh& mesh = scene.meshes[instance.mesh_index];
      QVector3D bounds_min = mesh.bounds_min, bounds_max = mesh.bounds_max;
      for(const auto& terrain : rt_terrains_) {
        // a deformable terrain's box is that of its refitted BVH
        if(terrain.mesh == instance.mesh_index &&
           terrain.first_node < static_cast<int>(pick_nodes_.size())) {
          const BvhNode& root = pick_nodes_[terrain.first_node];
          bounds_min = QVector3D(root.box_min[0], root.box_min[1],
                                 root.box_min[2]);
          bounds_max = QVector3D(root.box_max[0], root.box_max[1],
                                 root.box_max[2]);
        }
      }
      TransformBox(pose, bounds_min, bounds_max, instances[i].box_min,
                   instances[i].box_max);
      CopyVector(mesh.albedo, mesh.normal_scale, instances[i].albedo);
      CopyMaterial(mesh, instances[i].material, instances[i].offsets + 1);
      instances[i].optical[0] = mesh.opacity;
      CopyEmission(scene, mesh, poses_, instances[i].optical + 1);
      CopySurfaces(mesh, instances[i].variation, instances[i].surface_layers,
                   instances[i].surface);
    } else {
      // CAD geometry: its triangles are scaled, its faces may have colors
      const SceneCad& cad = scene.cads[instance.mesh_index - mesh_count];
      TransformBox(pose, cad.bounds_min, cad.bounds_max, instances[i].box_min,
                   instances[i].box_max);
      CopyVector(cad.albedo, 1.f, instances[i].albedo);
      instances[i].material[0] = cad.metallic;
      instances[i].material[1] = cad.roughness;
      instances[i].material[2] = 0.f;
      instances[i].material[3] = 1.f;
      instances[i].optical[0] = cad.opacity;
      instances[i].offsets[1] = instances[i].offsets[2] =
        instances[i].offsets[3] = -1;
      CopySurfaces(SceneMesh(), instances[i].variation,
                   instances[i].surface_layers, instances[i].surface);
    }
    instances[i].offsets[0] = rt_roots_[instance.mesh_index];
    if(!is_body_shown(instance.body_index)) {
      // far beyond anything a ray reaches: its triangles are never tested
      for(int k = 0; k < 3; k++) {
        instances[i].box_min[k] = instances[i].box_max[k] = 1e30f;
      }
    }
  }
  batch->uploadStaticBuffer(rt_instance_buffer_.get(), 0,
                            quint32(instances.size() * sizeof(RtInstance)),
                            instances.data());
  // the BVH over the instances' world boxes; the shader skips it without
  // instances
  std::vector<float> box_min(4 * instance_count), box_max(4 * instance_count);
  for(int i = 0; i < instance_count; i++) {
    std::copy(instances[i].box_min, instances[i].box_min + 4, &box_min[4 * i]);
    std::copy(instances[i].box_max, instances[i].box_max + 4, &box_max[4 * i]);
  }
  const std::vector<BvhNode> instance_nodes = BuildBoxBvh(box_min, box_max);
  if(!instance_nodes.empty()) {
    batch->uploadStaticBuffer(
      rt_instance_node_buffer_.get(), 0,
      quint32(instance_nodes.size() * sizeof(BvhNode)), instance_nodes.data());
  }
  rt_previous_poses_.resize(instance_count);
  for(int i = 0; i < instance_count; i++) {
    rt_previous_poses_[i] = poses_.bodies[rt_instance_list_[i].body_index];
  }
  return moved;
}

void shift::gui::Viewport::RenderRayTraced(QRhiCommandBuffer* cb,
                                           QRhiResourceUpdateBatch* batch) {
  const QSize size = get_render_target()->pixelSize();
  const float aspect = float(size.width()) / float(std::max(size.height(), 1));
  const QMatrix4x4 view = camera_.get_view_matrix();
  const QMatrix4x4 projection =
    rhi_->clipSpaceCorrMatrix() * camera_.get_projection_matrix(aspect);
  const QMatrix4x4 view_projection = projection * view;

  // restart the progressive mean whenever anything in view changes; the
  // denoiser's history survives through reprojection
  const bool camera_moved = view_projection != rt_last_view_projection_;
  if(camera_moved) {
    rt_last_view_projection_ = view_projection;
    rt_frame_ = 0;
  }
  if(!rt_history_valid_) {
    rt_previous_view_projection_ = view_projection;
  }

  const bool have_poses = simulation_ && !poses_.bodies.empty();
  int instance_count = 0;
  bool bodies_moved = false;
  if(have_poses) {
    instance_count = static_cast<int>(rt_instance_list_.size());
    // a light switched on or off changes the glow of its meshes, which the
    // instances hold, and what the progressive mean converges to
    if(poses_.lights != rt_light_values_) {
      rt_light_values_ = poses_.lights;
      rt_instances_dirty_ = true;
      rt_frame_ = 0;
    }
    // after a frame with motion the instances still hold the old previous
    // poses, so upload once more with previous = current
    if(rt_instances_dirty_ || rt_instances_moving_) {
      bodies_moved = UploadInstances(batch, instance_count);
      rt_instances_moving_ = bodies_moved;
      rt_instances_dirty_ = false;
      if(bodies_moved) {
        rt_frame_ = 0;
      }
    }
  }

  if(rt_frame_ < kRtMaxFrames) {
    RtUniforms uniforms{};
    CopyMatrix(view.inverted(), uniforms.inverse_view);
    CopyMatrix(projection.inverted(), uniforms.inverse_projection);
    CopyVector(camera_.get_position(), 1.f, uniforms.camera_position);
    const bool sun = !UpdateLights(batch);
    CopyVector(sun_direction_, sun ? kSunIntensity : 0.f,
               uniforms.sun_direction);
    CopyArray(sun ? kSunColor : kNoSun, uniforms.sun_color);
    CopyArray(kSkyZenith, uniforms.sky_zenith);
    CopyArray(kSkyHorizon, uniforms.sky_horizon);
    const float floor_params[4] = {0.f, floor_spacing_,
                                   (floor_enabled_ && have_poses) ? 1.f : 0.f,
                                   floor_extent_};
    CopyArray(floor_params, uniforms.floor_params);
    CopyFloorFrame(uniforms.floor_frame);
    // angle subtended by a pixel, to select texture levels of detail and to
    // scale the denoiser's depth tolerances
    const float spread =
      2.f * std::tan(0.5f * camera_.get_fov_y() * float(M_PI) / 180.f) /
      float(std::max(size.height(), 1));
    const float texture_params[4] = {
      spread, float(texture_array_->pixelSize().width()), 0.f, 0.f};
    CopyArray(texture_params, uniforms.texture_params);
    uniforms.control[0] = instance_count;
    uniforms.control[1] = rt_sample_;
    uniforms.control[2] = rt_frame_;
    batch->updateDynamicBuffer(rt_ubo_.get(), 0, sizeof(uniforms), &uniforms);

    DenoiseUniforms denoise{};
    CopyMatrix(rt_previous_view_projection_, denoise.previous_view_projection);
    CopyMatrix(view_projection, denoise.view_projection);
    denoise.params[0] = spread;
    denoise.params[1] = bodies_moved   ? kHistoryMoving
                        : camera_moved ? kHistoryCamera
                                       : kHistoryStatic;
    denoise.params[2] = rt_history_valid_ ? 1.f : 0.f;
    denoise.blend[0] = float(rt_frame_ + 1);
    denoise.blend[1] = kBlendStart;
    denoise.blend[2] = kBlendEnd;
    denoise.control[0] = instance_count;
    for(int i = 0; i <= kAtrousIterations; i++) {
      denoise.params[3] = i == 0 ? 0.f : float(1 << (i - 1));
      batch->updateDynamicBuffer(svgf_ubo_.get(), quint32(i) * svgf_ubo_stride_,
                                 sizeof(denoise), &denoise);
    }

    // separate passes so that QRhi orders each pass's image reads after the
    // previous pass's writes
    const int groups_x = (size.width() + 7) / 8;
    const int groups_y = (size.height() + 7) / 8;
    auto dispatch = [&](QRhiComputePipeline* pipeline,
                        QRhiShaderResourceBindings* bindings,
                        const QRhiCommandBuffer::DynamicOffset* offset) {
      cb->beginComputePass(batch);
      batch = nullptr;
      cb->setComputePipeline(pipeline);
      cb->setShaderResources(bindings, offset ? 1 : 0, offset);
      cb->dispatch(groups_x, groups_y, 1);
      cb->endComputePass();
    };
    const int p = rt_parity_;
    dispatch(rt_pipeline_.get(), rt_bindings_[p].get(), nullptr);
    dispatch(svgf_temporal_pipeline_.get(), svgf_temporal_bindings_[p].get(),
             nullptr);
    dispatch(svgf_variance_pipeline_.get(), svgf_variance_bindings_[p].get(),
             nullptr);
    for(int i = 0; i < kAtrousIterations; i++) {
      const QRhiCommandBuffer::DynamicOffset offset(
        0, quint32(i + 1) * svgf_ubo_stride_);
      dispatch(svgf_atrous_pipeline_.get(),
               svgf_atrous_bindings_[p][i % 2].get(), &offset);
    }
    dispatch(svgf_composite_pipeline_.get(),
             svgf_composite_bindings_[p].get(), nullptr);
    dispatch(svgf_taa_pipeline_.get(), svgf_taa_bindings_[p].get(), nullptr);

    rt_parity_ = 1 - p;
    rt_sample_++;
    rt_frame_++;
    rt_history_valid_ = true;
    rt_previous_view_projection_ = view_projection;
  }

  batch = UploadFrameAxes(batch, view_projection, true);
  cb->beginPass(get_render_target(), QColor(0, 0, 0), {1.0f, 0}, batch);
  const QRhiViewport viewport(0, 0, float(size.width()), float(size.height()));
  cb->setGraphicsPipeline(blit_pipeline_.get());
  cb->setViewport(viewport);
  cb->setShaderResources(blit_bindings_.get());
  cb->draw(6);
  DrawFrameAxes(cb, viewport);
  cb->endPass();
}

QRhiResourceUpdateBatch* shift::gui::Viewport::UploadFrameAxes(
  QRhiResourceUpdateBatch* batch, const QMatrix4x4& view_projection,
  bool frame_uniforms) {
  axes_instance_count_ = 0;
  axes_frame_instance_count_ = 0;
  if((frame_axes_.empty() && joint_axes_.empty()) || !simulation_ ||
     poses_.bodies.empty() || !axes_pipeline_) {
    return batch;
  }
  if(!batch) {
    batch = rhi_->nextResourceUpdateBatch();
  }
  if(!axes_mesh_uploaded_) {
    std::vector<AxesVertex> vertices;
    std::vector<uint32_t> indices;
    MakeAxesMesh(vertices, indices, axes_arrow_index_count_);
    batch->uploadStaticBuffer(axes_vertices_.get(), vertices.data());
    batch->uploadStaticBuffer(axes_indices_.get(), indices.data());
    axes_mesh_uploaded_ = true;
  }
  if(frame_uniforms) {
    // ray traced, the raster pass did not set the camera: only the fields
    // the arrows read, view_projection to camera_position
    FrameUniforms frame{};
    CopyMatrix(view_projection, frame.view_projection);
    CopyMatrix(view_projection.inverted(), frame.inverse_view_projection);
    CopyVector(camera_.get_position(), 1.f, frame.camera_position);
    batch->updateDynamicBuffer(frame_ubo_.get(), 0,
                               offsetof(FrameUniforms, sun_direction), &frame);
  }
  std::vector<AxesInstance> instances;
  instances.reserve(frame_axes_.size() + joint_axes_.size());
  // radius across the z axis (x, y), scale along it
  auto add = [&](int body, const QMatrix4x4& frame, const QVector3D& tint,
                 float radius) {
    if(body < 0 || body >= static_cast<int>(poses_.bodies.size())) {
      return;
    }
    QMatrix4x4 pose = poses_.bodies[body] * frame;
    pose.scale(radius, radius, frame_axes_scale_);
    instances.emplace_back();
    CopyMatrix(pose, instances.back().model);
    CopyVector(tint, 1.f, instances.back().tint);
  };
  for(const FrameAxes& axes : frame_axes_) {
    add(axes.body_index, axes.frame, QVector3D(1.f, 1.f, 1.f),
        frame_axes_scale_);
  }
  axes_frame_instance_count_ = static_cast<uint32_t>(instances.size());
  for(const JointAxis& axis : joint_axes_) {
    add(axis.body_index, axis.line, joint_color_, 0.5f * joint_thickness_);
  }
  if(instances.empty()) {
    return batch;
  }
  const quint32 bytes = quint32(instances.size() * sizeof(AxesInstance));
  if(!axes_instances_ || axes_instances_->size() < bytes) {
    axes_instances_.reset(rhi_->newBuffer(QRhiBuffer::Dynamic,
                                          QRhiBuffer::VertexBuffer, bytes));
    axes_instances_->create();
  }
  batch->updateDynamicBuffer(axes_instances_.get(), 0, bytes,
                             instances.data());
  axes_instance_count_ = static_cast<uint32_t>(instances.size());
  return batch;
}

void shift::gui::Viewport::DrawFrameAxes(QRhiCommandBuffer* cb,
                                         const QRhiViewport& viewport) {
  if(axes_instance_count_ == 0) {
    return;
  }
  const QRhiCommandBuffer::DynamicOffset zero_offset(1, 0);
  cb->setGraphicsPipeline(axes_pipeline_.get());
  cb->setViewport(viewport);
  cb->setShaderResources(raster_bindings_.get(), 1, &zero_offset);
  const QRhiCommandBuffer::VertexInput inputs[2] = {
    {axes_vertices_.get(), 0}, {axes_instances_.get(), 0}};
  cb->setVertexInput(0, 2, inputs, axes_indices_.get(), 0,
                     QRhiCommandBuffer::IndexUInt32);
  // the arrows of the frames, then the lines of the joints
  if(axes_frame_instance_count_ > 0) {
    cb->drawIndexed(axes_arrow_index_count_, axes_frame_instance_count_);
  }
  if(axes_instance_count_ > axes_frame_instance_count_) {
    cb->drawIndexed(axes_index_count_ - axes_arrow_index_count_,
                    axes_instance_count_ - axes_frame_instance_count_,
                    axes_arrow_index_count_, 0, axes_frame_instance_count_);
  }
}

void shift::gui::Viewport::mousePressEvent(QMouseEvent* event) {
  if(drag_button_ == Qt::NoButton) {
    drag_button_ = event->button();
    last_mouse_ = event->position().toPoint();
  }
}

void shift::gui::Viewport::mouseMoveEvent(QMouseEvent* event) {
  const QPoint position = event->position().toPoint();
  const QPoint delta = position - last_mouse_;
  last_mouse_ = position;
  const float scale = 1.f / float(std::max(height(), 1));
  if(drag_button_ == Qt::LeftButton) {
    camera_.Orbit(-delta.x() * 3.f * scale, delta.y() * 3.f * scale);
  } else if(drag_button_ == Qt::RightButton) {
    camera_.Pan(delta.x() * scale, delta.y() * scale);
  } else if(drag_button_ == Qt::MiddleButton) {
    // drag down to move closer, as in the original workspace
    camera_.Zoom(std::exp(-2.f * delta.y() * scale));
  } else {
    return;
  }
  update();
}

void shift::gui::Viewport::mouseReleaseEvent(QMouseEvent* event) {
  if(event->button() == drag_button_) {
    drag_button_ = Qt::NoButton;
  }
}

void shift::gui::Viewport::wheelEvent(QWheelEvent* event) {
  const float steps = event->angleDelta().y() / 120.f;
  const float factor = std::pow(0.88f, steps);
  // zoom towards the surface under the cursor, so that it can be seen up
  // close without the camera passing through it; elsewhere (the sky, the
  // floor, terrain) towards the target
  QVector3D point;
  if(Pick(event->position(), point)) {
    camera_.ZoomTowards(point, factor);
  } else {
    camera_.Zoom(factor);
  }
  update();
}

void shift::gui::Viewport::TriangulateCad() {
  cad_triangles_dirty_ = false;
  cad_triangles_.clear();
  cad_triangle_faces_.clear();
  if(!rhi_ || !simulation_ || !rhi_->isFeatureSupported(QRhi::Compute)) {
    return;
  }
  const Scene& scene = simulation_->get_scene();
  cad_triangles_.resize(scene.cads.size());
  cad_triangle_faces_.resize(scene.cads.size());

  struct Params {
    float scale[4];
    int32_t control[4];
  };
  using Binding = QRhiShaderResourceBinding;
  const auto compute = Binding::ComputeStage;
  std::unique_ptr<QRhiComputePipeline> pipeline;
  for(size_t c = 0; c < scene.cads.size(); c++) {
    const SceneCad& cad = scene.cads[c];
    const size_t patch_count = cad.patches.size();
    const size_t triangle_count =
      cad.first_triangles.empty() ? 0 : size_t(cad.first_triangles.back());
    if(patch_count == 0 || triangle_count == 0) {
      continue;
    }
    // storage buffers are addressed with 32 bit sizes, and a dispatch has at
    // most 65535 groups of 64 patches
    if(triangle_count * sizeof(RtTriangle) >= (size_t(1) << 31) ||
       patch_count > 64 * size_t(65535)) {
      qWarning("CAD geometry has too many triangles to ray trace (%zu)",
               triangle_count);
      continue;
    }
    std::vector<int32_t> jobs(4 * patch_count, 0);
    for(size_t p = 0; p < patch_count; p++) {
      jobs[4 * p + 0] = cad.grids[2 * p];
      jobs[4 * p + 1] = cad.grids[2 * p + 1];
      jobs[4 * p + 2] = cad.first_triangles[p];
    }

    QRhiResourceUpdateBatch* batch = rhi_->nextResourceUpdateBatch();
    std::vector<std::unique_ptr<QRhiBuffer>> inputs;
    auto input = [&](const void* data, size_t bytes) {
      std::unique_ptr<QRhiBuffer> b(
        rhi_->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::StorageBuffer,
                        quint32(std::max<size_t>(bytes, 16))));
      b->create();
      if(bytes > 0) {
        batch->uploadStaticBuffer(b.get(), 0, quint32(bytes), data);
      }
      inputs.push_back(std::move(b));
      return inputs.back().get();
    };
    QRhiBuffer* faces =
      input(cad.faces.data(), cad.faces.size() * sizeof(CadGpuFace));
    QRhiBuffer* curves =
      input(cad.curves.data(), cad.curves.size() * sizeof(CadGpuCurve));
    QRhiBuffer* patches =
      input(cad.patches.data(), patch_count * sizeof(CadGpuPatch));
    QRhiBuffer* points =
      input(cad.points.data(), cad.points.size() * sizeof(float));
    QRhiBuffer* knots =
      input(cad.knots.data(), cad.knots.size() * sizeof(float));
    QRhiBuffer* job_buffer = input(jobs.data(), jobs.size() * sizeof(int32_t));
    // the outputs are read back, so not Dynamic
    std::unique_ptr<QRhiBuffer> triangles(rhi_->newBuffer(
      QRhiBuffer::Static, QRhiBuffer::StorageBuffer,
      quint32(triangle_count * sizeof(RtTriangle))));
    triangles->create();
    std::unique_ptr<QRhiBuffer> triangle_faces(rhi_->newBuffer(
      QRhiBuffer::Static, QRhiBuffer::StorageBuffer,
      quint32(std::max<size_t>(triangle_count * sizeof(int32_t), 16))));
    triangle_faces->create();
    std::unique_ptr<QRhiBuffer> params_buffer(rhi_->newBuffer(
      QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(Params)));
    params_buffer->create();
    Params params{{cad.scale, 0.f, 0.f, 0.f},
                  {static_cast<int32_t>(patch_count), 0, 0, 0}};
    batch->updateDynamicBuffer(params_buffer.get(), 0, sizeof(params),
                               &params);

    std::unique_ptr<QRhiShaderResourceBindings> bindings(
      rhi_->newShaderResourceBindings());
    bindings->setBindings(
      {Binding::uniformBuffer(0, compute, params_buffer.get()),
       Binding::bufferLoad(1, compute, job_buffer),
       Binding::bufferStore(2, compute, triangles.get()),
       Binding::bufferStore(3, compute, triangle_faces.get()),
       Binding::bufferLoad(10, compute, faces),
       Binding::bufferLoad(11, compute, curves),
       Binding::bufferLoad(12, compute, patches),
       Binding::bufferLoad(13, compute, points),
       Binding::bufferLoad(14, compute, knots)});
    bindings->create();
    if(!pipeline) {
      pipeline.reset(rhi_->newComputePipeline());
      pipeline->setShaderStage(
        {QRhiShaderStage::Compute, LoadShader("cad_triangulate.comp.qsb")});
      pipeline->setShaderResourceBindings(bindings.get());
      if(!pipeline->create()) {
        qWarning("Cannot create the CAD triangulation pipeline; CAD geometry "
                 "is not ray traced");
        batch->release();
        return;
      }
    }

    QRhiCommandBuffer* cb = nullptr;
    if(rhi_->beginOffscreenFrame(&cb) != QRhi::FrameOpSuccess) {
      batch->release();
      return;
    }
    cb->beginComputePass(batch);
    cb->setComputePipeline(pipeline.get());
    cb->setShaderResources(bindings.get());
    cb->dispatch(int((patch_count + 63) / 64), 1, 1);
    QRhiReadbackResult triangles_read, faces_read;
    QRhiResourceUpdateBatch* readback = rhi_->nextResourceUpdateBatch();
    readback->readBackBuffer(triangles.get(), 0,
                             quint32(triangle_count * sizeof(RtTriangle)),
                             &triangles_read);
    readback->readBackBuffer(triangle_faces.get(), 0,
                             quint32(triangle_count * sizeof(int32_t)),
                             &faces_read);
    cb->endComputePass(readback);
    // waits for the GPU, so the readbacks are complete
    if(rhi_->endOffscreenFrame() != QRhi::FrameOpSuccess ||
       size_t(triangles_read.data.size()) <
         triangle_count * sizeof(RtTriangle) ||
       size_t(faces_read.data.size()) < triangle_count * sizeof(int32_t)) {
      qWarning("Cannot read back the triangles of CAD geometry");
      continue;
    }
    cad_triangles_[c].resize(triangle_count);
    std::memcpy(cad_triangles_[c].data(), triangles_read.data.constData(),
                triangle_count * sizeof(RtTriangle));
    cad_triangle_faces_[c].resize(triangle_count);
    std::memcpy(cad_triangle_faces_[c].data(), faces_read.data.constData(),
                triangle_count * sizeof(int32_t));
  }
}

void shift::gui::Viewport::BuildPickBvh() {
  pick_triangles_.clear();
  pick_triangle_faces_.clear();
  pick_nodes_.clear();
  pick_roots_.clear();
  pick_cad_trims_ = CadTrimTables();
  if(simulation_) {
    const Scene& scene = simulation_->get_scene();
    pick_cad_trims_ = MergeCadTrims(scene.cads);
    BuildMeshBvhs(scene, cad_triangles_, cad_triangle_faces_, pick_cad_trims_,
                  pick_triangles_, pick_triangle_faces_, pick_nodes_,
                  pick_roots_);
  }
  pick_built_ = true;
}

bool shift::gui::Viewport::Pick(const QPointF& pixel, QVector3D& point) {
  if(!simulation_ || poses_.bodies.empty()) {
    return false;
  }
  if(!pick_built_) {
    BuildPickBvh();
  }
  // the ray from the camera through the pixel
  const float w = float(std::max(width(), 1));
  const float h = float(std::max(height(), 1));
  const QMatrix4x4 inverse_view_projection =
    (camera_.get_projection_matrix(w / h) * camera_.get_view_matrix())
      .inverted();
  const float x = 2.f * float(pixel.x()) / w - 1.f;
  const float y = 1.f - 2.f * float(pixel.y()) / h;
  const QVector3D near_point =
    inverse_view_projection.map(QVector3D(x, y, -1.f));
  const QVector3D far_point = inverse_view_projection.map(QVector3D(x, y, 1.f));
  const QVector3D origin = camera_.get_position();
  const QVector3D direction = (far_point - near_point).normalized();

  const Scene& scene = simulation_->get_scene();
  float t_best = std::numeric_limits<float>::max();
  std::vector<int> stack;
  for(const auto& instance : RtInstanceList(scene)) {
    if(instance.mesh_index >= static_cast<int>(pick_roots_.size()) ||
       instance.body_index >= static_cast<int>(poses_.bodies.size()) ||
       !is_body_shown(instance.body_index)) {
      continue;
    }
    // the ray in body coordinates; poses are rigid, so t is unchanged
    const QMatrix4x4 world_to_body =
      poses_.bodies[instance.body_index].inverted();
    const QVector3D o = world_to_body.map(origin);
    const QVector3D d = world_to_body.mapVector(direction);
    const QVector3D inverse_d(1.f / d.x(), 1.f / d.y(), 1.f / d.z());
    stack.assign(1, pick_roots_[instance.mesh_index]);
    while(!stack.empty()) {
      const BvhNode& node = pick_nodes_[stack.back()];
      const int index = stack.back();
      stack.pop_back();
      if(!RayHitsBox(o, inverse_d, node.box_min, node.box_max, t_best)) {
        continue;
      }
      if(node.triangle_count < 0) {
        stack.push_back(node.right_or_first);
        stack.push_back(index + 1); // the left child follows its parent
      } else {
        for(int i = 0; i < node.triangle_count; i++) {
          const int k = node.right_or_first + i;
          const RtTriangle& triangle = pick_triangles_[k];
          float t, b1, b2;
          if(!RayHitsTriangle(o, d, triangle, t_best, t, b1, b2)) {
            continue;
          }
          // a CAD face's triangle sticking out of its trims
          const int32_t face = pick_triangle_faces_[k];
          if(face >= 0 && (face & 1) != 0) {
            const float b0 = 1.f - b1 - b2;
            const float u =
              b0 * triangle.v0[3] + b1 * triangle.v1[3] + b2 * triangle.v2[3];
            const float v =
              b0 * triangle.n0[3] + b1 * triangle.n1[3] + b2 * triangle.n2[3];
            if(!CadInside(pick_cad_trims_, face >> 1, u, v)) {
              continue;
            }
          }
          t_best = t;
        }
      }
    }
  }
  if(t_best == std::numeric_limits<float>::max()) {
    return false;
  }
  point = origin + direction * t_best;
  return true;
}

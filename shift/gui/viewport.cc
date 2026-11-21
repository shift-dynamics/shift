#include "shift/gui/viewport.h"

#include "shift/core/log.h"
#include "shift/gui/opacity_micromap.h"

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
#include <rhi/qrhi_platform.h>
#include <rhi/qshader.h>

#include <QFloat16>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <map>
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
  float sky_zenith[4]; // w = exposure (stops), see tonemap.glsl
  float sky_horizon[4]; // w = tone curve (0 Reinhard, 1 ACES)
  float floor_params[4];
  float floor_frame[16]; // column major: axes u, v, normal, origin
  float fog_params[4]; // density, height falloff, height, 1 if any fog
  float fog_color[4];
  float water_params[4]; // height (up), wave height, wavelength, 1 if water
  float water_wave[4];   // direction of travel (x, y), time
  float water_color[4];
  float water_extinction[4];
  // the colors of the stress drawn on flexible bodies (set_color_map),
  // sRGB; only mesh.frag declares them
  float color_map[shift::gui::Viewport::kColorMapSamples][4];
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
  float variation[4]; // x = 1: vary the tiles of triplanar textures, y =
                      // material map + 1, z = alpha cutoff (0: opaque),
                      // w = opacity (1: opaque)
  int32_t surface_layers[4]; // base color and normal of surfaces 0 and 1
  float surface[2][4]; // x = 1 / texture scale, y = normal scale, z = noise
                       // level, w = 1 / patch size
  float emission[4]; // rgb: factor of the base color added to the shading
  float holes[4][4]; // of a terrain, in its uv: x0, y0, x1, y1 (SceneMesh)
  float flex[4]; // x = 1: a flexible body's stress drawn (FlexUpdate)
};

/*! the material maps of the terrains, as in ground.glsl (std430), followed
 * by their samples' indices, four per 32 bit word */
struct GroundMaterialData {
  float color[4];     // rgb: factor of the texture, w = roughness factor
  float scale[4];     // x = 1 / texture scale, y = normal scale
  int32_t layers[4];  // base color, normal, roughness; -1: none
};
struct GroundMapData {
  float world_to_map[16];
  float scale[4];  // x, y = 1 / spacing of the samples, z = blend width
  int32_t size[4]; // samples along x, y; first index byte, first material
};
struct GroundBuffer {
  int32_t counts[4];
  GroundMapData maps[shift::gui::Scene::kMaxMaterialMaps];
  GroundMaterialData materials[shift::gui::Scene::kMaxGroundMaterials];
};
static_assert(sizeof(GroundMaterialData) == 48);
static_assert(sizeof(GroundMapData) == 96);
static_assert(sizeof(GroundBuffer) == 16 + 8 * 96 + 64 * 48);

/*! the contents of the ground buffer for the material maps of a scene,
 * which keeps to Scene::kMaxMaterialMaps and kMaxGroundMaterials */
static QByteArray GroundContents(const shift::gui::Scene& scene) {
  GroundBuffer header{};
  std::vector<uint8_t> bytes;
  int materials = 0;
  int map_count = 0;
  for(const auto& map : scene.material_maps) {
    if(map_count == shift::gui::Scene::kMaxMaterialMaps ||
       materials + int(map.materials.size()) >
         shift::gui::Scene::kMaxGroundMaterials) {
      break; // not in a scene from Simulation
    }
    GroundMapData& m = header.maps[map_count];
    ksl_mat4x4f_t world_to_map;
    ksl_SE3f_toMat4x4f(&map.world_to_map, &world_to_map);
    std::memcpy(m.world_to_map, world_to_map.as_array, 16 * sizeof(float));
    m.scale[0] = 1.f / map.x_scale;
    m.scale[1] = 1.f / map.y_scale;
    m.scale[2] = map.blend_width;
    m.size[0] = map.width;
    m.size[1] = map.height;
    m.size[2] = int32_t(bytes.size());
    m.size[3] = materials;
    bytes.insert(bytes.end(), map.indices.begin(), map.indices.end());
    for(const auto& g : map.materials) {
      GroundMaterialData& d = header.materials[materials++];
      d.color[0] = g.color.x;
      d.color[1] = g.color.y;
      d.color[2] = g.color.z;
      d.color[3] = g.roughness;
      d.scale[0] = 1.f / g.texture_scale;
      d.scale[1] = g.normal_scale;
      d.scale[2] = g.height_scale;
      d.layers[0] = g.base_color_layer;
      d.layers[1] = g.normal_layer;
      d.layers[2] = g.roughness_layer;
      d.layers[3] = g.height_layer;
    }
    map_count++;
  }
  header.counts[0] = map_count;
  header.counts[1] = materials;
  bytes.resize((bytes.size() + 3) / 4 * 4 + 4, 0); // whole words, not empty
  QByteArray data(int(sizeof(GroundBuffer) + bytes.size()), 0);
  std::memcpy(data.data(), &header, sizeof(GroundBuffer));
  std::memcpy(data.data() + sizeof(GroundBuffer), bytes.data(), bytes.size());
  return data;
}

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

/*! a x + b y */
static ksl_vec3f_t Combined(float a, const ksl_vec3f_t& x, float b,
                            const ksl_vec3f_t& y) {
  ksl_vec3f_t sum;
  ksl_product_avf(a, &x, &sum);
  ksl_axpy_vvf(b, &y, &sum);
  return sum;
}

/*! the unit vector along a + b */
static ksl_vec3f_t Bisector(const ksl_vec3f_t& a, const ksl_vec3f_t& b) {
  ksl_vec3f_t sum;
  ksl_add_vvf(&a, &b, &sum);
  ksl_vec3f_normalize(&sum);
  return sum;
}

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
  const ksl_vec3f_t zero = ksl_vec3f(0.f, 0.f, 0.f);
  vertices.clear();
  indices.clear();
  for(int axis = 0; axis < 3; axis++) {
    ksl_vec3f_t e = zero, u = zero, v = zero;
    e.at[axis] = 1.f;
    u.at[(axis + 1) % 3] = 1.f;
    v.at[(axis + 2) % 3] = 1.f;
    ksl_vec3f_t minus_e;
    ksl_product_avf(-1.f, &e, &minus_e);
    const float* color = kColors[axis];
    auto add = [&](const ksl_vec3f_t& p, const ksl_vec3f_t& n) {
      AxesVertex vertex;
      for(int i = 0; i < 3; i++) {
        vertex.position[i] = p.at[i];
        vertex.normal[i] = n.at[i];
        vertex.color[i] = color[i];
      }
      vertices.push_back(vertex);
      return static_cast<uint32_t>(vertices.size() - 1);
    };
    // the slope of the cone's side
    const float head_length = 1.f - kShaftLength;
    const float slope = std::hypot(head_length, kHeadRadius);
    const ksl_vec3f_t tip = e;
    const ksl_vec3f_t base = Combined(kShaftLength, e, 0.f, e);
    const uint32_t base_center = add(base, minus_e);
    for(int k = 0; k < kSegments; k++) {
      const float a0 = 2.f * float(M_PI) * float(k) / kSegments;
      const float a1 = 2.f * float(M_PI) * float(k + 1) / kSegments;
      const ksl_vec3f_t r0 = Combined(std::cos(a0), u, std::sin(a0), v);
      const ksl_vec3f_t r1 = Combined(std::cos(a1), u, std::sin(a1), v);
      // shaft
      const uint32_t s0 = add(Combined(kShaftRadius, r0, 0.f, r0), r0);
      const uint32_t s1 = add(Combined(kShaftRadius, r1, 0.f, r1), r1);
      const uint32_t s2 = add(Combined(1.f, base, kShaftRadius, r1), r1);
      const uint32_t s3 = add(Combined(1.f, base, kShaftRadius, r0), r0);
      indices.insert(indices.end(), {s0, s1, s2, s0, s2, s3});
      // the base of the head
      const uint32_t b0 = add(Combined(1.f, base, kHeadRadius, r0), minus_e);
      const uint32_t b1 = add(Combined(1.f, base, kHeadRadius, r1), minus_e);
      indices.insert(indices.end(), {base_center, b1, b0});
      // the side of the head
      const ksl_vec3f_t n0 =
        Combined(head_length / slope, r0, kHeadRadius / slope, e);
      const ksl_vec3f_t n1 =
        Combined(head_length / slope, r1, kHeadRadius / slope, e);
      const uint32_t h0 = add(Combined(1.f, base, kHeadRadius, r0), n0);
      const uint32_t h1 = add(Combined(1.f, base, kHeadRadius, r1), n1);
      const uint32_t h2 = add(tip, Bisector(n0, n1));
      indices.insert(indices.end(), {h0, h1, h2});
    }
  }
  arrow_indices = static_cast<uint32_t>(indices.size());

  constexpr int kLineSegments = 16;
  constexpr float kLineRadius = 1.f;
  constexpr float kLineHeadRadius = 3.f;
  constexpr float kLineHeadLength = 0.24f; // of the length 2
  auto add = [&](const ksl_vec3f_t& p, const ksl_vec3f_t& n) {
    AxesVertex vertex;
    for(int i = 0; i < 3; i++) {
      vertex.position[i] = p.at[i];
      vertex.normal[i] = n.at[i];
      vertex.color[i] = 1.f;
    }
    vertices.push_back(vertex);
    return static_cast<uint32_t>(vertices.size() - 1);
  };
  const ksl_vec3f_t z = ksl_vec3f(0.f, 0.f, 1.f);
  const ksl_vec3f_t minus_z = ksl_vec3f(0.f, 0.f, -1.f);
  const ksl_vec3f_t head_base = ksl_vec3f(0.f, 0.f, 1.f - kLineHeadLength);
  const float head_slope = std::hypot(kLineHeadLength, kLineHeadRadius);
  const uint32_t tail_center = add(minus_z, minus_z);
  const uint32_t head_center = add(head_base, minus_z);
  for(int k = 0; k < kLineSegments; k++) {
    const float a0 = 2.f * float(M_PI) * float(k) / kLineSegments;
    const float a1 = 2.f * float(M_PI) * float(k + 1) / kLineSegments;
    const ksl_vec3f_t r0 = ksl_vec3f(std::cos(a0), std::sin(a0), 0.f);
    const ksl_vec3f_t r1 = ksl_vec3f(std::cos(a1), std::sin(a1), 0.f);
    // shaft, closed at the tail
    const uint32_t s0 = add(Combined(kLineRadius, r0, 1.f, minus_z), r0);
    const uint32_t s1 = add(Combined(kLineRadius, r1, 1.f, minus_z), r1);
    const uint32_t s2 = add(Combined(1.f, head_base, kLineRadius, r1), r1);
    const uint32_t s3 = add(Combined(1.f, head_base, kLineRadius, r0), r0);
    indices.insert(indices.end(), {s0, s1, s2, s0, s2, s3});
    const uint32_t t0 = add(Combined(kLineRadius, r0, 1.f, minus_z), minus_z);
    const uint32_t t1 = add(Combined(kLineRadius, r1, 1.f, minus_z), minus_z);
    indices.insert(indices.end(), {tail_center, t1, t0});
    // the base of the head
    const uint32_t b0 =
      add(Combined(1.f, head_base, kLineHeadRadius, r0), minus_z);
    const uint32_t b1 =
      add(Combined(1.f, head_base, kLineHeadRadius, r1), minus_z);
    indices.insert(indices.end(), {head_center, b1, b0});
    // the side of the head
    const ksl_vec3f_t n0 = Combined(kLineHeadLength / head_slope, r0,
                                    kLineHeadRadius / head_slope, z);
    const ksl_vec3f_t n1 = Combined(kLineHeadLength / head_slope, r1,
                                    kLineHeadRadius / head_slope, z);
    const uint32_t h0 = add(Combined(1.f, head_base, kLineHeadRadius, r0), n0);
    const uint32_t h1 = add(Combined(1.f, head_base, kLineHeadRadius, r1), n1);
    const uint32_t h2 = add(z, Bisector(n0, n1));
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
  float fog_params[4];   // as in FrameUniforms
  float fog_color[4];
  float water_params[4]; // as in FrameUniforms
  float water_wave[4];
  float water_color[4];
  float water_extinction[4];
  uint32_t hardware[4]; // xy = address of the top level acceleration
                        // structure (low, high bits), z = 1 if any
  float color_map[shift::gui::Viewport::kColorMapSamples][4]; // as in
                                                              // FrameUniforms
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
  int32_t lods[4]; // x = the levels in lod_roots, yzw = the mesh's own
                   // texture layers (as offsets 1..3), which they take
  int32_t lod_roots[4]; // the mesh and its simplified levels, finest first,
                        // which shadow and bounce rays choose from
  float lod_errors[4];  // how far each strays from the mesh (m, world)
  float flex[4]; // x = 1: colored by the stress (FlexUpdate; a mesh's in u,
                 // a CAD's in its motion's w); a flexible body's CAD's: y =
                 // its motion's entry - its triangle, z = 1
};
static_assert(sizeof(RtInstance) == 416);
static_assert(sizeof(MaterialUniforms) == 208);

struct DenoiseUniforms {
  float previous_view_projection[16];
  float params[4]; // x = pixel spread angle, y = maximum history length,
                   // z = history valid, w = a-trous step
  float blend[4];  // x = progressive frames, y, z = blend start, end,
                   // w = maximum history length of the unmoved
  int32_t control[4]; // x = instance count
  float view_projection[16]; // current frame
  float upscale[4]; // xy = the primary rays' jitter (ray traced pixels),
                    // zw = ray traced size / output size (1 = not scaled)
  float tone[4];    // x = exposure (stops), y = curve, see tonemap.glsl
};
static_assert(sizeof(DenoiseUniforms) == 208);

/* the radical inverse of index in base, as in raytrace.comp */
static float Halton(uint32_t index, uint32_t base) {
  float f = 1.f, r = 0.f;
  while(index > 0u) {
    f /= float(base);
    r += f * float(index % base);
    index /= base;
  }
  return r;
}



/*! Vulkan instance shared by all viewports, created on first use */
static QVulkanInstance* GetVulkanInstance() {
  static QVulkanInstance* instance = [] {
    auto* created = new QVulkanInstance;
    created->setApiVersion(created->supportedApiVersion());
    created->setExtensions(QRhiVulkanInitParams::preferredInstanceExtensions() +
                           shift::gui::Dlss::InstanceExtensions(*created));
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

/*! a matrix with a projection, column major */
static void CopyMatrix(const QMatrix4x4& m, float* out) {
  std::memcpy(out, m.constData(), 16 * sizeof(float));
}

/*! a rigid transform as a 4 by 4 matrix, column major */
static void CopyMatrix(const ksl_SE3f_t& d, float* out) {
  ksl_mat4x4f_t m;
  ksl_SE3f_toMat4x4f(&d, &m);
  std::memcpy(out, m.as_array, 16 * sizeof(float));
}

static void CopyVector(const ksl_vec3f_t& v, float w, float* out) {
  out[0] = v.x;
  out[1] = v.y;
  out[2] = v.z;
  out[3] = w;
}

/*! k v */
static ksl_vec3f_t Scaled(float k, const ksl_vec3f_t& v) {
  ksl_vec3f_t kv;
  ksl_product_avf(k, &v, &kv);
  return kv;
}

/*! the rigid part of the mesh of instance in the world, for a body of pose
 * body; the mesh is scaled by instance.scale before it */
static ksl_SE3f_t Placed(const ksl_SE3f_t& body,
                         const shift::gui::SceneInstance& instance) {
  ksl_SE3f_t placed;
  ksl_product_ddf(&body, &instance.placement, &placed);
  return placed;
}

/*! a rigid transform that follows a uniform scale, as a 4 by 4 matrix,
 * column major */
static void CopyScaledMatrix(const ksl_SE3f_t& pose, float scale, float* out) {
  CopyMatrix(pose, out);
  for(int k = 0; k < 12; k++) {
    out[k] *= scale;
  }
}

/*! the inverse of CopyScaledMatrix's, column major */
static void CopyInverseScaledMatrix(const ksl_SE3f_t& pose, float scale,
                                    float* out) {
  ksl_SE3f_t inverse;
  ksl_SE3f_inverted(&pose, &inverse);
  CopyMatrix(inverse, out);
  for(int c = 0; c < 4; c++) {
    for(int r = 0; r < 3; r++) {
      out[4 * c + r] /= scale;
    }
  }
}

/*! the mesh of instance in the world as a 4 by 4 matrix, column major: the
 * rigid placement with the instance's uniform scale */
static void CopyPlacedMatrix(const ksl_SE3f_t& body,
                             const shift::gui::SceneInstance& instance,
                             float* out) {
  CopyScaledMatrix(Placed(body, instance), instance.scale, out);
}

/*! v divided by its length */
static ksl_vec3f_t Normalized(const ksl_vec3f_t& v) {
  ksl_vec3f_t n;
  ksl_vec3f_normalized(&v, &n);
  return n;
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
  variation[1] = float(mesh.material_map + 1);
  variation[2] = mesh.alpha_cutoff;
  variation[3] = mesh.opacity;
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
    emission[k] = light.diffuse.at[k] * intensity;
  }
}

/*! world space box of a body space box under a pose */
static void TransformBox(const ksl_SE3f_t& pose, const ksl_vec3f_t& box_min,
                         const ksl_vec3f_t& box_max, float* out_min,
                         float* out_max) {
  for(int k = 0; k < 3; k++) {
    out_min[k] = 1e30f;
    out_max[k] = -1e30f;
  }
  for(int corner = 0; corner < 8; corner++) {
    const ksl_vec3f_t local =
      ksl_vec3f((corner & 1) ? box_max.x : box_min.x,
                (corner & 2) ? box_max.y : box_min.y,
                (corner & 4) ? box_max.z : box_min.z);
    ksl_vec3f_t p;
    ksl_product_dvf(&pose, &local, &p);
    for(int k = 0; k < 3; k++) {
      out_min[k] = std::min(out_min[k], p.at[k]);
      out_max[k] = std::max(out_max[k], p.at[k]);
    }
  }
  out_min[3] = out_max[3] = 0.f;
}


/*! the lights of a scene in the given body poses; returns their number */
static int FillLights(const shift::gui::Scene& scene,
                      const shift::gui::Poses& poses, LightUniforms& out) {
  out = LightUniforms{};
  int count = 0;
  ksl_vec3f_t ambient = ksl_vec3f(0.f, 0.f, 0.f);
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
    ksl_SE3f_t frame;
    ksl_product_ddf(&poses.bodies[light.body_index], &light.frame, &frame);
    LightData& d = out.lights[count++];
    // the light shines along -z of its frame
    const ksl_vec3f_t shine = Normalized(Scaled(-1.f, frame.R.v2));
    const float cos_outer = std::cos(light.cone_angle);
    const float cos_inner =
      std::max(std::cos(light.inner_cone_angle), cos_outer + 1e-4f);
    CopyVector(frame.t, float(light.type), d.position);
    CopyVector(shine, cos_outer, d.direction);
    CopyVector(Normalized(frame.R.v0), cos_inner, d.axis_x);
    // pi: a white matte surface facing the light shows its color Kd
    CopyVector(Scaled(float(M_PI) * intensity, light.diffuse),
               float(light.ies_profile), d.diffuse);
    CopyVector(Scaled(float(M_PI) * intensity, light.specular), 0.f,
               d.specular);
    CopyVector(light.attenuation, 0.f, d.attenuation);
    ksl_xpy_vvf(&light.ambient, &ambient);
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
  sun_direction_ = environment_.sun_direction;
}

shift::gui::Viewport::~Viewport() {
  ReleaseRhi();
}

bool shift::gui::Viewport::CreatePipeline(QRhiGraphicsPipeline* pipeline,
                                          std::function<void()> failed) {
  return CreatePipeline([pipeline]() { return pipeline->create(); },
                        std::move(failed));
}

bool shift::gui::Viewport::CreatePipeline(QRhiComputePipeline* pipeline,
                                          std::function<void()> failed) {
  return CreatePipeline([pipeline]() { return pipeline->create(); },
                        std::move(failed));
}

bool shift::gui::Viewport::CreatePipeline(std::function<bool()> create,
                                          std::function<void()> failed) {
  if(!offscreen_target_ && rhi_ && rhi_->backend() == QRhi::Vulkan) {
    pending_pipelines_.push_back(
      {std::move(create), std::move(failed), false, creating_pipelines_});
    return true;
  }
  const bool ok = create();
  if(!ok && failed) {
    failed();
  }
  return ok;
}

void shift::gui::Viewport::StartPipelineCompile() {
  if(pending_pipelines_.empty() || pipeline_thread_.joinable()) {
    return;
  }
  pipelines_made_ = false;
  emit CompilingPipelines(true);
  // the GUI thread leaves the device alone until it is joined
  pipeline_thread_ = std::thread([this]() {
    for(auto& pending : pending_pipelines_) {
      pending.ok = pending.create();
    }
    pipelines_made_ = true;
    QMetaObject::invokeMethod(this, [this]() { requestUpdate(); },
                              Qt::QueuedConnection);
  });
}

bool shift::gui::Viewport::FinishPipelineCompile(bool wait) {
  if(!pipeline_thread_.joinable()) {
    return true;
  }
  if(!wait && !pipelines_made_) {
    return false;
  }
  pipeline_thread_.join();
  std::vector<PendingPipeline> made;
  made.swap(pending_pipelines_);
  for(auto& pending : made) {
    if(!pending.ok && pending.failed) {
      pending.failed();
    }
  }
  emit CompilingPipelines(false);
  return true;
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

/* the tone curve as the shaders take it (tonemap.glsl) */
static float ToneCurveValue(shift::gui::Viewport::ToneCurve curve) {
  return curve == shift::gui::Viewport::ToneCurve::kAces ? 1.f : 0.f;
}

void shift::gui::Viewport::set_tone_curve(ToneCurve curve) {
  tone_curve_ = curve;
  // as the exposure, applied after the ray traced history
  update();
}

void shift::gui::Viewport::set_exposure(float stops) {
  exposure_ = stops;
  // the ray traced history is kept before the filmic curve, so it stays
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

void shift::gui::Viewport::set_joint_color(const ksl_vec3f_t& color) {
  joint_color_ = color;
  update();
}

void shift::gui::Viewport::set_joint_thickness(float thickness) {
  joint_thickness_ = thickness;
  update();
}

void shift::gui::Viewport::set_render_scale(float scale) {
  render_scale_ = std::clamp(scale, 0.25f, 1.f);
  // initialize makes the images of the new size
  update();
}

QSize shift::gui::Viewport::RayTracedSize() const {
  const QSize full = get_render_target()->pixelSize();
  if(is_offscreen()) {
    return full;
  }
  if(WantsDlss()) {
    const QSize render_size = dlss_.RenderSize(full, dlss_quality_);
    if(!render_size.isEmpty()) {
      return render_size; // DLSS's mode, in place of the render scale
    }
  }
  if(render_scale_ >= 1.f) {
    return full;
  }
  return QSize(std::max(int(std::lround(full.width() * render_scale_)), 1),
               std::max(int(std::lround(full.height() * render_scale_)), 1));
}

void shift::gui::Viewport::set_lod_tolerance(float pixels) {
  lod_tolerance_ = std::max(pixels, 0.f);
  // the ray tracer's instances hold the levels they trace
  rt_instances_dirty_ = true;
  rt_frame_ = 0;
  update();
}

void shift::gui::Viewport::set_hidden_bodies(std::vector<bool> hidden) {
  hidden_bodies_ = std::move(hidden);
  // the ray tracer's instances hold the bodies' boxes
  rt_instances_dirty_ = true;
  rt_frame_ = 0;
  update();
}

void shift::gui::Viewport::set_flex_mesh_surfaces(
  std::vector<bool> mesh_surface) {
  flex_mesh_surfaces_ = std::move(mesh_surface);
  // the ray tracer's instances: those of the surface drawn
  rt_instances_dirty_ = true;
  rt_frame_ = 0;
  update();
}

void shift::gui::Viewport::set_flex_mesh_edges(std::vector<bool> edges) {
  flex_mesh_edges_ = std::move(edges);
  update();
}

void shift::gui::Viewport::set_color_map(
  const std::vector<ksl_vec3f_t>& colors) {
  color_map_ = colors;
  rt_frame_ = 0;
  update();
}

float shift::gui::Viewport::get_flex_stress_maximum() const {
  float largest = 0.f;
  for(const GpuMesh& gpu : gpu_meshes_) {
    if(gpu.flex_shown.stress == FlexUpdate::Stress::kNone) {
      continue;
    }
    largest = std::max(largest, gpu.flex >= 0 && flex_gpu_
                                  ? flex_gpu_->get_stress_maximum(gpu.flex)
                                  : gpu.stress_maximum);
  }
  return largest;
}

float shift::gui::Viewport::get_flex_safety_factor_minimum() const {
  float smallest = 0.f;
  for(const GpuMesh& gpu : gpu_meshes_) {
    const FlexUpdate& shown = gpu.flex_shown;
    if(shown.stress == FlexUpdate::Stress::kNone ||
       shown.quantity != FlexUpdate::Quantity::kSafetyFactor) {
      continue;
    }
    const float largest = gpu.flex >= 0 && flex_gpu_
                            ? flex_gpu_->get_stress_maximum(gpu.flex)
                            : gpu.stress_maximum;
    const float factor = largest > 0.f
                           ? shown.yield_strength / largest
                           : std::numeric_limits<float>::infinity();
    smallest = smallest > 0.f ? std::min(smallest, factor) : factor;
  }
  return smallest;
}

void shift::gui::Viewport::set_flex_constrained_nodes(
  std::vector<bool> nodes) {
  flex_constrained_nodes_ = std::move(nodes);
  update();
}

void shift::gui::Viewport::FrameScene() {
  if(simulation_ && !poses_.bodies.empty()) {
    ksl_vec3f_t box_min, box_max;
    SceneBounds(simulation_->get_scene(), poses_, box_min, box_max);
    camera_.Frame(box_min, box_max);
    UpdateEnvironment();
  }
  rt_frame_ = 0;
  update();
}

void shift::gui::Viewport::FrameSphere(const ksl_vec3f_t& center,
                                       float radius) {
  const ksl_vec3f_t reach = ksl_vec3f(radius, radius, radius);
  ksl_vec3f_t box_min, box_max;
  ksl_subtract_vvf(&center, &reach, &box_min);
  ksl_add_vvf(&center, &reach, &box_max);
  camera_.Frame(box_min, box_max);
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
      const ksl_SE3f_t frame = FollowFrame(follow_body_, follow_mode_);
      ksl_SE3f_t follow;
      ksl_product_ddf(&frame, &follow_offset_, &follow);
      camera_.set_follow(follow);
    }
  }
}

ksl_SE3f_t shift::gui::Viewport::FollowFrame(int body, FollowMode mode) const {
  const ksl_SE3f_t& pose = poses_.bodies[body];
  if(mode == FollowMode::kBody) {
    return pose;
  }
  // the height of the ground under the body; without a terrain the ground is
  // flat at z = 0, as it is for the tires
  float ground_z = 0.f;
  const ksl_vec3_t point = {{pose.t.x, pose.t.y, pose.t.z}};
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
    const ksl_SE3f_t frame = FollowFrame(body, mode);
    ksl_product_dinvdf(&frame, &camera_.get_follow(), &follow_offset_);
  }
  rt_frame_ = 0;
  update();
}

void shift::gui::Viewport::set_floor_below_model(float z) {
  PlaceFloor(ksl_vec3f(0.f, 0.f, z), ksl_vec3f(0.f, 0.f, -1.f));
  floor_below_model_ = true;
  LowerFloorBelowModel();
}

void shift::gui::Viewport::LowerFloorBelowModel() {
  if(!floor_below_model_ || !simulation_ || poses_.bodies.empty()) {
    return;
  }
  ksl_vec3f_t box_min, box_max, size;
  SceneBounds(simulation_->get_scene(), poses_, box_min, box_max);
  // +z is down: the lowest point has the largest z; leave a small gap so
  // the floor does not touch the model
  ksl_subtract_vvf(&box_max, &box_min, &size);
  const float gap = 0.01f * ksl_vec3f_l2norm(&size);
  const float level = box_max.z + gap;
  if(level > floor_origin_.z) {
    // lowered as the model settles, by a little in each frame
    PlaceFloor(ksl_vec3f(0.5f * (box_min.x + box_max.x),
                         0.5f * (box_min.y + box_max.y), level),
               ksl_vec3f(0.f, 0.f, -1.f), true);
  }
}

void shift::gui::Viewport::set_floor(const ksl_vec3f_t& point,
                                     const ksl_vec3f_t& normal) {
  floor_below_model_ = false;
  PlaceFloor(point, normal);
}

void shift::gui::Viewport::PlaceFloor(const ksl_vec3f_t& point,
                                      const ksl_vec3f_t& normal,
                                      bool keep_history) {
  floor_origin_ = point;
  floor_normal_ = Normalized(normal);
  // in-plane axes: the x axis projected onto the plane, so a level floor
  // keeps its grid along x and y
  ksl_vec3f_t x = ksl_vec3f(1.f, 0.f, 0.f);
  if(std::fabs(ksl_dot_vvf(&x, &floor_normal_)) > 0.9f) {
    x = ksl_vec3f(0.f, 1.f, 0.f);
  }
  ksl_axpy_vvf(-ksl_dot_vvf(&x, &floor_normal_), &floor_normal_, &x);
  floor_u_ = Normalized(x);
  ksl_cross_vvf(&floor_normal_, &floor_u_, &floor_v_);
  if(floor_enabled_) {
    rt_frame_ = 0;
    if(!keep_history) {
      rt_history_valid_ = false;
    }
  }
  update();
}

void shift::gui::Viewport::CopyFloorFrame(float out[16]) const {
  const ksl_vec3f_t* columns[4] = {&floor_u_, &floor_v_, &floor_normal_,
                                   &floor_origin_};
  for(int c = 0; c < 4; c++) {
    out[4 * c + 0] = columns[c]->x;
    out[4 * c + 1] = columns[c]->y;
    out[4 * c + 2] = columns[c]->z;
    out[4 * c + 3] = c == 3 ? 1.f : 0.f;
  }
}

void shift::gui::Viewport::UpdateEnvironment() {
  // size the floor grid to the model in its current pose; the floor itself
  // is the floor plane of the fixed body (see set_floor)
  ksl_vec3f_t box_min, box_max, extent;
  SceneBounds(simulation_->get_scene(), poses_, box_min, box_max);
  ksl_subtract_vvf(&box_max, &box_min, &extent);
  const float size = std::max(ksl_vec3f_l2norm(&extent), 1e-3f);
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
  // the device extensions of DLSS, on whichever device is made
  params.deviceExtensions = Dlss::DeviceExtensions(*vulkanInstance());
  // a device that traces rays in hardware, if there is one, given to QRhi;
  // otherwise QRhi makes its own
  hardware_ = HardwareRayTracing::Create(vulkanInstance(), this,
                                         params.deviceExtensions);
  if(hardware_) {
    QRhiVulkanNativeHandles handles;
    hardware_->FillNativeHandles(&handles);
    rhi_.reset(QRhi::create(QRhi::Vulkan, &params, {}, &handles));
    if(rhi_) {
      SHIFT_INFO("Tracing rays in hardware on " +
                 hardware_->get_device_name());
    } else {
      hardware_.reset();
    }
  }
  if(!rhi_) {
    rhi_.reset(QRhi::create(QRhi::Vulkan, &params));
  }
  if(!rhi_) {
    qWarning("Failed to create a Vulkan QRhi");
    return;
  }
  if(!dlss_.Initialize(rhi_.get()) && Dlss::is_built()) {
    SHIFT_INFO("NVIDIA DLSS is not available: " +
               dlss_.get_reason().toStdString() +
               "; ray tracing uses the viewer's own anti-aliasing");
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

/* SHIFT_GPU_PROFILE: the mean GPU time of each pass of the ray traced
frames, to the standard error */
void shift::gui::Viewport::PrintGpuProfile() {
  if(hardware_ && hardware_->is_profiling()) {
    std::vector<std::string> names = {"top level structure", "ray tracing",
                                      "sun shadow", "denoise: temporal",
                                      "denoise: variance"};
    for(int i = 0; i < kAtrousIterations; i++) {
      names.push_back("denoise: a-trous " + std::to_string(i + 1));
    }
    names.push_back("denoise: composite");
    if(dlss_profiled_) {
      names.push_back("DLSS: inputs");
      names.push_back("DLSS");
    } else {
      names.push_back("anti-aliasing");
    }
    std::vector<const char*> pointers;
    for(const auto& name : names) {
      pointers.push_back(name.c_str());
    }
    std::fprintf(stderr, "%s",
                 hardware_->TakeProfileReport(pointers.data(),
                                          int(pointers.size()))
                   .c_str());
  }
}

void shift::gui::Viewport::ReleaseRhi() {
  FinishPipelineCompile(true);
  offscreen_target_.reset();
  offscreen_render_pass_.reset();
  offscreen_depth_.reset();
  offscreen_texture_.reset();
  ReleaseAll();
  dlss_.Shutdown(); // before the device goes
  dlss_failed_ = false;
  device_resources_ready_ = false;
  swapchain_ready_ = false;
  swapchain_render_pass_.reset();
  swapchain_.reset();
  depth_stencil_.reset();
  rhi_.reset();
  PrintGpuProfile();
  // the device last, as QRhi used it
  hardware_scene_ = false;
  hardware_.reset();
}

QRhiRenderTarget* shift::gui::Viewport::get_render_target() const {
  if(offscreen_target_) {
    return offscreen_target_.get();
  }
  return swapchain_->currentFrameRenderTarget();
}

bool shift::gui::Viewport::BeginOffscreen(const QSize& size, QString& error) {
  FinishPipelineCompile(true);
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
  PrintGpuProfile();
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
  if(!FinishPipelineCompile(false)) {
    return; // drawn once the worker has made the pipelines
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
    if(!pending_pipelines_.empty()) {
      StartPipelineCompile(); // and triangulated once it is made
      return;
    }
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
  // the pipelines this frame queued, made before the next draws
  StartPipelineCompile();
}

void shift::gui::Viewport::initialize() {
  if(!device_resources_ready_) {
    CreateDeviceResources();
    device_resources_ready_ = true;
    scene_dirty_ = simulation_ != nullptr;
  }
  if(output_size_ != get_render_target()->pixelSize() ||
     (ray_tracing_supported_ && (rt_size_ != RayTracedSize() ||
                                 WantsDlss() != dlss_active_))) {
    CreateSizeDependentResources();
  }
  if(pipeline_render_pass_ != get_render_target()->renderPassDescriptor()) {
    CreatePipelines();
  }
}

void shift::gui::Viewport::ReleaseAll() {
  FinishPipelineCompile(true);
  pending_pipelines_.clear();
  cad_triangulate_pipeline_.reset();
  cad_triangulate_layout_.reset();
  cad_triangulate_dummies_.clear();
  axes_pipeline_.reset();
  axes_instances_.reset();
  axes_indices_.reset();
  axes_vertices_.reset();
  axes_mesh_uploaded_ = false;
  axes_instance_count_ = 0;
  cad_pipeline_.reset();
  gpu_cads_.clear();
  cad_rigid_grids_.reset();
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
  waves_texture_.reset();
  waves_sampler_.reset();
  lights_ubo_.reset();
  texture_sampler_.reset();
  texture_array_.reset();
  large_texture_array_.reset();
  linear_sampler_.reset();
  environment_brdf_.reset();
  environment_irradiance_.reset();
  environment_specular_.reset();
  blit_pipeline_.reset();
  blit_bindings_.reset();
  sampler_.reset();
  svgf_taa_pipeline_.reset();
  sun_shadow_pipeline_.reset();
  svgf_composite_pipeline_.reset();
  svgf_atrous_pipeline_.reset();
  svgf_variance_pipeline_.reset();
  svgf_temporal_pipeline_.reset();
  rt_pipeline_.reset();
  dlss_.Release();
  dlss_active_ = false;
  dlss_prepare_pipeline_.reset();
  dlss_prepare_bindings_[0].reset();
  dlss_prepare_bindings_[1].reset();
  dlss_output_.reset();
  dlss_motion_.reset();
  dlss_depth_.reset();
  dlss_color_.reset();
  for(int p = 0; p < 2; p++) {
    svgf_taa_bindings_[p].reset();
    sun_shadow_bindings_[p].reset();
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
    rt_sun_passed_[p].reset();
  }
  rt_motion_.reset();
  rt_triangle_motion_buffer_.reset();
  rt_color_.reset();
  rt_sun_.reset();
  rt_direct_.reset();
  rt_albedo_.reset();
  rt_illumination_.reset();
  svgf_ubo_.reset();
  rt_accumulation_.reset();
  rt_upscaled_mean_.reset();
  rt_output_.reset();
  rt_instance_node_buffer_.reset();
  rt_instance_buffer_.reset();
  rt_node_buffer_.reset();
  rt_cad_trim_index_buffer_.reset();
  ground_buffer_.reset();
  rt_cad_segment_buffer_.reset();
  rt_cad_face_buffer_.reset();
  rt_triangle_face_buffer_.reset();
  rt_triangle_buffer_.reset();
  rt_ubo_.reset();
  floor_pipeline_.reset();
  sky_pipeline_.reset();
  edges_pipeline_.reset();
  points_pipeline_.reset();
  spider_pipeline_.reset();
  transparent_pipeline_.reset();
  mesh_pipeline_.reset();
  raster_bindings_.reset();
  instance_buffer_.reset();
  material_ubo_.reset();
  frame_ubo_.reset();
  flex_gpu_.reset();
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

  // the water's waves, uploaded with the first frame that draws water
  waves_texture_.reset(rhi_->newTextureArray(
    QRhiTexture::RGBA16F, OceanWaves::kCascades,
    QSize(OceanWaves::kSize, OceanWaves::kSize), 1, QRhiTexture::MipMapped));
  waves_texture_->create();
  waves_sampler_.reset(rhi_->newSampler(
    QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::Linear,
    QRhiSampler::Repeat, QRhiSampler::Repeat));
  waves_sampler_->create();
  waves_uploaded_ = false;

  // placeholder until a scene with material maps is loaded: not read
  ground_buffer_.reset(rhi_->newBuffer(QRhiBuffer::Immutable,
                                       QRhiBuffer::StorageBuffer,
                                       sizeof(GroundBuffer) + 16));
  ground_buffer_->create();

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
  environment_radiance_ =
    make_table(Tables::kRadianceWidth, Tables::kRadianceHeight);
  environment_specular_ =
    make_table(Tables::kSpecularWidth,
               Tables::kSpecularHeight * Tables::kRoughnessCount);
  environment_irradiance_ =
    make_table(Tables::kIrradianceWidth, Tables::kIrradianceHeight);
  environment_brdf_ = make_table(Tables::kBrdfCount, Tables::kBrdfCount);
  linear_sampler_.reset(rhi_->newSampler(
    QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::None,
    QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge));
  linear_sampler_->create();
  environment_sampler_.reset(rhi_->newSampler(
    QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::None,
    QRhiSampler::Repeat, QRhiSampler::ClampToEdge));
  environment_sampler_->create();
  environment_dirty_ = true;

  texture_sampler_.reset(rhi_->newSampler(
    QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::Linear,
    QRhiSampler::Repeat, QRhiSampler::Repeat));
  texture_sampler_->create();
  // placeholders until a scene is loaded
  CreateTextureArray(texture_array_, 1, 1);
  CreateTextureArray(large_texture_array_, 1, 1);

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
  // DLSS's views of its images go before the images do
  dlss_.Release();
  dlss_active_ = false;
  dlss_color_.reset();
  dlss_depth_.reset();
  dlss_motion_.reset();
  dlss_output_.reset();
  if(!ray_tracing_supported_) {
    return;
  }
  rt_size_ = RayTracedSize();
  dlss_active_ =
    WantsDlss() && !dlss_.RenderSize(output_size_, dlss_quality_).isEmpty();
  // the anti-aliasing pass and its history at the output's size: it scales
  // the ray traced images up (svgf_taa.comp)
  auto make_image = [&](QRhiTexture::Format format, QSize size = QSize()) {
    std::unique_ptr<QRhiTexture> texture(
      rhi_->newTexture(format, size.isEmpty() ? rt_size_ : size, 1,
                       QRhiTexture::UsedWithLoadStore));
    texture->create();
    return texture;
  };
  rt_output_ = make_image(QRhiTexture::RGBA8, output_size_);
  // the progressive mean at the output's size, only read when scaled
  rt_upscaled_mean_ =
    make_image(QRhiTexture::RGBA32F,
               rt_size_ == output_size_ ? QSize(1, 1) : output_size_);
  rt_accumulation_ = make_image(QRhiTexture::RGBA32F);
  rt_illumination_ = make_image(QRhiTexture::RGBA16F);
  rt_albedo_ = make_image(QRhiTexture::RGBA16F);
  rt_direct_ = make_image(QRhiTexture::RGBA16F);
  rt_sun_ = make_image(QRhiTexture::RGBA16F);
  rt_color_ = make_image(QRhiTexture::RGBA16F);
  rt_motion_ = make_image(QRhiTexture::RGBA16F);
  for(int p = 0; p < 2; p++) {
    rt_position_[p] = make_image(QRhiTexture::RGBA32F);
    rt_sun_passed_[p] = make_image(QRhiTexture::R32F);
    rt_normal_[p] = make_image(QRhiTexture::RGBA16F);
    rt_history_illumination_[p] = make_image(QRhiTexture::RGBA16F);
    rt_history_moments_[p] = make_image(QRhiTexture::RGBA32F);
    rt_history_albedo_[p] = make_image(QRhiTexture::RGBA16F);
    rt_filter_[p] = make_image(QRhiTexture::RGBA16F);
    rt_taa_[p] = make_image(QRhiTexture::RGBA16F, output_size_);
  }
  if(dlss_active_) {
    // DLSS's inputs at the ray traced size, its output at the target's
    dlss_color_ = make_image(QRhiTexture::RGBA16F);
    dlss_depth_ = make_image(QRhiTexture::R32F);
    dlss_motion_ = make_image(QRhiTexture::RGBA16F);
    dlss_output_ = make_image(QRhiTexture::RGBA16F, output_size_);
    const Dlss::Images images{dlss_color_.get(), dlss_depth_.get(),
                              dlss_motion_.get(), dlss_output_.get()};
    if(!dlss_.Prepare(images, dlss_quality_)) {
      SHIFT_WARNING("NVIDIA DLSS: " + dlss_.get_reason().toStdString() +
                    "; ray tracing uses the viewer's own anti-aliasing");
      dlss_failed_ = true;
      CreateSizeDependentResources(); // again, without DLSS
      return;
    }
  }
  // bindings reference the textures; rebuild them along with the pipelines
  pipeline_render_pass_ = nullptr;
}

bool shift::gui::Viewport::WantsDlss() const {
  return dlss_enabled_ && !dlss_failed_ && dlss_.is_available() &&
         ray_tracing_supported_ && render_mode_ == RenderMode::kRayTraced &&
         !is_offscreen();
}

void shift::gui::Viewport::set_dlss_enabled(bool enabled) {
  dlss_enabled_ = enabled;
  update(); // initialize makes the images of the new size
}

void shift::gui::Viewport::set_dlss_quality(Dlss::Quality quality) {
  if(quality != dlss_quality_) {
    dlss_quality_ = quality;
    output_size_ = QSize(); // DLSS is prepared for a mode
    update();
  }
}

QSize shift::gui::Viewport::get_dlss_render_size(Dlss::Quality quality) const {
  const QSize output = output_size_.isValid()
                         ? output_size_
                         : QSize(int(std::lround(width() * devicePixelRatio())),
                                 int(std::lround(height() * devicePixelRatio())));
  return dlss_.RenderSize(output, quality);
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
  // a second call before the worker has started (a scene uploaded in the
  // frame whose initialize made them) replaces the pipelines the first one
  // queued: its entries would make pipelines that are gone
  pending_pipelines_.erase(
    std::remove_if(pending_pipelines_.begin(), pending_pipelines_.end(),
                   [](const PendingPipeline& pending) {
                     return pending.from_create_pipelines;
                   }),
    pending_pipelines_.end());
  creating_pipelines_ = true;
  struct Done {
    bool& creating;
    ~Done() {
      creating = false;
    }
  } done{creating_pipelines_};
  using Binding = QRhiShaderResourceBinding;
  const auto vertex_fragment = Binding::VertexStage | Binding::FragmentStage;

  raster_bindings_->setBindings(
    {Binding::uniformBuffer(0, vertex_fragment, frame_ubo_.get()),
     Binding::uniformBufferWithDynamicOffset(
       1, vertex_fragment, material_ubo_.get(), sizeof(MaterialUniforms)),
     Binding::sampledTexture(2, Binding::FragmentStage,
                             environment_specular_.get(),
                             environment_sampler_.get()),
     Binding::sampledTexture(3, Binding::FragmentStage,
                             environment_irradiance_.get(),
                             environment_sampler_.get()),
     Binding::sampledTexture(4, Binding::FragmentStage,
                             environment_brdf_.get(), linear_sampler_.get()),
     Binding::sampledTexture(5, Binding::FragmentStage, texture_array_.get(),
                             texture_sampler_.get()),
     Binding::uniformBuffer(8, Binding::FragmentStage, lights_ubo_.get()),
     Binding::sampledTexture(9, Binding::FragmentStage, ies_texture_.get(),
                             linear_sampler_.get()),
     Binding::sampledTexture(10, Binding::FragmentStage,
                             environment_radiance_.get(),
                             environment_sampler_.get()),
     Binding::bufferLoad(17, Binding::FragmentStage, ground_buffer_.get()),
     Binding::sampledTexture(18, Binding::FragmentStage,
                             environment_radiance_.get(),
                             environment_sampler_.get()),
     Binding::sampledTexture(20, Binding::FragmentStage, waves_texture_.get(),
                             waves_sampler_.get()),
     Binding::sampledTexture(21, Binding::FragmentStage,
                             large_texture_array_.get(),
                             texture_sampler_.get())});
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
  CreatePipeline(mesh_pipeline_.get());

  // the edges of flexible bodies' elements: lines of the meshes' vertices,
  // tested against the depth of the surfaces but not written
  edges_pipeline_.reset(make_pipeline("mesh_edges.vert.qsb",
                                      "mesh_edges.frag.qsb",
                                      raster_bindings_.get(), true));
  edges_pipeline_->setTopology(QRhiGraphicsPipeline::Lines);
  edges_pipeline_->setDepthWrite(false);
  edges_pipeline_->setVertexInputLayout(layout);
  CreatePipeline(edges_pipeline_.get(), [this]() {
    qWarning("Cannot create the pipeline of the edges of flexible bodies' "
             "elements; they are not drawn");
    edges_pipeline_.reset();
  });
  // their constrained nodes: points of the meshes' vertices, as the edges
  points_pipeline_.reset(make_pipeline("mesh_points.vert.qsb",
                                       "mesh_points.frag.qsb",
                                       raster_bindings_.get(), true));
  points_pipeline_->setTopology(QRhiGraphicsPipeline::Points);
  points_pipeline_->setDepthWrite(false);
  points_pipeline_->setVertexInputLayout(layout);
  CreatePipeline(points_pipeline_.get(), [this]() {
    qWarning("Cannot create the pipeline of the constrained nodes of "
             "flexible bodies; they are not drawn");
    points_pipeline_.reset();
  });
  // the spiders of their attachments: lines from each attachment's frame to
  // its nodes, over everything (many lie inside the material)
  spider_pipeline_.reset(make_pipeline("mesh_points.vert.qsb", "spider.frag.qsb",
                                       raster_bindings_.get(), false));
  spider_pipeline_->setTopology(QRhiGraphicsPipeline::Lines);
  {
    QRhiGraphicsPipeline::TargetBlend blend;
    blend.enable = true;
    blend.srcColor = QRhiGraphicsPipeline::SrcAlpha;
    blend.dstColor = QRhiGraphicsPipeline::OneMinusSrcAlpha;
    blend.srcAlpha = QRhiGraphicsPipeline::One;
    blend.dstAlpha = QRhiGraphicsPipeline::OneMinusSrcAlpha;
    spider_pipeline_->setTargetBlends({blend});
  }
  spider_pipeline_->setVertexInputLayout(layout);
  CreatePipeline(spider_pipeline_.get(), [this]() {
    qWarning("Cannot create the pipeline of the spiders of flexible bodies' "
             "attachments; they are not drawn");
    spider_pipeline_.reset();
  });

  // the meshes that let light through (opacity below 1, as glass), drawn
  // over the others, blended by their opacity, without hiding what is
  // drawn after them
  transparent_pipeline_.reset(make_pipeline(
    "mesh.vert.qsb", "mesh.frag.qsb", raster_bindings_.get(), true));
  transparent_pipeline_->setDepthWrite(false);
  QRhiGraphicsPipeline::TargetBlend blend;
  blend.enable = true;
  blend.srcColor = QRhiGraphicsPipeline::SrcAlpha;
  blend.dstColor = QRhiGraphicsPipeline::OneMinusSrcAlpha;
  blend.srcAlpha = QRhiGraphicsPipeline::One;
  blend.dstAlpha = QRhiGraphicsPipeline::OneMinusSrcAlpha;
  transparent_pipeline_->setTargetBlends({blend});
  transparent_pipeline_->setVertexInputLayout(layout);
  transparent_pipeline_->create();

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
  CreatePipeline(particle_pipeline_.get());

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
  CreatePipeline(fluid_sphere_pipeline_.get());

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
    CreatePipeline(fluid_blur_pipeline_[k].get());
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
  CreatePipeline(fluid_pipeline_.get());

  sky_pipeline_.reset(make_pipeline("fullscreen.vert.qsb", "sky.frag.qsb",
                                    raster_bindings_.get(), false));
  CreatePipeline(sky_pipeline_.get());

  floor_pipeline_.reset(make_pipeline("floor.vert.qsb", "floor.frag.qsb",
                                      raster_bindings_.get(), true));
  CreatePipeline(floor_pipeline_.get());

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
  CreatePipeline(axes_pipeline_.get());

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
                                 environment_sampler_.get()),
         Binding::sampledTexture(3, Binding::FragmentStage,
                                 environment_irradiance_.get(),
                                 environment_sampler_.get()),
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
                                 ies_texture_.get(), linear_sampler_.get()),
         Binding::bufferLoad(17, Binding::FragmentStage,
                             ground_buffer_.get()),
         Binding::sampledTexture(18, Binding::FragmentStage,
                                 environment_radiance_.get(),
                                 environment_sampler_.get()),
         Binding::sampledTexture(20, Binding::FragmentStage,
                                 waves_texture_.get(), waves_sampler_.get()),
         Binding::sampledTexture(21, Binding::FragmentStage,
                                 large_texture_array_.get(),
                                 texture_sampler_.get())});
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
    CreatePipeline(terrain_pipeline_.get(), [this]() {
      qWarning("Cannot create the terrain tessellation pipeline; terrains "
               "are drawn as meshes");
      terrain_pipeline_.reset();
    });
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
                                 environment_sampler_.get()),
         Binding::sampledTexture(3, Binding::FragmentStage,
                                 environment_irradiance_.get(),
                                 environment_sampler_.get()),
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
                             gpu.trim_index.get()),
         Binding::bufferLoad(17, Binding::FragmentStage,
                             ground_buffer_.get()),
         Binding::sampledTexture(18, Binding::FragmentStage,
                                 environment_radiance_.get(),
                                 environment_sampler_.get()),
         Binding::sampledTexture(20, Binding::FragmentStage,
                                 waves_texture_.get(), waves_sampler_.get()),
         Binding::sampledTexture(21, Binding::FragmentStage,
                                 large_texture_array_.get(),
                                 texture_sampler_.get()),
         // the deformation of a flexible body's (flex_field.glsl)
         Binding::bufferLoad(22, tessellation,
                             gpu.flex >= 0 ? flex_gpu_->get_cad_grids(gpu.flex)
                                           : cad_rigid_grids_.get()),
         Binding::bufferLoad(23, tessellation,
                             gpu.flex >= 0 ? flex_gpu_->get_cad_field(gpu.flex)
                                           : cad_rigid_grids_.get())});
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
    CreatePipeline(cad_pipeline_.get(), [this]() {
      qWarning("Cannot create the CAD tessellation pipeline; CAD geometry is "
               "not drawn");
      cad_pipeline_.reset();
    });
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
       Binding::bufferLoad(17, compute, rt_cad_trim_index_buffer_.get()),
       Binding::sampledTexture(18, compute, environment_radiance_.get(),
                               environment_sampler_.get()),
       Binding::bufferLoad(19, compute, ground_buffer_.get()),
       Binding::sampledTexture(20, compute, waves_texture_.get(),
                               waves_sampler_.get()),
       Binding::sampledTexture(21, compute, large_texture_array_.get(),
                               texture_sampler_.get()),
       store(22, rt_sun_),
       Binding::bufferLoad(23, compute, rt_triangle_motion_buffer_.get()),
       store(24, rt_motion_)});
    sun_shadow_bindings_[p] = make_bindings(
      {denoise_uniforms, load(1, rt_position_[p]), load(2, rt_normal_[p]),
       load(3, rt_sun_),
       Binding::imageLoadStore(4, compute, rt_direct_.get(), 0),
       Binding::imageLoadStore(5, compute, rt_accumulation_.get(), 0),
       store(6, rt_sun_passed_[p])});
    svgf_temporal_bindings_[p] = make_bindings(
      {denoise_uniforms, instances, load(2, rt_illumination_),
       load(3, rt_albedo_), load(4, rt_position_[p]), load(5, rt_normal_[p]),
       load(6, rt_position_[q]), load(7, rt_normal_[q]),
       load(8, rt_history_illumination_[q]), load(9, rt_history_moments_[q]),
       load(10, rt_history_albedo_[q]), store(11, rt_history_illumination_[p]),
       store(12, rt_history_moments_[p]), store(13, rt_history_albedo_[p]),
       load(14, rt_motion_)});
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
       load(4, rt_filter_[1]),
       store(6, rt_color_), load(7, rt_direct_), load(8, rt_albedo_)});
    svgf_taa_bindings_[p] = make_bindings(
      {denoise_uniforms, instances, load(2, rt_position_[p]),
       load(3, rt_color_), load(4, rt_taa_[q]), store(5, rt_taa_[p]),
       store(6, rt_output_), load(7, rt_accumulation_),
       load(8, rt_position_[q]), load(9, rt_normal_[p]),
       load(10, rt_normal_[q]), load(11, rt_illumination_),
       load(12, rt_albedo_), load(13, rt_direct_),
       Binding::imageLoadStore(14, compute, rt_upscaled_mean_.get(), 0),
       load(15, rt_sun_passed_[p]), load(16, rt_sun_passed_[q]),
       load(17, rt_motion_)});
    if(dlss_active_) {
      dlss_prepare_bindings_[p] = make_bindings(
        {denoise_uniforms, instances, load(2, rt_position_[p]),
         load(3, rt_color_), store(4, dlss_color_), store(5, dlss_motion_),
         store(6, dlss_depth_), load(7, rt_motion_)});
    }
  }

  // the variant with ray queries on a device that traces rays in hardware,
  // with the same bindings
  QShader raytrace = LoadShader("raytrace.comp.qsb");
  if(hardware_) {
    raytrace.setShader(QShaderKey(QShader::SpirvShader, QShaderVersion(100)),
                       QShaderCode(HardwareRayTracing::get_spirv(), "main"));
  }
  rt_pipeline_.reset(rhi_->newComputePipeline());
  rt_pipeline_->setShaderStage({QRhiShaderStage::Compute, raytrace});
  rt_pipeline_->setShaderResourceBindings(rt_bindings_[0].get());
  rt_pipeline_->create();
  sun_shadow_pipeline_.reset(NewComputePipeline(
    "sun_shadow.comp.qsb", sun_shadow_bindings_[0].get()));
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
  dlss_prepare_pipeline_.reset();
  if(dlss_active_) {
    dlss_prepare_pipeline_.reset(NewComputePipeline(
      "dlss_prepare.comp.qsb", dlss_prepare_bindings_[0].get()));
  }

  // DLSS's output is at the target's size, as rt_output_ is
  blit_bindings_->setBindings({Binding::sampledTexture(
    0, Binding::FragmentStage,
    dlss_active_ ? dlss_output_.get() : rt_output_.get(), sampler_.get())});
  blit_bindings_->create();
  blit_pipeline_.reset(make_pipeline("fullscreen.vert.qsb", "blit.frag.qsb",
                                     blit_bindings_.get(), false));
  CreatePipeline(blit_pipeline_.get());
}

QRhiComputePipeline* shift::gui::Viewport::NewComputePipeline(
  const char* shader, QRhiShaderResourceBindings* bindings) {
  QRhiComputePipeline* pipeline = rhi_->newComputePipeline();
  pipeline->setShaderStage({QRhiShaderStage::Compute, LoadShader(shader)});
  pipeline->setShaderResourceBindings(bindings);
  CreatePipeline(pipeline);
  return pipeline;
}

void shift::gui::Viewport::CreateTextureArray(
  std::unique_ptr<QRhiTexture>& array, int layer_count, int size) {
  QRhiTexture::Flags flags;
  if(size > 1) {
    flags |= QRhiTexture::MipMapped | QRhiTexture::UsedWithGenerateMips;
  }
  array.reset(rhi_->newTextureArray(QRhiTexture::RGBA8, layer_count,
                                    QSize(size, size), 1, flags));
  array->create();
}

void shift::gui::Viewport::UploadEnvironment(QRhiResourceUpdateBatch* batch) {
  environment_ = simulation_ && simulation_->is_loaded()
                   ? simulation_->get_scene().environment
                   : SceneEnvironment();
  std::string error;
  const SkyMap map = BuildSkyMap(environment_, &error);
  if(!error.empty()) {
    qWarning("environment: sky: %s; drawn as a gradient sky", error.c_str());
  }
  const EnvironmentTables tables = ComputeEnvironmentTables(map);
  sky_zenith_ = tables.zenith;
  sky_horizon_ = tables.horizon;
  if(environment_.sky_type == SceneEnvironment::kPhysical &&
     !environment_.sun_color_given) {
    environment_.sun_color =
      SunlightColor(environment_.sun_direction, environment_.turbidity);
  }
  if(!environment_.fog_color_given) {
    environment_.fog_color = sky_horizon_;
  }
  sun_direction_ = environment_.sun_direction;
  auto upload = [&](QRhiTexture* texture, const std::vector<float>& values) {
    batch->uploadTexture(texture,
                         QRhiTextureUploadDescription(
                           {0, 0, QRhiTextureSubresourceUploadDescription(
                                    ToHalf(values))}));
  };
  upload(environment_radiance_.get(), tables.radiance);
  upload(environment_specular_.get(), tables.specular);
  upload(environment_irradiance_.get(), tables.irradiance);
  upload(environment_brdf_.get(), tables.brdf);
  environment_dirty_ = false;
  rt_frame_ = 0;
}

void shift::gui::Viewport::SunUniforms(bool lights_on, float direction[4],
                                       float color[4]) const {
  // without an environment, the lights of the model replace the sun
  const bool sun = environment_.given || !lights_on;
  CopyVector(sun_direction_, sun ? environment_.sun_intensity : 0.f,
             direction);
  CopyVector(sun ? environment_.sun_color : ksl_vec3f(0.f, 0.f, 0.f), 0.f,
             color);
}

void shift::gui::Viewport::FogUniforms(float params[4], float color[4]) const {
  const float fog[4] = {environment_.fog_density,
                        environment_.fog_height_falloff,
                        environment_.fog_height,
                        environment_.fog_density > 0.f ? 1.f : 0.f};
  CopyArray(fog, params);
  // the air of the physical sky: its haze, or none (-1)
  const bool air = environment_.given &&
                   environment_.sky_type == SceneEnvironment::kPhysical;
  CopyVector(environment_.fog_color,
             air ? AngstromBeta(environment_.turbidity) : -1.f, color);
}

void shift::gui::Viewport::WaterUniforms(float params[4], float wave[4],
                                         float color[4],
                                         float extinction[4]) const {
  const auto& e = environment_;
  const float p[4] = {e.water_height, e.wave_height, e.wavelength,
                      e.water ? 1.f : 0.f};
  CopyArray(p, params);
  const float w[4] = {std::cos(e.wave_direction), std::sin(e.wave_direction),
                      float(poses_.time), float(ocean_waves_.get_crest())};
  CopyArray(w, wave);
  CopyVector(e.water_color, 0.f, color);
  CopyVector(e.water_extinction, float(ocean_waves_.get_steepest()),
             extinction);
}

void shift::gui::Viewport::UploadWaves(QRhiResourceUpdateBatch* batch) {
  const auto& e = environment_;
  if(!e.water || !waves_texture_) {
    return;
  }
  const bool changed =
    ocean_waves_.Set(e.wave_height, e.wavelength, e.wave_direction,
                     e.wind_speed);
  if(!ocean_waves_.Update(poses_.time) && !changed && waves_uploaded_) {
    return;
  }
  QVector<QRhiTextureUploadEntry> entries;
  for(int c = 0; c < OceanWaves::kCascades; c++) {
    for(int l = 0; l < OceanWaves::get_level_count(); l++) {
      const auto& level = ocean_waves_.get_level(c, l);
      QByteArray half(qsizetype(level.size() * sizeof(qfloat16)),
                      Qt::Uninitialized);
      qFloatToFloat16(reinterpret_cast<qfloat16*>(half.data()), level.data(),
                      qsizetype(level.size()));
      entries.append(QRhiTextureUploadEntry(
        c, l, QRhiTextureSubresourceUploadDescription(half)));
    }
  }
  QRhiTextureUploadDescription description;
  description.setEntries(entries.cbegin(), entries.cend());
  batch->uploadTexture(waves_texture_.get(), description);
  waves_uploaded_ = true;
}

/* the instances of ray tracing and picking: the meshes' instances, then one
per body of each SceneCad, with mesh_index meshes.size() + its index; those
of the CADs not drawn left out */
static std::vector<shift::gui::SceneInstance>
RtInstanceList(const shift::gui::Scene& scene,
               const std::vector<bool>& cad_drawn) {
  std::vector<shift::gui::SceneInstance> list(scene.instances);
  for(size_t c = 0; c < scene.cads.size(); c++) {
    if(c < cad_drawn.size() && !cad_drawn[c]) {
      continue;
    }
    for(int body : scene.cads[c].bodies) {
      list.push_back({static_cast<int>(scene.meshes.size() + c), body});
    }
  }
  return list;
}

bool shift::gui::Viewport::is_cad_drawn(int c) const {
  const Scene& scene = simulation_->get_scene();
  if(c < 0 || size_t(c) >= scene.cads.size()) {
    return false;
  }
  if(scene.cads[size_t(c)].flex_mesh < 0) {
    return true;
  }
  for(const auto& gpu : gpu_cads_) {
    if(gpu.cad_index == c && gpu.flex >= 0) {
      return true;
    }
  }
  return false;
}

std::vector<int> shift::gui::Viewport::CadFlexMeshes() const {
  const Scene& scene = simulation_->get_scene();
  std::vector<int> cad_flex(scene.cads.size(), -1);
  for(const auto& gpu : gpu_cads_) {
    if(gpu.flex >= 0) {
      cad_flex[size_t(gpu.cad_index)] =
        scene.cads[size_t(gpu.cad_index)].flex_mesh;
    }
  }
  return cad_flex;
}

std::vector<shift::gui::SceneInstance>
shift::gui::Viewport::DrawnInstances() const {
  const Scene& scene = simulation_->get_scene();
  std::vector<bool> drawn(scene.cads.size(), false);
  for(size_t c = 0; c < drawn.size(); c++) {
    drawn[c] = is_cad_drawn(static_cast<int>(c));
  }
  return RtInstanceList(scene, drawn);
}

bool shift::gui::Viewport::is_instance_drawn(
  const SceneInstance& instance) const {
  if(!is_body_shown(instance.body_index)) {
    return false;
  }
  const Scene& scene = simulation_->get_scene();
  const size_t meshes = scene.meshes.size();
  const size_t m = size_t(instance.mesh_index);
  if(m < meshes) {
    // a flexible body's mesh, unless its CAD is drawn in its place
    return m >= gpu_meshes_.size() || !gpu_meshes_[m].has_cad ||
           is_flex_mesh_surface(instance.body_index);
  }
  // a flexible body's CAD, unless its mesh's surface is
  return m - meshes >= scene.cads.size() ||
         scene.cads[m - meshes].flex_mesh < 0 ||
         !is_flex_mesh_surface(instance.body_index);
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
  std::vector<shift::gui::RtDeformableTerrain>* deformables = nullptr,
  std::vector<shift::gui::SceneMeshBvh>* prebuilt = nullptr,
  std::vector<std::vector<int>>* lod_roots = nullptr,
  std::vector<shift::gui::RtFlexMesh>* flex_meshes = nullptr,
  const std::vector<int>* cad_flex = nullptr) {
  int last_node_count = 0;
  auto append = [&](std::vector<shift::gui::RtTriangle>& triangles,
                    std::vector<int32_t>& faces,
                    std::vector<shift::gui::BvhNode> nodes) {
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
  if(flex_meshes) {
    flex_meshes->clear();
  }
  auto next_motion = [&]() {
    return flex_meshes->empty()
             ? 0
             : flex_meshes->back().motion_base +
                 static_cast<int>(flex_meshes->back().order.size());
  };
  for(size_t m = 0; m < scene.meshes.size(); m++) {
    const auto& mesh = scene.meshes[m];
    const bool deformable =
      deformables && mesh.terrain >= 0 &&
      mesh.terrain < static_cast<int>(scene.terrains.size()) &&
      scene.terrains[mesh.terrain].deformable;
    const bool flex =
      flex_meshes && mesh.deformable && !mesh.indices.empty();
    // built while the model loaded (Simulation::TakeMeshBvhs), a
    // deformable terrain's and a flexible body's tagged, the terrain's as
    // it was then: ShapeRtTerrain fits it to the terrain as shown
    shift::gui::SceneMeshBvh bvh;
    if(prebuilt && m < prebuilt->size() &&
       (*prebuilt)[m].triangles.size() == mesh.indices.size() / 3 &&
       (!(deformable || flex) || (*prebuilt)[m].faces.empty() ||
        (*prebuilt)[m].faces[0] >= 0)) {
      bvh = std::move((*prebuilt)[m]);
    } else {
      bvh = shift::gui::BuildSceneMeshBvh(mesh, deformable || flex);
    }
    std::vector<shift::gui::RtTriangle>& triangles = bvh.triangles;
    std::vector<int32_t>& faces = bvh.faces;
    const int first_triangle = static_cast<int>(all_triangles.size());
    append(triangles, faces, std::move(bvh.nodes));
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
    if(flex) {
      shift::gui::RtFlexMesh f;
      f.mesh = static_cast<int>(m);
      f.first_triangle = first_triangle;
      f.first_node = roots.back();
      f.node_count = last_node_count;
      f.order.assign(faces.begin(), faces.end());
      f.motion_base = next_motion();
      for(size_t k = 0; k < faces.size(); k++) {
        all_faces[size_t(first_triangle) + k] =
          -2 - (f.motion_base + static_cast<int32_t>(k));
      }
      flex_meshes->push_back(std::move(f));
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
    // a flexible body's CAD, deformed: where the BVH puts each triangle
    const bool flex = flex_meshes && cad_flex && c < cad_flex->size() &&
                      (*cad_flex)[c] >= 0 && !triangles.empty();
    std::vector<int32_t> global_faces;
    if(flex) {
      global_faces = faces;
      for(size_t i = 0; i < faces.size(); i++) {
        faces[i] = static_cast<int32_t>(i);
      }
    }
    const int first_triangle = static_cast<int>(all_triangles.size());
    std::vector<shift::gui::BvhNode> nodes =
      shift::gui::BuildBvh(triangles, &faces);
    append(triangles, faces, std::move(nodes));
    if(flex) {
      shift::gui::RtFlexMesh f;
      f.mesh = (*cad_flex)[c];
      f.cad = static_cast<int>(c);
      f.first_triangle = first_triangle;
      f.first_node = roots.back();
      f.node_count = last_node_count;
      f.order.assign(faces.begin(), faces.end());
      f.motion_base = next_motion();
      for(size_t k = 0; k < faces.size(); k++) {
        all_faces[size_t(first_triangle) + k] =
          global_faces[size_t(faces[k])];
      }
      flex_meshes->push_back(std::move(f));
    }
  }
  // the levels of detail of each mesh (SceneMesh::lods), after all the
  // others so that roots stays one per mesh and SceneCad: lod_roots holds
  // the root of the mesh and of each of its levels
  if(lod_roots) {
    lod_roots->assign(scene.meshes.size(), {});
    for(size_t m = 0; m < scene.meshes.size(); m++) {
      const auto& mesh = scene.meshes[m];
      (*lod_roots)[m].push_back(roots[m]);
      for(const auto& lod : mesh.lods) {
        shift::gui::SceneMesh level;
        level.vertices = lod.vertices;
        level.indices = lod.indices;
        shift::gui::SceneMeshBvh bvh =
          shift::gui::BuildSceneMeshBvh(level, false);
        append(bvh.triangles, bvh.faces, std::move(bvh.nodes));
        (*lod_roots)[m].push_back(roots.back());
        roots.pop_back();
      }
    }
  }
}

/* the mesh whose material level (as LodLevel) of scene.meshes[mesh] is
drawn with: an impostor's, or the mesh's own */
static const shift::gui::SceneMesh& LodMaterial(const shift::gui::Scene& scene,
                                                int mesh, int level) {
  const shift::gui::SceneMesh& own = scene.meshes[size_t(mesh)];
  if(level > 0 && own.lods[size_t(level - 1)].material >= 0) {
    return scene.meshes[size_t(own.lods[size_t(level - 1)].material)];
  }
  return own;
}

/* the pixels across the screen that the bounding sphere of mesh drawn with
model spans, seen from eye; pixels_per_slope: the pixels across the screen
of a sphere of radius 1 at distance 1 */
static float LodPixels(const shift::gui::SceneMesh& mesh,
                       const ksl_SE3f_t& pose, float scale,
                       const ksl_vec3f_t& eye, float pixels_per_slope,
                       ksl_vec3f_t* world_center = nullptr,
                       float* world_radius = nullptr) {
  ksl_vec3f_t local, center, side, offset;
  ksl_add_vvf(&mesh.bounds_min, &mesh.bounds_max, &local);
  ksl_vec3f_scale(&local, 0.5f * scale);
  ksl_product_dvf(&pose, &local, &center);
  ksl_subtract_vvf(&mesh.bounds_max, &mesh.bounds_min, &side);
  const float radius = 0.5f * ksl_vec3f_l2norm(&side) * scale;
  ksl_subtract_vvf(&center, &eye, &offset);
  const float distance = ksl_vec3f_l2norm(&offset);
  if(world_center) {
    *world_center = center;
  }
  if(world_radius) {
    *world_radius = radius;
  }
  return pixels_per_slope * radius / std::max(distance, radius);
}

/* the coarsest level of detail of mesh (0 the mesh itself, k
SceneMeshLod k - 1) made for as few pixels as it spans, with its errors
allowed to span looseness times the pixels they were made for
(SceneMeshLod::kPixelError); 0 keeps the mesh */
static int LodLevel(const shift::gui::SceneMesh& mesh, float pixels,
                    float looseness) {
  int level = 0;
  for(size_t k = 0; k < mesh.lods.size(); k++) {
    if(pixels < looseness * mesh.lods[k].max_pixels) {
      level = int(k) + 1;
    }
  }
  return level;
}

/* distance along the ray o + t d to the box, if it is hit before t_max */
static bool RayHitsBox(const ksl_vec3f_t& o, const ksl_vec3f_t& inverse_d,
                       const float box_min[3], const float box_max[3],
                       float t_max) {
  float t_near = 0.f, t_far = t_max;
  for(int k = 0; k < 3; k++) {
    float t0 = (box_min[k] - o.at[k]) * inverse_d.at[k];
    float t1 = (box_max[k] - o.at[k]) * inverse_d.at[k];
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
static bool RayHitsTriangle(const ksl_vec3f_t& o, const ksl_vec3f_t& d,
                            const shift::gui::RtTriangle& triangle, float t_max,
                            float& t, float& b1_out, float& b2_out) {
  const ksl_vec3f_t v0 =
    ksl_vec3f(triangle.v0[0], triangle.v0[1], triangle.v0[2]);
  const ksl_vec3f_t v1 =
    ksl_vec3f(triangle.v1[0], triangle.v1[1], triangle.v1[2]);
  const ksl_vec3f_t v2 =
    ksl_vec3f(triangle.v2[0], triangle.v2[1], triangle.v2[2]);
  ksl_vec3f_t e1, e2, p, s, q;
  ksl_subtract_vvf(&v1, &v0, &e1);
  ksl_subtract_vvf(&v2, &v0, &e2);
  ksl_cross_vvf(&d, &e2, &p);
  const float det = ksl_dot_vvf(&e1, &p);
  if(std::fabs(det) < 1e-12f) {
    return false;
  }
  const float inverse_det = 1.f / det;
  ksl_subtract_vvf(&o, &v0, &s);
  const float b1 = ksl_dot_vvf(&s, &p) * inverse_det;
  if(b1 < 0.f || b1 > 1.f) {
    return false;
  }
  ksl_cross_vvf(&s, &e1, &q);
  const float b2 = ksl_dot_vvf(&d, &q) * inverse_det;
  if(b2 < 0.f || b1 + b2 > 1.f) {
    return false;
  }
  const float hit = ksl_dot_vvf(&e2, &q) * inverse_det;
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

void shift::gui::Viewport::UploadFlexUpdates(QRhiResourceUpdateBatch* batch,
                                             bool all) {
  simulation_->TakeFlexUpdates(flex_updates_, all);
  const Scene& scene = simulation_->get_scene();
  for(const auto& update : flex_updates_) {
    if(update.mesh < 0 || update.mesh >= static_cast<int>(gpu_meshes_.size()) ||
       update.mesh >= static_cast<int>(scene.meshes.size())) {
      continue;
    }
    GpuMesh& gpu = gpu_meshes_[update.mesh];
    const SceneMesh& mesh = scene.meshes[update.mesh];
    if(update.coefficients.size() != mesh.flex_columns) {
      continue;
    }
    gpu.flex_shown = update;
    gpu.flex_shown.peak_samples.clear(); // folded in below
    gpu.pick_stale = true;
    if(gpu.flex >= 0 && flex_gpu_) {
      flex_gpu_->Update(gpu.flex, mesh, update, batch);
      if(flex_gpu_->is_ray_traced(gpu.flex) || gpu.has_cad) {
        rt_instances_dirty_ = true; // the instance's box
        rt_frame_ = 0;
      }
      continue;
    }
    gpu.stress_maximum =
      DeformFlexMesh(mesh, update, flex_vertices_, &gpu.peak);
    const quint32 bytes = quint32(flex_vertices_.size() * sizeof(Vertex));
    if(gpu.vertices && bytes == gpu.vertices->size()) {
      batch->uploadStaticBuffer(gpu.vertices.get(), 0, bytes,
                                flex_vertices_.data());
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
      if(hardware_scene_) {
        hardware_->RebuildMesh(pick_triangles_, pick_nodes_,
                               terrain.first_node);
      }
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
    const ksl_vec3f_t local = ksl_vec3f(t.x_scale * (float(i) + a),
                                        t.y_scale * (float(j) + b), h);
    ksl_vec3f_t world;
    ksl_product_dvf(&t.pose, &local, &world);
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
    const ksl_vec3f_t n = ksl_vec3f(z_x / length, z_y / length, -1.f / length);
    ksl_vec3f_t world_normal;
    ksl_product_drvf(&t.pose.R, &n, &world_normal);
    for(int c = 0; c < 3; c++) {
      position[c] = world.at[c];
      normal[c] = world_normal.at[c];
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
  if(hardware_) {
    hardware_->ReleaseMeshes();
  }
  hardware_scene_ = false;
  rt_terrains_.clear();
  flex_gpu_.reset();
  gpu_meshes_.clear();
  rt_triangles_.clear();
  rt_triangle_faces_.clear();
  rt_nodes_.clear();
  rt_roots_.clear();
  rt_lod_roots_.clear();
  rt_instance_list_.clear();
  pick_triangles_.clear();
  pick_triangle_faces_.clear();
  pick_nodes_.clear();
  pick_roots_.clear();
  pick_cad_trims_ = CadTrimTables();
  pick_flex_.clear();
  pick_built_ = false;
  if(!simulation_) {
    return;
  }
  const Scene& scene = simulation_->get_scene();

  const bool compute = rhi_->isFeatureSupported(QRhi::Compute);
  flex_gpu_ = std::make_unique<FlexGpu>(rhi_.get());
  flex_gpu_->set_pipeline_creator(
    [this](QRhiComputePipeline* pipeline) { return CreatePipeline(pipeline); });
  for(const auto& mesh : scene.meshes) {
    GpuMesh gpu;
    const quint32 vertex_bytes = quint32(mesh.vertices.size() * sizeof(Vertex));
    const quint32 index_bytes = quint32(mesh.indices.size() * sizeof(uint32_t));
    // a deformable mesh's vertices are written again as it deforms, by the
    // compute shaders where the device has them, which read its indices
    const bool on_gpu = mesh.deformable && compute && !mesh.vertices.empty();
    QRhiBuffer::UsageFlags vertex_usage = QRhiBuffer::VertexBuffer;
    QRhiBuffer::UsageFlags index_usage = QRhiBuffer::IndexBuffer;
    if(on_gpu) {
      vertex_usage |= QRhiBuffer::StorageBuffer;
      index_usage |= QRhiBuffer::StorageBuffer;
    }
    gpu.vertices.reset(rhi_->newBuffer(
      mesh.deformable ? QRhiBuffer::Static : QRhiBuffer::Immutable,
      vertex_usage, vertex_bytes));
    gpu.vertices->create();
    gpu.indices.reset(
      rhi_->newBuffer(QRhiBuffer::Immutable, index_usage, index_bytes));
    gpu.indices->create();
    batch->uploadStaticBuffer(gpu.vertices.get(), mesh.vertices.data());
    batch->uploadStaticBuffer(gpu.indices.get(), mesh.indices.data());
    gpu.index_count = quint32(mesh.indices.size());
    if(on_gpu) {
      gpu.flex =
        flex_gpu_->Add(mesh, gpu.vertices.get(), gpu.indices.get(), batch);
    }
    if(!mesh.edge_indices.empty()) {
      gpu.edge_indices.reset(rhi_->newBuffer(
        QRhiBuffer::Immutable, QRhiBuffer::IndexBuffer,
        quint32(mesh.edge_indices.size() * sizeof(uint32_t))));
      gpu.edge_indices->create();
      batch->uploadStaticBuffer(gpu.edge_indices.get(),
                                mesh.edge_indices.data());
      gpu.edge_index_count = quint32(mesh.edge_indices.size());
    }
    if(!mesh.constrained_vertices.empty()) {
      gpu.point_indices.reset(rhi_->newBuffer(
        QRhiBuffer::Immutable, QRhiBuffer::IndexBuffer,
        quint32(mesh.constrained_vertices.size() * sizeof(uint32_t))));
      gpu.point_indices->create();
      batch->uploadStaticBuffer(gpu.point_indices.get(),
                                mesh.constrained_vertices.data());
      gpu.point_index_count = quint32(mesh.constrained_vertices.size());
    }
    if(!mesh.spider_indices.empty()) {
      gpu.spider_indices.reset(rhi_->newBuffer(
        QRhiBuffer::Immutable, QRhiBuffer::IndexBuffer,
        quint32(mesh.spider_indices.size() * sizeof(uint32_t))));
      gpu.spider_indices->create();
      batch->uploadStaticBuffer(gpu.spider_indices.get(),
                                mesh.spider_indices.data());
      gpu.spider_index_count = quint32(mesh.spider_indices.size());
    }
    for(const SceneMeshLod& lod : mesh.lods) {
      GpuMesh::Lod gpu_lod;
      gpu_lod.vertices.reset(rhi_->newBuffer(
        QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer,
        quint32(lod.vertices.size() * sizeof(Vertex))));
      gpu_lod.vertices->create();
      gpu_lod.indices.reset(rhi_->newBuffer(
        QRhiBuffer::Immutable, QRhiBuffer::IndexBuffer,
        quint32(lod.indices.size() * sizeof(uint32_t))));
      gpu_lod.indices->create();
      batch->uploadStaticBuffer(gpu_lod.vertices.get(), lod.vertices.data());
      batch->uploadStaticBuffer(gpu_lod.indices.get(), lod.indices.data());
      gpu_lod.index_count = quint32(lod.indices.size());
      gpu.lods.push_back(std::move(gpu_lod));
    }
    gpu.level_first.assign(mesh.lods.size() + 1, 0);
    gpu.level_count.assign(mesh.lods.size() + 1, 0);
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
  cad_rigid_grids_.reset();
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
    const CadGpuSampleGrid rigid{};
    cad_rigid_grids_ = storage(&rigid, sizeof(rigid));
    for(size_t c = 0; c < scene.cads.size(); c++) {
      const SceneCad& cad = scene.cads[c];
      if(cad.patches.empty() || cad.bodies.empty()) {
        continue;
      }
      // a flexible body's: only where its mesh deforms on the GPU
      if(cad.flex_mesh >= 0 &&
         (size_t(cad.flex_mesh) >= gpu_meshes_.size() ||
          gpu_meshes_[size_t(cad.flex_mesh)].flex < 0)) {
        continue;
      }
      GpuCad gpu;
      gpu.cad_index = static_cast<int>(c);
      if(cad.flex_mesh >= 0) {
        gpu.flex = flex_gpu_->AddCad(gpu_meshes_[size_t(cad.flex_mesh)].flex,
                                     cad, batch);
        if(gpu.flex < 0) {
          continue;
        }
        gpu_meshes_[size_t(cad.flex_mesh)].has_cad = true;
      }
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

  // texture layers, and the larger ones; without any, a placeholder layer
  // keeps the bindings valid
  auto upload_layers = [&](std::unique_ptr<QRhiTexture>& array,
                           const std::vector<QImage>& layers, int size) {
    if(layers.empty()) {
      CreateTextureArray(array, 1, 1);
      return;
    }
    CreateTextureArray(array, static_cast<int>(layers.size()), size);
    QVector<QRhiTextureUploadEntry> entries;
    for(size_t i = 0; i < layers.size(); i++) {
      entries.append(QRhiTextureUploadEntry(
        int(i), 0, QRhiTextureSubresourceUploadDescription(layers[i])));
    }
    QRhiTextureUploadDescription description;
    description.setEntries(entries.cbegin(), entries.cend());
    batch->uploadTexture(array.get(), description);
    batch->generateMips(array.get());
  };
  upload_layers(texture_array_, scene.texture_layers, Scene::kTextureSize);
  upload_layers(large_texture_array_, scene.large_texture_layers,
                Scene::kLargeTextureSize);

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

  // the material maps of the terrains
  {
    const QByteArray ground = GroundContents(scene);
    ground_buffer_.reset(rhi_->newBuffer(QRhiBuffer::Immutable,
                                         QRhiBuffer::StorageBuffer,
                                         quint32(ground.size())));
    ground_buffer_->create();
    batch->uploadStaticBuffer(ground_buffer_.get(), ground.constData());
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

  rt_instance_list_ = DrawnInstances();
  if(ray_tracing_supported_) {
    CadTrimTables trims = MergeCadTrims(scene.cads);
    // the meshes' hierarchies built while the model loaded, once
    std::vector<SceneMeshBvh> prebuilt = simulation_->TakeMeshBvhs();
    std::vector<RtFlexMesh>& rt_flex = pick_flex_;
    const std::vector<int> cad_flex = CadFlexMeshes();
    BuildMeshBvhs(scene, cad_triangles_, cad_triangle_faces_, trims,
                  rt_triangles_, rt_triangle_faces_, rt_nodes_, rt_roots_,
                  &rt_terrains_, &prebuilt, &rt_lod_roots_, &rt_flex,
                  &cad_flex);
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
    size_t motion_triangles = 0;
    for(const auto& flex : rt_flex) {
      motion_triangles += flex.order.size();
    }
    rt_triangle_motion_buffer_.reset(rhi_->newBuffer(
      QRhiBuffer::Static, QRhiBuffer::StorageBuffer,
      quint32(std::max<size_t>(3 * motion_triangles * 4 * sizeof(float), 16))));
    rt_triangle_motion_buffer_->create();
    if(motion_triangles > 0) {
      const std::vector<float> zero(12 * motion_triangles, 0.f);
      batch->uploadStaticBuffer(rt_triangle_motion_buffer_.get(), 0,
                                quint32(zero.size() * sizeof(float)),
                                zero.data());
    }
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
    for(const auto& flex : rt_flex) {
      if(flex.cad < 0) {
        flex_gpu_->AddRayTracing(gpu_meshes_[size_t(flex.mesh)].flex, flex,
                                 rt_nodes_, rt_triangle_buffer_.get(),
                                 rt_node_buffer_.get(),
                                 rt_triangle_motion_buffer_.get(), batch);
        continue;
      }
      for(const auto& gpu : gpu_cads_) {
        if(gpu.cad_index != flex.cad || gpu.flex < 0) {
          continue;
        }
        FlexGpu::CadSource source;
        source.cad = &scene.cads[size_t(flex.cad)];
        source.triangles = &cad_triangles_[size_t(flex.cad)];
        source.faces = gpu.faces.get();
        source.curves = gpu.curves.get();
        source.patches = gpu.patches.get();
        source.points = gpu.points.get();
        source.knots = gpu.knots.get();
        flex_gpu_->AddRayTracing(gpu.flex, flex, rt_nodes_,
                                 rt_triangle_buffer_.get(),
                                 rt_node_buffer_.get(),
                                 rt_triangle_motion_buffer_.get(), batch,
                                 &source);
      }
    }
    // the GPU copies are all that ray tracing needs from here on; picking
    // keeps the CPU copies
    pick_triangles_ = std::move(rt_triangles_);
    pick_triangle_faces_ = std::move(rt_triangle_faces_);
    pick_nodes_ = std::move(rt_nodes_);
    pick_roots_ = rt_roots_;
    pick_cad_trims_ = std::move(trims);
    pick_built_ = true;
    // the acceleration structures of the roots: of the meshes and CAD
    // geometry, and of the meshes' levels of detail; not with flexible
    // bodies, whose triangles deform on the device (FlexGpu), which the
    // structures built here would not follow
    if(hardware_ && pick_flex_.empty()) {
      // with the alpha textures that cut-out cards test their hits against
      // (AcceptHit in raytrace.comp), at the resolution of the small
      // layers, which the large ones are read at by a level of detail
      std::map<int, std::unique_ptr<AlphaCoverage>> alpha_of_layer;
      auto coverage = [&](const SceneMesh& material,
                          float cutoff) -> const AlphaCoverage* {
        const int layer = material.base_color_layer;
        if(cutoff <= 0.f || layer < 0) {
          return nullptr;
        }
        auto found = alpha_of_layer.find(layer);
        if(found == alpha_of_layer.end()) {
          const QImage* image = nullptr;
          if(layer >= Scene::kLargeLayer) {
            const size_t k = size_t(layer - Scene::kLargeLayer);
            image = k < scene.large_texture_layers.size()
                      ? &scene.large_texture_layers[k]
                      : nullptr;
          } else if(size_t(layer) < scene.texture_layers.size()) {
            image = &scene.texture_layers[size_t(layer)];
          }
          std::unique_ptr<AlphaCoverage> alpha;
          if(image && image->format() == QImage::Format_RGBA8888 &&
             image->width() == image->height() &&
             image->width() % Scene::kTextureSize == 0 &&
             image->bytesPerLine() == 4 * image->width()) {
            const std::vector<uint8_t> texels = AlphaCoverage::Downsample(
              image->constBits(), image->width(), Scene::kTextureSize);
            alpha = std::make_unique<AlphaCoverage>(texels.data(),
                                                    Scene::kTextureSize);
          }
          found = alpha_of_layer.emplace(layer, std::move(alpha)).first;
        }
        return found->second.get();
      };
      std::vector<int> roots = rt_roots_;
      std::vector<const AlphaCoverage*> coverages(roots.size(), nullptr);
      for(size_t m = 0; m < scene.meshes.size() && m < rt_roots_.size();
          m++) {
        coverages[m] = coverage(scene.meshes[m], scene.meshes[m].alpha_cutoff);
      }
      for(size_t m = 0; m < rt_lod_roots_.size(); m++) {
        const std::vector<int>& levels = rt_lod_roots_[m];
        for(size_t level = 0; level < levels.size(); level++) {
          roots.push_back(levels[level]);
          coverages.push_back(
            coverage(LodMaterial(scene, int(m), int(level)),
                     scene.meshes[m].alpha_cutoff));
        }
      }
      hardware_scene_ = hardware_->BuildMeshes(pick_triangles_, pick_nodes_,
                                               roots, coverages);
    }
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
  if(scene_dirty_) {
    environment_dirty_ = true; // the model's environment
  }
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
  if(!pending_pipelines_.empty()) {
    // pipelines queued (CreatePipeline): nothing drawn with them yet; the
    // uploads go with an empty pass, and the frame after they are made
    // draws
    cb->beginPass(get_render_target(), QColor(0, 0, 0), {1.0f, 0}, batch);
    cb->endPass();
    return;
  }
  if(simulation_) {
    // all of them after the scene was uploaded again
    UploadParticleUpdates(batch, gpu_particles_reset_);
    UploadFlexUpdates(batch, gpu_particles_reset_);
    gpu_particles_reset_ = false;
    if(flex_gpu_) {
      batch = flex_gpu_->Deform(
        cb, batch,
        render_mode_ == RenderMode::kRayTraced && ray_tracing_supported_);
    }
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
  const QMatrix4x4 view_projection =
    rhi_->clipSpaceCorrMatrix() * camera_.get_projection_matrix(aspect) *
    ToQMatrix4x4(camera_.get_view_matrix());

  const bool lights_on = UpdateLights(batch);
  FrameUniforms frame{};
  CopyMatrix(view_projection, frame.view_projection);
  CopyMatrix(view_projection.inverted(), frame.inverse_view_projection);
  CopyVector(camera_.get_position(), 1.f, frame.camera_position);
  SunUniforms(lights_on, frame.sun_direction, frame.sun_color);
  CopyVector(sky_zenith_, exposure_, frame.sky_zenith);
  CopyVector(sky_horizon_, ToneCurveValue(tone_curve_), frame.sky_horizon);
  FogUniforms(frame.fog_params, frame.fog_color);
  UploadWaves(batch);
  WaterUniforms(frame.water_params, frame.water_wave, frame.water_color,
                frame.water_extinction);
  const float floor_params[4] = {0.f, floor_spacing_,
                                 floor_enabled_ ? 1.f : 0.f, floor_extent_};
  CopyArray(floor_params, frame.floor_params);
  CopyFloorFrame(frame.floor_frame);
  for(size_t k = 0; k < color_map_.size() && k < size_t(kColorMapSamples);
      k++) {
    CopyVector(color_map_[k], 1.f, frame.color_map[k]);
  }
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
      // a flexible body's stress drawn: colored by its uv[0]
      // (FlexUpdate::stress)
      if(mesh.deformable && i < gpu_meshes_.size() &&
         gpu_meshes_[i].flex_shown.stress != FlexUpdate::Stress::kNone) {
        material.flex[0] = 1.f;
      }
      CopyEmission(scene, mesh, poses_, material.emission);
      const int holes = std::min<int>(int(mesh.holes.size()), 4);
      material.layers[3] = holes;
      for(int k = 0; k < holes; k++) {
        std::copy(mesh.holes[k].begin(), mesh.holes[k].end(),
                  material.holes[k]);
      }
      batch->updateDynamicBuffer(material_ubo_.get(),
                                 quint32(i) * material_stride_,
                                 sizeof(material), &material);
    }
    // the planes of the view frustum, a x + b y + c z + d >= 0 inside, of
    // unit normal (Gribb and Hartmann): placed instances outside it, or
    // beyond their mesh's draw distance, are not drawn
    QVector4D planes[6];
    for(int k = 0; k < 3; k++) {
      planes[2 * k] = view_projection.row(3) + view_projection.row(k);
      planes[2 * k + 1] = view_projection.row(3) - view_projection.row(k);
    }
    for(auto& plane : planes) {
      plane /= std::max(plane.toVector3D().length(), 1e-20f);
    }
    const ksl_vec3f_t eye = camera_.get_position();
    // the pixels across the screen of a sphere of radius 1 at distance 1
    const float pixels_per_slope =
      camera_.get_projection_matrix(aspect)(1, 1) * float(size.height());
    // each instance's level of detail (0 the mesh itself), -1 if not drawn
    std::vector<int> levels(scene.instances.size(), 0);
    // each instance's mesh in the world, without its scale
    std::vector<ksl_SE3f_t> models(scene.instances.size());
    for(auto& gpu : gpu_meshes_) {
      std::fill(gpu.level_count.begin(), gpu.level_count.end(), 0u);
    }
    for(size_t i = 0; i < scene.instances.size(); i++) {
      const SceneInstance& instance = scene.instances[i];
      const SceneMesh& mesh = scene.meshes[instance.mesh_index];
      GpuMesh& gpu = gpu_meshes_[instance.mesh_index];
      const int body = instance.body_index;
      models[i] = Placed(poses_.bodies[body], instance);
      const bool placed = instance.is_placed();
      if(placed || !mesh.lods.empty()) {
        ksl_vec3f_t center;
        float radius;
        const float pixels = LodPixels(mesh, models[i], instance.scale, eye,
                                       pixels_per_slope, &center, &radius);
        ksl_vec3f_t offset;
        ksl_subtract_vvf(&center, &eye, &offset);
        const float distance = ksl_vec3f_l2norm(&offset);
        if(placed) {
          bool shown = is_body_shown(body);
          if(mesh.draw_distance > 0.f) {
            shown = shown && distance - radius < mesh.draw_distance;
          }
          for(const auto& plane : planes) {
            shown = shown && plane.x() * center.x + plane.y() * center.y +
                                 plane.z() * center.z + plane.w() >=
                               -radius;
          }
          if(!shown) {
            levels[i] = -1;
            continue;
          }
        }
        levels[i] =
          LodLevel(mesh, pixels, lod_tolerance_ / SceneMeshLod::kPixelError);
      }
      gpu.level_count[size_t(levels[i])]++;
    }
    // the drawn instances first in each mesh's range, by level; a hidden
    // body's meshes collapse to a point, and draw nothing
    for(auto& gpu : gpu_meshes_) {
      uint32_t first = gpu.first_instance;
      gpu.drawn_count = 0;
      for(size_t l = 0; l < gpu.level_count.size(); l++) {
        gpu.level_first[l] = first;
        first += gpu.level_count[l];
        gpu.drawn_count += gpu.level_count[l];
      }
      std::fill(gpu.level_count.begin(), gpu.level_count.end(), 0u);
    }
    std::vector<InstanceAttributes> instances(scene.instances.size());
    for(size_t i = 0; i < scene.instances.size(); i++) {
      if(levels[i] < 0) {
        continue;
      }
      const SceneInstance& instance = scene.instances[i];
      GpuMesh& gpu = gpu_meshes_[instance.mesh_index];
      const size_t l = size_t(levels[i]);
      InstanceAttributes& drawn =
        instances[gpu.level_first[l] + gpu.level_count[l]++];
      if(is_body_shown(instance.body_index)) {
        CopyPlacedMatrix(poses_.bodies[instance.body_index], instance,
                         drawn.model);
      } else {
        std::fill(drawn.model, drawn.model + 16, 0.f);
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
      // a flexible body's with its stress drawn: colored by it (cad.tese)
      if(gpu.flex >= 0 && cad.flex_mesh >= 0 &&
         size_t(cad.flex_mesh) < gpu_meshes_.size() &&
         gpu_meshes_[size_t(cad.flex_mesh)].flex_shown.stress !=
           FlexUpdate::Stress::kNone) {
        material.flex[0] = 1.f;
      }
      batch->updateDynamicBuffer(
        material_ubo_.get(),
        quint32(scene.meshes.size() + gpu.cad_index) * material_stride_,
        sizeof(material), &material);
      std::vector<InstanceAttributes> instances(cad.bodies.size());
      for(size_t i = 0; i < instances.size(); i++) {
        // the pose, its rotation scaled by the CAD's scale (zero: hidden,
        // or a flexible body drawn by its mesh's surface)
        const bool shown =
          is_body_shown(cad.bodies[i]) &&
          (cad.flex_mesh < 0 || !is_flex_mesh_surface(cad.bodies[i]));
        const float scale = shown ? cad.scale : 0.f;
        CopyMatrix(poses_.bodies[cad.bodies[i]], instances[i].model);
        for(int c = 0; c < 3; c++) {
          for(int r = 0; r < 3; r++) {
            instances[i].model[4 * c + r] *= scale;
          }
        }
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

  // one instanced draw per mesh, either those that hide what is behind
  // them or those that let light through (Scene::meshes opacity below 1)
  auto draw_meshes = [&](bool transparent) {
    cb->setGraphicsPipeline(transparent ? transparent_pipeline_.get()
                                        : mesh_pipeline_.get());
    cb->setViewport(viewport);
    const Scene& scene = simulation_->get_scene();
    for(size_t i = 0; i < gpu_meshes_.size(); i++) {
      const GpuMesh& gpu = gpu_meshes_[i];
      if(gpu.drawn_count == 0 ||
         (tessellate && scene.meshes[i].terrain >= 0) ||
         (scene.meshes[i].opacity < 1.f) != transparent) {
        continue; // terrains are tessellated below
      }
      if(gpu.has_cad && cad_pipeline_ &&
         !is_flex_mesh_surface(
           scene.instances[gpu.first_instance].body_index)) {
        continue; // a flexible body's CAD is drawn below, deformed
      }
      // the mesh, then each of its levels of detail, for its instances,
      // each with its material (an impostor's its own)
      int bound = -1;
      for(size_t l = 0; l < gpu.level_count.size(); l++) {
        if(gpu.level_count[l] == 0) {
          continue;
        }
        const int material =
          l == 0 || scene.meshes[i].lods[l - 1].material < 0
            ? int(i)
            : scene.meshes[i].lods[l - 1].material;
        if(material != bound) {
          const QRhiCommandBuffer::DynamicOffset offset(
            1, quint32(material) * material_stride_);
          cb->setShaderResources(raster_bindings_.get(), 1, &offset);
          bound = material;
        }
        QRhiBuffer* vertices =
          l == 0 ? gpu.vertices.get() : gpu.lods[l - 1].vertices.get();
        QRhiBuffer* indices =
          l == 0 ? gpu.indices.get() : gpu.lods[l - 1].indices.get();
        const quint32 index_count =
          l == 0 ? gpu.index_count : gpu.lods[l - 1].index_count;
        const QRhiCommandBuffer::VertexInput inputs[2] = {
          {vertices, 0},
          {instance_buffer_.get(),
           gpu.level_first[l] * quint32(sizeof(InstanceAttributes))}};
        cb->setVertexInput(0, 2, inputs, indices, 0,
                           QRhiCommandBuffer::IndexUInt32);
        cb->drawIndexed(index_count, gpu.level_count[l]);
      }
    }
  };
  if(have_poses) {
    draw_meshes(false);
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

  // the edges of flexible bodies' elements, and their constrained nodes,
  // over the surface drawn: indices into the meshes' vertices as deformed
  auto draw_over = [&](QRhiGraphicsPipeline* pipeline,
                       std::unique_ptr<QRhiBuffer> GpuMesh::*indices,
                       uint32_t GpuMesh::*count,
                       const std::vector<bool>& bodies) {
    if(!have_poses || !pipeline) {
      return;
    }
    const Scene& scene = simulation_->get_scene();
    bool bound = false;
    for(size_t i = 0; i < gpu_meshes_.size(); i++) {
      const GpuMesh& gpu = gpu_meshes_[i];
      if(gpu.*count == 0 || gpu.instance_count == 0) {
        continue;
      }
      const int body = scene.instances[gpu.first_instance].body_index;
      if(body >= static_cast<int>(bodies.size()) || !bodies[size_t(body)]) {
        continue;
      }
      if(!bound) {
        cb->setGraphicsPipeline(pipeline);
        cb->setViewport(viewport);
        bound = true;
      }
      const QRhiCommandBuffer::DynamicOffset offset(
        1, quint32(i) * material_stride_);
      cb->setShaderResources(raster_bindings_.get(), 1, &offset);
      const QRhiCommandBuffer::VertexInput inputs[2] = {
        {gpu.vertices.get(), 0},
        {instance_buffer_.get(),
         gpu.first_instance * quint32(sizeof(InstanceAttributes))}};
      cb->setVertexInput(0, 2, inputs, (gpu.*indices).get(), 0,
                         QRhiCommandBuffer::IndexUInt32);
      cb->drawIndexed(gpu.*count, gpu.instance_count);
    }
  };
  draw_over(edges_pipeline_.get(), &GpuMesh::edge_indices,
            &GpuMesh::edge_index_count, flex_mesh_edges_);
  draw_over(spider_pipeline_.get(), &GpuMesh::spider_indices,
            &GpuMesh::spider_index_count, flex_constrained_nodes_);
  draw_over(points_pipeline_.get(), &GpuMesh::point_indices,
            &GpuMesh::point_index_count, flex_constrained_nodes_);

  if(floor_enabled_ && have_poses) {
    cb->setGraphicsPipeline(floor_pipeline_.get());
    cb->setViewport(viewport);
    cb->setShaderResources(raster_bindings_.get(), 1, &zero_offset);
    cb->draw(6);
  }
  if(have_poses) {
    draw_meshes(true);
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
  CopyVector(Normalized(camera_.get_forward()), 0.f, u.camera_forward);
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
  // the levels of detail of camera rays follow the camera; shadow and
  // bounce rays choose their own (raytrace.comp, Trace)
  const ksl_vec3f_t eye = camera_.get_position();
  const float pixels_per_slope =
    camera_.get_projection_matrix(1.f)(1, 1) *
    float(get_render_target()->pixelSize().height());
  std::vector<RtInstance> instances(std::max(instance_count, 1));
  const int mesh_count = static_cast<int>(scene.meshes.size());
  for(int i = 0; i < instance_count; i++) {
    const SceneInstance& instance = rt_instance_list_[i];
    const ksl_SE3f_t pose =
      Placed(poses_.bodies[instance.body_index], instance);
    const ksl_SE3f_t& previous =
      rt_previous_poses_.empty() ? pose : rt_previous_poses_[i];
    moved = moved || !std::equal(std::begin(previous.as_array),
                                 std::end(previous.as_array),
                                 std::begin(pose.as_array));
    // scenery is scaled before it is placed
    const float scale = instance.scale;
    CopyScaledMatrix(pose, scale, instances[i].body_to_world);
    CopyInverseScaledMatrix(pose, scale, instances[i].world_to_body);
    CopyScaledMatrix(previous, scale, instances[i].previous_body_to_world);
    if(instance.mesh_index < mesh_count) {
      const SceneMesh& mesh = scene.meshes[instance.mesh_index];
      ksl_vec3f_t bounds_min = mesh.bounds_min, bounds_max = mesh.bounds_max;
      // a flexible body's: a box that holds its deformation
      if(size_t(instance.mesh_index) < gpu_meshes_.size()) {
        const GpuMesh& gpu = gpu_meshes_[size_t(instance.mesh_index)];
        if(flex_gpu_ && flex_gpu_->is_ray_traced(gpu.flex)) {
          flex_gpu_->get_bounds(gpu.flex, bounds_min, bounds_max);
        }
      }
      ksl_vec3f_scale(&bounds_min, scale);
      ksl_vec3f_scale(&bounds_max, scale);
      for(const auto& terrain : rt_terrains_) {
        // a deformable terrain's box is that of its refitted BVH
        if(terrain.mesh == instance.mesh_index &&
           terrain.first_node < static_cast<int>(pick_nodes_.size())) {
          const BvhNode& root = pick_nodes_[terrain.first_node];
          bounds_min =
            ksl_vec3f(root.box_min[0], root.box_min[1], root.box_min[2]);
          bounds_max =
            ksl_vec3f(root.box_max[0], root.box_max[1], root.box_max[2]);
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
      if(mesh.deformable &&
         size_t(instance.mesh_index) < gpu_meshes_.size() &&
         gpu_meshes_[size_t(instance.mesh_index)].flex_shown.stress !=
           FlexUpdate::Stress::kNone) {
        instances[i].flex[0] = 1.f;
      }
    } else {
      // CAD geometry: its triangles are scaled, its faces may have colors
      const int c = instance.mesh_index - mesh_count;
      const SceneCad& cad = scene.cads[size_t(c)];
      ksl_vec3f_t bounds_min = cad.bounds_min, bounds_max = cad.bounds_max;
      // a flexible body's: a box that holds its deformation
      int flex = -1;
      for(const auto& gpu : gpu_cads_) {
        if(gpu.cad_index == c && gpu.flex >= 0) {
          flex = gpu.flex;
          flex_gpu_->get_bounds(flex, bounds_min, bounds_max);
        }
      }
      TransformBox(pose, bounds_min, bounds_max, instances[i].box_min,
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
      // its triangles' motion
      for(const auto& rt : pick_flex_) {
        if(flex >= 0 && rt.cad == c && flex_gpu_->is_ray_traced(flex)) {
          instances[i].flex[1] = float(rt.motion_base - rt.first_triangle);
          instances[i].flex[2] = 1.f;
          // its stress drawn: colored by it (flex_cad.comp)
          if(size_t(cad.flex_mesh) < gpu_meshes_.size() &&
             gpu_meshes_[size_t(cad.flex_mesh)].flex_shown.stress !=
               FlexUpdate::Stress::kNone) {
            instances[i].flex[0] = 1.f;
          }
        }
      }
    }
    instances[i].offsets[0] = rt_roots_[instance.mesh_index];
    instances[i].lods[0] = 1;
    for(int k = 1; k < 4; k++) {
      instances[i].lods[k] = instances[i].offsets[k];
    }
    for(int k = 0; k < 4; k++) {
      instances[i].lod_roots[k] = instances[i].offsets[0];
      instances[i].lod_errors[k] = 0.f;
    }
    if(size_t(instance.mesh_index) < rt_lod_roots_.size()) {
      const SceneMesh& mesh = scene.meshes[instance.mesh_index];
      const std::vector<int>& roots = rt_lod_roots_[instance.mesh_index];
      // camera rays: the level the rasterizer draws, with its textures (an
      // impostor's its own)
      const float pixels =
        LodPixels(mesh, pose, scale, eye, pixels_per_slope);
      const int level = LodLevel(mesh, pixels,
                                 lod_tolerance_ / SceneMeshLod::kPixelError);
      instances[i].offsets[0] = roots[size_t(level)];
      float unused[4];
      CopyMaterial(LodMaterial(scene, instance.mesh_index, level), unused,
                   instances[i].offsets + 1);
      // shadow and bounce rays: the mesh and its simplified levels (not
      // the impostors, which show it to the camera only), with their errors
      // in the world
      ksl_vec3f_t sides;
      ksl_subtract_vvf(&mesh.bounds_max, &mesh.bounds_min, &sides);
      const float extent = std::max({sides.x, sides.y, sides.z}) * scale;
      int count = 1;
      for(size_t k = 0; k < mesh.lods.size() && count < 4; k++) {
        if(mesh.lods[k].material >= 0) {
          continue;
        }
        instances[i].lod_roots[count] = roots[k + 1];
        instances[i].lod_errors[count] = mesh.lods[k].error * extent;
        count++;
      }
      instances[i].lods[0] = count;
    }
    if(!is_instance_drawn(instance)) {
      // far beyond anything a ray reaches: its triangles are never tested
      for(int k = 0; k < 3; k++) {
        instances[i].box_min[k] = instances[i].box_max[k] = 1e30f;
      }
    }
  }
  batch->uploadStaticBuffer(rt_instance_buffer_.get(), 0,
                            quint32(instances.size() * sizeof(RtInstance)),
                            instances.data());
  if(hardware_scene_) {
    // the camera's level and the mesh itself; opaque where AcceptHit in
    // raytrace.comp would accept every hit
    std::vector<HardwareInstance> hardware;
    hardware.reserve(size_t(instance_count));
    for(int i = 0; i < instance_count; i++) {
      if(!is_instance_drawn(rt_instance_list_[i])) {
        continue;
      }
      const RtInstance& r = instances[i];
      const bool cad = rt_instance_list_[i].mesh_index >= mesh_count;
      const bool solid = r.optical[0] >= 1.f && !cad;
      const bool cut = r.variation[2] > 0.f;
      HardwareInstance h;
      std::copy(r.body_to_world, r.body_to_world + 16, h.body_to_world);
      h.index = i;
      h.camera_root = r.offsets[0];
      h.full_root = r.lod_roots[0];
      h.camera_opaque = solid && !(cut && r.offsets[1] >= 0);
      h.full_opaque = solid && !(cut && r.lods[1] >= 0);
      hardware.push_back(h);
    }
    hardware_->SetInstances(hardware);
  }
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
    rt_previous_poses_[i] =
      Placed(poses_.bodies[rt_instance_list_[i].body_index],
             rt_instance_list_[i]);
  }
  return moved;
}

void shift::gui::Viewport::RenderRayTraced(QRhiCommandBuffer* cb,
                                           QRhiResourceUpdateBatch* batch) {
  // the ray tracer's images, which the anti-aliasing pass scales up to the
  // target's
  const QSize size = rt_size_;
  const QSize target_size = get_render_target()->pixelSize();
  const float aspect =
    float(target_size.width()) / float(std::max(target_size.height(), 1));
  const ksl_SE3f_t view = camera_.get_view_matrix();
  const QMatrix4x4 projection =
    rhi_->clipSpaceCorrMatrix() * camera_.get_projection_matrix(aspect);
  const QMatrix4x4 view_projection = projection * ToQMatrix4x4(view);

  // restart the progressive mean whenever anything in view changes; the
  // denoiser's history survives through reprojection
  const bool camera_moved = view_projection != rt_last_view_projection_;
  if(camera_moved) {
    rt_last_view_projection_ = view_projection;
    rt_frame_ = 0;
    rt_instances_dirty_ = true; // the levels of detail follow the camera
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
    ksl_SE3f_t inverse_view;
    ksl_SE3f_inverted(&view, &inverse_view);
    CopyMatrix(inverse_view, uniforms.inverse_view);
    CopyMatrix(projection.inverted(), uniforms.inverse_projection);
    CopyVector(camera_.get_position(), 1.f, uniforms.camera_position);
    const bool lights_on = UpdateLights(batch);
    SunUniforms(lights_on, uniforms.sun_direction, uniforms.sun_color);
    CopyVector(sky_zenith_, 0.f, uniforms.sky_zenith);
    CopyVector(sky_horizon_, 0.f, uniforms.sky_horizon);
    FogUniforms(uniforms.fog_params, uniforms.fog_color);
    UploadWaves(batch);
    WaterUniforms(uniforms.water_params, uniforms.water_wave,
                  uniforms.water_color, uniforms.water_extinction);
    const float floor_params[4] = {0.f, floor_spacing_,
                                   (floor_enabled_ && have_poses) ? 1.f : 0.f,
                                   floor_extent_};
    CopyArray(floor_params, uniforms.floor_params);
    CopyFloorFrame(uniforms.floor_frame);
    for(size_t k = 0; k < color_map_.size() && k < size_t(kColorMapSamples);
        k++) {
      CopyVector(color_map_[k], 1.f, uniforms.color_map[k]);
    }
    // angle subtended by a pixel, to scale the denoiser's depth tolerances
    const float spread =
      2.f * std::tan(0.5f * camera_.get_fov_y() * float(M_PI) / 180.f) /
      float(std::max(size.height(), 1));
    // and by a pixel of the output, to select texture levels of detail:
    // scaled up (set_render_scale), the anti-aliasing pass recovers the
    // detail of the output's pixels from the jittered samples, as the
    // meshes' levels of detail are chosen for them (UploadInstances)
    const float output_spread =
      spread * float(size.height()) / float(std::max(target_size.height(), 1));
    const float texture_params[4] = {
      output_spread, float(texture_array_->pixelSize().width()),
      std::log2(float(large_texture_array_->pixelSize().width()) /
                float(texture_array_->pixelSize().width())),
      lod_tolerance_};
    CopyArray(texture_params, uniforms.texture_params);
    uniforms.control[0] = instance_count;
    uniforms.control[1] = rt_sample_;
    uniforms.control[2] = rt_frame_;
    const bool profile = hardware_scene_ && hardware_->is_profiling();
    int stamp = 0;
    if(hardware_scene_) {
      // the instances' acceleration structure, built ahead of the passes
      cb->beginExternal();
      const auto* native =
        static_cast<const QRhiVulkanCommandBufferNativeHandles*>(
          cb->nativeHandles());
      if(profile) {
        hardware_->Collect(rhi_->currentFrameSlot());
        // in the viewer, every so often too
        if(!is_offscreen() && ++profile_frames_ % 300 == 0) {
          PrintGpuProfile();
        }
        hardware_->Stamp(native->commandBuffer, rhi_->currentFrameSlot(),
                         stamp++);
      }
      const uint64_t address = hardware_->RecordTopLevel(
        native->commandBuffer, rhi_->currentFrameSlot(),
        rhi_->resourceLimit(QRhi::FramesInFlight));
      if(profile) {
        hardware_->Stamp(native->commandBuffer, rhi_->currentFrameSlot(),
                         stamp++);
      }
      cb->endExternal();
      uniforms.hardware[0] = uint32_t(address);
      uniforms.hardware[1] = uint32_t(address >> 32);
      uniforms.hardware[2] = address != 0 ? 1u : 0u;
    }
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
    // surfaces of bodies that did not move keep the history of a moving
    // camera while others move, so that a still view of a running
    // simulation settles where nothing moves
    denoise.blend[3] = bodies_moved ? kHistoryCamera : denoise.params[1];
    denoise.control[0] = instance_count;
    denoise.tone[0] = exposure_;
    denoise.tone[1] = ToneCurveValue(tone_curve_);
    // the jitter of this frame's primary rays (raytrace.comp), from which
    // the anti-aliasing pass places the samples in its larger image
    const uint32_t halton_index = uint32_t(rt_sample_) + 1u;
    denoise.upscale[0] = Halton(halton_index, 2u) - 0.5f;
    denoise.upscale[1] = Halton(halton_index, 3u) - 0.5f;
    denoise.upscale[2] = size == output_size_ ? 1.f
                                              : float(size.width()) /
                                                  float(output_size_.width());
    denoise.upscale[3] = size == output_size_
                           ? 1.f
                           : float(size.height()) /
                               float(output_size_.height());
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
                        const QRhiCommandBuffer::DynamicOffset* offset,
                        bool output_size = false) {
      cb->beginComputePass(batch);
      batch = nullptr;
      cb->setComputePipeline(pipeline);
      cb->setShaderResources(bindings, offset ? 1 : 0, offset);
      if(output_size) {
        cb->dispatch((output_size_.width() + 7) / 8,
                     (output_size_.height() + 7) / 8, 1);
      } else {
        cb->dispatch(groups_x, groups_y, 1);
      }
      cb->endComputePass();
      if(profile) {
        // SHIFT_GPU_PROFILE: the GPU time of each pass
        cb->beginExternal();
        hardware_->Stamp(
          static_cast<const QRhiVulkanCommandBufferNativeHandles*>(
            cb->nativeHandles())
            ->commandBuffer,
          rhi_->currentFrameSlot(), stamp++);
        cb->endExternal();
      }
    };
    const int p = rt_parity_;
    dispatch(rt_pipeline_.get(), rt_bindings_[p].get(), nullptr);
    dispatch(sun_shadow_pipeline_.get(), sun_shadow_bindings_[p].get(),
             nullptr);
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
    dlss_profiled_ = dlss_active_;
    if(dlss_active_) {
      // DLSS in place of the anti-aliasing pass: its inputs, then DLSS
      // itself, given the jitter of this frame's primary rays
      dispatch(dlss_prepare_pipeline_.get(), dlss_prepare_bindings_[p].get(),
               nullptr);
      if(!dlss_.Evaluate(cb, denoise.upscale[0], denoise.upscale[1],
                         !rt_history_valid_)) {
        SHIFT_WARNING("NVIDIA DLSS: " + dlss_.get_reason().toStdString() +
                      "; ray tracing uses the viewer's own anti-aliasing");
        dlss_failed_ = true; // the next frame is made without it
        update();
      }
      if(profile) {
        cb->beginExternal();
        hardware_->Stamp(
          static_cast<const QRhiVulkanCommandBufferNativeHandles*>(
            cb->nativeHandles())
            ->commandBuffer,
          rhi_->currentFrameSlot(), stamp++);
        cb->endExternal();
      }
    } else {
      dispatch(svgf_taa_pipeline_.get(), svgf_taa_bindings_[p].get(), nullptr,
               true);
    }

    rt_parity_ = 1 - p;
    rt_sample_++;
    rt_frame_++;
    rt_history_valid_ = true;
    rt_previous_view_projection_ = view_projection;
  }

  batch = UploadFrameAxes(batch, view_projection, true);
  cb->beginPass(get_render_target(), QColor(0, 0, 0), {1.0f, 0}, batch);
  const QRhiViewport viewport(0, 0, float(target_size.width()),
                              float(target_size.height()));
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
  auto add = [&](int body, const ksl_SE3f_t& frame, const ksl_vec3f_t& tint,
                 float radius) {
    if(body < 0 || body >= static_cast<int>(poses_.bodies.size())) {
      return;
    }
    ksl_SE3f_t pose;
    ksl_product_ddf(&poses_.bodies[body], &frame, &pose);
    instances.emplace_back();
    float* model = instances.back().model;
    CopyMatrix(pose, model);
    // the columns of the rotation scaled: x and y by the radius, z by the
    // length
    const float scale[3] = {radius, radius, frame_axes_scale_};
    for(int c = 0; c < 3; c++) {
      for(int r = 0; r < 3; r++) {
        model[4 * c + r] *= scale[c];
      }
    }
    CopyVector(tint, 1.f, instances.back().tint);
  };
  for(const FrameAxes& axes : frame_axes_) {
    add(axes.body_index, axes.frame, ksl_vec3f(1.f, 1.f, 1.f),
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
  ksl_vec3f_t point;
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
  // the pipeline, made once (on a worker thread, CreatePipeline: the CADs
  // are triangulated in the frame after it is made); laid out with
  // placeholder buffers, as the bindings of each CAD are
  if(!cad_triangulate_pipeline_) {
    auto dummy = [&](QRhiBuffer::Type type, QRhiBuffer::UsageFlags usage,
                     quint32 bytes) {
      cad_triangulate_dummies_.emplace_back(
        rhi_->newBuffer(type, usage, bytes));
      cad_triangulate_dummies_.back()->create();
      return cad_triangulate_dummies_.back().get();
    };
    auto storage = [&]() {
      return dummy(QRhiBuffer::Static, QRhiBuffer::StorageBuffer, 16);
    };
    cad_triangulate_layout_.reset(rhi_->newShaderResourceBindings());
    cad_triangulate_layout_->setBindings(
      {Binding::uniformBuffer(0, compute,
                              dummy(QRhiBuffer::Dynamic,
                                    QRhiBuffer::UniformBuffer,
                                    sizeof(Params))),
       Binding::bufferLoad(1, compute, storage()),
       Binding::bufferStore(2, compute, storage()),
       Binding::bufferStore(3, compute, storage()),
       Binding::bufferLoad(10, compute, storage()),
       Binding::bufferLoad(11, compute, storage()),
       Binding::bufferLoad(12, compute, storage()),
       Binding::bufferLoad(13, compute, storage()),
       Binding::bufferLoad(14, compute, storage())});
    cad_triangulate_layout_->create();
    cad_triangulate_pipeline_.reset(rhi_->newComputePipeline());
    cad_triangulate_pipeline_->setShaderStage(
      {QRhiShaderStage::Compute, LoadShader("cad_triangulate.comp.qsb")});
    cad_triangulate_pipeline_->setShaderResourceBindings(
      cad_triangulate_layout_.get());
    CreatePipeline(cad_triangulate_pipeline_.get(), [this]() {
      qWarning("Cannot create the CAD triangulation pipeline; CAD geometry "
               "is not ray traced");
      cad_triangulate_pipeline_.reset();
    });
    if(!pending_pipelines_.empty()) {
      cad_triangles_dirty_ = true; // once it is made
      return;
    }
  }
  if(!cad_triangulate_pipeline_) {
    return;
  }
  QRhiComputePipeline* pipeline = cad_triangulate_pipeline_.get();
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

    QRhiCommandBuffer* cb = nullptr;
    if(rhi_->beginOffscreenFrame(&cb) != QRhi::FrameOpSuccess) {
      batch->release();
      return;
    }
    cb->beginComputePass(batch);
    cb->setComputePipeline(pipeline);
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
    const std::vector<int> cad_flex = CadFlexMeshes();
    BuildMeshBvhs(scene, cad_triangles_, cad_triangle_faces_, pick_cad_trims_,
                  pick_triangles_, pick_triangle_faces_, pick_nodes_,
                  pick_roots_, nullptr, nullptr, nullptr, &pick_flex_,
                  &cad_flex);
    for(auto& gpu : gpu_meshes_) {
      gpu.pick_stale = true;
    }
  }
  pick_built_ = true;
}

void shift::gui::Viewport::DeformPickBvh() {
  const Scene& scene = simulation_->get_scene();
  std::vector<Vertex> vertices;
  std::vector<float> field;
  std::vector<size_t> refreshed;
  for(const auto& flex : pick_flex_) {
    if(size_t(flex.mesh) >= gpu_meshes_.size() ||
       size_t(flex.mesh) >= scene.meshes.size()) {
      continue;
    }
    GpuMesh& gpu = gpu_meshes_[size_t(flex.mesh)];
    const SceneMesh& mesh = scene.meshes[size_t(flex.mesh)];
    if(!gpu.pick_stale || gpu.flex_shown.coefficients.size() !=
                            mesh.flex_columns) {
      continue;
    }
    refreshed.push_back(size_t(flex.mesh));
    if(flex.cad >= 0) {
      /* a flexible body's CAD: each corner of its undeformed triangles
      displaced by the field, as flex_cad.comp */
      const SceneCad& cad = scene.cads[size_t(flex.cad)];
      const std::vector<RtTriangle>& undeformed =
        cad_triangles_[size_t(flex.cad)];
      FlexCadField(cad, mesh, gpu.flex_shown, field);
      for(size_t k = 0; k < flex.order.size(); k++) {
        const size_t t = size_t(flex.order[k]);
        const auto& first = cad.first_triangles;
        const int patch = static_cast<int>(
          std::upper_bound(first.begin(), first.end(), int32_t(t)) -
          first.begin() - 1);
        const RtTriangle& source = undeformed[t];
        RtTriangle& triangle = pick_triangles_[size_t(flex.first_triangle) + k];
        const float* from[3][2] = {{source.v0, source.n0},
                                   {source.v1, source.n1},
                                   {source.v2, source.n2}};
        float* to[3] = {triangle.v0, triangle.v1, triangle.v2};
        for(int j = 0; j < 3; j++) {
          const ksl_vec3f_t d = FlexCadInterpolate(
            cad, field, patch, from[j][0][3], from[j][1][3], nullptr, nullptr);
          for(int c = 0; c < 3; c++) {
            to[j][c] = from[j][0][c] + d.at[c];
          }
        }
      }
      RefitBvh(pick_triangles_, pick_nodes_, flex.first_node, flex.node_count);
      continue;
    }
    DeformFlexMesh(mesh, gpu.flex_shown, vertices);
    for(size_t k = 0; k < flex.order.size(); k++) {
      RtTriangle& triangle = pick_triangles_[size_t(flex.first_triangle) + k];
      float* positions[3] = {triangle.v0, triangle.v1, triangle.v2};
      for(int j = 0; j < 3; j++) {
        const Vertex& v =
          vertices[mesh.indices[3 * size_t(flex.order[k]) + size_t(j)]];
        std::copy(v.position, v.position + 3, positions[j]);
      }
    }
    RefitBvh(pick_triangles_, pick_nodes_, flex.first_node, flex.node_count);
  }
  for(size_t m : refreshed) {
    gpu_meshes_[m].pick_stale = false;
  }
}

bool shift::gui::Viewport::Pick(const QPointF& pixel, ksl_vec3f_t& point) {
  if(!simulation_ || poses_.bodies.empty()) {
    return false;
  }
  if(!pick_built_) {
    BuildPickBvh();
  }
  DeformPickBvh();
  // the ray from the camera through the pixel
  const float w = float(std::max(width(), 1));
  const float h = float(std::max(height(), 1));
  // the pixel on the near and far planes, in the camera's coordinates:
  // undoing the projection, the only Qt matrix
  const QMatrix4x4 inverse_projection =
    camera_.get_projection_matrix(w / h).inverted();
  const float x = 2.f * float(pixel.x()) / w - 1.f;
  const float y = 1.f - 2.f * float(pixel.y()) / h;
  const QVector3D near_eye = inverse_projection.map(QVector3D(x, y, -1.f));
  const QVector3D far_eye = inverse_projection.map(QVector3D(x, y, 1.f));
  const ksl_vec3f_t along_eye =
    ksl_vec3f(far_eye.x() - near_eye.x(), far_eye.y() - near_eye.y(),
              far_eye.z() - near_eye.z());
  // then in world coordinates, from the camera's position
  const ksl_SE3f_t view = camera_.get_view_matrix();
  ksl_vec3f_t along;
  ksl_product_drinvvf(&view.R, &along_eye, &along);
  const ksl_vec3f_t origin = camera_.get_position();
  const ksl_vec3f_t direction = Normalized(along);

  float t_best = std::numeric_limits<float>::max();
  std::vector<int> stack;
  for(const auto& instance : DrawnInstances()) {
    if(instance.mesh_index >= static_cast<int>(pick_roots_.size()) ||
       instance.body_index >= static_cast<int>(poses_.bodies.size()) ||
       !is_instance_drawn(instance)) {
      continue;
    }
    // the ray in mesh coordinates; poses are rigid and placements add a
    // uniform scale, which divides o and d alike, so t is unchanged
    const ksl_SE3f_t pose =
      Placed(poses_.bodies[instance.body_index], instance);
    ksl_vec3f_t o, d;
    ksl_product_dinvvf(&pose, &origin, &o);
    ksl_product_drinvvf(&pose.R, &direction, &d);
    ksl_vec3f_scale(&o, 1.f / instance.scale);
    ksl_vec3f_scale(&d, 1.f / instance.scale);
    const ksl_vec3f_t inverse_d = ksl_vec3f(1.f / d.x, 1.f / d.y, 1.f / d.z);
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
  point = origin;
  ksl_axpy_vvf(t_best, &direction, &point);
  return true;
}

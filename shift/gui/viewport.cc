#include "shift/gui/viewport.h"

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
  float material[4]; // x = metallic, y = roughness, z = triplanar,
                     // w = 1 / texture scale
  int32_t layers[4]; // base color, metallic roughness, normal; -1: none
};

/*! per instance vertex attributes: the body origin pose, column major */
struct InstanceAttributes {
  float model[16];
};
static_assert(sizeof(InstanceAttributes) == 64);

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
};
static_assert(sizeof(RtInstance) == 272);

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
  material[2] = mesh.triplanar ? 1.f : 0.f;
  material[3] = 1.f / mesh.texture_scale;
  layers[0] = mesh.base_color_layer;
  layers[1] = mesh.metallic_roughness_layer;
  layers[2] = mesh.normal_layer;
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
  for(const auto& light : scene.lights) {
    if(count == kMaxLights) {
      break;
    }
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
    CopyVector(float(M_PI) * light.diffuse, float(light.ies_profile),
               d.diffuse);
    CopyVector(float(M_PI) * light.specular, 0.f, d.specular);
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

void shift::gui::Viewport::set_simulation(Simulation* simulation) {
  simulation_ = simulation;
  poses_ = Poses();
  if(simulation_) {
    simulation_->TakePoses(poses_);
  }
  scene_dirty_ = true;
  FrameScene();
}

void shift::gui::Viewport::set_render_mode(RenderMode mode) {
  render_mode_ = mode;
  rt_frame_ = 0;
  rt_history_valid_ = false;
  update();
}

void shift::gui::Viewport::set_floor_enabled(bool enabled) {
  floor_enabled_ = enabled;
  rt_frame_ = 0;
  rt_history_valid_ = false;
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

void shift::gui::Viewport::set_floor(const QVector3D& point,
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
  terrain_pipeline_.reset();
  gpu_terrains_.clear();
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
  // placeholder until a scene with IES lights is loaded
  ies_texture_.reset(rhi_->newTexture(QRhiTexture::R16F, QSize(1, 1)));
  ies_texture_->create();

  raster_bindings_.reset(rhi_->newShaderResourceBindings());

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

  sky_pipeline_.reset(make_pipeline("fullscreen.vert.qsb", "sky.frag.qsb",
                                    raster_bindings_.get(), false));
  sky_pipeline_->create();

  floor_pipeline_.reset(make_pipeline("floor.vert.qsb", "floor.frag.qsb",
                                      raster_bindings_.get(), true));
  floor_pipeline_->create();

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
                               linear_sampler_.get())});
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
       store(6, rt_output_), load(7, rt_accumulation_)});
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

/* one BVH per mesh in body coordinates, shared by its instances and
concatenated with absolute node and triangle indices; roots holds the root
node of each mesh */
static void BuildMeshBvhs(const shift::gui::Scene& scene,
                          std::vector<shift::gui::RtTriangle>& all_triangles,
                          std::vector<shift::gui::BvhNode>& all_nodes,
                          std::vector<int>& roots) {
  for(const auto& mesh : scene.meshes) {
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
    std::vector<shift::gui::BvhNode> nodes = shift::gui::BuildBvh(triangles);
    const int node_offset = static_cast<int>(all_nodes.size());
    const int triangle_offset = static_cast<int>(all_triangles.size());
    for(auto& node : nodes) {
      node.right_or_first +=
        node.triangle_count < 0 ? node_offset : triangle_offset;
    }
    roots.push_back(node_offset);
    all_nodes.insert(all_nodes.end(), nodes.begin(), nodes.end());
    all_triangles.insert(all_triangles.end(), triangles.begin(),
                         triangles.end());
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
                            float& t) {
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
  return true;
}

void shift::gui::Viewport::UploadScene(QRhiResourceUpdateBatch* batch) {
  scene_dirty_ = false;
  gpu_meshes_.clear();
  rt_triangles_.clear();
  rt_nodes_.clear();
  rt_roots_.clear();
  pick_triangles_.clear();
  pick_nodes_.clear();
  pick_roots_.clear();
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

  const quint32 mesh_count = quint32(std::max<size_t>(scene.meshes.size(), 1));
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

  if(ray_tracing_supported_) {
    BuildMeshBvhs(scene, rt_triangles_, rt_nodes_, rt_roots_);
    if(rt_triangles_.empty()) {
      rt_triangles_.resize(1);
      rt_nodes_.resize(1);
    }

    rt_triangle_buffer_.reset(rhi_->newBuffer(
      QRhiBuffer::Immutable, QRhiBuffer::StorageBuffer,
      quint32(rt_triangles_.size() * sizeof(RtTriangle))));
    rt_triangle_buffer_->create();
    rt_node_buffer_.reset(
      rhi_->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::StorageBuffer,
                      quint32(rt_nodes_.size() * sizeof(BvhNode))));
    rt_node_buffer_->create();
    rt_instance_buffer_.reset(
      rhi_->newBuffer(QRhiBuffer::Static, QRhiBuffer::StorageBuffer,
                      quint32(instance_count * sizeof(RtInstance))));
    rt_instance_buffer_->create();
    // one leaf per instance: 2 n - 1 nodes
    rt_instance_node_buffer_.reset(rhi_->newBuffer(
      QRhiBuffer::Static, QRhiBuffer::StorageBuffer,
      quint32((2 * instance_count - 1) * sizeof(BvhNode))));
    rt_instance_node_buffer_->create();
    batch->uploadStaticBuffer(rt_triangle_buffer_.get(), rt_triangles_.data());
    batch->uploadStaticBuffer(rt_node_buffer_.get(), rt_nodes_.data());
    // the GPU copies are all that ray tracing needs from here on; picking
    // keeps the CPU copies
    pick_triangles_ = std::move(rt_triangles_);
    pick_nodes_ = std::move(rt_nodes_);
    pick_roots_ = rt_roots_;
    pick_built_ = true;
    rt_triangles_ = std::vector<RtTriangle>();
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
  if(simulation_ && simulation_->TakePoses(poses_)) {
    rt_instances_dirty_ = true;
  }

  QRhiResourceUpdateBatch* batch = rhi_->nextResourceUpdateBatch();
  if(environment_dirty_) {
    UploadEnvironment(batch);
  }
  if(scene_dirty_) {
    UploadScene(batch);
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
      material.layers[3] = -1; // unused
      batch->updateDynamicBuffer(material_ubo_.get(),
                                 quint32(i) * material_stride_,
                                 sizeof(material), &material);
    }
    std::vector<InstanceAttributes> instances(scene.instances.size());
    for(size_t i = 0; i < instances.size(); i++) {
      CopyMatrix(poses_.bodies[scene.instances[i].body_index],
                 instances[i].model);
    }
    if(!instances.empty()) {
      batch->updateDynamicBuffer(
        instance_buffer_.get(), 0,
        quint32(instances.size() * sizeof(InstanceAttributes)),
        instances.data());
    }
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
      u.samples[2] = terrain.bicubic ? 1 : 0;
      batch->updateDynamicBuffer(gpu.uniforms.get(), 0, sizeof(u), &u);
    }
  }

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

  if(tessellate) {
    const Scene& scene = simulation_->get_scene();
    cb->setGraphicsPipeline(terrain_pipeline_.get());
    cb->setViewport(viewport);
    for(const auto& gpu : gpu_terrains_) {
      const int mesh_index = scene.terrains[gpu.terrain_index].mesh_index;
      const QRhiCommandBuffer::DynamicOffset offset(1, quint32(mesh_index) *
                                                         material_stride_);
      cb->setShaderResources(gpu.bindings.get(), 1, &offset);
      cb->draw(4, quint32(gpu.patches_x * gpu.patches_y));
    }
  }

  if(floor_enabled_ && have_poses) {
    cb->setGraphicsPipeline(floor_pipeline_.get());
    cb->setViewport(viewport);
    cb->setShaderResources(raster_bindings_.get(), 1, &zero_offset);
    cb->draw(6);
  }
  cb->endPass();
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
  for(int i = 0; i < instance_count; i++) {
    const SceneInstance& instance = scene.instances[i];
    const SceneMesh& mesh = scene.meshes[instance.mesh_index];
    const QMatrix4x4& pose = poses_.bodies[instance.body_index];
    const QMatrix4x4& previous =
      rt_previous_poses_.empty() ? pose : rt_previous_poses_[i];
    moved = moved || previous != pose;
    CopyMatrix(pose, instances[i].body_to_world);
    CopyMatrix(pose.inverted(), instances[i].world_to_body);
    CopyMatrix(previous, instances[i].previous_body_to_world);
    TransformBox(pose, mesh.bounds_min, mesh.bounds_max, instances[i].box_min,
                 instances[i].box_max);
    CopyVector(mesh.albedo, mesh.normal_scale, instances[i].albedo);
    CopyMaterial(mesh, instances[i].material, instances[i].offsets + 1);
    instances[i].offsets[0] = rt_roots_[instance.mesh_index];
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
    rt_previous_poses_[i] = poses_.bodies[scene.instances[i].body_index];
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
    instance_count =
      static_cast<int>(simulation_->get_scene().instances.size());
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

  cb->beginPass(get_render_target(), QColor(0, 0, 0), {1.0f, 0}, batch);
  cb->setGraphicsPipeline(blit_pipeline_.get());
  cb->setViewport(QRhiViewport(0, 0, float(size.width()), float(size.height())));
  cb->setShaderResources(blit_bindings_.get());
  cb->draw(6);
  cb->endPass();
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

void shift::gui::Viewport::BuildPickBvh() {
  pick_triangles_.clear();
  pick_nodes_.clear();
  pick_roots_.clear();
  if(simulation_) {
    BuildMeshBvhs(simulation_->get_scene(), pick_triangles_, pick_nodes_,
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
  for(const auto& instance : scene.instances) {
    if(instance.mesh_index >= static_cast<int>(pick_roots_.size()) ||
       instance.body_index >= static_cast<int>(poses_.bodies.size())) {
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
          float t;
          if(RayHitsTriangle(o, d, pick_triangles_[node.right_or_first + i],
                             t_best, t)) {
            t_best = t;
          }
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

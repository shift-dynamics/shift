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

namespace {

constexpr int kRtMaxFrames = 1024; //!< accumulated samples per pixel

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
};

struct InstanceUniforms {
  float model[16];
  float albedo[4];   // w = normal map scale
  float material[4]; // x = metallic, y = roughness, z = triplanar,
                     // w = 1 / texture scale
  int32_t layers[4]; // base color, metallic roughness, normal; -1: none
};

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
};

struct RtInstance {
  float body_to_world[16];
  float world_to_body[16];
  float box_min[4];
  float box_max[4];
  float albedo[4];
  float material[4]; // as in InstanceUniforms
  int32_t offsets[4]; // x = root node, y..w = texture layers as in
                      // InstanceUniforms
};
static_assert(sizeof(RtInstance) == 208);

const float kSunColor[4] = {1.f, 0.96f, 0.9f, 0.f};
const float kSunIntensity = 3.f;
const float kSkyZenith[4] = {0.22f, 0.36f, 0.62f, 0.f};
const float kSkyHorizon[4] = {0.62f, 0.68f, 0.74f, 0.f};

QShader LoadShader(const char* name) {
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
QVulkanInstance* GetVulkanInstance() {
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
QByteArray ToHalf(const std::vector<float>& values) {
  QByteArray data(int(values.size() * sizeof(qfloat16)), Qt::Uninitialized);
  qFloatToFloat16(reinterpret_cast<qfloat16*>(data.data()), values.data(),
                  qsizetype(values.size()));
  return data;
}

void CopyMatrix(const QMatrix4x4& m, float* out) {
  std::memcpy(out, m.constData(), 16 * sizeof(float));
}

void CopyVector(const QVector3D& v, float w, float* out) {
  out[0] = v.x();
  out[1] = v.y();
  out[2] = v.z();
  out[3] = w;
}

void CopyArray(const float* in, float* out) {
  std::memcpy(out, in, 4 * sizeof(float));
}

/*! material parameters and texture layers of a mesh, as in InstanceUniforms */
void CopyMaterial(const shift::gui::SceneMesh& mesh, float* material,
                  int32_t* layers) {
  material[0] = mesh.metallic;
  material[1] = mesh.roughness;
  material[2] = mesh.triplanar ? 1.f : 0.f;
  material[3] = 1.f / mesh.texture_scale;
  layers[0] = mesh.base_color_layer;
  layers[1] = mesh.metallic_roughness_layer;
  layers[2] = mesh.normal_layer;
  layers[3] = -1;
}

/*! world space box of a body space box under a pose */
void TransformBox(const QMatrix4x4& pose, const QVector3D& box_min,
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

} // namespace

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
  update();
}

void shift::gui::Viewport::set_floor_enabled(bool enabled) {
  floor_enabled_ = enabled;
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

void shift::gui::Viewport::UpdateEnvironment() {
  // place the floor under the model in its current pose; the lowest point
  // has the largest z since +z is down
  QVector3D box_min, box_max;
  SceneBounds(simulation_->get_scene(), poses_, box_min, box_max);
  const float size = std::max((box_max - box_min).length(), 1e-3f);
  floor_z_ = box_max.z();
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
  ReleaseAll();
  device_resources_ready_ = false;
  swapchain_ready_ = false;
  swapchain_render_pass_.reset();
  swapchain_.reset();
  depth_stencil_.reset();
  rhi_.reset();
}

QRhiRenderTarget* shift::gui::Viewport::get_render_target() const {
  return swapchain_->currentFrameRenderTarget();
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
  texture_sampler_.reset();
  texture_array_.reset();
  linear_sampler_.reset();
  environment_brdf_.reset();
  environment_irradiance_.reset();
  environment_specular_.reset();
  blit_pipeline_.reset();
  blit_bindings_.reset();
  sampler_.reset();
  rt_pipeline_.reset();
  rt_bindings_.reset();
  rt_accumulation_.reset();
  rt_output_.reset();
  rt_instance_buffer_.reset();
  rt_node_buffer_.reset();
  rt_triangle_buffer_.reset();
  rt_ubo_.reset();
  floor_pipeline_.reset();
  sky_pipeline_.reset();
  mesh_pipeline_.reset();
  raster_bindings_.reset();
  instance_ubo_.reset();
  frame_ubo_.reset();
  gpu_meshes_.clear();
  pipeline_render_pass_ = nullptr;
  output_size_ = QSize();
  scene_dirty_ = simulation_ != nullptr;
}

void shift::gui::Viewport::CreateDeviceResources() {
  instance_stride_ = rhi_->ubufAligned(sizeof(InstanceUniforms));

  frame_ubo_.reset(rhi_->newBuffer(QRhiBuffer::Dynamic,
                                   QRhiBuffer::UniformBuffer,
                                   sizeof(FrameUniforms)));
  frame_ubo_->create();
  instance_ubo_.reset(rhi_->newBuffer(
    QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, instance_stride_));
  instance_ubo_->create();

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
    rt_bindings_.reset(rhi_->newShaderResourceBindings());
  }
}

void shift::gui::Viewport::CreateSizeDependentResources() {
  output_size_ = get_render_target()->pixelSize();
  rt_frame_ = 0;
  if(!ray_tracing_supported_) {
    return;
  }
  rt_output_.reset(rhi_->newTexture(QRhiTexture::RGBA8, output_size_, 1,
                                    QRhiTexture::UsedWithLoadStore));
  rt_output_->create();
  rt_accumulation_.reset(rhi_->newTexture(QRhiTexture::RGBA32F, output_size_,
                                          1, QRhiTexture::UsedWithLoadStore));
  rt_accumulation_->create();
  // bindings reference the textures; rebuild them along with the pipelines
  pipeline_render_pass_ = nullptr;
}

void shift::gui::Viewport::CreatePipelines() {
  using Binding = QRhiShaderResourceBinding;
  const auto vertex_fragment = Binding::VertexStage | Binding::FragmentStage;

  raster_bindings_->setBindings(
    {Binding::uniformBuffer(0, vertex_fragment, frame_ubo_.get()),
     Binding::uniformBufferWithDynamicOffset(
       1, vertex_fragment, instance_ubo_.get(), sizeof(InstanceUniforms)),
     Binding::sampledTexture(2, Binding::FragmentStage,
                             environment_specular_.get(),
                             linear_sampler_.get()),
     Binding::sampledTexture(3, Binding::FragmentStage,
                             environment_irradiance_.get(),
                             linear_sampler_.get()),
     Binding::sampledTexture(4, Binding::FragmentStage,
                             environment_brdf_.get(), linear_sampler_.get()),
     Binding::sampledTexture(5, Binding::FragmentStage, texture_array_.get(),
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
  QRhiVertexInputLayout layout;
  layout.setBindings({QRhiVertexInputBinding(sizeof(Vertex))});
  layout.setAttributes(
    {QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3,
                              offsetof(Vertex, position)),
     QRhiVertexInputAttribute(0, 1, QRhiVertexInputAttribute::Float3,
                              offsetof(Vertex, normal)),
     QRhiVertexInputAttribute(0, 2, QRhiVertexInputAttribute::Float2,
                              offsetof(Vertex, uv))});
  mesh_pipeline_->setVertexInputLayout(layout);
  mesh_pipeline_->create();

  sky_pipeline_.reset(make_pipeline("fullscreen.vert.qsb", "sky.frag.qsb",
                                    raster_bindings_.get(), false));
  sky_pipeline_->create();

  floor_pipeline_.reset(make_pipeline("floor.vert.qsb", "floor.frag.qsb",
                                      raster_bindings_.get(), true));
  floor_pipeline_->create();

  if(!ray_tracing_supported_) {
    return;
  }

  const auto compute = Binding::ComputeStage;
  rt_bindings_->setBindings(
    {Binding::imageStore(0, compute, rt_output_.get(), 0),
     Binding::uniformBuffer(1, compute, rt_ubo_.get()),
     Binding::bufferLoad(2, compute, rt_triangle_buffer_.get()),
     Binding::bufferLoad(3, compute, rt_node_buffer_.get()),
     Binding::bufferLoad(4, compute, rt_instance_buffer_.get()),
     Binding::imageLoadStore(5, compute, rt_accumulation_.get(), 0),
     Binding::sampledTexture(6, compute, texture_array_.get(),
                             texture_sampler_.get())});
  rt_bindings_->create();

  rt_pipeline_.reset(rhi_->newComputePipeline());
  rt_pipeline_->setShaderStage(
    {QRhiShaderStage::Compute, LoadShader("raytrace.comp.qsb")});
  rt_pipeline_->setShaderResourceBindings(rt_bindings_.get());
  rt_pipeline_->create();

  blit_bindings_->setBindings({Binding::sampledTexture(
    0, Binding::FragmentStage, rt_output_.get(), sampler_.get())});
  blit_bindings_->create();
  blit_pipeline_.reset(make_pipeline("fullscreen.vert.qsb", "blit.frag.qsb",
                                     blit_bindings_.get(), false));
  blit_pipeline_->create();
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

void shift::gui::Viewport::UploadScene(QRhiResourceUpdateBatch* batch) {
  scene_dirty_ = false;
  gpu_meshes_.clear();
  rt_triangles_.clear();
  rt_nodes_.clear();
  rt_roots_.clear();
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

  const quint32 instance_count = quint32(std::max<size_t>(scene.meshes.size(), 1));
  instance_ubo_.reset(rhi_->newBuffer(QRhiBuffer::Dynamic,
                                      QRhiBuffer::UniformBuffer,
                                      instance_stride_ * instance_count));
  instance_ubo_->create();

  if(ray_tracing_supported_) {
    // one BVH per mesh in body coordinates, concatenated with absolute node
    // and triangle indices
    for(const auto& mesh : scene.meshes) {
      std::vector<RtTriangle> triangles(mesh.indices.size() / 3);
      for(size_t i = 0; i < triangles.size(); i++) {
        float* positions[3] = {triangles[i].v0, triangles[i].v1,
                               triangles[i].v2};
        float* normals[3] = {triangles[i].n0, triangles[i].n1,
                             triangles[i].n2};
        for(int j = 0; j < 3; j++) {
          const Vertex& v = mesh.vertices[mesh.indices[3 * i + j]];
          std::copy(v.position, v.position + 3, positions[j]);
          std::copy(v.normal, v.normal + 3, normals[j]);
          // texture coordinates in the spare components
          positions[j][3] = v.uv[0];
          normals[j][3] = v.uv[1];
        }
      }
      std::vector<BvhNode> nodes = BuildBvh(triangles);
      const int node_offset = static_cast<int>(rt_nodes_.size());
      const int triangle_offset = static_cast<int>(rt_triangles_.size());
      for(auto& node : nodes) {
        node.right_or_first +=
          node.triangle_count < 0 ? node_offset : triangle_offset;
      }
      rt_roots_.push_back(node_offset);
      rt_nodes_.insert(rt_nodes_.end(), nodes.begin(), nodes.end());
      rt_triangles_.insert(rt_triangles_.end(), triangles.begin(),
                           triangles.end());
    }
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
    batch->uploadStaticBuffer(rt_triangle_buffer_.get(), rt_triangles_.data());
    batch->uploadStaticBuffer(rt_node_buffer_.get(), rt_nodes_.data());
    // the GPU copies are all that is needed from here on
    rt_triangles_ = std::vector<RtTriangle>();
    rt_nodes_ = std::vector<BvhNode>();
  }

  CreatePipelines();
  rt_instances_dirty_ = true;
  rt_frame_ = 0;
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

void shift::gui::Viewport::RenderRaster(QRhiCommandBuffer* cb,
                                        QRhiResourceUpdateBatch* batch) {
  const QSize size = get_render_target()->pixelSize();
  const float aspect = float(size.width()) / float(std::max(size.height(), 1));
  const QMatrix4x4 view_projection = rhi_->clipSpaceCorrMatrix() *
                                     camera_.get_projection_matrix(aspect) *
                                     camera_.get_view_matrix();

  FrameUniforms frame{};
  CopyMatrix(view_projection, frame.view_projection);
  CopyMatrix(view_projection.inverted(), frame.inverse_view_projection);
  CopyVector(camera_.get_position(), 1.f, frame.camera_position);
  CopyVector(sun_direction_, kSunIntensity, frame.sun_direction);
  CopyArray(kSunColor, frame.sun_color);
  CopyArray(kSkyZenith, frame.sky_zenith);
  CopyArray(kSkyHorizon, frame.sky_horizon);
  const float floor_params[4] = {floor_z_, floor_spacing_,
                                 floor_enabled_ ? 1.f : 0.f, floor_extent_};
  CopyArray(floor_params, frame.floor_params);
  batch->updateDynamicBuffer(frame_ubo_.get(), 0, sizeof(frame), &frame);

  const bool have_poses = simulation_ && !poses_.bodies.empty();
  if(have_poses) {
    const Scene& scene = simulation_->get_scene();
    for(size_t i = 0; i < scene.meshes.size(); i++) {
      const SceneMesh& mesh = scene.meshes[i];
      InstanceUniforms instance{};
      CopyMatrix(poses_.bodies[mesh.body_index], instance.model);
      CopyVector(mesh.albedo, mesh.normal_scale, instance.albedo);
      CopyMaterial(mesh, instance.material, instance.layers);
      batch->updateDynamicBuffer(instance_ubo_.get(),
                                 quint32(i) * instance_stride_,
                                 sizeof(instance), &instance);
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
    for(size_t i = 0; i < gpu_meshes_.size(); i++) {
      const QRhiCommandBuffer::DynamicOffset offset(
        1, quint32(i) * instance_stride_);
      cb->setShaderResources(raster_bindings_.get(), 1, &offset);
      const QRhiCommandBuffer::VertexInput input(gpu_meshes_[i].vertices.get(),
                                                 0);
      cb->setVertexInput(0, 1, &input, gpu_meshes_[i].indices.get(), 0,
                         QRhiCommandBuffer::IndexUInt32);
      cb->drawIndexed(gpu_meshes_[i].index_count);
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

void shift::gui::Viewport::RenderRayTraced(QRhiCommandBuffer* cb,
                                           QRhiResourceUpdateBatch* batch) {
  const QSize size = get_render_target()->pixelSize();
  const float aspect = float(size.width()) / float(std::max(size.height(), 1));
  const QMatrix4x4 view = camera_.get_view_matrix();
  const QMatrix4x4 projection =
    rhi_->clipSpaceCorrMatrix() * camera_.get_projection_matrix(aspect);
  const QMatrix4x4 view_projection = projection * view;

  // restart accumulation whenever anything in view changes
  if(view_projection != rt_last_view_projection_) {
    rt_last_view_projection_ = view_projection;
    rt_frame_ = 0;
  }

  const bool have_poses = simulation_ && !poses_.bodies.empty();
  int instance_count = 0;
  if(have_poses) {
    const Scene& scene = simulation_->get_scene();
    instance_count = static_cast<int>(scene.meshes.size());
    if(rt_instances_dirty_) {
      std::vector<RtInstance> instances(std::max(instance_count, 1));
      for(int i = 0; i < instance_count; i++) {
        const SceneMesh& mesh = scene.meshes[i];
        const QMatrix4x4& pose = poses_.bodies[mesh.body_index];
        CopyMatrix(pose, instances[i].body_to_world);
        CopyMatrix(pose.inverted(), instances[i].world_to_body);
        TransformBox(pose, mesh.bounds_min, mesh.bounds_max,
                     instances[i].box_min, instances[i].box_max);
        CopyVector(mesh.albedo, mesh.normal_scale, instances[i].albedo);
        CopyMaterial(mesh, instances[i].material, instances[i].offsets + 1);
        instances[i].offsets[0] = rt_roots_[i];
      }
      batch->uploadStaticBuffer(rt_instance_buffer_.get(), 0,
                                quint32(instances.size() * sizeof(RtInstance)),
                                instances.data());
      rt_instances_dirty_ = false;
      rt_frame_ = 0;
    }
  }

  if(rt_frame_ < kRtMaxFrames) {
    RtUniforms uniforms{};
    CopyMatrix(view.inverted(), uniforms.inverse_view);
    CopyMatrix(projection.inverted(), uniforms.inverse_projection);
    CopyVector(camera_.get_position(), 1.f, uniforms.camera_position);
    CopyVector(sun_direction_, kSunIntensity, uniforms.sun_direction);
    CopyArray(kSunColor, uniforms.sun_color);
    CopyArray(kSkyZenith, uniforms.sky_zenith);
    CopyArray(kSkyHorizon, uniforms.sky_horizon);
    const float floor_params[4] = {
      floor_z_, floor_spacing_, (floor_enabled_ && have_poses) ? 1.f : 0.f,
      floor_extent_};
    CopyArray(floor_params, uniforms.floor_params);
    // angle subtended by a pixel, to select texture levels of detail
    const float spread =
      2.f * std::tan(0.5f * camera_.get_fov_y() * float(M_PI) / 180.f) /
      float(std::max(size.height(), 1));
    const float texture_params[4] = {
      spread, float(texture_array_->pixelSize().width()), 0.f, 0.f};
    CopyArray(texture_params, uniforms.texture_params);
    uniforms.control[0] = instance_count;
    uniforms.control[1] = rt_frame_;
    batch->updateDynamicBuffer(rt_ubo_.get(), 0, sizeof(uniforms), &uniforms);

    cb->beginComputePass(batch);
    cb->setComputePipeline(rt_pipeline_.get());
    cb->setShaderResources(rt_bindings_.get());
    cb->dispatch((size.width() + 7) / 8, (size.height() + 7) / 8, 1);
    cb->endComputePass();
    batch = nullptr;
    rt_frame_++;
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
  camera_.Zoom(std::pow(0.88f, steps));
  update();
}

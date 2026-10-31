#ifndef SHIFT_GUI_VIEWPORT_H_
#define SHIFT_GUI_VIEWPORT_H_

#include "shift/gui/bvh.h"
#include "shift/gui/camera.h"
#include "shift/gui/scene.h"

#include <QPoint>
#include <QWindow>

#include <memory>
#include <vector>

class QRhi;
class QRhiCommandBuffer;
class QRhiRenderBuffer;
class QRhiRenderTarget;
class QRhiSwapChain;
class QRhiBuffer;
class QRhiComputePipeline;
class QRhiGraphicsPipeline;
class QRhiRenderPassDescriptor;
class QRhiResourceUpdateBatch;
class QRhiSampler;
class QRhiShaderResourceBindings;
class QRhiTexture;

namespace shift {
namespace gui {

class Simulation;

/*!
@brief 3D view of a simulation, rendered with Qt RHI (Vulkan)

Two render modes:
- raster: GGX shading with a hemispherical ambient term
- ray traced: compute shader path tracer with sun shadows, one diffuse
  bounce and progressive accumulation while the view is static

Both work directly in shift world coordinates (+z down).

The viewport is a native window with its own Vulkan swap chain, embedded in
the workspace with QWidget::createWindowContainer. Rendering the whole main
window through Vulkan (QRhiWidget) leaves it without client side decorations
on Wayland compositors that don't provide server side decorations (GNOME).
*/
class Viewport : public QWindow {
  Q_OBJECT

public:
  enum class RenderMode { kRaster, kRayTraced };

  explicit Viewport(QWindow* parent = nullptr);
  ~Viewport() override;

  /*! \brief show the scene of a newly loaded simulation */
  void set_simulation(Simulation* simulation);

  void set_render_mode(RenderMode mode);
  RenderMode get_render_mode() const {
    return render_mode_;
  }

  void set_floor_enabled(bool enabled);

  /*! \brief frame the camera on the current poses */
  void FrameScene();

  /*! \brief the poses currently drawn */
  void get_poses(Poses& poses) const {
    poses = poses_;
  }

  /*! \brief frames rendered since the previous call */
  int TakeFrameCount();

  /*! \brief schedule a frame */
  void update() {
    requestUpdate();
  }

  /*! \brief false if the device has no compute support */
  bool is_ray_tracing_supported() const {
    return ray_tracing_supported_;
  }

protected:
  bool event(QEvent* event) override;
  void exposeEvent(QExposeEvent* event) override;

  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;

private:
  struct GpuMesh {
    std::unique_ptr<QRhiBuffer> vertices;
    std::unique_ptr<QRhiBuffer> indices;
    uint32_t index_count = 0;
  };

  void InitializeRhi();
  void ReleaseRhi();
  void RenderFrame();
  void initialize();
  void render(QRhiCommandBuffer* cb);
  QRhiRenderTarget* get_render_target() const;

  void CreateDeviceResources();
  void CreateSizeDependentResources();
  void CreatePipelines();
  void ReleaseAll();

  void UploadScene(QRhiResourceUpdateBatch* batch);
  void UploadEnvironment(QRhiResourceUpdateBatch* batch);
  void CreateTextureArray(int layer_count, int size);
  void UpdateEnvironment();
  void RenderRaster(QRhiCommandBuffer* cb, QRhiResourceUpdateBatch* batch);
  void RenderRayTraced(QRhiCommandBuffer* cb, QRhiResourceUpdateBatch* batch);

  Simulation* simulation_ = nullptr;
  Poses poses_;
  Camera camera_;
  RenderMode render_mode_ = RenderMode::kRaster;
  bool floor_enabled_ = true;
  bool scene_dirty_ = false;
  bool frame_pending_ = false;
  int frame_count_ = 0;

  // environment, in world coordinates
  QVector3D sun_direction_;
  float floor_z_ = 0.f;
  float floor_spacing_ = 1.f;
  float floor_extent_ = 100.f;

  // mouse
  QPoint last_mouse_;
  Qt::MouseButton drag_button_ = Qt::NoButton;

  std::unique_ptr<QRhi> rhi_;
  std::unique_ptr<QRhiSwapChain> swapchain_;
  std::unique_ptr<QRhiRenderBuffer> depth_stencil_;
  std::unique_ptr<QRhiRenderPassDescriptor> swapchain_render_pass_;
  bool swapchain_ready_ = false;
  bool device_resources_ready_ = false;
  QRhiRenderPassDescriptor* pipeline_render_pass_ = nullptr;
  QSize output_size_;

  // raster
  std::vector<GpuMesh> gpu_meshes_;
  std::unique_ptr<QRhiBuffer> frame_ubo_;
  std::unique_ptr<QRhiBuffer> instance_ubo_;
  quint32 instance_stride_ = 0;
  std::unique_ptr<QRhiShaderResourceBindings> raster_bindings_;
  std::unique_ptr<QRhiGraphicsPipeline> mesh_pipeline_;
  std::unique_ptr<QRhiGraphicsPipeline> sky_pipeline_;
  std::unique_ptr<QRhiGraphicsPipeline> floor_pipeline_;

  // image based lighting tables of the sky (see environment.h)
  std::unique_ptr<QRhiTexture> environment_specular_;
  std::unique_ptr<QRhiTexture> environment_irradiance_;
  std::unique_ptr<QRhiTexture> environment_brdf_;
  std::unique_ptr<QRhiSampler> linear_sampler_;
  bool environment_dirty_ = false;

  // texture layers of the scene (Scene::texture_layers), shared by both
  // render modes
  std::unique_ptr<QRhiTexture> texture_array_;
  std::unique_ptr<QRhiSampler> texture_sampler_;

  // ray tracing
  bool ray_tracing_supported_ = false;
  std::vector<RtTriangle> rt_triangles_;
  std::vector<BvhNode> rt_nodes_;
  std::vector<int> rt_roots_; /*!< root node of each mesh BVH */
  std::unique_ptr<QRhiBuffer> rt_ubo_;
  std::unique_ptr<QRhiBuffer> rt_triangle_buffer_;
  std::unique_ptr<QRhiBuffer> rt_node_buffer_;
  std::unique_ptr<QRhiBuffer> rt_instance_buffer_;
  std::unique_ptr<QRhiTexture> rt_output_;
  std::unique_ptr<QRhiTexture> rt_accumulation_;
  std::unique_ptr<QRhiShaderResourceBindings> rt_bindings_;
  std::unique_ptr<QRhiComputePipeline> rt_pipeline_;
  std::unique_ptr<QRhiSampler> sampler_;
  std::unique_ptr<QRhiShaderResourceBindings> blit_bindings_;
  std::unique_ptr<QRhiGraphicsPipeline> blit_pipeline_;
  int rt_frame_ = 0;
  QMatrix4x4 rt_last_view_projection_;
  bool rt_instances_dirty_ = true;
};

} // namespace gui
} // namespace shift

#endif

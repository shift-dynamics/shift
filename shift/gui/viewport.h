#ifndef SHIFT_GUI_VIEWPORT_H_
#define SHIFT_GUI_VIEWPORT_H_

#include "shift/gui/bvh.h"
#include "shift/gui/camera.h"
#include "shift/gui/scene.h"

#include <QPoint>
#include <QWindow>

#include <cstdint>
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
class QRhiTextureRenderTarget;

namespace shift {
namespace gui {

class Simulation;

/*! \brief the triangles of a deformable terrain's mesh (SceneTerrain::
 * deformable) among those of ray tracing, moved with its control points: the
 * mesh has s = mesh_subdivisions squares along each side of a cell, and two
 * triangles per square, those of square (u, v) at 2 (v s (width - 1) + u)
 * and the next in the mesh's order */
struct RtDeformableTerrain {
  int mesh = 0;    /*!< in Scene::meshes */
  int terrain = 0; /*!< in Scene::terrains */
  int first_triangle = 0;
  int first_node = 0; /*!< its root */
  int node_count = 0;
  /*! the place of each triangle of the mesh after first_triangle, as the
   * BVH ordered them */
  std::vector<int32_t> position;
};

/*!
@brief 3D view of a simulation, rendered with Qt RHI (Vulkan)

Two render modes:
- raster: GGX shading with a hemispherical ambient term. Terrains are
  tessellated on the GPU (terrain.tesc, terrain.tese): each patch is divided
  by the size of its edges on screen, and the generated vertices are placed
  on the surface evaluated from the terrain's control points, the cubic
  B-spline of the physics (terrain::Basic).
- ray traced: compute shader path tracer with sun shadows and one bounce at
  one sample per pixel and frame, denoised with SVGF (spatiotemporal
  variance-guided filtering: temporal reprojection, including moving bodies,
  and an edge-aware a-trous wavelet filter) and temporal anti-aliasing
  while the view changes, blending into the exact progressive mean once it
  is static

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
  /*! What the camera holds its transform to: the body itself, or the frame
   * on the ground under it (Camera::GroundFrame), which shows the body
   * bounce and tilt */
  enum class FollowMode { kBody, kGround };

  explicit Viewport(QWindow* parent = nullptr);
  ~Viewport() override;

  /*! \brief show the scene of a newly loaded simulation; frame_scene
   * places the camera to see all of it, else the camera stays (as for the
   * same model loaded again) */
  void set_simulation(Simulation* simulation, bool frame_scene = true);

  void set_render_mode(RenderMode mode);
  RenderMode get_render_mode() const {
    return render_mode_;
  }

  void set_floor_enabled(bool enabled);
  /*! \brief draw deformable terrains (SceneTerrain::deformable) by the
   * Catmull-Rom surface through their nodes, smooth but for overshoots
   * beside sharp ruts, rather than bilinearly between them as the physics
   * does, which shows the facets of the grid (default true) */
  void set_smooth_terrain(bool smooth);
  /*! \brief place the floor: a point on it and its normal, up (out of the
   * solid), in the inertial frame */
  void set_floor(const QVector3D& point, const QVector3D& normal);

  /*! \brief place a level floor at height z (+z down) that only marks the
   * ground: it is lowered whenever a pose shown reaches below it, so the
   * model never passes through it. For a floor plane without contact, e.g.
   * the default floor of a model without a ground plane. */
  void set_floor_below_model(float z);

  /*! \brief frame the camera on the current poses */
  void FrameScene();

  /*! \brief frame a sphere of the scene, to look closely at a part of it */
  void FrameSphere(const QVector3D& center, float radius);
  /*! \brief look from azimuth and elevation (radians, see Camera) and frame
   * the camera on the current poses */
  void set_view(float azimuth, float elevation);
  /*! \brief follow a body, index into the poses: from now on the transform
   * from the camera to the body stays as it is at this moment, and orbiting,
   * panning and zooming change it. -1 stops following and leaves the camera
   * where it is. */
  void set_follow_body(int body, FollowMode mode = FollowMode::kBody);
  int get_follow_body() const {
    return follow_body_;
  }

  /*! \brief take the shown poses of the simulation, if they changed */
  void UpdatePoses();
  /*! \brief with set_floor_below_model, lower the floor below the poses
   * shown if they reach below it */
  void LowerFloorBelowModel();
  /*! \brief move the floor, see set_floor */
  void PlaceFloor(const QVector3D& point, const QVector3D& normal);

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

  /*! @name Offscreen rendering, e.g. the frames of a video

  Between BeginOffscreen and EndOffscreen the viewport renders into an image
  of its own size instead of the window, which is not updated meanwhile. */
  /*! @{ */
  /*! \brief render offscreen at size (pixels); returns false, with a
   * message in error, if the device cannot */
  bool BeginOffscreen(const QSize& size, QString& error);
  /*! \brief render the poses shown and read the image back: RGBA, 8 bits
   * per channel, top row first, 4 width bytes per row. Ray traced, samples
   * frames of the same poses are rendered, so the progressive mean
   * converges; raster takes one. */
  bool RenderOffscreen(int samples, std::vector<uint8_t>& rgba);
  /*! \brief render to the window again */
  void EndOffscreen();
  bool is_offscreen() const {
    return offscreen_target_ != nullptr;
  }
  /*! @} */

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
    /*! range of Scene::instances drawn with this mesh */
    uint32_t first_instance = 0;
    uint32_t instance_count = 0;
  };

  /*! the control points of a terrain (SceneTerrain) for the tessellation
   * pipeline */
  struct GpuTerrain {
    std::unique_ptr<QRhiTexture> control_points; /*!< R32F */
    std::unique_ptr<QRhiBuffer> uniforms;
    std::unique_ptr<QRhiShaderResourceBindings> bindings;
    int terrain_index = 0; /*!< in Scene::terrains */
    int patches_x = 1;
    int patches_y = 1;
  };

  /*! CAD geometry (SceneCad) for the tessellation pipeline: its records
   * in storage buffers, and the poses of the bodies drawn with it */
  struct GpuCad {
    std::unique_ptr<QRhiBuffer> faces;
    std::unique_ptr<QRhiBuffer> curves;
    std::unique_ptr<QRhiBuffer> patches;
    std::unique_ptr<QRhiBuffer> points;
    std::unique_ptr<QRhiBuffer> knots;
    std::unique_ptr<QRhiBuffer> segments;
    std::unique_ptr<QRhiBuffer> trim_index;
    std::unique_ptr<QRhiBuffer> instances; /*!< poses times the scale */
    std::unique_ptr<QRhiShaderResourceBindings> bindings;
    int cad_index = 0; /*!< in Scene::cads */
  };

  /*! \brief upload the rectangles of the deformable terrains that changed
   * (Simulation::TakeTerrainUpdates) */
  void UploadTerrainUpdates(QRhiResourceUpdateBatch* batch);
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
  /*! \brief the floor's axes and origin, a column major 4x4 matrix */
  void CopyFloorFrame(float out[16]) const;
  /*! \brief uploads the lights of the model in the current poses; false
   * if it has none, and the sun lights the scene */
  bool UpdateLights(QRhiResourceUpdateBatch* batch);
  void RenderRaster(QRhiCommandBuffer* cb, QRhiResourceUpdateBatch* batch);
  void RenderRayTraced(QRhiCommandBuffer* cb, QRhiResourceUpdateBatch* batch);
  /*! \brief the frame the camera holds its transform to */
  QMatrix4x4 FollowFrame(int body, FollowMode mode) const;
  /*! \brief upload the instances (body poses, and the poses of the previous
   * frame for reprojection); returns true if a body moved */
  bool UploadInstances(QRhiResourceUpdateBatch* batch, int instance_count);
  /*! \brief the triangles of a deformable terrain over the cells i0..i1,
   * j0..j1 (clamped to the grid) from the control points of the scene, and
   * the boxes of its BVH refitted around them; the range of triangles
   * changed (relative to first_triangle) grows lo..hi. The surface is that
   * drawn by the tessellation shaders: Catmull-Rom with smooth_terrain_,
   * else bilinear between the control points */
  void ShapeRtTerrain(const RtDeformableTerrain& terrain, int i0, int j0,
                      int i1, int j1, std::vector<RtTriangle>& triangles,
                      std::vector<BvhNode>& nodes, int& lo, int& hi) const;
  /*! \brief compute pipeline of a shader, laid out as bindings */
  QRhiComputePipeline* NewComputePipeline(const char* shader,
                                          QRhiShaderResourceBindings* bindings);

  Simulation* simulation_ = nullptr;
  Poses poses_;
  Camera camera_;
  int follow_body_ = -1;        /*!< see set_follow_body */
  FollowMode follow_mode_ = FollowMode::kBody;
  QMatrix4x4 follow_offset_;    /*!< the motion of the camera in the frame
                                 * followed, see FollowFrame */
  RenderMode render_mode_ = RenderMode::kRaster;
  bool floor_enabled_ = true;
  bool smooth_terrain_ = true;
  /*! smooth_terrain_ when the ray tracer's terrains were shaped */
  bool rt_terrain_smooth_ = true;
  bool scene_dirty_ = false;
  bool frame_pending_ = false;
  int frame_count_ = 0;

  // environment, in world coordinates
  QVector3D sun_direction_;
  QVector3D floor_origin_{0.f, 0.f, 0.f};
  QVector3D floor_u_{1.f, 0.f, 0.f};
  QVector3D floor_v_{0.f, -1.f, 0.f};
  QVector3D floor_normal_{0.f, 0.f, -1.f}; // up, +z is down
  bool floor_below_model_ = false; /*!< see set_floor_below_model */
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

  // offscreen rendering (BeginOffscreen)
  std::unique_ptr<QRhiTexture> offscreen_texture_;
  std::unique_ptr<QRhiRenderBuffer> offscreen_depth_;
  std::unique_ptr<QRhiTextureRenderTarget> offscreen_target_;
  std::unique_ptr<QRhiRenderPassDescriptor> offscreen_render_pass_;

  // raster
  std::vector<GpuMesh> gpu_meshes_;
  std::unique_ptr<QRhiBuffer> frame_ubo_;
  std::unique_ptr<QRhiBuffer> material_ubo_; /*!< per mesh */
  std::unique_ptr<QRhiBuffer> lights_ubo_; /*!< LightUniforms */
  /*! IES distributions of the lights, R16F: see Scene::ies_profiles */
  std::unique_ptr<QRhiTexture> ies_texture_;
  quint32 material_stride_ = 0;
  /*! per instance vertex attributes (poses), in Scene::instances order */
  std::unique_ptr<QRhiBuffer> instance_buffer_;
  std::unique_ptr<QRhiShaderResourceBindings> raster_bindings_;
  std::unique_ptr<QRhiGraphicsPipeline> mesh_pipeline_;
  std::unique_ptr<QRhiGraphicsPipeline> sky_pipeline_;
  std::unique_ptr<QRhiGraphicsPipeline> floor_pipeline_;

  // terrains, tessellated on the GPU from their control points
  bool tessellation_supported_ = false;
  std::vector<GpuTerrain> gpu_terrains_;
  std::vector<TerrainUpdate> terrain_updates_; /*!< reused each frame */
  std::unique_ptr<QRhiGraphicsPipeline> terrain_pipeline_;

  // CAD geometry, tessellated on the GPU from its exact surfaces
  std::vector<GpuCad> gpu_cads_;
  std::unique_ptr<QRhiBuffer> cad_ubo_; /*!< CadUniforms */
  std::unique_ptr<QRhiGraphicsPipeline> cad_pipeline_;

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

  // triangles of the CAD geometry for ray tracing and picking, per
  // SceneCad, made on the GPU (cad_triangulate.comp) when a scene is set and
  // read back: body coordinates (scaled), and per triangle its face (see
  // raytrace.comp's triangle_faces)
  std::vector<std::vector<RtTriangle>> cad_triangles_;
  std::vector<std::vector<int32_t>> cad_triangle_faces_;
  bool cad_triangles_dirty_ = false;
  /*! \brief makes cad_triangles_ in a frame of its own, outside any other */
  void TriangulateCad();

  // ray tracing
  bool ray_tracing_supported_ = false;
  std::vector<RtTriangle> rt_triangles_;
  std::vector<int32_t> rt_triangle_faces_; /*!< -1 for meshes' triangles */
  std::vector<BvhNode> rt_nodes_;
  /*! root node of each mesh BVH, then of each SceneCad's */
  std::vector<int> rt_roots_;
  /*! what ray tracing instances: the meshes' (Scene::instances), then the
   * CAD geometry's, mesh_index meshes.size() + its index in Scene::cads */
  std::vector<SceneInstance> rt_instance_list_;
  /*! the deformable terrains among the meshes, whose triangles the
   * ray tracer moves with their control points (in pick_triangles_ and
   * pick_nodes_, the CPU copies of its arrays) */
  std::vector<RtDeformableTerrain> rt_terrains_;

  // picking: the point of the scene meshes under the cursor, found on the
  // CPU with one BVH per mesh (those of ray tracing when it is supported,
  // else built on the first pick), for zooming towards it
  std::vector<RtTriangle> pick_triangles_;
  std::vector<int32_t> pick_triangle_faces_;
  std::vector<BvhNode> pick_nodes_;
  std::vector<int> pick_roots_; /*!< root node of each mesh BVH */
  CadTrimTables pick_cad_trims_;
  bool pick_built_ = false;
  void BuildPickBvh();
  /*! \brief the nearest point of the scene meshes under a point of the
   * window (logical pixels), false if none */
  bool Pick(const QPointF& pixel, QVector3D& point);
  std::unique_ptr<QRhiBuffer> rt_ubo_;
  std::unique_ptr<QRhiBuffer> rt_triangle_buffer_;
  std::unique_ptr<QRhiBuffer> rt_triangle_face_buffer_;
  std::unique_ptr<QRhiBuffer> rt_cad_face_buffer_;
  std::unique_ptr<QRhiBuffer> rt_cad_segment_buffer_;
  std::unique_ptr<QRhiBuffer> rt_cad_trim_index_buffer_;
  std::unique_ptr<QRhiBuffer> rt_node_buffer_;
  std::unique_ptr<QRhiBuffer> rt_instance_buffer_;
  /*! BVH over the world space boxes of the instances, rebuilt with them */
  std::unique_ptr<QRhiBuffer> rt_instance_node_buffer_;
  std::unique_ptr<QRhiTexture> rt_output_;
  std::unique_ptr<QRhiTexture> rt_accumulation_; /*!< progressive mean */
  std::unique_ptr<QRhiComputePipeline> rt_pipeline_;

  // SVGF denoiser. The G-buffer and the history alternate between two sets
  // by frame parity: a frame writes set p and reads the previous frame's set
  // 1 - p, so each pass has a binding set per parity.
  std::unique_ptr<QRhiTexture> rt_illumination_; /*!< radiance / albedo */
  std::unique_ptr<QRhiTexture> rt_albedo_;
  std::unique_ptr<QRhiTexture> rt_direct_; /*!< sunlight, not denoised */
  std::unique_ptr<QRhiTexture> rt_position_[2]; /*!< xyz, w = instance */
  std::unique_ptr<QRhiTexture> rt_normal_[2];   /*!< xyz, w = distance */
  std::unique_ptr<QRhiTexture> rt_history_illumination_[2]; /*!< w = length */
  std::unique_ptr<QRhiTexture> rt_history_moments_[2];
  std::unique_ptr<QRhiTexture> rt_history_albedo_[2];
  std::unique_ptr<QRhiTexture> rt_filter_[2]; /*!< a-trous ping-pong */
  std::unique_ptr<QRhiTexture> rt_color_;  /*!< composited, tonemapped */
  std::unique_ptr<QRhiTexture> rt_taa_[2]; /*!< anti-aliased history */
  std::unique_ptr<QRhiBuffer> svgf_ubo_; /*!< one slot per pass/iteration */
  quint32 svgf_ubo_stride_ = 0;
  std::unique_ptr<QRhiShaderResourceBindings> rt_bindings_[2];
  std::unique_ptr<QRhiShaderResourceBindings> svgf_temporal_bindings_[2];
  std::unique_ptr<QRhiShaderResourceBindings> svgf_variance_bindings_[2];
  /*! [parity][direction], direction 0 filters rt_filter_[0] into [1] */
  std::unique_ptr<QRhiShaderResourceBindings> svgf_atrous_bindings_[2][2];
  std::unique_ptr<QRhiShaderResourceBindings> svgf_composite_bindings_[2];
  std::unique_ptr<QRhiShaderResourceBindings> svgf_taa_bindings_[2];
  std::unique_ptr<QRhiComputePipeline> svgf_temporal_pipeline_;
  std::unique_ptr<QRhiComputePipeline> svgf_variance_pipeline_;
  std::unique_ptr<QRhiComputePipeline> svgf_atrous_pipeline_;
  std::unique_ptr<QRhiComputePipeline> svgf_composite_pipeline_;
  std::unique_ptr<QRhiComputePipeline> svgf_taa_pipeline_;
  std::unique_ptr<QRhiSampler> sampler_;
  std::unique_ptr<QRhiShaderResourceBindings> blit_bindings_;
  std::unique_ptr<QRhiGraphicsPipeline> blit_pipeline_;
  int rt_frame_ = 0;  /*!< frames in the progressive mean */
  int rt_sample_ = 0; /*!< frames rendered, seeds the random numbers */
  int rt_parity_ = 0;
  QMatrix4x4 rt_last_view_projection_;
  bool rt_instances_dirty_ = true;
  /*! false after anything that invalidates the SVGF history (new scene,
   * resize, floor toggled, switching to ray tracing) */
  bool rt_history_valid_ = false;
  QMatrix4x4 rt_previous_view_projection_;
  std::vector<QMatrix4x4> rt_previous_poses_; /*!< per instance */
  /*! the last upload had a previous pose different from the current one, so
   * the instances must be uploaded again with previous = current */
  bool rt_instances_moving_ = false;
};

} // namespace gui
} // namespace shift

#endif

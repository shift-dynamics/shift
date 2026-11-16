#ifndef SHIFT_GUI_FLEX_GPU_H_
#define SHIFT_GUI_FLEX_GPU_H_

#include <rhi/qrhi.h>
#include <rhi/qshader.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "shift/gui/bvh.h"
#include "shift/gui/scene.h"

namespace shift {
namespace gui {

/*! \brief a shader bundle of the resources (":/shaders/<name>") */
QShader LoadShader(const char* name);

/*! the ray tracer's triangles of a deformable mesh (a flexible body's
 * surface), which the compute shaders write and refit as it deforms */
struct RtFlexMesh {
  int mesh = 0; /*!< in Scene::meshes */
  int first_triangle = 0;
  int first_node = 0; /*!< its root */
  int node_count = 0;
  /*! the mesh's triangle at each place after first_triangle, as the BVH
   * ordered them */
  std::vector<int32_t> order;
  /*! the first of its triangles' entries in the motion buffer; its
   * triangles' faces (the ray tracer's triangle faces) are -2 - the
   * entry */
  int motion_base = 0;
};

/*!
@brief the compute shaders that deform the meshes of flexible bodies
(SceneMesh::deformable) on the GPU

The shapes of a mesh (SceneMesh::flex_basis) are uploaded once; as it
deforms, only a FlexUpdate, a coefficient per shape and a rigid correction,
is. Deform then writes, in compute passes:
- flex_deform.comp: the deformed positions;
- flex_normals.comp: the vertex buffer drawn, with the area weighted normals
  of the deformed triangles (as DeformFlexMesh on the CPU);
- flex_refit.comp, while ray tracing: the ray tracer's triangles of the mesh
  in its BVH's order and the motion of their vertices since they were last
  written (for the denoiser's reprojection), then the BVH's boxes, the
  leaves and then the interior nodes depth by depth, the deepest first.

Each pass after the first reads what the one before wrote: QRhi orders the
passes.
*/
class FlexGpu {
public:
  explicit FlexGpu(QRhi* rhi);
  ~FlexGpu();
  FlexGpu(const FlexGpu&) = delete;
  FlexGpu& operator=(const FlexGpu&) = delete;

  /*! \brief a deformable mesh drawn from vertices (VertexBuffer and
   * StorageBuffer, Vertex) and indices (IndexBuffer and StorageBuffer);
   * returns its index here */
  int Add(const SceneMesh& mesh, QRhiBuffer* vertices, QRhiBuffer* indices,
          QRhiResourceUpdateBatch* batch);

  /*! \brief the ray tracer's triangles of mesh index (Add), in triangles
   * (RtTriangle) and their BVH in bvh (BvhNode; nodes as uploaded), and
   * the motion of the vertices of the deforming triangles (3 vec4 each) */
  void AddRayTracing(int index, const RtFlexMesh& rt,
                     const std::vector<BvhNode>& nodes, QRhiBuffer* triangles,
                     QRhiBuffer* bvh, QRhiBuffer* motion,
                     QRhiResourceUpdateBatch* batch);

  /*! \brief the deformation of mesh index now; it is deformed by the
   * next Deform */
  void Update(int index, const SceneMesh& mesh, const FlexUpdate& update,
              QRhiResourceUpdateBatch* batch);

  /*! \brief whether mesh index has the ray tracer's triangles */
  bool is_ray_traced(int index) const;

  /*! \brief a box that holds mesh index as last updated, body
   * coordinates: its undeformed box grown by a bound of the deformation,
   * sum |a_j| max |s_j| + |t| + |theta| max |x0 - c| */
  void get_bounds(int index, ksl_vec3f_t& bounds_min,
                  ksl_vec3f_t& bounds_max) const;

  /*! \brief deform the meshes updated since the last Deform, and while
   * ray_tracing, their ray tracer's triangles and BVH (those deformed
   * while not ray tracing catch up); the passes take batch. Returns the
   * batch for what follows: batch if nothing was done, a new one if it was
   * taken */
  QRhiResourceUpdateBatch* Deform(QRhiCommandBuffer* cb,
                                  QRhiResourceUpdateBatch* batch,
                                  bool ray_tracing);

private:
  struct Mesh;
  QRhiComputePipeline* NewPipeline(const char* shader,
                                   QRhiShaderResourceBindings* bindings);
  QRhiBuffer* NewStorage(QRhiBuffer::Type type, size_t bytes,
                         const void* data, QRhiResourceUpdateBatch* batch);

  QRhi* rhi_;
  std::vector<std::unique_ptr<Mesh>> meshes_;
  std::unique_ptr<QRhiComputePipeline> deform_pipeline_;
  std::unique_ptr<QRhiComputePipeline> normals_pipeline_;
  std::unique_ptr<QRhiComputePipeline> refit_pipeline_;
  quint32 step_stride_ = 0; /*!< of the refit's steps, aligned */
};

} // namespace gui
} // namespace shift

#endif

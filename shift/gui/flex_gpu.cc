#include "shift/gui/flex_gpu.h"

#include <QFile>

#include <algorithm>
#include <cmath>

QShader shift::gui::LoadShader(const char* name) {
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

/* FlexParams of flex_common.glsl */
struct FlexParams {
  uint32_t counts[4];
  float centroid[4];
  float translation[4];
  float rotation[4];
};

/* Step of flex_refit.comp */
struct FlexRefitStep {
  uint32_t step[4];
};

struct shift::gui::FlexGpu::Mesh {
  QRhiBuffer* vertices = nullptr; /*!< drawn, not owned */
  QRhiBuffer* indices = nullptr;  /*!< drawn, not owned */
  std::unique_ptr<QRhiBuffer> params;       /*!< uniform, FlexParams */
  std::unique_ptr<QRhiBuffer> reference;    /*!< the undeformed vertices */
  std::unique_ptr<QRhiBuffer> basis;        /*!< SceneMesh::flex_basis */
  std::unique_ptr<QRhiBuffer> coefficients; /*!< FlexUpdate's */
  std::unique_ptr<QRhiBuffer> deformed;     /*!< positions, vec4 */
  std::unique_ptr<QRhiBuffer> offsets;      /*!< triangles of a vertex */
  std::unique_ptr<QRhiBuffer> triangles;
  std::unique_ptr<QRhiShaderResourceBindings> deform_bindings;
  std::unique_ptr<QRhiShaderResourceBindings> normal_bindings;
  uint32_t vertex_count = 0;
  bool dispatch = false; /*!< updated since deformed */
  /*! the farthest any vertex moves in each shape, and the farthest vertex
   * from SceneMesh::flex_centroid: a bound of the deformation */
  std::vector<float> reach;
  float centroid_reach = 0.f;
  ksl_vec3f_t bounds_min = ksl_vec3f(0.f, 0.f, 0.f);
  ksl_vec3f_t bounds_max = ksl_vec3f(0.f, 0.f, 0.f);
  /*! the ray tracer's (RtFlexMesh), written by flex_refit.comp in steps:
   * the triangles, the leaves, then the interior nodes by depth, the
   * deepest first; then the step that zeroes the motion */
  std::unique_ptr<QRhiBuffer> order;
  std::unique_ptr<QRhiBuffer> list;
  std::unique_ptr<QRhiBuffer> steps; /*!< uniform, a FlexRefitStep each */
  /*! the bindings of each step, its FlexRefitStep at its offset */
  std::vector<std::unique_ptr<QRhiShaderResourceBindings>> refit_bindings;
  std::vector<uint32_t> step_counts; /*!< the work of each refit step */
  uint32_t zero_step = 0;
  bool rt_stale = false; /*!< deformed since its triangles were written */
  bool rt_moved = false; /*!< its motion is not zero */
};

shift::gui::FlexGpu::FlexGpu(QRhi* rhi) : rhi_(rhi) {}

shift::gui::FlexGpu::~FlexGpu() = default;

QRhiComputePipeline*
shift::gui::FlexGpu::NewPipeline(const char* shader,
                                 QRhiShaderResourceBindings* bindings) {
  QRhiComputePipeline* pipeline = rhi_->newComputePipeline();
  pipeline->setShaderStage({QRhiShaderStage::Compute, LoadShader(shader)});
  pipeline->setShaderResourceBindings(bindings);
  pipeline->create();
  return pipeline;
}

QRhiBuffer* shift::gui::FlexGpu::NewStorage(QRhiBuffer::Type type,
                                            size_t bytes, const void* data,
                                            QRhiResourceUpdateBatch* batch) {
  /* a buffer may not be empty */
  QRhiBuffer* buffer = rhi_->newBuffer(type, QRhiBuffer::StorageBuffer,
                                       quint32(std::max<size_t>(bytes, 16)));
  buffer->create();
  if(data && bytes > 0) {
    batch->uploadStaticBuffer(buffer, 0, quint32(bytes), data);
  }
  return buffer;
}

int shift::gui::FlexGpu::Add(const SceneMesh& mesh, QRhiBuffer* vertices,
                             QRhiBuffer* indices,
                             QRhiResourceUpdateBatch* batch) {
  auto flex = std::make_unique<Mesh>();
  flex->vertices = vertices;
  flex->indices = indices;
  const uint32_t vertex_count = uint32_t(mesh.vertices.size());
  flex->vertex_count = vertex_count;
  /* the triangles of each vertex */
  std::vector<uint32_t> offsets(size_t(vertex_count) + 1, 0);
  for(uint32_t index : mesh.indices) {
    offsets[size_t(index) + 1]++;
  }
  for(size_t v = 0; v < vertex_count; v++) {
    offsets[v + 1] += offsets[v];
  }
  std::vector<uint32_t> triangles(offsets.back());
  std::vector<uint32_t> next(offsets.begin(), offsets.end() - 1);
  for(size_t i = 0; i < mesh.indices.size(); i++) {
    triangles[next[mesh.indices[i]]++] = uint32_t(i / 3);
  }
  flex->params.reset(rhi_->newBuffer(
    QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(FlexParams)));
  flex->params->create();
  flex->reference.reset(NewStorage(QRhiBuffer::Immutable,
                                   mesh.vertices.size() * sizeof(Vertex),
                                   mesh.vertices.data(), batch));
  flex->basis.reset(NewStorage(QRhiBuffer::Immutable,
                               mesh.flex_basis.size() * sizeof(float),
                               mesh.flex_basis.data(), batch));
  flex->coefficients.reset(
    NewStorage(QRhiBuffer::Static, size_t(mesh.flex_columns) * sizeof(float),
               nullptr, batch));
  flex->deformed.reset(NewStorage(
    QRhiBuffer::Static, size_t(vertex_count) * 4 * sizeof(float), nullptr,
    batch));
  flex->offsets.reset(NewStorage(QRhiBuffer::Immutable,
                                 offsets.size() * sizeof(uint32_t),
                                 offsets.data(), batch));
  flex->triangles.reset(NewStorage(QRhiBuffer::Immutable,
                                   triangles.size() * sizeof(uint32_t),
                                   triangles.data(), batch));
  const auto stage = QRhiShaderResourceBinding::ComputeStage;
  flex->deform_bindings.reset(rhi_->newShaderResourceBindings());
  flex->deform_bindings->setBindings(
    {QRhiShaderResourceBinding::uniformBuffer(0, stage, flex->params.get()),
     QRhiShaderResourceBinding::bufferLoad(1, stage, flex->reference.get()),
     QRhiShaderResourceBinding::bufferLoad(2, stage, flex->basis.get()),
     QRhiShaderResourceBinding::bufferLoad(3, stage, flex->coefficients.get()),
     QRhiShaderResourceBinding::bufferStore(4, stage, flex->deformed.get())});
  flex->deform_bindings->create();
  flex->normal_bindings.reset(rhi_->newShaderResourceBindings());
  flex->normal_bindings->setBindings(
    {QRhiShaderResourceBinding::uniformBuffer(0, stage, flex->params.get()),
     QRhiShaderResourceBinding::bufferLoad(1, stage, flex->reference.get()),
     QRhiShaderResourceBinding::bufferLoad(4, stage, flex->deformed.get()),
     QRhiShaderResourceBinding::bufferLoad(5, stage, indices),
     QRhiShaderResourceBinding::bufferLoad(6, stage, flex->offsets.get()),
     QRhiShaderResourceBinding::bufferLoad(7, stage, flex->triangles.get()),
     QRhiShaderResourceBinding::bufferStore(8, stage, vertices)});
  flex->normal_bindings->create();
  if(!deform_pipeline_) {
    deform_pipeline_.reset(
      NewPipeline("flex_deform.comp.qsb", flex->deform_bindings.get()));
    normals_pipeline_.reset(
      NewPipeline("flex_normals.comp.qsb", flex->normal_bindings.get()));
  }
  /* a bound of the deformation (Update) */
  const size_t columns = mesh.flex_columns;
  const bool basis = mesh.flex_basis.size() == 3 * columns * vertex_count;
  flex->reach.assign(columns, 0.f);
  for(size_t v = 0; v < vertex_count; v++) {
    const float* x = mesh.vertices[v].position;
    const ksl_vec3f_t d =
      ksl_vec3f(x[0] - mesh.flex_centroid.x, x[1] - mesh.flex_centroid.y,
                x[2] - mesh.flex_centroid.z);
    flex->centroid_reach =
      std::max(flex->centroid_reach, ksl_vec3f_l2norm(&d));
    for(size_t j = 0; basis && j < columns; j++) {
      const float* u = mesh.flex_basis.data() + 3 * (v * columns + j);
      const ksl_vec3f_t shape = ksl_vec3f(u[0], u[1], u[2]);
      flex->reach[j] = std::max(flex->reach[j], ksl_vec3f_l2norm(&shape));
    }
  }
  flex->bounds_min = mesh.bounds_min;
  flex->bounds_max = mesh.bounds_max;
  meshes_.push_back(std::move(flex));
  return static_cast<int>(meshes_.size()) - 1;
}

void shift::gui::FlexGpu::AddRayTracing(
  int index, const RtFlexMesh& rt, const std::vector<BvhNode>& nodes,
  QRhiBuffer* triangles, QRhiBuffer* bvh, QRhiBuffer* motion,
  QRhiResourceUpdateBatch* batch) {
  if(index < 0 || size_t(index) >= meshes_.size()) {
    return;
  }
  Mesh& flex = *meshes_[size_t(index)];
  /* the depth of each node of the mesh's BVH: the left child follows its
  node, and both children come after it */
  const int first = rt.first_node;
  const int count = rt.node_count;
  if(first < 0 || count <= 0 || first + count > static_cast<int>(nodes.size())) {
    return;
  }
  std::vector<int> depth(size_t(count), 0);
  int deepest = 0;
  std::vector<uint32_t> leaves;
  for(int i = 0; i < count; i++) {
    const BvhNode& node = nodes[size_t(first + i)];
    if(node.triangle_count >= 0) {
      leaves.push_back(uint32_t(first + i));
      continue;
    }
    for(int child : {first + i + 1, node.right_or_first}) {
      depth[size_t(child - first)] = depth[size_t(i)] + 1;
    }
    deepest = std::max(deepest, depth[size_t(i)]);
  }
  /* the list: the leaves, then the interior nodes the deepest first */
  std::vector<uint32_t> list = leaves;
  std::vector<FlexRefitStep> steps;
  const uint32_t first_triangle = uint32_t(rt.first_triangle);
  const uint32_t motion_base = uint32_t(rt.motion_base);
  const uint32_t triangle_count = uint32_t(rt.order.size());
  steps.push_back({{motion_base, triangle_count, 0, first_triangle}});
  steps.push_back({{0, uint32_t(leaves.size()), 1, first_triangle}});
  for(int d = deepest; d >= 0; d--) {
    const uint32_t start = uint32_t(list.size());
    for(int i = 0; i < count; i++) {
      if(nodes[size_t(first + i)].triangle_count < 0 &&
         depth[size_t(i)] == d) {
        list.push_back(uint32_t(first + i));
      }
    }
    steps.push_back(
      {{start, uint32_t(list.size()) - start, 2, first_triangle}});
  }
  flex.zero_step = uint32_t(steps.size());
  steps.push_back({{motion_base, triangle_count, 3, first_triangle}});
  step_stride_ = quint32(rhi_->ubufAligned(int(sizeof(FlexRefitStep))));
  flex.steps.reset(rhi_->newBuffer(QRhiBuffer::Dynamic,
                                   QRhiBuffer::UniformBuffer,
                                   quint32(steps.size()) * step_stride_));
  flex.steps->create();
  flex.step_counts.clear();
  for(size_t k = 0; k < steps.size(); k++) {
    batch->updateDynamicBuffer(flex.steps.get(), quint32(k) * step_stride_,
                               sizeof(FlexRefitStep), &steps[k]);
    if(k < flex.zero_step) {
      flex.step_counts.push_back(steps[k].step[1]);
    }
  }
  const std::vector<uint32_t> order(rt.order.begin(), rt.order.end());
  flex.order.reset(NewStorage(QRhiBuffer::Immutable,
                              order.size() * sizeof(uint32_t), order.data(),
                              batch));
  flex.list.reset(NewStorage(QRhiBuffer::Immutable,
                             list.size() * sizeof(uint32_t), list.data(),
                             batch));
  const auto stage = QRhiShaderResourceBinding::ComputeStage;
  flex.refit_bindings.clear();
  for(size_t k = 0; k < steps.size(); k++) {
    std::unique_ptr<QRhiShaderResourceBindings> bindings(
      rhi_->newShaderResourceBindings());
    bindings->setBindings(
      {QRhiShaderResourceBinding::uniformBuffer(
         0, stage, flex.steps.get(), quint32(k) * step_stride_,
         sizeof(FlexRefitStep)),
       QRhiShaderResourceBinding::bufferLoad(1, stage, flex.vertices),
       QRhiShaderResourceBinding::bufferLoad(2, stage, flex.indices),
       QRhiShaderResourceBinding::bufferLoad(3, stage, flex.order.get()),
       QRhiShaderResourceBinding::bufferLoad(4, stage, flex.list.get()),
       QRhiShaderResourceBinding::bufferLoadStore(5, stage, triangles),
       QRhiShaderResourceBinding::bufferLoadStore(6, stage, bvh),
       QRhiShaderResourceBinding::bufferLoadStore(7, stage, motion)});
    bindings->create();
    flex.refit_bindings.push_back(std::move(bindings));
  }
  if(!refit_pipeline_) {
    refit_pipeline_.reset(
      NewPipeline("flex_refit.comp.qsb", flex.refit_bindings[0].get()));
  }
  flex.rt_stale = true;
}

void shift::gui::FlexGpu::Update(int index, const SceneMesh& mesh,
                                 const FlexUpdate& update,
                                 QRhiResourceUpdateBatch* batch) {
  if(index < 0 || size_t(index) >= meshes_.size() ||
     update.coefficients.size() != mesh.flex_columns) {
    return;
  }
  Mesh& flex = *meshes_[size_t(index)];
  FlexParams params{};
  params.counts[0] = flex.vertex_count;
  params.counts[1] = mesh.flex_columns;
  for(int c = 0; c < 3; c++) {
    params.centroid[c] = mesh.flex_centroid.at[c];
    params.translation[c] = update.translation.at[c];
    params.rotation[c] = update.rotation.at[c];
  }
  batch->updateDynamicBuffer(flex.params.get(), 0, sizeof(params), &params);
  if(!update.coefficients.empty()) {
    batch->uploadStaticBuffer(
      flex.coefficients.get(), 0,
      quint32(update.coefficients.size() * sizeof(float)),
      update.coefficients.data());
  }
  flex.dispatch = true;
  flex.rt_stale = true;
  /* no vertex moves farther than sum |a_j| reach_j + |t| + |theta|
  centroid_reach */
  float radius = ksl_vec3f_l2norm(&update.translation) +
                 ksl_vec3f_l2norm(&update.rotation) * flex.centroid_reach;
  for(size_t j = 0; j < flex.reach.size(); j++) {
    radius += std::fabs(update.coefficients[j]) * flex.reach[j];
  }
  for(int c = 0; c < 3; c++) {
    flex.bounds_min.at[c] = mesh.bounds_min.at[c] - radius;
    flex.bounds_max.at[c] = mesh.bounds_max.at[c] + radius;
  }
}

bool shift::gui::FlexGpu::is_ray_traced(int index) const {
  return index >= 0 && size_t(index) < meshes_.size() &&
         !meshes_[size_t(index)]->refit_bindings.empty();
}

void shift::gui::FlexGpu::get_bounds(int index, ksl_vec3f_t& bounds_min,
                                     ksl_vec3f_t& bounds_max) const {
  if(index < 0 || size_t(index) >= meshes_.size()) {
    return;
  }
  bounds_min = meshes_[size_t(index)]->bounds_min;
  bounds_max = meshes_[size_t(index)]->bounds_max;
}

QRhiResourceUpdateBatch*
shift::gui::FlexGpu::Deform(QRhiCommandBuffer* cb,
                            QRhiResourceUpdateBatch* batch, bool ray_tracing) {
  QRhiResourceUpdateBatch* const given = batch;
  std::vector<Mesh*> pending;
  for(auto& flex : meshes_) {
    if(flex->dispatch) {
      flex->dispatch = false;
      pending.push_back(flex.get());
    }
  }
  if(!pending.empty() && deform_pipeline_ && normals_pipeline_) {
    // the normals read the positions of the vertices around: a pass of
    // their own
    cb->beginComputePass(batch);
    batch = nullptr;
    cb->setComputePipeline(deform_pipeline_.get());
    for(Mesh* flex : pending) {
      cb->setShaderResources(flex->deform_bindings.get());
      cb->dispatch(int((flex->vertex_count + 63) / 64), 1, 1);
    }
    cb->endComputePass();
    cb->beginComputePass();
    cb->setComputePipeline(normals_pipeline_.get());
    for(Mesh* flex : pending) {
      cb->setShaderResources(flex->normal_bindings.get());
      cb->dispatch(int((flex->vertex_count + 63) / 64), 1, 1);
    }
    cb->endComputePass();
  }
  // the ray tracer's triangles and BVH, while ray tracing; a mesh that
  // moved in the previous frame and not in this one: its motion zeroed
  std::vector<Mesh*> stale, still;
  if(ray_tracing && refit_pipeline_) {
    for(auto& flex : meshes_) {
      if(flex->refit_bindings.empty()) {
        continue;
      }
      if(flex->rt_stale) {
        flex->rt_stale = false;
        flex->rt_moved = true;
        stale.push_back(flex.get());
      } else if(flex->rt_moved) {
        flex->rt_moved = false;
        still.push_back(flex.get());
      }
    }
  }
  auto dispatch = [&](Mesh* flex, uint32_t step, uint32_t count) {
    cb->setShaderResources(flex->refit_bindings[step].get());
    cb->dispatch(int((count + 63) / 64), 1, 1);
  };
  if(!still.empty()) {
    cb->beginComputePass(batch);
    batch = nullptr;
    cb->setComputePipeline(refit_pipeline_.get());
    for(Mesh* flex : still) {
      dispatch(flex, flex->zero_step, flex->step_counts[0]);
    }
    cb->endComputePass();
  }
  // each step in a pass of its own, after the one whose results it reads
  size_t step_count = 0;
  for(Mesh* flex : stale) {
    step_count = std::max(step_count, flex->step_counts.size());
  }
  for(size_t k = 0; k < step_count; k++) {
    cb->beginComputePass(batch);
    batch = nullptr;
    cb->setComputePipeline(refit_pipeline_.get());
    for(Mesh* flex : stale) {
      if(k < flex->step_counts.size() && flex->step_counts[k] > 0) {
        dispatch(flex, uint32_t(k), flex->step_counts[k]);
      }
    }
    cb->endComputePass();
  }
  if(batch == given) {
    return batch;
  }
  return rhi_->nextResourceUpdateBatch();
}

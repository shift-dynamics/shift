#include "shift/core/sph/gpu_solver.h"

#include <algorithm>
#include <cstring>

#include "shift/config.h"
#include "shift/core/exception.h"

#if SHIFT_HAVE_GPU
static const uint32_t kCount[] =
#include "shift/core/sph/sph_count.comp.inc"
  ;
static const uint32_t kScatter[] =
#include "shift/core/sph/sph_scatter.comp.inc"
  ;
static const uint32_t kCellSort[] =
#include "shift/core/sph/sph_cell_sort.comp.inc"
  ;
static const uint32_t kGather[] =
#include "shift/core/sph/sph_gather.comp.inc"
  ;
static const uint32_t kMarkers[] =
#include "shift/core/sph/sph_markers.comp.inc"
  ;
static const uint32_t kRates[] =
#include "shift/core/sph/sph_rates.comp.inc"
  ;
static const uint32_t kMomentum[] =
#include "shift/core/sph/sph_momentum.comp.inc"
  ;
static const uint32_t kBodies[] =
#include "shift/core/sph/sph_bodies.comp.inc"
  ;
#endif

/* the push constants of the shaders (sph_common.glsl) */
struct SphConstants {
  float dt;
  float h;
  float cell;
  uint32_t count;
  uint32_t mask;
  float viscosity;
  uint32_t unused0;
  uint32_t unused1;
  float gravity[4];
  uint32_t bodies[4]; // first body marker, body markers, bodies, substep
};

static const long kBatch = 64; // substeps per submission

static const uint32_t kGroup = 128; // local_size_x of the shaders

static uint32_t Groups(uint32_t n) {
  return (n + kGroup - 1) / kGroup;
}

shift::core::sph::GpuSolver::GpuSolver(gpu::Device& device,
                                       const Settings& settings,
                                       const std::vector<Material>& materials,
                                       const State& state)
  : Solver(settings, materials, state.body_count()), device_(device) {
#if !SHIFT_HAVE_GPU
  throw shift::core::Exception("SPH: shift was built without Vulkan");
#else
  const std::vector<std::array<float, 8>> table =
    MaterialTable(settings, materials);
  if(state.size() == 0 || state.size() > (1u << 30)) {
    throw shift::core::Exception("SPH: no particles, or too many");
  }
  CheckBodyMarkers(state);
  for(uint32_t kind : state.kind) {
    if(!(kind & State::kMarker) && kind >= table.size()) {
      throw shift::core::Exception("SPH: a particle of no material");
    }
  }
  count_ = static_cast<uint32_t>(state.size());
  table_size_ = HashTableSize(state.size());
  h_ = settings.smoothing_ratio * settings.spacing;
  const size_t n = count_;
  for(Arrays* arrays : {&a_, &b_}) {
    arrays->position = std::make_unique<gpu::Buffer>(device, n * 16);
    arrays->velocity = std::make_unique<gpu::Buffer>(device, n * 16);
    arrays->density = std::make_unique<gpu::Buffer>(device, n * 4);
    arrays->stress = std::make_unique<gpu::Buffer>(device, n * 32);
    arrays->id = std::make_unique<gpu::Buffer>(device, n * 4);
    arrays->kind = std::make_unique<gpu::Buffer>(device, n * 4);
  }
  a_.position->Upload(state.position.data(), n * 16);
  a_.velocity->Upload(state.velocity.data(), n * 16);
  a_.density->Upload(state.density.data(), n * 4);
  a_.stress->Upload(state.stress.data(), n * 32);
  a_.id->Upload(state.id.data(), n * 4);
  a_.kind->Upload(state.kind.data(), n * 4);
  materials_buffer_ =
    std::make_unique<gpu::Buffer>(device, table.size() * sizeof(table[0]));
  materials_buffer_->Upload(table.data(), table.size() * sizeof(table[0]));
  /* the grid: a count per cell and one more, 0, so that the scan gives the
  end of the last cell */
  hashes_ = std::make_unique<gpu::Buffer>(device, n * 4);
  counts_ = std::make_unique<gpu::Buffer>(device, (size_t(table_size_) + 1) * 4);
  start_ = std::make_unique<gpu::Buffer>(device, (size_t(table_size_) + 1) * 4);
  cursor_ = std::make_unique<gpu::Buffer>(device, size_t(table_size_) * 4);
  order_ = std::make_unique<gpu::Buffer>(device, n * 4);
  scan_ = std::make_unique<gpu::Scan>(device, table_size_ + 1);
  const size_t constants = sizeof(SphConstants);
  count_kernel_ =
    std::make_unique<gpu::Kernel>(device, kCount, sizeof(kCount), 3, constants);
  scatter_ = std::make_unique<gpu::Kernel>(device, kScatter, sizeof(kScatter),
                                           4, constants);
  cell_sort_ = std::make_unique<gpu::Kernel>(device, kCellSort,
                                             sizeof(kCellSort), 3, constants);
  gather_ = std::make_unique<gpu::Kernel>(device, kGather, sizeof(kGather), 13,
                                          constants);
  markers_ = std::make_unique<gpu::Kernel>(device, kMarkers, sizeof(kMarkers),
                                           6, constants);
  rates_ =
    std::make_unique<gpu::Kernel>(device, kRates, sizeof(kRates), 9, constants);
  momentum_ = std::make_unique<gpu::Kernel>(device, kMomentum,
                                            sizeof(kMomentum), 13, constants);
  bodies_kernel_ = std::make_unique<gpu::Kernel>(device, kBodies,
                                                 sizeof(kBodies), 6, constants);
  /* the bodies; without any, buffers of one element for the bindings */
  first_body_marker_ = state.first_body_marker;
  body_markers_ = static_cast<uint32_t>(state.body_local.size());
  const size_t markers = std::max<size_t>(1, body_markers_);
  body_local_ = std::make_unique<gpu::Buffer>(device, markers * 16);
  if(body_markers_ > 0) {
    body_local_->Upload(state.body_local.data(), body_markers_ * 16);
  }
  poses_ = std::make_unique<gpu::Buffer>(
    device, size_t(kBatch) * std::max<size_t>(1, bodies_) * 6 * 16);
  impulse_ = std::make_unique<gpu::Buffer>(device, 6 * markers * 4);
  sums_ = std::make_unique<gpu::Buffer>(device,
                                        6 * std::max<size_t>(1, bodies_) * 4);
  if(bodies_ > 0) {
    /* a segment per component and body, component by component */
    std::vector<uint32_t> offsets;
    for(uint32_t k = 0; k < 6; k++) {
      for(size_t b = 0; b < bodies_; b++) {
        offsets.push_back(k * body_markers_ + state.body_offsets[b]);
      }
    }
    offsets.push_back(6 * body_markers_);
    segment_sum_ = std::make_unique<gpu::SegmentSum>(device, offsets);
  }
  commands_ = std::make_unique<gpu::Commands>(device);
  ClearImpulses();
#endif
}

shift::core::sph::GpuSolver::~GpuSolver() {}

void shift::core::sph::GpuSolver::Record(float dt, uint32_t substep) {
  SphConstants c{};
  c.bodies[0] = first_body_marker_;
  c.bodies[1] = body_markers_;
  c.bodies[2] = static_cast<uint32_t>(bodies_);
  c.bodies[3] = substep;
  c.dt = dt;
  c.h = h_;
  c.cell = 2.0f * h_;
  c.count = count_;
  c.mask = table_size_ - 1;
  c.viscosity = settings_.artificial_viscosity;
  for(int k = 0; k < 3; k++) {
    c.gravity[k] = settings_.gravity[k];
  }
  const void* p = &c;
  const size_t size = sizeof(c);
  gpu::Commands& commands = *commands_;
  /* the bodies' markers on their poses */
  if(placed_) {
    commands.Dispatch(*bodies_kernel_,
                      {a_.position.get(), a_.velocity.get(), a_.kind.get(),
                       a_.id.get(), body_local_.get(), poses_.get()},
                      p, size, Groups(count_));
  }
  /* sort by cell, then id */
  commands.Fill(*counts_, 0u);
  commands.Fill(*cursor_, 0u);
  commands.Dispatch(*count_kernel_,
                    {a_.position.get(), hashes_.get(), counts_.get()}, p, size,
                    Groups(count_));
  scan_->Record(commands, *counts_, *start_);
  commands.Dispatch(*scatter_,
                    {hashes_.get(), start_.get(), cursor_.get(), order_.get()},
                    p, size, Groups(count_));
  commands.Dispatch(*cell_sort_, {start_.get(), order_.get(), a_.id.get()}, p,
                    size, Groups(table_size_));
  commands.Dispatch(*gather_,
                    {order_.get(), a_.position.get(), a_.velocity.get(),
                     a_.density.get(), a_.stress.get(), a_.id.get(),
                     a_.kind.get(), b_.position.get(), b_.velocity.get(),
                     b_.density.get(), b_.stress.get(), b_.id.get(),
                     b_.kind.get()},
                    p, size, Groups(count_));
  /* the markers, the rates, the motion */
  commands.Dispatch(*markers_,
                    {b_.position.get(), b_.density.get(), b_.stress.get(),
                     b_.kind.get(), start_.get(), materials_buffer_.get()},
                    p, size, Groups(count_));
  commands.Dispatch(*rates_,
                    {b_.position.get(), b_.velocity.get(), b_.density.get(),
                     b_.stress.get(), b_.kind.get(), start_.get(),
                     materials_buffer_.get(), a_.density.get(),
                     a_.stress.get()},
                    p, size, Groups(count_));
  commands.Dispatch(*momentum_,
                    {b_.position.get(), b_.velocity.get(), b_.kind.get(),
                     b_.id.get(), start_.get(), materials_buffer_.get(),
                     a_.density.get(), a_.stress.get(), a_.position.get(),
                     a_.velocity.get(), a_.id.get(), a_.kind.get(),
                     impulse_.get()},
                    p, size, Groups(count_));
}

/* the poses as sph_bodies.comp reads them: per substep and body, the
columns of R, t, v and w, as floats */
void shift::core::sph::GpuSolver::UploadPoses(float dt, long n) {
  placed_ = false;
  if(bodies_ == 0) {
    return;
  }
  std::vector<float> table(size_t(n) * bodies_ * 24, 0.0f);
  std::vector<BodyPose> poses;
  for(long k = 0; k < n; k++) {
    /* the time of the substep as the CPU solver's, a sum of the steps */
    double time = time_;
    for(long m = 0; m < k; m++) {
      time += dt;
    }
    PosesAt(time, poses);
    if(poses.empty()) {
      return;
    }
    for(size_t b = 0; b < bodies_; b++) {
      float* p = &table[(size_t(k) * bodies_ + b) * 24];
      const BodyPose& pose = poses[b];
      for(int c = 0; c < 3; c++) {
        for(int r = 0; r < 3; r++) {
          p[4 * c + r] = static_cast<float>(pose.R[3 * c + r]);
        }
        p[12 + c] = static_cast<float>(pose.t[c]);
        p[16 + c] = static_cast<float>(pose.v[c]);
        p[20 + c] = static_cast<float>(pose.w[c]);
      }
    }
  }
  poses_->Upload(table.data(), table.size() * 4);
  placed_ = true;
}

void shift::core::sph::GpuSolver::Step(float dt) {
  Steps(dt, 1);
}

void shift::core::sph::GpuSolver::Steps(float dt, long n) {
  /* in batches, so that a submission does not run for long; the poses of
  the bodies at the substeps of each batch go up before it */
  for(long first = 0; first < n; first += kBatch) {
    const long count = std::min(kBatch, n - first);
    UploadPoses(dt, count);
    for(long k = 0; k < count; k++) {
      Record(dt, static_cast<uint32_t>(k));
    }
    commands_->Submit();
    for(long k = 0; k < count; k++) {
      time_ += dt;
    }
  }
}

void shift::core::sph::GpuSolver::ClearImpulses() {
  /* recorded, and run with the next submission (of Steps or GetImpulses),
  before it */
  commands_->Fill(*impulse_, 0u);
}

void shift::core::sph::GpuSolver::GetImpulses(
  std::vector<std::array<double, 6>>& impulses) {
  impulses.assign(bodies_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  if(bodies_ == 0) {
    return;
  }
  segment_sum_->Record(*commands_, *impulse_, *sums_);
  commands_->Submit();
  std::vector<float> sums(6 * bodies_);
  sums_->Download(sums.data(), sums.size() * 4);
  for(size_t b = 0; b < bodies_; b++) {
    for(int k = 0; k < 3 * 2; k++) {
      impulses[b][k] = sums[k * bodies_ + b];
    }
  }
}

void shift::core::sph::GpuSolver::RemoveParticles(
  const std::vector<uint32_t>& ids) {
  if(ids.empty()) {
    return;
  }
  State state;
  GetState(state);
  state.RemoveParticles(ids);
  if(state.size() == 0) {
    throw shift::core::Exception("SPH: no particles or markers left");
  }
  /* the buffers keep their size, and the hash table its: the grid only
  gets sparser */
  count_ = static_cast<uint32_t>(state.size());
  const size_t n = count_;
  a_.position->Upload(state.position.data(), n * 16);
  a_.velocity->Upload(state.velocity.data(), n * 16);
  a_.density->Upload(state.density.data(), n * 4);
  a_.stress->Upload(state.stress.data(), n * 32);
  a_.id->Upload(state.id.data(), n * 4);
  a_.kind->Upload(state.kind.data(), n * 4);
}

void shift::core::sph::GpuSolver::GetState(State& state) {
  const size_t n = count_;
  state.resize(n);
  a_.position->Download(state.position.data(), n * 16);
  a_.velocity->Download(state.velocity.data(), n * 16);
  a_.density->Download(state.density.data(), n * 4);
  a_.stress->Download(state.stress.data(), n * 32);
  a_.id->Download(state.id.data(), n * 4);
  a_.kind->Download(state.kind.data(), n * 4);
}

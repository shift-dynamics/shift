#include "shift/core/dem/gpu_solver.h"

#include <algorithm>
#include <cstring>
#include <string>

#include "shift/config.h"
#include "shift/core/exception.h"

#if SHIFT_HAVE_GPU
static const uint32_t kSpheres[] =
#include "shift/core/dem/dem_spheres.comp.inc"
  ;
static const uint32_t kCount[] =
#include "shift/core/dem/dem_count.comp.inc"
  ;
static const uint32_t kScatter[] =
#include "shift/core/dem/dem_scatter.comp.inc"
  ;
static const uint32_t kCellSort[] =
#include "shift/core/dem/dem_cell_sort.comp.inc"
  ;
static const uint32_t kNeighbors[] =
#include "shift/core/dem/dem_neighbors.comp.inc"
  ;
static const uint32_t kFlags[] =
#include "shift/core/dem/dem_flags.comp.inc"
  ;
static const uint32_t kForces[] =
#include "shift/core/dem/dem_forces.comp.inc"
  ;
static const uint32_t kGrains[] =
#include "shift/core/dem/dem_grains.comp.inc"
  ;
static const uint32_t kTriangles[] =
#include "shift/core/dem/dem_triangles.comp.inc"
  ;
static const uint32_t kTriangleCount[] =
#include "shift/core/dem/dem_triangle_count.comp.inc"
  ;
static const uint32_t kNscContacts[] =
#include "shift/core/dem/nsc_contacts.comp.inc"
  ;
static const uint32_t kNscReverse[] =
#include "shift/core/dem/nsc_reverse.comp.inc"
  ;
static const uint32_t kNscPrepare[] =
#include "shift/core/dem/nsc_prepare.comp.inc"
  ;
static const uint32_t kNscVelocities[] =
#include "shift/core/dem/nsc_velocities.comp.inc"
  ;
static const uint32_t kNscBodyReduce[] =
#include "shift/core/dem/nsc_body_reduce.comp.inc"
  ;
static const uint32_t kNscBodySum[] =
#include "shift/core/dem/nsc_body_sum.comp.inc"
  ;
static const uint32_t kNscGradient[] =
#include "shift/core/dem/nsc_gradient.comp.inc"
  ;
static const uint32_t kNscUpdate[] =
#include "shift/core/dem/nsc_update.comp.inc"
  ;
static const uint32_t kNscReduce[] =
#include "shift/core/dem/nsc_reduce.comp.inc"
  ;
static const uint32_t kNscControl[] =
#include "shift/core/dem/nsc_control.comp.inc"
  ;
static const uint32_t kNscFinish[] =
#include "shift/core/dem/nsc_finish.comp.inc"
  ;
static const uint32_t kNscGrains[] =
#include "shift/core/dem/nsc_grains.comp.inc"
  ;
#endif

/* the push constants of the shaders (dem_common.glsl) */
struct DemConstants {
  float dt;
  float cell;
  float margin;
  uint32_t spheres;
  uint32_t grains;
  uint32_t mask;
  uint32_t max_neighbors;
  uint32_t walls;
  float gravity[4];
  uint32_t materials;
  uint32_t triangles;
  uint32_t triangle_mask;
  uint32_t max_triangles;
  uint32_t bodies;
  uint32_t substep;
  /* the non-smooth method */
  uint32_t capacity;
  uint32_t mode;
  uint32_t option;
  uint32_t stage;
  float omega;
  float recovery_speed;
  float min_bounce_speed;
  uint32_t splitting;
  uint32_t unused[2];
};

static_assert(sizeof(DemConstants) == 112, "the constants of dem_common.glsl");

static const long kBatch = 256; // steps per submission
static const long kNscBatch = 4; // steps of the non-smooth method

/* the words of NscControl (nsc_common.glsl), and the indirect dispatches'
workgroup counts, 4 words each, of the contacts in groups of 128 and of
1024, of the same while APGD tries, of the grains while it tries, and of
the contacts if the iterate is the best */
static const size_t kControlWords = 12;
static const size_t kArgsContacts = 0, kArgsReduce = 1,
                    kArgsContactsTrying = 2, kArgsReduceTrying = 3,
                    kArgsGrainsTrying = 4, kArgsBest = 5;

static const uint32_t kGroup = 128; // local_size_x of the shaders

static const uint32_t kMaxWalls = 8;

static uint32_t Groups(size_t n) {
  return static_cast<uint32_t>((n + kGroup - 1) / kGroup);
}

/* a buffer of n bytes, at least 16 */
static std::unique_ptr<shift::core::gpu::Buffer> Bytes(
  shift::core::gpu::Device& device, size_t n) {
  return std::make_unique<shift::core::gpu::Buffer>(device,
                                                    std::max<size_t>(16, n));
}

/* a buffer holding values, or one element of size bytes if there are none */
template <typename T>
static std::unique_ptr<shift::core::gpu::Buffer> Make(
  shift::core::gpu::Device& device, const std::vector<T>& values) {
  auto buffer = std::make_unique<shift::core::gpu::Buffer>(
    device, std::max<size_t>(1, values.size()) * sizeof(T));
  if(!values.empty()) {
    buffer->Upload(values.data(), values.size() * sizeof(T));
  }
  return buffer;
}

shift::core::dem::GpuSolver::GpuSolver(gpu::Device& device,
                                       const Settings& settings,
                                       const std::vector<Material>& materials,
                                       const std::vector<Template>& templates,
                                       const State& state,
                                       const std::vector<Body>& bodies)
  : Solver(settings, materials, templates, state, bodies), device_(device) {
#if !SHIFT_HAVE_GPU
  throw shift::core::Exception("DEM: shift was built without Vulkan");
#else
  nsc_ = settings.method == ContactMethod::kNsc;
  if(nsc_ && settings.nsc.solver == NscSolver::kGaussSeidel) {
    throw shift::core::Exception(
      "DEM: projected Gauss-Seidel (NSC) runs on the CPU only, device "
      "\"cpu\"; on the GPU, use APGD or Jacobi");
  }
  const Layout& l = layout_;
  grains_ = static_cast<uint32_t>(state.size());
  const size_t S = l.spheres, K = settings.max_neighbors;
  table_size_ = l.mask + 1;
  /* the grains */
  std::vector<std::array<float, 4>> velocity(grains_), omega(grains_);
  for(size_t g = 0; g < grains_; g++) {
    velocity[g] = {state.velocity[g][0], state.velocity[g][1],
                   state.velocity[g][2], 0.0f};
    omega[g] = {state.angular_velocity[g][0], state.angular_velocity[g][1],
                state.angular_velocity[g][2], 0.0f};
  }
  grain_anchor_ = Make(device, l.anchor);
  grain_offset_ = Make(device, l.offset);
  grain_velocity_ = Make(device, velocity);
  grain_omega_ = Make(device, omega);
  grain_quaternion_ = Make(device, state.orientation);
  first_sphere_ = Make(device, l.first_sphere);
  template_mass_ = Make(device, l.template_mass);
  template_spheres_ = Make(device, l.template_spheres);
  template_sphere_ = Make(device, l.template_sphere);
  template_material_ = Make(device, l.template_material);
  /* the hulls: a buffer holds at least one element */
  template_hull_ = Make(device, l.template_hull);
  hull_vertex_ = Make(device, l.hull_vertex.empty()
                                ? std::vector<std::array<float, 4>>(1)
                                : l.hull_vertex);
  hull_plane_ = Make(device, l.hull_plane.empty()
                               ? std::vector<std::array<float, 4>>(1)
                               : l.hull_plane);
  hull_face_ = Make(device, l.hull_face.empty()
                              ? std::vector<std::array<uint32_t, 4>>(1)
                              : l.hull_face);
  wall_anchor_ = Make(device, l.wall_anchor);
  wall_offset_ = Make(device, l.wall_offset);
  wall_normal_ = Make(device, l.wall_normal);
  pairs_ = Make(device, l.pairs);
  /* the spheres */
  sphere_info_ = Make(device, l.sphere_info);
  sphere_anchor_ = std::make_unique<gpu::Buffer>(device, S * 16);
  sphere_offset_ = std::make_unique<gpu::Buffer>(device, S * 16);
  sphere_arm_ = std::make_unique<gpu::Buffer>(device, S * 16);
  sphere_velocity_ = std::make_unique<gpu::Buffer>(device, S * 16);
  sphere_omega_ = std::make_unique<gpu::Buffer>(device, S * 16);
  reference_anchor_ = std::make_unique<gpu::Buffer>(device, S * 16);
  reference_offset_ = std::make_unique<gpu::Buffer>(device, S * 16);
  sphere_force_ = std::make_unique<gpu::Buffer>(device, S * 16);
  sphere_torque_ = std::make_unique<gpu::Buffer>(device, S * 16);
  neighbor_count_ = std::make_unique<gpu::Buffer>(device, S * 4);
  neighbor_ = std::make_unique<gpu::Buffer>(device, S * K * 4);
  history_ = std::make_unique<gpu::Buffer>(device, S * K * 16);
  wall_history_ = std::make_unique<gpu::Buffer>(device, S * kMaxWalls * 16);
  /* the grid: a count per cell and one more, 0, so that the scan gives the
  end of the last cell */
  hashes_ = std::make_unique<gpu::Buffer>(device, S * 4);
  counts_ =
    std::make_unique<gpu::Buffer>(device, (size_t(table_size_) + 1) * 4);
  start_ = std::make_unique<gpu::Buffer>(device, (size_t(table_size_) + 1) * 4);
  cursor_ = std::make_unique<gpu::Buffer>(device, size_t(table_size_) * 4);
  order_ = std::make_unique<gpu::Buffer>(device, S * 4);
  flags_ = std::make_unique<gpu::Buffer>(device, 32);
  scan_ = std::make_unique<gpu::Scan>(device, table_size_ + 1);
  /* the triangles, their grid (8 slots per triangle) and each sphere's
  triangles */
  const size_t T = l.triangles, M = settings.max_triangles, B = bodies.size();
  triangle_table_size_ = l.triangle_mask + 1;
  triangle_local_ = Make(device, l.triangle_local);
  triangle_anchor_ = Bytes(device, T * 16);
  triangle_vertex_ = Bytes(device, 3 * T * 16);
  reference_triangle_anchor_ = Bytes(device, T * 16);
  reference_triangle_vertex_ = Bytes(device, 3 * T * 16);
  triangle_hashes_ = Bytes(device, 8 * T * 4);
  triangle_counts_ = Bytes(device, (size_t(triangle_table_size_) + 1) * 4);
  triangle_start_ = Bytes(device, (size_t(triangle_table_size_) + 1) * 4);
  triangle_cursor_ = Bytes(device, size_t(triangle_table_size_) * 4);
  triangle_order_ = Bytes(device, 8 * T * 4);
  triangle_count_ = Bytes(device, S * 4);
  triangle_list_ = Bytes(device, S * M * 4);
  triangle_history_ = Bytes(device, S * M * 16);
  poses_ = Bytes(device, size_t(kBatch + 1) * B * 7 * 16);
  impulse_ = Bytes(device, 6 * B * S * 4);
  impulse_sums_ = Bytes(device, 6 * B * 4);
  readback_ = Bytes(device, 32 + 6 * B * 4 + kControlWords * 4);
  sums_.assign(6 * B, 0.0f);
  if(T > 0) {
    triangle_scan_ =
      std::make_unique<gpu::Scan>(device, triangle_table_size_ + 1);
  }
  if(B > 0) {
    std::vector<uint32_t> offsets(6 * B + 1);
    for(size_t j = 0; j <= 6 * B; j++) {
      offsets[j] = static_cast<uint32_t>(j * S);
    }
    impulse_sum_ = std::make_unique<gpu::SegmentSum>(device, offsets);
  }
  const size_t c = sizeof(DemConstants);
  spheres_kernel_ =
    std::make_unique<gpu::Kernel>(device, kSpheres, sizeof(kSpheres), 17, c);
  count_kernel_ =
    std::make_unique<gpu::Kernel>(device, kCount, sizeof(kCount), 5, c);
  scatter_ =
    std::make_unique<gpu::Kernel>(device, kScatter, sizeof(kScatter), 5, c);
  cell_sort_ =
    std::make_unique<gpu::Kernel>(device, kCellSort, sizeof(kCellSort), 3, c);
  neighbors_ = std::make_unique<gpu::Kernel>(device, kNeighbors,
                                             sizeof(kNeighbors), 17, c);
  flags_kernel_ =
    std::make_unique<gpu::Kernel>(device, kFlags, sizeof(kFlags), 1, c);
  forces_ =
    std::make_unique<gpu::Kernel>(device, kForces, sizeof(kForces), 28, c);
  grains_kernel_ =
    std::make_unique<gpu::Kernel>(device, kGrains, sizeof(kGrains), 10, c);
  triangles_kernel_ = std::make_unique<gpu::Kernel>(device, kTriangles,
                                                    sizeof(kTriangles), 7, c);
  triangle_count_kernel_ = std::make_unique<gpu::Kernel>(
    device, kTriangleCount, sizeof(kTriangleCount), 7, c);
  if(nsc_) {
    /* room for half the neighbors of each sphere, each pair once, and one
    wall or triangle */
    const size_t capacity = S * (K / 2 + 1);
    if(capacity >= (size_t(1) << 30)) {
      throw shift::core::Exception(
        "DEM: too many spheres for the non-smooth method on the GPU");
    }
    capacity_ = static_cast<uint32_t>(capacity);
    for(const Body& body : bodies) {
      massive_bodies_ = massive_bodies_ || body.mass > 0.0f;
    }
    const size_t C = capacity_, groups = (C + 1023) / 1024;
    contact_count_ = Bytes(device, (S + 1) * 4);
    contact_start_ = Bytes(device, (S + 1) * 4);
    contact_scan_ = std::make_unique<gpu::Scan>(device, uint32_t(S + 1));
    contacts_ = Bytes(device, C * 9 * 16);
    slot_contact_ = Bytes(device, S * K * 4);
    other_contact_ = Bytes(device, S * K * 4);
    body_rows_ = Bytes(device, size_t(kNscBatch) * B * 5 * 16);
    end_velocity_ = Bytes(device, size_t(grains_) * 16);
    end_omega_ = Bytes(device, size_t(grains_) * 16);
    body_delta_ = Bytes(device, B * 2 * 16);
    body_partials_ = Bytes(device, groups * B * 6 * 4);
    partials_ = Bytes(device, groups * 16);
    control_ = Bytes(device, kControlWords * 4);
    grain_contacts_ = Bytes(device, size_t(grains_) * 4);
    body_contacts_ = Bytes(device, B * 4);
    args_ = Bytes(device, 6 * 16);
    x_ = Bytes(device, C * 32);
    x_next_ = Bytes(device, C * 32);
    g_x_ = Bytes(device, C * 32);
    if(settings.nsc.solver == NscSolver::kApgd) {
      y_ = Bytes(device, C * 32);
      best_ = Bytes(device, C * 32);
      g_y_ = Bytes(device, C * 32);
    }
    auto kernel = [&](const uint32_t* code, size_t size, int buffers) {
      return std::make_unique<gpu::Kernel>(device, code, size, buffers, c);
    };
    nsc_contacts_ = kernel(kNscContacts, sizeof(kNscContacts), 27);
    nsc_reverse_ = kernel(kNscReverse, sizeof(kNscReverse), 4);
    nsc_prepare_ = kernel(kNscPrepare, sizeof(kNscPrepare), 12);
    nsc_velocities_ = kernel(kNscVelocities, sizeof(kNscVelocities), 15);
    nsc_body_reduce_ = kernel(kNscBodyReduce, sizeof(kNscBodyReduce), 4);
    nsc_body_sum_ = kernel(kNscBodySum, sizeof(kNscBodySum), 4);
    nsc_gradient_ = kernel(kNscGradient, sizeof(kNscGradient), 9);
    nsc_update_ = kernel(kNscUpdate, sizeof(kNscUpdate), 5);
    nsc_reduce_ = kernel(kNscReduce, sizeof(kNscReduce), 8);
    nsc_control_ = kernel(kNscControl, sizeof(kNscControl), 5);
    nsc_finish_ = kernel(kNscFinish, sizeof(kNscFinish), 8);
    nsc_grains_ = kernel(kNscGrains, sizeof(kNscGrains), 7);
  }
  commands_ = std::make_unique<gpu::Commands>(device);
  if(nsc_) {
    commands_->Fill(*body_delta_, 0u);
    commands_->Fill(*control_, 0u);
    commands_->Fill(*args_, 0u);
  }
  /* no neighbors or histories yet, and a search at the first step */
  commands_->Fill(*neighbor_count_, 0u);
  commands_->Fill(*history_, 0u);
  commands_->Fill(*wall_history_, 0u);
  commands_->Fill(*reference_anchor_, 0u);
  commands_->Fill(*reference_offset_, 0u);
  commands_->Fill(*triangle_count_, 0u);
  commands_->Fill(*triangle_history_, 0u);
  commands_->Fill(*reference_triangle_anchor_, 0u);
  commands_->Fill(*reference_triangle_vertex_, 0u);
  commands_->Fill(*impulse_, 0u);
  commands_->Submit();
  const uint32_t flags[8] = {1u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
  flags_->Upload(flags, sizeof(flags));
#endif
}

shift::core::dem::GpuSolver::~GpuSolver() {}

void shift::core::dem::GpuSolver::Record(float dt, uint32_t substep) {
  const Layout& l = layout_;
  DemConstants c{};
  c.dt = dt;
  c.cell = l.cell;
  c.margin = l.margin;
  c.spheres = l.spheres;
  c.grains = grains_;
  c.mask = l.mask;
  c.max_neighbors = settings_.max_neighbors;
  c.walls = static_cast<uint32_t>(l.wall_normal.size());
  for(int k = 0; k < 3; k++) {
    c.gravity[k] = settings_.gravity[k];
  }
  c.materials = l.materials;
  c.triangles = l.triangles;
  c.triangle_mask = l.triangle_mask;
  c.max_triangles = settings_.max_triangles;
  c.bodies = static_cast<uint32_t>(bodies_.size());
  c.substep = substep;
  c.capacity = capacity_;
  c.recovery_speed = settings_.nsc.recovery_speed;
  c.min_bounce_speed = settings_.nsc.min_bounce_speed;
  c.omega = Relaxation(settings_.nsc);
  c.splitting = SplitsMasses(settings_.nsc) ? 1u : 0u;
  /* the scatter and cell sort of the triangles' slots */
  DemConstants t = c;
  t.spheres = 8 * l.triangles;
  t.mask = l.triangle_mask;
  const void* p = &c;
  const size_t size = sizeof(c);
  const uint32_t groups = Groups(l.spheres);
  gpu::Commands& commands = *commands_;
  const bool triangles = l.triangles > 0;
  commands.Dispatch(
    *spheres_kernel_,
    {grain_anchor_.get(), grain_offset_.get(), grain_velocity_.get(),
     grain_omega_.get(), grain_quaternion_.get(), template_mass_.get(),
     template_sphere_.get(), template_material_.get(), sphere_info_.get(),
     sphere_anchor_.get(), sphere_offset_.get(), sphere_arm_.get(),
     sphere_velocity_.get(), sphere_omega_.get(), reference_anchor_.get(),
     reference_offset_.get(), flags_.get()},
    p, size, groups);
  if(triangles) {
    commands.Dispatch(*triangles_kernel_,
                      {triangle_local_.get(), poses_.get(),
                       triangle_anchor_.get(), triangle_vertex_.get(),
                       reference_triangle_anchor_.get(),
                       reference_triangle_vertex_.get(), flags_.get()},
                      p, size, Groups(l.triangles));
  }
  /* the search, which does nothing unless flags[0] is set */
  commands.Fill(*counts_, 0u);
  commands.Fill(*cursor_, 0u);
  if(triangles) {
    commands.Fill(*triangle_counts_, 0u);
    commands.Fill(*triangle_cursor_, 0u);
    commands.Dispatch(*triangle_count_kernel_,
                      {triangle_anchor_.get(), triangle_vertex_.get(),
                       triangle_hashes_.get(), triangle_counts_.get(),
                       flags_.get(), reference_triangle_anchor_.get(),
                       reference_triangle_vertex_.get()},
                      p, size, Groups(l.triangles));
    triangle_scan_->Record(commands, *triangle_counts_, *triangle_start_);
    commands.Dispatch(*scatter_,
                      {triangle_hashes_.get(), triangle_start_.get(),
                       triangle_cursor_.get(), triangle_order_.get(),
                       flags_.get()},
                      &t, size, Groups(8 * size_t(l.triangles)));
    commands.Dispatch(*cell_sort_,
                      {triangle_start_.get(), triangle_order_.get(),
                       flags_.get()},
                      &t, size, Groups(triangle_table_size_));
  }
  commands.Dispatch(*count_kernel_,
                    {sphere_anchor_.get(), sphere_offset_.get(), hashes_.get(),
                     counts_.get(), flags_.get()},
                    p, size, groups);
  scan_->Record(commands, *counts_, *start_);
  commands.Dispatch(*scatter_,
                    {hashes_.get(), start_.get(), cursor_.get(), order_.get(),
                     flags_.get()},
                    p, size, groups);
  commands.Dispatch(*cell_sort_, {start_.get(), order_.get(), flags_.get()}, p,
                    size, Groups(table_size_));
  commands.Dispatch(*neighbors_,
                    {sphere_anchor_.get(), sphere_offset_.get(), start_.get(),
                     order_.get(), neighbor_count_.get(), neighbor_.get(),
                     history_.get(), reference_anchor_.get(),
                     reference_offset_.get(), flags_.get(),
                     triangle_start_.get(), triangle_order_.get(),
                     triangle_anchor_.get(), triangle_vertex_.get(),
                     triangle_count_.get(), triangle_list_.get(),
                     triangle_history_.get()},
                    p, size, groups);
  commands.Dispatch(*flags_kernel_, {flags_.get()}, p, size, 1);
  if(nsc_) {
    RecordNsc(p);
    return;
  }
  /* the forces and the motion */
  commands.Dispatch(*forces_,
                    {sphere_anchor_.get(), sphere_offset_.get(),
                     sphere_arm_.get(), sphere_velocity_.get(),
                     sphere_omega_.get(), neighbor_count_.get(),
                     neighbor_.get(), history_.get(), wall_history_.get(),
                     wall_anchor_.get(), wall_offset_.get(),
                     wall_normal_.get(), pairs_.get(), sphere_force_.get(),
                     sphere_torque_.get(), triangle_anchor_.get(),
                     triangle_vertex_.get(), triangle_count_.get(),
                     triangle_list_.get(), triangle_history_.get(),
                     poses_.get(), impulse_.get(), sphere_info_.get(),
                     grain_quaternion_.get(), template_hull_.get(),
                     hull_vertex_.get(), hull_plane_.get(),
                     hull_face_.get()},
                    p, size, groups);
  commands.Dispatch(*grains_kernel_,
                    {grain_anchor_.get(), grain_offset_.get(),
                     grain_velocity_.get(), grain_omega_.get(),
                     grain_quaternion_.get(), first_sphere_.get(),
                     template_mass_.get(), template_spheres_.get(),
                     sphere_force_.get(), sphere_torque_.get()},
                    p, size, Groups(grains_));
}

/* the non-smooth method */

void shift::core::dem::GpuSolver::Control(const void* constants,
                                          uint32_t stage, uint32_t option) {
  DemConstants c = *static_cast<const DemConstants*>(constants);
  c.stage = stage;
  c.option = option;
  commands_->Dispatch(*nsc_control_,
                      {partials_.get(), control_.get(), args_.get(),
                       contact_start_.get(), flags_.get()},
                      &c, sizeof(c), 1);
}

/* the grains' velocities for the impulses (option, see nsc_velocities.comp),
and the bodies' changes of velocity; only while APGD tries if trying */
void shift::core::dem::GpuSolver::Velocities(const void* constants,
                                             uint32_t option,
                                             const gpu::Buffer& impulse,
                                             bool trying) {
  DemConstants c = *static_cast<const DemConstants*>(constants);
  c.option = option;
  const std::vector<const gpu::Buffer*> buffers = {
    grain_anchor_.get(),    grain_velocity_.get(), grain_omega_.get(),
    grain_quaternion_.get(), first_sphere_.get(),  template_mass_.get(),
    template_spheres_.get(), neighbor_count_.get(), other_contact_.get(),
    contact_start_.get(),   contacts_.get(),       &impulse,
    end_velocity_.get(),    end_omega_.get(),      control_.get()};
  if(trying) {
    commands_->DispatchIndirect(*nsc_velocities_, buffers, &c, sizeof(c),
                                *args_, kArgsGrainsTrying * 16);
  } else {
    commands_->Dispatch(*nsc_velocities_, buffers, &c, sizeof(c),
                        Groups(grains_));
  }
  if(!massive_bodies_) {
    return;
  }
  const std::vector<const gpu::Buffer*> reduce = {
    contacts_.get(), &impulse, body_partials_.get(), control_.get()};
  commands_->DispatchIndirect(*nsc_body_reduce_, reduce, &c, sizeof(c),
                              *args_,
                              (trying ? kArgsReduceTrying : kArgsReduce) * 16);
  commands_->Dispatch(*nsc_body_sum_,
                      {body_partials_.get(), body_rows_.get(),
                       body_delta_.get(), control_.get()},
                      &c, sizeof(c), 1);
}

/* the gradient of APGD at z (option of nsc_velocities.comp) */
void shift::core::dem::GpuSolver::NscGradient(const void* constants,
                                              uint32_t option,
                                              const gpu::Buffer& z,
                                              const gpu::Buffer& gradient,
                                              bool trying) {
  Velocities(constants, option, z, trying);
  DemConstants c = *static_cast<const DemConstants*>(constants);
  c.mode = 0;
  commands_->DispatchIndirect(
    *nsc_gradient_,
    {contacts_.get(), grain_quaternion_.get(), end_velocity_.get(),
     end_omega_.get(), body_delta_.get(), &z, x_next_.get(), &gradient,
     control_.get()},
    &c, sizeof(c), *args_,
    (trying ? kArgsContactsTrying : kArgsContacts) * 16);
}

void shift::core::dem::GpuSolver::Reduce(const void* constants, uint32_t mode,
                                         const gpu::Buffer& a,
                                         const gpu::Buffer& b,
                                         const gpu::Buffer& c_,
                                         const gpu::Buffer& d,
                                         const gpu::Buffer& e, bool trying) {
  DemConstants c = *static_cast<const DemConstants*>(constants);
  c.mode = mode;
  commands_->DispatchIndirect(
    *nsc_reduce_,
    {contacts_.get(), &a, &b, &c_, &d, &e, partials_.get(), control_.get()},
    &c, sizeof(c), *args_, (trying ? kArgsReduceTrying : kArgsReduce) * 16);
}

void shift::core::dem::GpuSolver::Update(const void* constants, uint32_t mode,
                                         const gpu::Buffer& in0,
                                         const gpu::Buffer& in1,
                                         const gpu::Buffer& out,
                                         size_t args) {
  DemConstants c = *static_cast<const DemConstants*>(constants);
  c.mode = mode;
  commands_->DispatchIndirect(
    *nsc_update_, {contacts_.get(), &in0, &in1, &out, control_.get()}, &c,
    sizeof(c), *args_, args * 16);
}

void shift::core::dem::GpuSolver::RecordNsc(const void* constants) {
  gpu::Commands& commands = *commands_;
  DemConstants c = *static_cast<const DemConstants*>(constants);
  const size_t size = sizeof(c);
  const uint32_t groups = Groups(layout_.spheres);
  const bool apgd = settings_.nsc.solver == NscSolver::kApgd;
  /* the contacts: counted, summed, written; the pairs' contacts for the
  higher sphere; the step sizes and the impulses to start from */
  const std::vector<const gpu::Buffer*> contact_buffers = {
    sphere_anchor_.get(),   sphere_offset_.get(),   sphere_arm_.get(),
    sphere_velocity_.get(), sphere_omega_.get(),    neighbor_count_.get(),
    neighbor_.get(),        history_.get(),         wall_history_.get(),
    wall_anchor_.get(),     wall_offset_.get(),     wall_normal_.get(),
    pairs_.get(),           triangle_anchor_.get(), triangle_vertex_.get(),
    triangle_count_.get(),  triangle_list_.get(),   triangle_history_.get(),
    poses_.get(),           body_rows_.get(),       contact_count_.get(),
    contact_start_.get(),   contacts_.get(),        slot_contact_.get(),
    flags_.get(),           grain_contacts_.get(),  body_contacts_.get()};
  commands.Fill(*contact_count_, 0u);
  commands.Fill(*grain_contacts_, 0u);
  commands.Fill(*body_contacts_, 0u);
  c.mode = 0;
  commands.Dispatch(*nsc_contacts_, contact_buffers, &c, size, groups);
  contact_scan_->Record(commands, *contact_count_, *contact_start_);
  Control(&c, 0);
  c.mode = 1;
  commands.Dispatch(*nsc_contacts_, contact_buffers, &c, size, groups);
  commands.Dispatch(*nsc_reverse_,
                    {neighbor_count_.get(), neighbor_.get(),
                     slot_contact_.get(), other_contact_.get()},
                    &c, size, groups);
  c.option = apgd ? 0u : 1u;
  commands.DispatchIndirect(
    *nsc_prepare_,
    {contacts_.get(), grain_quaternion_.get(), grain_anchor_.get(),
     template_mass_.get(), body_rows_.get(), history_.get(),
     wall_history_.get(), triangle_history_.get(), x_.get(), control_.get(),
     grain_contacts_.get(), body_contacts_.get()},
    &c, size, *args_, kArgsContacts * 16);
  const uint32_t iterations = settings_.nsc.iterations;
  if(!apgd) {
    /* Jacobi: each iteration from the impulses of the one before */
    c.mode = 1;
    for(uint32_t it = 0; it < iterations; it++) {
      Velocities(&c, 1, *x_, false);
      commands.DispatchIndirect(
        *nsc_gradient_,
        {contacts_.get(), grain_quaternion_.get(), end_velocity_.get(),
         end_omega_.get(), body_delta_.get(), x_.get(), x_next_.get(),
         g_x_.get(), control_.get()},
        &c, size, *args_, kArgsContacts * 16);
      std::swap(x_, x_next_);
    }
    Reduce(&c, 4, *x_, *g_x_, *x_, *x_, *x_, false);
    Control(&c, 5);
    c.option = 1;
  } else {
    /* APGD (as CpuSolver::NscStep): the gradient at the start and at
    ones, for the first Lipschitz constant */
    gpu::Buffer& g_ones = *g_y_;
    NscGradient(&c, 0, *x_, *g_x_, false);
    NscGradient(&c, 3, *x_, g_ones, false);
    Reduce(&c, 0, *x_, *g_x_, g_ones, *x_, *x_, false);
    Control(&c, 1);
    Update(&c, 1, *x_, *x_, *y_, kArgsContacts);
    Update(&c, 1, *x_, *x_, *best_, kArgsContacts);
    for(uint32_t it = 0; it < iterations; it++) {
      NscGradient(&c, 0, *y_, *g_y_, false);
      Control(&c, 2);
      for(uint32_t t = 0; t < kApgdTries; t++) {
        Update(&c, 0, *y_, *g_y_, *x_next_, kArgsContactsTrying);
        NscGradient(&c, 0, *x_next_, *g_x_, true);
        Reduce(&c, 2, *x_next_, *g_x_, *g_x_, *g_y_, *y_, true);
        Control(&c, 3, t);
      }
      Reduce(&c, 3, *x_next_, *g_x_, *x_next_, *g_y_, *x_, false);
      Control(&c, 4);
      Update(&c, 1, *x_next_, *x_next_, *best_, kArgsBest);
      Update(&c, 2, *x_next_, *x_, *y_, kArgsContacts);
      std::swap(x_, x_next_);
    }
    c.option = 0;
  }
  /* the velocities of the impulses found, which the contacts keep and the
  bodies take, and the grains' motion */
  const gpu::Buffer& found = apgd ? *best_ : *x_;
  Velocities(&c, c.option, found, false);
  commands.Dispatch(*nsc_finish_,
                    {contact_start_.get(), contacts_.get(), &found,
                     history_.get(), wall_history_.get(),
                     triangle_history_.get(), impulse_.get(), control_.get()},
                    &c, size, groups);
  commands.Dispatch(*nsc_grains_,
                    {grain_anchor_.get(), grain_offset_.get(),
                     grain_velocity_.get(), grain_omega_.get(),
                     grain_quaternion_.get(), end_velocity_.get(),
                     end_omega_.get()},
                    &c, size, Groups(grains_));
}

void shift::core::dem::GpuSolver::CheckFlags(const uint32_t* flags) {
  statistics_.searches = flags[3];
  statistics_.most_neighbors = flags[2];
  statistics_.most_triangles = flags[4];
  if((flags[1] & 1u) != 0u) {
    throw shift::core::Exception(
      "DEM: a sphere has " + std::to_string(flags[2]) +
      " neighbors, more than max_neighbors (" +
      std::to_string(settings_.max_neighbors) + ")");
  }
  if((flags[1] & 2u) != 0u) {
    throw shift::core::Exception(
      "DEM: a triangle is in more than 8 cells of the grid");
  }
  if((flags[1] & 8u) != 0u) {
    throw shift::core::Exception(
      "DEM: " + std::to_string(flags[5]) +
      " contacts within the margin (NSC), more than the room for them (" +
      std::to_string(capacity_) + ", max_neighbors / 2 + 1 per sphere)");
  }
  if((flags[1] & 4u) != 0u) {
    throw shift::core::Exception(
      "DEM: a sphere is near " + std::to_string(flags[4]) +
      " triangles, more than max_triangles (" +
      std::to_string(settings_.max_triangles) + ")");
  }
}

void shift::core::dem::GpuSolver::Step(float dt) {
  Steps(dt, 1);
}

void shift::core::dem::GpuSolver::Steps(float dt, long n) {
  const long batch = nsc_ ? kNscBatch : kBatch;
  for(long first = 0; first < n; first += batch) {
    const long count = std::min(batch, n - first);
    /* the bodies' poses at the steps, at the times the steps reach by
    adding dt, as the CPU solver does, and, for the non-smooth method, at
    the end of the last, and their masses */
    const size_t B = bodies_.size();
    if(B > 0) {
      const long poses = nsc_ ? count + 1 : count;
      std::vector<std::array<float, 4>> rows(size_t(poses) * 7 * B), step,
        mass_rows, masses;
      double time = time_;
      for(long k = 0; k < poses; k++) {
        AnchoredPosesAt(time, step);
        std::copy(step.begin(), step.end(), rows.begin() + k * 7 * B);
        if(nsc_ && k < count) {
          BodyMassRows(step, mass_rows);
          masses.insert(masses.end(), mass_rows.begin(), mass_rows.end());
        }
        time += dt;
      }
      commands_->Update(*poses_, rows.data(), rows.size() * 16);
      if(nsc_) {
        commands_->Update(*body_rows_, masses.data(), masses.size() * 16);
      }
    }
    for(long k = 0; k < count; k++) {
      Record(dt, static_cast<uint32_t>(k));
    }
    /* the flags, and after the last batch the impulses' sums, read back
    together */
    const bool last = first + count >= n;
    commands_->Copy(*flags_, *readback_, 32);
    if(last && B > 0) {
      impulse_sum_->Record(*commands_, *impulse_, *impulse_sums_);
      commands_->Copy(*impulse_sums_, *readback_, 6 * B * 4, 0, 32);
    }
    const size_t control_at = 8 + 6 * B;
    if(nsc_) {
      commands_->Copy(*control_, *readback_, kControlWords * 4, 0,
                      control_at * 4);
    }
    commands_->Submit();
    std::vector<uint32_t> back(nsc_ ? control_at + kControlWords
                                    : 8 + (last ? 6 * B : 0));
    readback_->Download(back.data(), back.size() * 4);
    if(last && B > 0) {
      sums_.resize(6 * B);
      std::memcpy(sums_.data(), back.data() + 8, 6 * B * 4);
    }
    if(nsc_) {
      /* the count and the residual of NscControl */
      statistics_.contacts = back[control_at];
      std::memcpy(&statistics_.residual, &back[control_at + 10], 4);
    }
    for(long k = 0; k < count; k++) {
      time_ += dt;
    }
    statistics_.steps += count;
    CheckFlags(back.data());
  }
}

void shift::core::dem::GpuSolver::GetState(State& state) {
  std::vector<std::array<int32_t, 4>> anchor(grains_);
  std::vector<std::array<float, 4>> offset(grains_), velocity(grains_),
    omega(grains_);
  grain_anchor_->Download(anchor.data(), grains_ * 16);
  grain_offset_->Download(offset.data(), grains_ * 16);
  grain_velocity_->Download(velocity.data(), grains_ * 16);
  grain_omega_->Download(omega.data(), grains_ * 16);
  FromLayout(anchor, offset, state);
  grain_quaternion_->Download(state.orientation.data(), grains_ * 16);
  for(size_t g = 0; g < grains_; g++) {
    for(int k = 0; k < 3; k++) {
      state.velocity[g][k] = velocity[g][k];
      state.angular_velocity[g][k] = omega[g][k];
    }
  }
}

shift::core::dem::Statistics shift::core::dem::GpuSolver::GetStatistics() {
  return statistics_;
}

void shift::core::dem::GpuSolver::ClearImpulses() {
  /* run with the next steps */
  commands_->Fill(*impulse_, 0u);
  std::fill(sums_.begin(), sums_.end(), 0.0f);
}

void shift::core::dem::GpuSolver::GetImpulses(
  std::vector<std::array<double, 6>>& impulses) {
  const size_t B = bodies_.size();
  impulses.assign(B, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  if(B == 0) {
    return;
  }
  for(size_t c = 0; c < 6; c++) {
    for(size_t b = 0; b < B; b++) {
      impulses[b][c] = sums_[c * B + b];
    }
  }
}

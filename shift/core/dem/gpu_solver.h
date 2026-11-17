#ifndef SHIFT_DEM_GPU_SOLVER_H_
#define SHIFT_DEM_GPU_SOLVER_H_

#include <memory>
#include <vector>

#include "shift/core/dem/dem.h"
#include "shift/core/gpu/device.h"
#include "shift/core/gpu/scan.h"
#include "shift/core/gpu/segment_sum.h"

namespace shift {
namespace core {
namespace dem {

/*!
@brief DEM on the GPU (see dem.h), on a gpu::Device

The grains, spheres and neighbor lists stay on the device; each step is a
sequence of the passes in shift/core/dem/shaders, and up to 256 steps go in
one submission. Whether a step searches for neighbors is decided on the
device: the search passes run every step and return at once unless a
sphere has moved half the margin. After each submission the solver reads
the flags back, and throws if a sphere had more neighbors than
Settings::max_neighbors.

The bodies' poses at each step of a submission are recorded into it, and the
impulses on the bodies, which stay on the device per sphere, are summed
there (gpu::SegmentSum) at the end of the steps and read back with the
flags: Steps waits for the device once per submission.

The non-smooth method (ContactMethod::kNsc) runs with APGD or Jacobi; the
Gauss-Seidel method, one contact after another, has no parallel form here.
Each step lists the contacts within the margin, sphere by sphere in the
order of the CPU solver, into room for max_neighbors / 2 + 1 per sphere
(the counts summed by gpu::Scan), and each grain sums the impulses of its
own contacts in the CPU's order, so that no sum needs atomic additions.
The sums over all contacts that APGD needs are trees over workgroups, and
its decisions (whether a step descends, whether the momentum restarts,
whether an iterate is the best) are made on the device, which sets the
workgroup counts of the dispatches that follow (indirect dispatches): its
kApgdTries tries of each iteration are recorded, and those after the step
is accepted run no workgroups. The steps go 4 to a submission.
*/
class GpuSolver : public Solver {
public:
  GpuSolver(gpu::Device& device, const Settings& settings,
            const std::vector<Material>& materials,
            const std::vector<Template>& templates, const State& state,
            const std::vector<Body>& bodies = {});
  ~GpuSolver() override;

  void Step(float dt) override;
  /*! \brief n steps of dt, run in submissions of up to 256 */
  void Steps(float dt, long n) override;
  void GetState(State& state) override;
  Statistics GetStatistics() override;
  void ClearImpulses() override;
  void GetImpulses(std::vector<std::array<double, 6>>& impulses) override;

private:
  /* a step, reading the bodies' poses of substep of the submission */
  void Record(float dt, uint32_t substep);
  /* the passes of a step of the non-smooth method after the search, and
  their parts */
  void RecordNsc(const void* constants);
  void Velocities(const void* constants, uint32_t option,
                  const gpu::Buffer& impulse, bool trying);
  void NscGradient(const void* constants, uint32_t option,
                   const gpu::Buffer& z, const gpu::Buffer& gradient,
                   bool trying);
  void Reduce(const void* constants, uint32_t mode, const gpu::Buffer& a,
              const gpu::Buffer& b, const gpu::Buffer& c, const gpu::Buffer& d,
              const gpu::Buffer& e, bool trying);
  void Control(const void* constants, uint32_t stage, uint32_t option = 0);
  void Update(const void* constants, uint32_t mode, const gpu::Buffer& in0,
              const gpu::Buffer& in1, const gpu::Buffer& out, size_t args);
  /* the flags read back after a submission; throws on too many neighbors
  or triangles, or a triangle in too many cells */
  void CheckFlags(const uint32_t* flags);

  gpu::Device& device_;
  uint32_t grains_ = 0;
  uint32_t table_size_ = 0;
  /* the grains */
  std::unique_ptr<gpu::Buffer> grain_anchor_, grain_offset_, grain_velocity_,
    grain_omega_, grain_quaternion_, first_sphere_;
  /* the templates, walls and pairs of materials */
  std::unique_ptr<gpu::Buffer> template_mass_, template_spheres_,
    template_sphere_, template_material_, template_hull_, hull_vertex_,
    hull_plane_, hull_face_, wall_anchor_, wall_offset_, wall_normal_, pairs_;
  /* the spheres */
  std::unique_ptr<gpu::Buffer> sphere_info_, sphere_anchor_, sphere_offset_,
    sphere_arm_, sphere_velocity_, sphere_omega_, reference_anchor_,
    reference_offset_, sphere_force_, sphere_torque_;
  /* the neighbors and the walls' histories */
  std::unique_ptr<gpu::Buffer> neighbor_count_, neighbor_, history_,
    wall_history_;
  /* the grid, and the flags: search wanted, too many neighbors, the most
  neighbors, the searches run */
  std::unique_ptr<gpu::Buffer> hashes_, counts_, start_, cursor_, order_,
    flags_;
  std::unique_ptr<gpu::Scan> scan_;
  /* the triangles of the bodies and their grid; each sphere's triangles;
  the bodies' poses of a submission, and the impulses on them per
  component, body and sphere, and their sums */
  uint32_t triangle_table_size_ = 0;
  std::unique_ptr<gpu::Buffer> triangle_local_, triangle_anchor_,
    triangle_vertex_, reference_triangle_anchor_, reference_triangle_vertex_,
    triangle_hashes_, triangle_counts_, triangle_start_, triangle_cursor_,
    triangle_order_, triangle_count_, triangle_list_, triangle_history_,
    poses_, impulse_, impulse_sums_;
  /* what the host reads after each submission: the flags (8 words) and,
  after the last of the steps, the impulses' sums; the sums, per component
  and body */
  std::unique_ptr<gpu::Buffer> readback_;
  std::vector<float> sums_;
  std::unique_ptr<gpu::Scan> triangle_scan_;
  std::unique_ptr<gpu::SegmentSum> impulse_sum_;
  std::unique_ptr<gpu::Kernel> spheres_kernel_, count_kernel_, scatter_,
    cell_sort_, neighbors_, flags_kernel_, forces_, grains_kernel_,
    triangles_kernel_, triangle_count_kernel_;
  std::unique_ptr<gpu::Commands> commands_;
  /* the non-smooth method: the room for contacts; the contacts of each
  sphere, counted and summed, the contacts; each sphere's contact of each
  of its pairs it is the lower sphere of, and of those it is the higher;
  the bodies' masses and inverse inertias of a submission; the grains'
  velocities for the impulses, the bodies' changes of velocity, the
  partial sums of the bodies and of the reductions, the solver's state
  (NscControl) and the workgroup counts of the indirect dispatches; the
  impulses (Jacobi: x and its next, x_next) or APGD's vectors */
  bool nsc_ = false;
  bool massive_bodies_ = false;
  uint32_t capacity_ = 0;
  std::unique_ptr<gpu::Buffer> contact_count_, contact_start_, contacts_,
    slot_contact_, other_contact_, body_rows_, end_velocity_, end_omega_,
    body_delta_, body_partials_, partials_, control_, args_, x_, x_next_, y_,
    best_, g_x_, g_y_, grain_contacts_, body_contacts_;
  std::unique_ptr<gpu::Scan> contact_scan_;
  std::unique_ptr<gpu::Kernel> nsc_contacts_, nsc_reverse_, nsc_prepare_,
    nsc_velocities_, nsc_body_reduce_, nsc_body_sum_, nsc_gradient_,
    nsc_update_, nsc_reduce_, nsc_control_, nsc_finish_, nsc_grains_;
  Statistics statistics_;
};

} // namespace dem
} // namespace core
} // namespace shift

#endif

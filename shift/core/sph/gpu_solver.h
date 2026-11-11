#ifndef SHIFT_SPH_GPU_SOLVER_H_
#define SHIFT_SPH_GPU_SOLVER_H_

#include <memory>
#include <vector>

#include "shift/core/gpu/device.h"
#include "shift/core/gpu/scan.h"
#include "shift/core/gpu/segment_sum.h"
#include "shift/core/sph/sph.h"

namespace shift {
namespace core {
namespace sph {

/*!
@brief SPH on the GPU (see sph.h), on a gpu::Device

The particles and markers stay on the device; each substep is one submission
of the passes in shift/core/sph/shaders. The sums run in the order of the
sort, by cell and then id, so a run gives the same results each time.
*/
class GpuSolver : public Solver {
public:
  GpuSolver(gpu::Device& device, const Settings& settings,
            const std::vector<Material>& materials, const State& state);
  ~GpuSolver() override;

  void Step(float dt) override;
  /*! \brief n substeps of dt, run in submissions of up to 64 */
  void Steps(float dt, long n) override;
  void GetState(State& state) override;
  void ClearImpulses() override;
  void GetImpulses(std::vector<std::array<double, 6>>& impulses) override;

private:
  /* the arrays of the particles and markers */
  struct Arrays {
    std::unique_ptr<gpu::Buffer> position, velocity, density, stress, id,
      kind;
  };

  /* a substep; substep is its index in the batch, for the bodies' poses */
  void Record(float dt, uint32_t substep);
  /* the bodies' poses at the substeps of a batch, n from time_ */
  void UploadPoses(float dt, long n);

  gpu::Device& device_;
  uint32_t count_ = 0;
  uint32_t table_size_ = 0;
  float h_ = 0.0f;
  Arrays a_, b_; //!< a: after a substep; b: sorted
  std::unique_ptr<gpu::Buffer> materials_buffer_;
  std::unique_ptr<gpu::Buffer> hashes_, counts_, start_, cursor_, order_;
  std::unique_ptr<gpu::Scan> scan_;
  std::unique_ptr<gpu::Kernel> count_kernel_, scatter_, cell_sort_, gather_,
    markers_, rates_, momentum_, bodies_kernel_;
  /* the bodies: their markers in them, their poses at the substeps of a
  batch, the impulses of the markers and their sums per body */
  uint32_t first_body_marker_ = 0;
  uint32_t body_markers_ = 0;
  bool placed_ = false; //!< whether the poses of the batch are uploaded
  std::unique_ptr<gpu::Buffer> body_local_, poses_, impulse_, sums_;
  std::unique_ptr<gpu::SegmentSum> segment_sum_;
  std::unique_ptr<gpu::Commands> commands_;
};

} // namespace sph
} // namespace core
} // namespace shift

#endif

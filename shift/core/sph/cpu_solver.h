#ifndef SHIFT_SPH_CPU_SOLVER_H_
#define SHIFT_SPH_CPU_SOLVER_H_

#include <array>
#include <cstdint>
#include <vector>

#include "shift/core/sph/sph.h"

namespace shift {
namespace core {
namespace sph {

/*!
@brief SPH on the CPU, single threaded: the reference for the GPU solver on
small problems (see sph.h); the same passes, sums in the same order, float32
*/
class CpuSolver : public Solver {
public:
  CpuSolver(const Settings& settings, const std::vector<Material>& materials,
            const State& state);

  void Step(float dt) override;
  void GetState(State& state) override;
  void RemoveParticles(const std::vector<uint32_t>& ids) override;
  size_t size() const override {
    return state_.size();
  }
  void ClearImpulses() override;
  void GetImpulses(std::vector<std::array<double, 6>>& impulses) override;

private:
  /* the neighbors' cells of a point: their hashes, each once, in the order
  the GPU visits them */
  int NeighborCells(const std::array<float, 4>& x, uint32_t hashes[27]) const;
  /* the passes of a substep, on sorted_ */
  void PlaceBodies();
  void Sort();
  void Markers();
  void Rates(float dt);
  void Momentum(float dt);
  /* the force on body marker i of sorted_, into its impulse */
  void BodyMarker(size_t i, float dt);

  std::vector<std::array<float, 8>> table_;
  float h_ = 0.0f;
  float cell_ = 0.0f;
  uint32_t mask_ = 0;
  State state_;  //!< after the last substep
  State sorted_; //!< sorted by cell, then id
  std::vector<uint32_t> start_;
  /* the bodies: their poses now, as the GPU's table, and the impulses of
  their markers, six floats each by id, component by component */
  std::vector<BodyPose> poses_;
  std::vector<float> impulse_;
};

} // namespace sph
} // namespace core
} // namespace shift

#endif

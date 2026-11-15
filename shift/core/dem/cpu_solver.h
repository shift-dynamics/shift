#ifndef SHIFT_DEM_CPU_SOLVER_H_
#define SHIFT_DEM_CPU_SOLVER_H_

#include <array>
#include <cstdint>
#include <vector>

#include "shift/core/dem/dem.h"

namespace shift {
namespace core {
namespace dem {

/*!
@brief DEM on the CPU, single threaded: the reference for the GPU solver on
small problems (see dem.h); the same passes as the shaders in
shift/core/dem/shaders, the sums in the same order, float32
*/
class CpuSolver : public Solver {
public:
  CpuSolver(const Settings& settings, const std::vector<Material>& materials,
            const std::vector<Template>& templates, const State& state,
            const std::vector<Body>& bodies = {});

  void Step(float dt) override;
  void GetState(State& state) override;
  Statistics GetStatistics() override;
  void ClearImpulses() override;
  void GetImpulses(std::vector<std::array<double, 6>>& impulses) override;

private:
  /* the passes of a step */
  bool Spheres();
  bool Triangles();
  void Search();
  void SearchTriangles();
  void Forces(float dt);
  void TriangleForces(uint32_t s, float dt, float force[3], float torque[3]);
  void HullTriangleForces(uint32_t s, float dt, float force[3],
                          float torque[3]);
  void Grains(float dt);
  /* a step of the non-smooth method */
  void NscStep(float dt);

  /*! a contact of the non-smooth method: grain a, and grain b or a wall or
   * a body; the arms from their centers of mass (from the body's origin),
   * the normal from b to a and two tangents, the arm from a body's center
   * of mass, the given velocity and angular
   * velocity of a wall or body at the contact, the bias, the coefficients,
   * the step sizes of the iterations, the impulse in the normal and tangent
   * directions and the rolling impulse, and where the impulse is kept for
   * the next step */
  struct NscContact {
    uint32_t a = 0, b = 0, sphere = 0, body = 0;
    float la[3], lb[3], lc[3], n[3], t1[3], t2[3], vb[3], wb[3];
    float bias = 0.0f, mu = 0.0f, rolling = 0.0f;
    float eta = 0.0f, eta_r = 0.0f;
    float gamma[3] = {0.0f, 0.0f, 0.0f};
    float rho[3] = {0.0f, 0.0f, 0.0f};
    std::array<float, 4>* kept = nullptr;
  };
  std::vector<NscContact> contacts_;

  /* the grains */
  std::vector<std::array<float, 4>> velocity_, omega_, quaternion_;
  /* the spheres: anchor (w: grain), offset (w: radius), arm (w: material),
  velocity (w: grain mass), angular velocity, and where they were at the
  last search */
  std::vector<std::array<int32_t, 4>> sphere_anchor_, reference_anchor_;
  std::vector<std::array<float, 4>> sphere_offset_, sphere_arm_,
    sphere_velocity_, sphere_omega_, reference_offset_;
  std::vector<uint32_t> sphere_material_;
  /* the neighbors: max_neighbors per sphere, their histories, and the
  walls' histories, 8 per sphere */
  std::vector<uint32_t> neighbor_count_, neighbor_;
  std::vector<std::array<float, 4>> history_, wall_history_;
  std::vector<std::array<float, 4>> sphere_force_, sphere_torque_;
  /* the triangles: anchor (w: body) and vertices, where they were at the
  last search, each sphere's triangles and their histories; the bodies'
  poses at this step (AnchoredPosesAt) and the impulses on them, per
  component, body and sphere */
  std::vector<std::array<int32_t, 4>> triangle_anchor_,
    reference_triangle_anchor_;
  std::vector<std::array<float, 4>> triangle_vertex_,
    reference_triangle_vertex_;
  std::vector<uint32_t> triangle_count_, triangle_list_;
  std::vector<std::array<float, 4>> triangle_history_;
  std::vector<std::array<float, 4>> poses_;
  std::vector<float> impulse_;
  bool search_ = true;
  Statistics statistics_;
};

} // namespace dem
} // namespace core
} // namespace shift

#endif

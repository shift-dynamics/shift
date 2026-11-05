#ifndef SHIFT_FORCEMODULE_RUBBER_MOUNT_H_
#define SHIFT_FORCEMODULE_RUBBER_MOUNT_H_

#include <memory>
#include <string>
#include <vector>

#include "shift/core/force_module/bushing.h"

namespace shift {
namespace core {
namespace force_module {

/*!
@brief A rubber mount or bushing whose stiffness and damping depend on the
frequency of the motion

Rubber stiffens and its damping falls as the motion grows faster. The mount
is a bushing (see Bushing: the same frames, relative coordinates x and rates
v, rest pose, static stiffness K, damping C and preload) with, in parallel,
a hardening term and spring-damper pairs in series (Maxwell elements) on
each coordinate:

  f = -K x - C v + preload - k3 x^3 - sum over elements k_e s_e

Each element is a spring of stiffness k_e in series with a damper of
coefficient c_e; s_e is the stretch of its spring, one state variable, with

  ds_e/dt = v - s_e / tau_e,  tau_e = c_e / k_e.

Slow motion relaxes the springs through their dampers, leaving the static
stiffness; motion much faster than 1 / tau_e loads the springs, adding k_e
to the stiffness. In between, an element damps the most, near the frequency
1 / (2 pi tau_e). Several elements with time constants spread over the
frequencies of interest fit measured dynamic stiffness and loss angle (a
Prony series). The springs start relaxed, s_e = 0. For the rotations the
rate v is the relative angular velocity, which is the rate of the rotation
vector for small rotations.

JSON:
\code
{"type": "rubber_mount", "name": "cab_mount",
 "frames": [["chassis", "cab_mount"], ["cab", "mount"]],
 "stiffness": [2e5, 2e5, 5e5, 3e3, 3e3, 1e3],
 "damping": [200, 200, 500, 3, 3, 1],
 "cubic_stiffness": [0, 0, 5e9, 0, 0, 0],
 "maxwell_stiffness": [1e5, 1e5, 2e5, 1e3, 1e3, 5e2,
                       5e4, 5e4, 1e5, 5e2, 5e2, 2e2],
 "maxwell_damping": [1e3, 1e3, 2e3, 10, 10, 5,
                     50, 50, 100, 0.5, 0.5, 0.2]}
\endcode

- frames, stiffness, damping, stiffness_file, damping_file, preload,
  rest_pose: as for Bushing.
- cubic_stiffness (optional): 6 m numbers, k3 of each coordinate, N/m^3 or
  N m/rad^3.
- maxwell_stiffness, maxwell_damping (optional): 6 m numbers for each
  element, element by element: the k_e and c_e of every coordinate of the
  first element, then of the second, and so on. A zero stiffness leaves the
  coordinate without that element.
*/
class RubberMount : public Bushing {
public:
  RubberMount()
    : Bushing() {
    DeclareMountParameters();
  }
  RubberMount(std::string name)
    : Bushing(name) {
    DeclareMountParameters();
  }
  virtual ~RubberMount() override {}

  static std::unique_ptr<RubberMount> Create();
  static bool is_registered() {
    return registered_;
  }

  void Load() override;

  /*! \brief number of Maxwell elements with nonzero stiffness, each one
   * state */
  size_t get_element_count() const {
    return elements_.size();
  }

protected:
  void AddForces(const double* y, double* dy) override;

private:
  struct Element {
    size_t coordinate = 0; //!< index into x of the pairs, 0 .. 6 m - 1
    double stiffness = 0.0;
    double rate = 0.0; //!< 1 / tau = k / c
  };

  void DeclareMountParameters();

  std::vector<double> cubic_stiffness_;
  std::vector<double> maxwell_stiffness_;
  std::vector<double> maxwell_damping_;
  std::vector<Element> elements_;

  static bool registered_;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif

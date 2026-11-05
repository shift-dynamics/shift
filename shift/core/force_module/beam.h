#ifndef SHIFT_FORCEMODULE_BEAM_H_
#define SHIFT_FORCEMODULE_BEAM_H_

#include <memory>
#include <string>
#include <vector>

#include "shift/core/force_module/bushing.h"

namespace shift {
namespace core {
namespace force_module {

/*!
@brief A straight, uniform, elastic beam between frames on two bodies

The beam runs from the origin of frame A, along A's x axis, to the origin of
frame B, which must lie on that axis as the model starts; its length
\f$l\f$ is the distance between them, and the axes of its cross section are A's y and
z. The beam is straight and unloaded as the model starts.

The forces are those of a linear beam element (Euler-Bernoulli): end B moves
relative to end A as the tip of a beam clamped at A, and the force and moment
on B that hold it there are

\f[
\begin{aligned}
\text{tension} &: & f_x &= \frac{e a}{l} u \\
\text{bending in x-y} &: & f_y &= \frac{12 e i_z}{l^3} v - \frac{6 e i_z}{l^2} r_z, &
  m_z &= -\frac{6 e i_z}{l^2} v + \frac{4 e i_z}{l} r_z \\
\text{bending in x-z} &: & f_z &= \frac{12 e i_y}{l^3} w + \frac{6 e i_y}{l^2} r_y, &
  m_y &= \frac{6 e i_y}{l^2} w + \frac{4 e i_y}{l} r_y \\
\text{torsion} &: & m_x &= \frac{g j}{l} r_x
\end{aligned}
\f]

with \f$(u, v, w)\f$ the displacement of B and \f$(r_x, r_y, r_z)\f$ its
rotation relative to A, in A's axes. The beam applies the opposite, restoring forces
to B and the equal and opposite load to A, so the two bodies are in
equilibrium with the beam. The relative motion is measured as for Bushing,
so the beam suits deflections that are small relative to its length; a
chain of beams on light bodies gives a simple flexible link.

Damping proportional to the stiffness, \f$C = \mathrm{damping} \, K\f$, makes
each mode decay with a damping ratio of \f$\mathrm{damping} \cdot \omega / 2\f$,
with \f$\omega\f$ its angular frequency.

JSON:
\code
{"type": "beam", "name": "link",
 "frames": [["base", "beam_root"], ["tip", "beam_end"]],
 "young_modulus": 200e9, "poisson_ratio": 0.3,
 "section": "tube", "diameter": 0.05, "inner_diameter": 0.04,
 "damping": 1e-4}
\endcode

- frames: [body, frame] of ends A and B.
- young_modulus; shear_modulus, or poisson_ratio for \f$g = e / (2 (1 + \nu))\f$.
- section: "circle" (diameter), "tube" (diameter, inner_diameter),
  "rectangle" (width along y, height along z) or "general" (default:
  area, second moments Iy and Iz about the y and z axes, and torsion
  constant J).
- damping (optional, default 0): the stiffness-proportional damping
  coefficient, in seconds.
*/
class Beam : public Bushing {
public:
  Beam()
    : Bushing() {
    DeclareBeamParameters();
  }
  Beam(std::string name)
    : Bushing(name) {
    DeclareBeamParameters();
  }
  virtual ~Beam() override {}

  static std::unique_ptr<Beam> Create();
  static bool is_registered() {
    return registered_;
  }

  void Load() override;

  double get_length() const {
    return length_;
  }
  /*! \brief the section's area, second moments about y and z, and torsion
   * constant */
  double get_area() const {
    return area_;
  }
  double get_Iy() const {
    return Iy_;
  }
  double get_Iz() const {
    return Iz_;
  }
  double get_J() const {
    return J_;
  }
  double get_shear_modulus() const {
    return G_;
  }

private:
  void DeclareBeamParameters();
  void Section(const std::string& prefix);

  double young_modulus_ = 0.0;
  double shear_modulus_ = 0.0;
  double poisson_ratio_ = -1.0;
  std::string section_ = "general";
  double diameter_ = 0.0;
  double inner_diameter_ = 0.0;
  double width_ = 0.0;
  double height_ = 0.0;
  double area_ = 0.0;
  double Iy_ = 0.0;
  double Iz_ = 0.0;
  double J_ = 0.0;
  double beam_damping_ = 0.0;

  double length_ = 0.0;
  double G_ = 0.0;

  static bool registered_;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif

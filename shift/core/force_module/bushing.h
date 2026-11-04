#ifndef SHIFT_FORCEMODULE_BUSHING_H_
#define SHIFT_FORCEMODULE_BUSHING_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/coscrew.h"

#include "shift/core/force_module/base.h"

namespace shift {
namespace core {

class Frame;

namespace body {
class Base;
}

namespace force_module {

/*!
@brief Forces and moments between frames on bodies from full stiffness and
damping matrices of their relative displacements and velocities

Two frames, I on one body and J on another, are joined as by a bushing whose
stiffness and damping have been measured or computed by finite elements:
every direction of motion may load every other. A group of n frames joins
each of its n (n - 1) / 2 pairs, with one set of matrices over all the
pairs, as for a simplified flexible structure.

For each pair (i, j), i < j, in the order (0, 1), (0, 2), ..., (0, n - 1),
(1, 2), ..., the six relative coordinates are those of frame j relative to
frame i, measured from the rest pose of the pair and expressed in the rest
pose's axes (the axes of frame i moved by the rest pose):

  x = [dx, dy, dz, rx, ry, rz]

where d is the displacement of the origin of j and r the rotation vector
(axis times angle, radians) of its rotation. Their rates are the velocity of
the origin of j relative to the point of i's body there, and the angular
velocity of j relative to i, in the same axes.

The force and moment on frame j, at its origin and in the same axes, is

  f = -K x - C dx/dt + preload

and the opposite force and moment act on frame i's body at the same point.
K and C are 6 m x 6 m for m pairs, in the order of the pairs, so the forces
of a pair may depend on the motion of every pair. The rotation vector makes
the coordinates exact for any rotation, but a constant matrix describes the
bushing best for small rotations, as measured stiffness does.

JSON:
\code
{"type": "bushing", "name": "cab_mount",
 "frames": [["chassis", "cab_mount"], ["cab", "mount"]],
 "stiffness": [2e5, 2e5, 5e5, 3e3, 3e3, 1e3],
 "damping": [2e3, 2e3, 5e3, 30, 30, 10]}
\endcode

- frames: [body, frame] of each frame, two or more; "fixed" is the fixed
  body.
- stiffness, damping (optional, default zero): 6 m numbers for a diagonal
  matrix, or (6 m)^2 numbers for the full matrix, row by row, in N/m, N m/rad,
  N s/m and N m s/rad (or the model's units); the force row of a pair times
  its displacement column, and so on.
- stiffness_file, damping_file (optional): the full matrix as text, (6 m)^2
  numbers separated by blanks, commas or line breaks, row by row, next to
  the model file or in an include directory; in place of stiffness or
  damping.
- preload (optional): 6 m numbers, the force and moment of each pair at
  rest.
- rest_pose (optional): "initial" (default), the relative poses of the
  frames as the model starts, or "frames", where the frames of each pair
  coincide.
*/
class Bushing : public Base {
public:
  Bushing()
    : Base() {
    DeclareParameters();
  }
  Bushing(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Bushing() override {}

  static std::unique_ptr<Bushing> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  /*! \brief logged outputs: the force and moment f of each pair k (as in
   * get_forces), pair_<k>_fx, _fy, _fz, _mx, _my, _mz */
  std::vector<std::string> get_output_names() const override;
  void UpdateOutputs(double t, const double* y) override;
  void AppendOutputs(std::vector<double>& values) const override;

  /*! \brief number of frame pairs, m */
  size_t get_pair_count() const {
    return pairs_.size();
  }
  /*! \brief relative coordinates x and rates of all pairs (6 m each), and
   * the force and moment f on frame j of each pair in the rest axes, as of
   * the last Service or UpdateOutputs */
  const std::vector<double>& get_displacements() const {
    return x_;
  }
  const std::vector<double>& get_velocities() const {
    return v_;
  }
  const std::vector<double>& get_forces() const {
    return f_;
  }

protected:
  struct Pair {
    size_t i = 0; //!< index of frame i in frames_
    size_t j = 0;
    ksl_SE3_t rest = ksl_SE3_identity(); //!< pose of j in i at rest
  };
  struct Attachment {
    body::Base* body = nullptr;
    const Frame* frame = nullptr;
    int load_index = -2; //!< see Context::GetBodyLoadIndex; -2: not yet
  };

  virtual void DeclareParameters() override;
  /*! \brief the relative coordinates x_ and rates v_ of every pair, from
   * the current poses and velocities of the bodies */
  void Measure();
  /*! \brief x_, v_ and f_ for the state y, and the rates of the states of
   * a derived module in dy unless it is null */
  void Compute(const double* y, double* dy);
  /*! \brief add forces of a derived module to f_, and set the rates of its
   * states, if dy is not null; after f_ = -K x - C v + preload */
  virtual void AddForces(const double* y, double* dy) {
    (void)y;
    (void)dy;
  }
  /*! \brief apply f_ of every pair to the bodies of its frames */
  void Apply();

  std::vector<Pair> pairs_;
  std::vector<double> x_, v_, f_;

private:
  /*! \brief the full matrix of a parameter: from its values (diagonal or
   * full) or its file; zero if neither is given */
  std::vector<double> Matrix(const std::string& key,
                             const std::vector<double>& values,
                             const std::string& file) const;
  std::string FindFile(const std::string& name) const;
  /*! \brief pose of attachment a in the fixed frame */
  ksl_SE3_t Pose(const Attachment& a) const;

  std::vector<std::vector<std::string>> frame_names_;
  std::vector<double> stiffness_values_;
  std::vector<double> damping_values_;
  std::string stiffness_file_;
  std::string damping_file_;
  std::vector<double> preload_;
  std::string rest_pose_ = "initial";

  std::vector<Attachment> attachments_;
  std::vector<double> stiffness_; //!< 6 m x 6 m, row major
  std::vector<double> damping_;
  std::vector<ksl_SE3_t> poses_;   //!< of the attachments, last Measure
  std::vector<ksl_mat3x3_t> axes_; //!< rest axes of the pairs in the fixed
                                   //!< frame, last Measure

  static bool registered_;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif

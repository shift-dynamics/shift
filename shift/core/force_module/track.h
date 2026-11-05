#ifndef SHIFT_CORE_FORCE_MODULE_TRACK_H_
#define SHIFT_CORE_FORCE_MODULE_TRACK_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/vec3.h"
#include "shift/core/force_module/base.h"
#include "shift/core/parameter/array.h"

namespace shift {
namespace core {

namespace body {
class Base;
}

namespace force_module {

/*!
@brief a span of a track between two supports (wheels or rollers), whose
sag the track module logs
*/
class TrackSpan : public parameter::List {
public:
  TrackSpan() {
    DeclareParameters();
  }
  virtual ~TrackSpan() {}

  static const std::string get_array_key() {
    return "spans";
  }

  std::string name;
  std::vector<std::string> supports; //!< the two bodies at its ends
  std::vector<double> radii; //!< from each one's origin to the bushings

protected:
  virtual void DeclareParameters() override;
};

/*!
@brief outputs of a track of links on their own joints (see the track
loops of examples/cad): the sag of its spans, the misguide of its links and
the side force on the flanges of its rollers. It applies no force.

The outputs are measured in the coordinates of the track frame (frame, a
body carrying the wheels and rollers, the fixed body by default), with its
directions up and lateral (along the axles):

- <span>_sag, of each span between two supports, the wheels or rollers at
  its ends: the greatest distance (along up) of the origins of the links
  (their bushings) between the two supports below the line through the
  supports' tops, each support's origin plus its radius along up; the
  radius of a wheel is that of the circle of the bushings around it, that of
  a roller the distance from its axle to the bushings of the links on it.
  Links whose bushings are above the line count as 0, and links below the
  line through the supports' origins (those of the other run) are left out.
- misguide: the greatest lateral distance of a link's origin from the
  middle of the track, the plane through the frame's origin plus center
  along lateral.
- side_force: the sum of the magnitudes of the lateral components of the
  contact forces between the links and the guides (the rollers whose
  flanges guide the links), as of the last contact force evaluation.

JSON:
\code
{"type": "track", "name": "track", "frame": "frame",
 "links": ["link1", "link2", ...], "guides": ["roller1", "carrier1"],
 "up": [0, 0, -1], "lateral": [0, 1, 0], "center": 0,
 "spans": [{"name": "front", "supports": ["idler", "carrier1"],
            "radii": [0.322, 0.1205]}]}
\endcode
*/
class Track : public Base {
public:
  Track()
    : Base() {
    DeclareParameters();
  }
  Track(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Track() override {}

  static std::unique_ptr<Track> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  std::vector<std::string> get_output_names() const override;
  void UpdateOutputs(double t, const double* y) override;
  void AppendOutputs(std::vector<double>& values) const override;

  /*! \brief the outputs of the current poses and contact forces, in the
   * order of get_output_names */
  std::vector<double> Measure();

protected:
  void DeclareParameters() override;

private:
  static bool registered_;
  std::string frame_name_ = "fixed";
  std::vector<std::string> link_names_;
  std::vector<std::string> guide_names_;
  std::vector<double> up_ = {0.0, 0.0, -1.0};
  std::vector<double> lateral_ = {0.0, 1.0, 0.0};
  double center_ = 0.0;
  parameter::Array<TrackSpan> spans_;

  body::Base* frame_ = nullptr;
  std::vector<body::Base*> links_;
  std::vector<body::Base*> guides_;
  std::vector<std::array<body::Base*, 2>> supports_;
  std::vector<double> outputs_;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif

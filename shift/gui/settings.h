#ifndef SHIFT_GUI_SETTINGS_H_
#define SHIFT_GUI_SETTINGS_H_

#include "shift/gui/dlss.h"

#include <json/json.h>

#include <QColor>

#include <string>
#include <vector>

#include "shift/gui/color_map.h"
#include "shift/gui/scene.h"

namespace shift {
namespace gui {

/*! \brief a quantity of the stress the viewer can draw: its key in the
 * settings file and its label */
struct StressQuantity {
  const char* key;
  const char* label;
  FlexUpdate::Quantity quantity;
};

/*! \brief the quantities of the stress, in the order of FlexUpdate::
 * Quantity */
const std::vector<StressQuantity>& StressQuantities();

/*!
@brief The viewer's settings, kept from one session to the next

They are the length of the arrows of the coordinate frames, the
thickness and color of the lines of the joint axes, the tolerance of the
meshes' levels of detail, the resolution of the ray traced view, whether it
is upscaled with NVIDIA DLSS and in which mode, and the quantity, color map
and range of the stress drawn on flexible bodies. They are read from and
written to a JSON file,
~/.shift/settings.json (get_default_path). Keys the file has that this
version does not know are kept when it is written again.
*/
class Settings {
public:
  /*! \brief the length of the arrows of a coordinate frame (m) */
  static constexpr double kDefaultFrameScale = 0.25;
  /*! \brief the diameter of the line of a joint axis (m) */
  static constexpr double kDefaultJointThickness = 0.0075;
  /*! \brief the color of the joint axes, purple */
  static QColor get_default_joint_color() {
    return QColor(0xa0, 0x40, 0xe0);
  }

  /*! \brief the pixels the error of a level of detail may span
   * (SceneMeshLod::kPixelError) */
  static constexpr double kDefaultLodTolerance = 8.0;
  /*! \brief the share of the window's width and height that the ray tracer
   * renders in the viewer: all of it */
  static constexpr double kDefaultRenderScale = 1.0;
  static constexpr double kMinimumRenderScale = 0.25;

  /*! \brief ~/.shift/settings.json */
  static std::string get_default_path();

  /*! \brief read the settings from path; a file that does not exist leaves
   * the defaults. Returns false, with a message in error, if it cannot be
   * read, and the defaults are kept. */
  bool Load(const std::string& path, std::string* error = nullptr);

  /*! \brief write the settings to path, making its directory if needed;
   * false, with a message in error, if it cannot be written */
  bool Save(const std::string& path, std::string* error = nullptr) const;

  double get_frame_scale() const {
    return frame_scale_;
  }
  void set_frame_scale(double scale) {
    frame_scale_ = scale;
  }
  /*! \brief the color of the joint axes, "joint_color" in the file as
   * "#rrggbb" */
  const QColor& get_joint_color() const {
    return joint_color_;
  }
  void set_joint_color(const QColor& color) {
    joint_color_ = color;
  }
  /*! \brief the diameter of the line of a joint axis (m),
   * "joint_thickness" in the file */
  double get_joint_thickness() const {
    return joint_thickness_;
  }
  void set_joint_thickness(double thickness) {
    joint_thickness_ = thickness;
  }
  /*! \brief whether the ray traced view uses DLSS where the GPU supports
   * it, "dlss" in the file (default true) */
  bool is_dlss_enabled() const {
    return dlss_enabled_;
  }
  void set_dlss_enabled(bool enabled) {
    dlss_enabled_ = enabled;
  }
  /*! \brief the DLSS mode, "dlss_mode" in the file as a key of
   * Dlss::get_quality_key (default "quality") */
  Dlss::Quality get_dlss_quality() const {
    return dlss_quality_;
  }
  void set_dlss_quality(Dlss::Quality quality) {
    dlss_quality_ = quality;
  }

  /*! \brief the color map of the stress drawn on flexible bodies (a key of
   * ColorMaps), "stress_color_map" in the file */
  const std::string& get_stress_color_map() const {
    return stress_color_map_;
  }
  void set_stress_color_map(const std::string& key) {
    stress_color_map_ = key;
  }
  /*! \brief whether the color map spans 0 to the largest stress on each
   * body as drawn ("stress_range": "automatic", the default), or from
   * get_stress_minimum to get_stress_maximum ("fixed"); in the units of
   * stress of the model (Pa in SI units), "stress_minimum" and
   * "stress_maximum" in the file */
  bool is_stress_range_automatic() const {
    return stress_range_automatic_;
  }
  void set_stress_range_automatic(bool automatic) {
    stress_range_automatic_ = automatic;
  }
  double get_stress_minimum() const {
    return stress_minimum_;
  }
  double get_stress_maximum() const {
    return stress_maximum_;
  }
  void set_stress_range(double minimum, double maximum) {
    stress_minimum_ = minimum;
    stress_maximum_ = maximum;
  }
  /*! \brief the quantity of the stress drawn ("stress_quantity" in the
   * file, a key of StressQuantities; "von_mises" by default) */
  FlexUpdate::Quantity get_stress_quantity() const {
    return stress_quantity_;
  }
  void set_stress_quantity(FlexUpdate::Quantity quantity) {
    stress_quantity_ = quantity;
  }
  /*! \brief whether the greatest over the run up to the time shown is
   * drawn ("stress_peak"; false by default) */
  bool is_stress_peak() const {
    return stress_peak_;
  }
  void set_stress_peak(bool peak) {
    stress_peak_ = peak;
  }
  /*! \brief the safety factors at the color map's end and at its start
   * ("safety_factor_minimum" and "safety_factor_maximum"; 1 and 4 by
   * default) */
  double get_safety_minimum() const {
    return safety_minimum_;
  }
  double get_safety_maximum() const {
    return safety_maximum_;
  }
  void set_safety_range(double minimum, double maximum) {
    safety_minimum_ = minimum;
    safety_maximum_ = maximum;
  }

  /*! \brief the pixels across the screen that the error of a mesh's level
   * of detail may span where it is drawn, 0 for the meshes themselves;
   * "lod_tolerance" in the file */
  double get_lod_tolerance() const {
    return lod_tolerance_;
  }
  void set_lod_tolerance(double pixels) {
    lod_tolerance_ = pixels;
  }

  /*! \brief the share (kMinimumRenderScale to 1) of the window's width and
   * height that the ray tracer renders in the viewer, scaled up to it;
   * images and videos are rendered at their full size. "render_scale" in
   * the file */
  double get_render_scale() const {
    return render_scale_;
  }
  void set_render_scale(double scale) {
    render_scale_ = scale;
  }

private:
  double frame_scale_ = kDefaultFrameScale;
  double lod_tolerance_ = kDefaultLodTolerance;
  double render_scale_ = kDefaultRenderScale;
  QColor joint_color_ = get_default_joint_color();
  double joint_thickness_ = kDefaultJointThickness;
  std::string stress_color_map_ = DefaultColorMap();
  FlexUpdate::Quantity stress_quantity_ = FlexUpdate::Quantity::kVonMises;
  bool stress_peak_ = false;
  double safety_minimum_ = 1.0;
  double safety_maximum_ = 4.0;
  bool stress_range_automatic_ = true;
  double stress_minimum_ = 0.0;
  double stress_maximum_ = 250e6;
  bool dlss_enabled_ = true;
  Dlss::Quality dlss_quality_ = Dlss::Quality::kQuality;
  Json::Value json_ = Json::Value(Json::objectValue); /*!< the file as read */
};

} // namespace gui
} // namespace shift

#endif

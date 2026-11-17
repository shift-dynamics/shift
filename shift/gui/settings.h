#ifndef SHIFT_GUI_SETTINGS_H_
#define SHIFT_GUI_SETTINGS_H_

#include <json/json.h>

#include <QColor>

#include <string>

#include "shift/gui/color_map.h"

namespace shift {
namespace gui {

/*!
@brief The viewer's settings, kept from one session to the next

They are the length of the arrows of the coordinate frames, the
thickness and color of the lines of the joint axes, and the color map and
range of the stress drawn on flexible bodies. They are read from and
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

private:
  double frame_scale_ = kDefaultFrameScale;
  QColor joint_color_ = get_default_joint_color();
  double joint_thickness_ = kDefaultJointThickness;
  std::string stress_color_map_ = DefaultColorMap();
  bool stress_range_automatic_ = true;
  double stress_minimum_ = 0.0;
  double stress_maximum_ = 250e6;
  Json::Value json_ = Json::Value(Json::objectValue); /*!< the file as read */
};

} // namespace gui
} // namespace shift

#endif

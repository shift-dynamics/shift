#ifndef SHIFT_GUI_SETTINGS_H_
#define SHIFT_GUI_SETTINGS_H_

#include <json/json.h>

#include <QColor>

#include <string>

namespace shift {
namespace gui {

/*!
@brief The viewer's settings, kept from one session to the next

They are the length of the arrows of the coordinate frames and the
thickness and color of the lines of the joint axes. They are read from and written to a JSON file,
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

private:
  double frame_scale_ = kDefaultFrameScale;
  QColor joint_color_ = get_default_joint_color();
  double joint_thickness_ = kDefaultJointThickness;
  Json::Value json_ = Json::Value(Json::objectValue); /*!< the file as read */
};

} // namespace gui
} // namespace shift

#endif

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
is upscaled with NVIDIA DLSS and in which mode, the tone curve and the
exposure, how deformable terrain and SPH soil are drawn, and the quantity,
color map and range of the stress drawn on flexible bodies, and how flexible
bodies are drawn. They are read from and written to a JSON file,
~/.shift/settings.json (get_default_path), whose keys are grouped as the
pages of the Settings dialog: "interface", "rendering" and
"flexible_bodies" (SectionOf). A file of an earlier version, with its keys
at the top level, is read as well and written grouped. Keys the file has
that this version does not know are kept when it is written again.
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

  /*! \brief the tone curve: ACES filmic if true, else Reinhard (the
   * default); "tone_mapping" in the file, "aces" or "reinhard" */
  bool is_tone_aces() const {
    return tone_aces_;
  }
  void set_tone_aces(bool aces) {
    tone_aces_ = aces;
  }
  /*! \brief the exposure in stops (default 0); "exposure" in the file */
  double get_exposure() const {
    return exposure_;
  }
  void set_exposure(double stops) {
    exposure_ = stops;
  }
  /*! \brief whether deformable terrain is drawn by a smooth surface through
   * its nodes (default true); "smooth_terrain" in the file */
  bool is_smooth_terrain() const {
    return smooth_terrain_;
  }
  void set_smooth_terrain(bool smooth) {
    smooth_terrain_ = smooth;
  }
  /*! \brief whether SPH soil is drawn as one surface over its particles
   * (default true); "soil_surface" in the file */
  bool is_soil_surface() const {
    return soil_surface_;
  }
  void set_soil_surface(bool surface) {
    soil_surface_ = surface;
  }

  /*! \brief the factor the deformation of flexible bodies is drawn with
   * (default 1); "deformation_scale" in the file */
  double get_deformation_scale() const {
    return deformation_scale_;
  }
  void set_deformation_scale(double scale) {
    deformation_scale_ = scale;
  }
  /*! \brief how every flexible body is drawn when a model opens (a body's
   * panel then sets its own): by its mesh's surface ("mesh_surface"), with
   * its elements' edges ("element_edges"), its constrained nodes
   * ("constrained_nodes") and its stress ("stresses"); all false by
   * default */
  bool is_flex_mesh_surface() const {
    return flex_mesh_surface_;
  }
  void set_flex_mesh_surface(bool on) {
    flex_mesh_surface_ = on;
  }
  bool is_flex_element_edges() const {
    return flex_element_edges_;
  }
  void set_flex_element_edges(bool on) {
    flex_element_edges_ = on;
  }
  bool is_flex_constrained_nodes() const {
    return flex_constrained_nodes_;
  }
  void set_flex_constrained_nodes(bool on) {
    flex_constrained_nodes_ = on;
  }
  bool is_flex_stresses() const {
    return flex_stresses_;
  }
  void set_flex_stresses(bool on) {
    flex_stresses_ = on;
  }

  /*! \brief the group of the file ("interface", "rendering" or
   * "flexible_bodies") that a key of the settings is written in */
  static const char* SectionOf(const std::string& key);

private:
  double frame_scale_ = kDefaultFrameScale;
  double deformation_scale_ = 1.0;
  bool flex_mesh_surface_ = false;
  bool flex_element_edges_ = false;
  bool flex_constrained_nodes_ = false;
  bool flex_stresses_ = false;
  bool tone_aces_ = false;
  double exposure_ = 0.0;
  bool smooth_terrain_ = true;
  bool soil_surface_ = true;
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

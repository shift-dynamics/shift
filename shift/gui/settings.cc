#include "shift/gui/settings.h"

#include <QDir>
#include <QFileInfo>

#include <fstream>

const std::vector<shift::gui::StressQuantity>&
shift::gui::StressQuantities() {
  static const std::vector<StressQuantity> kQuantities = {
    {"von_mises", "von Mises stress", FlexUpdate::Quantity::kVonMises},
    {"tresca", "Tresca stress (largest less smallest principal)", FlexUpdate::Quantity::kTresca},
    {"maximum_principal", "largest principal stress", FlexUpdate::Quantity::kMaxPrincipal},
    {"middle_principal", "middle principal stress", FlexUpdate::Quantity::kMiddlePrincipal},
    {"minimum_principal", "smallest principal stress", FlexUpdate::Quantity::kMinPrincipal},
    {"xx", "normal stress xx", FlexUpdate::Quantity::kXx},
    {"yy", "normal stress yy", FlexUpdate::Quantity::kYy},
    {"zz", "normal stress zz", FlexUpdate::Quantity::kZz},
    {"xy", "shear stress xy", FlexUpdate::Quantity::kXy},
    {"yz", "shear stress yz", FlexUpdate::Quantity::kYz},
    {"zx", "shear stress zx", FlexUpdate::Quantity::kZx},
    {"safety_factor", "safety factor (yield strength / von Mises)",
     FlexUpdate::Quantity::kSafetyFactor}};
  return kQuantities;
}

std::string shift::gui::Settings::get_default_path() {
  return QDir(QDir::homePath()).filePath(".shift/settings.json").toStdString();
}

const char* shift::gui::Settings::SectionOf(const std::string& key) {
  for(const char* interface : {"frame_scale", "joint_color",
                               "joint_thickness"}) {
    if(key == interface) {
      return "interface";
    }
  }
  for(const char* flexible :
      {"deformation_scale", "mesh_surface", "element_edges",
       "constrained_nodes", "stresses", "stress_quantity", "stress_peak",
       "stress_color_map", "stress_range", "stress_minimum", "stress_maximum",
       "safety_factor_minimum", "safety_factor_maximum"}) {
    if(key == flexible) {
      return "flexible_bodies";
    }
  }
  return "rendering";
}

bool shift::gui::Settings::Load(const std::string& path, std::string* error) {
  if(!QFileInfo::exists(QString::fromStdString(path))) {
    return true; // not saved yet: the defaults
  }
  std::ifstream file(path);
  if(!file) {
    if(error) {
      *error = "cannot read " + path;
    }
    return false;
  }
  Json::CharReaderBuilder builder;
  builder["allowComments"] = true;
  Json::Value file_json;
  std::string message;
  if(!Json::parseFromStream(builder, file, &file_json, &message) ||
     !file_json.isObject()) {
    if(error) {
      *error = path + ": " + (message.empty() ? "not a JSON object" : message);
    }
    return false;
  }
  // the keys of the groups, read as the top level keys of a file of an
  // earlier version are, which a group's take the place of
  Json::Value json = file_json;
  for(const char* section : {"interface", "rendering", "flexible_bodies"}) {
    json.removeMember(section);
    if(file_json.isMember(section) && file_json[section].isObject()) {
      for(const std::string& name : file_json[section].getMemberNames()) {
        json[name] = file_json[section][name];
      }
    }
  }
  // a positive number at key, default if the file has none
  auto positive = [&](const char* key, double& value) {
    if(!json.isMember(key)) {
      return true;
    }
    const Json::Value& number = json[key];
    if(!number.isNumeric() || number.asDouble() <= 0.0) {
      if(error) {
        *error = path + ": " + key + " must be a positive number";
      }
      return false;
    }
    value = number.asDouble();
    return true;
  };
  double frame_scale = kDefaultFrameScale;
  double joint_thickness = kDefaultJointThickness;
  if(!positive("frame_scale", frame_scale) ||
     !positive("joint_thickness", joint_thickness)) {
    return false;
  }
  double lod_tolerance = kDefaultLodTolerance;
  if(json.isMember("lod_tolerance")) {
    const Json::Value& number = json["lod_tolerance"];
    if(!number.isNumeric() || number.asDouble() < 0.0) {
      if(error) {
        *error = path + ": lod_tolerance must be a number of pixels, 0 or "
                        "more";
      }
      return false;
    }
    lod_tolerance = number.asDouble();
  }
  double render_scale = kDefaultRenderScale;
  if(json.isMember("render_scale")) {
    const Json::Value& number = json["render_scale"];
    if(!number.isNumeric() || number.asDouble() < kMinimumRenderScale ||
       number.asDouble() > 1.0) {
      if(error) {
        *error = path + ": render_scale must be a number from " +
                 std::to_string(kMinimumRenderScale) + " to 1";
      }
      return false;
    }
    render_scale = number.asDouble();
  }
  QColor joint_color = get_default_joint_color();
  if(json.isMember("joint_color")) {
    const Json::Value& color = json["joint_color"];
    if(color.isString()) {
      joint_color = QColor::fromString(color.asString());
    }
    if(!color.isString() || !joint_color.isValid()) {
      if(error) {
        *error = path + ": joint_color must be a color as \"#rrggbb\"";
      }
      return false;
    }
  }
  // the stress drawn: an unknown color map is the default's
  std::string color_map = DefaultColorMap();
  if(json["stress_color_map"].isString()) {
    const std::string key = json["stress_color_map"].asString();
    for(const ColorMap& map : ColorMaps()) {
      if(key == map.key) {
        color_map = key;
      }
    }
  }
  const bool automatic = !(json["stress_range"].isString() &&
                           json["stress_range"].asString() == "fixed");
  double stress_minimum = 0.0, stress_maximum = 250e6;
  if(json["stress_minimum"].isNumeric()) {
    stress_minimum = json["stress_minimum"].asDouble();
  }
  if(json["stress_maximum"].isNumeric()) {
    stress_maximum = json["stress_maximum"].asDouble();
  }
  FlexUpdate::Quantity quantity = FlexUpdate::Quantity::kVonMises;
  if(json["stress_quantity"].isString()) {
    for(const StressQuantity& known : StressQuantities()) {
      if(json["stress_quantity"].asString() == known.key) {
        quantity = known.quantity;
      }
    }
  }
  const bool peak =
    json["stress_peak"].isBool() && json["stress_peak"].asBool();
  double safety_minimum = 1.0, safety_maximum = 4.0;
  if(json["safety_factor_minimum"].isNumeric()) {
    safety_minimum = json["safety_factor_minimum"].asDouble();
  }
  if(json["safety_factor_maximum"].isNumeric()) {
    safety_maximum = json["safety_factor_maximum"].asDouble();
  }
  stress_quantity_ = quantity;
  stress_peak_ = peak;
  safety_minimum_ = safety_minimum;
  safety_maximum_ = safety_maximum;
  stress_color_map_ = color_map;
  stress_range_automatic_ = automatic;
  stress_minimum_ = stress_minimum;
  stress_maximum_ = stress_maximum;
  bool dlss_enabled = true;
  if(json.isMember("dlss")) {
    if(!json["dlss"].isBool()) {
      if(error) {
        *error = path + ": dlss must be true or false";
      }
      return false;
    }
    dlss_enabled = json["dlss"].asBool();
  }
  Dlss::Quality dlss_quality = Dlss::Quality::kQuality;
  if(json.isMember("dlss_mode")) {
    const Json::Value& mode = json["dlss_mode"];
    if(!mode.isString() || !Dlss::FindQuality(mode.asString(), dlss_quality)) {
      if(error) {
        *error = path + ": dlss_mode must be one of \"dlaa\", \"quality\", "
                        "\"balanced\", \"performance\", \"ultra_performance\"";
      }
      return false;
    }
  }
  bool tone_aces = false;
  if(json.isMember("tone_mapping")) {
    const Json::Value& curve = json["tone_mapping"];
    if(!curve.isString() ||
       (curve.asString() != "aces" && curve.asString() != "reinhard")) {
      if(error) {
        *error = path + ": tone_mapping must be \"reinhard\" or \"aces\"";
      }
      return false;
    }
    tone_aces = curve.asString() == "aces";
  }
  double exposure = 0.0;
  if(json.isMember("exposure")) {
    if(!json["exposure"].isNumeric()) {
      if(error) {
        *error = path + ": exposure must be a number of stops";
      }
      return false;
    }
    exposure = json["exposure"].asDouble();
  }
  // true or false at key, default if the file has none
  auto boolean = [&](const char* key, bool& value) {
    if(!json.isMember(key)) {
      return true;
    }
    if(!json[key].isBool()) {
      if(error) {
        *error = path + ": " + key + " must be true or false";
      }
      return false;
    }
    value = json[key].asBool();
    return true;
  };
  bool smooth_terrain = true;
  bool soil_surface = true;
  bool flex_mesh_surface = false;
  bool flex_element_edges = false;
  bool flex_constrained_nodes = false;
  bool flex_stresses = false;
  if(!boolean("smooth_terrain", smooth_terrain) ||
     !boolean("soil_surface", soil_surface) ||
     !boolean("mesh_surface", flex_mesh_surface) ||
     !boolean("element_edges", flex_element_edges) ||
     !boolean("constrained_nodes", flex_constrained_nodes) ||
     !boolean("stresses", flex_stresses)) {
    return false;
  }
  double deformation_scale = 1.0;
  if(json.isMember("deformation_scale")) {
    const Json::Value& number = json["deformation_scale"];
    if(!number.isNumeric() || number.asDouble() < 0.0) {
      if(error) {
        *error = path + ": deformation_scale must be a number of at least 0";
      }
      return false;
    }
    deformation_scale = number.asDouble();
  }
  frame_scale_ = frame_scale;
  tone_aces_ = tone_aces;
  exposure_ = exposure;
  smooth_terrain_ = smooth_terrain;
  soil_surface_ = soil_surface;
  flex_mesh_surface_ = flex_mesh_surface;
  flex_element_edges_ = flex_element_edges;
  flex_constrained_nodes_ = flex_constrained_nodes;
  flex_stresses_ = flex_stresses;
  deformation_scale_ = deformation_scale;
  joint_color_ = joint_color;
  joint_thickness_ = joint_thickness;
  lod_tolerance_ = lod_tolerance;
  render_scale_ = render_scale;
  dlss_enabled_ = dlss_enabled;
  dlss_quality_ = dlss_quality;
  json_ = file_json;
  return true;
}

bool shift::gui::Settings::Save(const std::string& path,
                                std::string* error) const {
  const QString directory = QFileInfo(QString::fromStdString(path)).path();
  if(!QDir().mkpath(directory)) {
    if(error) {
      *error = "cannot make the directory " + directory.toStdString();
    }
    return false;
  }
  Json::Value json = json_;
  // each key in its group, in place of where a file of an earlier version
  // had it
  auto put = [&json](const char* key, const Json::Value& value) {
    json.removeMember(key);
    const char* section = SectionOf(key);
    if(!json[section].isObject()) {
      json[section] = Json::Value(Json::objectValue);
    }
    json[section][key] = value;
  };
  put("frame_scale", frame_scale_);
  put("joint_color", joint_color_.name(QColor::HexRgb).toStdString());
  put("joint_thickness", joint_thickness_);
  put("stress_color_map", stress_color_map_);
  put("stress_range", stress_range_automatic_ ? "automatic" : "fixed");
  put("stress_minimum", stress_minimum_);
  put("stress_maximum", stress_maximum_);
  put("stress_quantity", StressQuantities()[size_t(stress_quantity_)].key);
  put("stress_peak", stress_peak_);
  put("safety_factor_minimum", safety_minimum_);
  put("safety_factor_maximum", safety_maximum_);
  put("lod_tolerance", lod_tolerance_);
  put("render_scale", render_scale_);
  put("dlss", dlss_enabled_);
  put("dlss_mode", Dlss::get_quality_key(dlss_quality_));
  put("tone_mapping", tone_aces_ ? "aces" : "reinhard");
  put("exposure", exposure_);
  put("smooth_terrain", smooth_terrain_);
  put("soil_surface", soil_surface_);
  put("deformation_scale", deformation_scale_);
  put("mesh_surface", flex_mesh_surface_);
  put("element_edges", flex_element_edges_);
  put("constrained_nodes", flex_constrained_nodes_);
  put("stresses", flex_stresses_);
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  std::ofstream file(path);
  file << Json::writeString(builder, json) << std::endl;
  if(!file) {
    if(error) {
      *error = "cannot write " + path;
    }
    return false;
  }
  return true;
}

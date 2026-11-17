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
  Json::Value json;
  std::string message;
  if(!Json::parseFromStream(builder, file, &json, &message) ||
     !json.isObject()) {
    if(error) {
      *error = path + ": " + (message.empty() ? "not a JSON object" : message);
    }
    return false;
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
  frame_scale_ = frame_scale;
  joint_color_ = joint_color;
  joint_thickness_ = joint_thickness;
  json_ = json;
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
  json["frame_scale"] = frame_scale_;
  json["joint_color"] = joint_color_.name(QColor::HexRgb).toStdString();
  json["joint_thickness"] = joint_thickness_;
  json["stress_color_map"] = stress_color_map_;
  json["stress_range"] = stress_range_automatic_ ? "automatic" : "fixed";
  json["stress_minimum"] = stress_minimum_;
  json["stress_maximum"] = stress_maximum_;
  json["stress_quantity"] =
    StressQuantities()[size_t(stress_quantity_)].key;
  json["stress_peak"] = stress_peak_;
  json["safety_factor_minimum"] = safety_minimum_;
  json["safety_factor_maximum"] = safety_maximum_;
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

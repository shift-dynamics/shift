#include "shift/gui/settings.h"

#include <QDir>
#include <QFileInfo>

#include <fstream>

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

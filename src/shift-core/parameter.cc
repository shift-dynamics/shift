#include "shift-core/parameter.h"
#include "shift-core/assembly.h"
#include "shift-core/context.h"
#include "shift-core/factory.h"
#include "shift-core/frame.h"

#include "ksl/print.h"

#include <algorithm>
#include <iostream>
#include <typeindex>
#include <typeinfo>

template <>
const Shift::Frame& Shift::Parameter::KeyValue::value<Shift::Frame>() const {
  return *dynamic_cast<Shift::Frame*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Assembly&
Shift::Parameter::KeyValue::value<Shift::Assembly>() const {
  return *dynamic_cast<Shift::Assembly*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Behavior::Base&
Shift::Parameter::KeyValue::value<Shift::Behavior::Base>() const {
  return *dynamic_cast<Shift::Behavior::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Body::Base&
Shift::Parameter::KeyValue::value<Shift::Body::Base>() const {
  return *dynamic_cast<Shift::Body::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::CollisionGeometry::Base&
Shift::Parameter::KeyValue::value<Shift::CollisionGeometry::Base>() const {
  return *dynamic_cast<Shift::CollisionGeometry::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Constraint::Base&
Shift::Parameter::KeyValue::value<Shift::Constraint::Base>() const {
  return *dynamic_cast<Shift::Constraint::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::ForceModule::Base&
Shift::Parameter::KeyValue::value<Shift::ForceModule::Base>() const {
  return *dynamic_cast<Shift::ForceModule::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Geometry::Base&
Shift::Parameter::KeyValue::value<Shift::Geometry::Base>() const {
  return *dynamic_cast<Shift::Geometry::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Integrator::Base&
Shift::Parameter::KeyValue::value<Shift::Integrator::Base>() const {
  return *dynamic_cast<Shift::Integrator::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Joint::Base&
Shift::Parameter::KeyValue::value<Shift::Joint::Base>() const {
  return *dynamic_cast<Shift::Joint::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Light::Base&
Shift::Parameter::KeyValue::value<Shift::Light::Base>() const {
  return *dynamic_cast<Shift::Light::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Logger::Base&
Shift::Parameter::KeyValue::value<Shift::Logger::Base>() const {
  return *dynamic_cast<Shift::Logger::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Sensor::Base&
Shift::Parameter::KeyValue::value<Shift::Sensor::Base>() const {
  return *dynamic_cast<Shift::Sensor::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Solver::Base&
Shift::Parameter::KeyValue::value<Shift::Solver::Base>() const {
  return *dynamic_cast<Shift::Solver::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Terrain::Base&
Shift::Parameter::KeyValue::value<Shift::Terrain::Base>() const {
  return *dynamic_cast<Shift::Terrain::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Frame* Shift::Parameter::KeyValue::data<Shift::Frame>() const {
  return dynamic_cast<Shift::Frame*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Assembly* Shift::Parameter::KeyValue::data<Shift::Assembly>() const {
  return dynamic_cast<Shift::Assembly*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Behavior::Base*
Shift::Parameter::KeyValue::data<Shift::Behavior::Base>() const {
  return dynamic_cast<Shift::Behavior::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Body::Base* Shift::Parameter::KeyValue::data<Shift::Body::Base>() const {
  return dynamic_cast<Shift::Body::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::CollisionGeometry::Base*
Shift::Parameter::KeyValue::data<Shift::CollisionGeometry::Base>() const {
  return dynamic_cast<Shift::CollisionGeometry::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Constraint::Base*
Shift::Parameter::KeyValue::data<Shift::Constraint::Base>() const {
  return dynamic_cast<Shift::Constraint::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::ForceModule::Base*
Shift::Parameter::KeyValue::data<Shift::ForceModule::Base>() const {
  return dynamic_cast<Shift::ForceModule::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Geometry::Base*
Shift::Parameter::KeyValue::data<Shift::Geometry::Base>() const {
  return dynamic_cast<Shift::Geometry::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Integrator::Base*
Shift::Parameter::KeyValue::data<Shift::Integrator::Base>() const {
  return dynamic_cast<Shift::Integrator::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Joint::Base*
Shift::Parameter::KeyValue::data<Shift::Joint::Base>() const {
  return dynamic_cast<Shift::Joint::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Light::Base*
Shift::Parameter::KeyValue::data<Shift::Light::Base>() const {
  return dynamic_cast<Shift::Light::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Logger::Base*
Shift::Parameter::KeyValue::data<Shift::Logger::Base>() const {
  return dynamic_cast<Shift::Logger::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Sensor::Base*
Shift::Parameter::KeyValue::data<Shift::Sensor::Base>() const {
  return dynamic_cast<Shift::Sensor::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Solver::Base*
Shift::Parameter::KeyValue::data<Shift::Solver::Base>() const {
  return dynamic_cast<Shift::Solver::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Terrain::Base*
Shift::Parameter::KeyValue::data<Shift::Terrain::Base>() const {
  return dynamic_cast<Shift::Terrain::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

Shift::Parameter::KeyValue& Shift::Parameter::KeyValue::
operator[](const std::string key_str) {
  if(is_list_) {
    return std::any_cast<Parameter::List*>(data_)->operator[](key_str);
  } else if(is_array_) {
    return std::any_cast<Parameter::ArrayBase*>(data_)->operator[](key_str);
  } else {
    throw "error: no parameter found";
    return std::any_cast<Parameter::List*>(data_)->operator[](key_str);
  }
}

Shift::Parameter::KeyValue& Shift::Parameter::KeyValue::
operator[](const char* key_str) {
  return operator[](std::string(key_str));
}

Shift::Parameter::KeyValue& Shift::Parameter::KeyValue::
operator[](unsigned int index) {
  if(is_array_) {
    return std::any_cast<Parameter::ArrayBase*>(data_)->operator[](index);
  } else {
    std::cerr << "error" << std::endl;
    throw "error: no parameter found";
    // return (std::any_cast<List>(data_))[key_str];
  }
}

void Shift::Parameter::KeyValue::setActiveObjects(bool parent_is_active) {
  if(is_array_) {
    std::any_cast<Parameter::ArrayBase*>(data_)->setActiveObjects(
      parent_is_active);
  } else if(is_list_) {
    std::any_cast<Parameter::List*>(data_)->setActiveObjects(parent_is_active);
  }
}

void Shift::Parameter::KeyValue::readJSON(Json::Value& object) {

  /* check if value exists */
  if(!object.isMember(key_)) {
    if(required_) {
      throw "JSON Parsing error: required key not found";
    } else {
      return;
    }
  }

  switch(parameter_type_) {
    case Parameter::Type::STRING: {
      readJSON_STRING(object[key_]);
      break;
    }
    case Parameter::Type::STRING_ARRAY: {
      readJSON_STRING_ARRAY(object[key_]);
      break;
    }
    case Parameter::Type::STRING_2DARRAY: {
      readJSON_STRING_2DARRAY(object[key_]);
      break;
    }
    case Parameter::Type::DOUBLE: {
      readJSON_DOUBLE(object[key_]);
      break;
    }
    case Parameter::Type::DOUBLE_ARRAY: {
      readJSON_DOUBLE_ARRAY(object[key_]);
      break;
    }
    case Parameter::Type::KSL_VEC3: {
      readJSON_KSL_VEC3(object[key_]);
      break;
    }
    case Parameter::Type::KSL_VEC4: {
      readJSON_KSL_VEC4(object[key_]);
      break;
    }
    case Parameter::Type::KSL_QUATERNION: {
      readJSON_KSL_QUATERNION(object[key_]);
      break;
    }
    case Parameter::Type::KSL_SCREW: {
      readJSON_KSL_SCREW(object[key_]);
      break;
    }
    case Parameter::Type::KSL_COSCREW: {
      readJSON_KSL_COSCREW(object[key_]);
      break;
    }
    case Parameter::Type::KSL_INERTIA: {
      readJSON_KSL_INERTIA(object[key_]);
      break;
    }
    case Parameter::Type::KSL_MAT3X3: {
      readJSON_KSL_MAT3X3(object[key_]);
      break;
    }
    case Parameter::Type::KSL_MAT4X4: {
      readJSON_KSL_MAT4X4(object[key_]);
      break;
    }
    case Parameter::Type::KSL_EULERANGLES: {
      readJSON_EULERANGLES(object[key_]);
      break;
    }
    case Parameter::Type::BOOLEAN: {
      readJSON_BOOLEAN(object[key_]);
      break;
    }
    case Parameter::Type::KSL_SE3: {
      readJSON_POSE(object[key_]);
      break;
    }
    case Parameter::Type::PARAMETER_LIST: {
      std::cout << "READING PARAMETER LIST: " << key_ << std::endl;
      std::any_cast<Parameter::List*>(data_)->readJSON(object[key_]);
      break;
    }
    case Parameter::Type::PARAMETER_ARRAY: {
      std::any_cast<Parameter::ArrayBase*>(data_)->readJSON(object);
      break;
    }
  }
}

void Shift::Parameter::KeyValue::readJSON_STRING(Json::Value& value) {
  if(!value.isString()) {
    if(required_) {
      throw "JSON Parsing error: required key has incorrectly formatted data; "
            "expected STRING";
    }
    return;
  }
  (*std::any_cast<std::string*>(data_)).assign(value.asString());
  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_STRING_ARRAY(Json::Value& value) {
  if(!value.isArray()) {
    if(required_) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected STRING_ARRAY";
    }
    return;
  }
  for(unsigned int i = 0; i < value.size(); i++) {
    if(!(value[i].isString())) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected STRING_ARRAY";
    }
  }

  for(unsigned int i = 0; i < value.size(); i++) {
    (*std::any_cast<std::vector<std::string>*>(data_))
      .push_back(value[i].asString());
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_STRING_2DARRAY(Json::Value& value) {
  if(value.isArray()) {
    for(unsigned int i = 0; i < value.size(); i++) {
      if(row_dim_ > 0 && row_dim_ != (int)value.size()) {
        throw "JSON Parsing error: required key has incorrectly "
              "formatted data; incorrect row dimension found in "
              "STRING_2DARRAY";
      }
      if(!(value[i].isArray()) && !(value[i].isString())) {
        throw "JSON Parsing error: required key has incorrectly formatted "
              "data; expected STRING_2DARRAY";
      }
      for(unsigned int j = 0; j < value[i].size(); j++) {
        if(col_dim_ > 0 && col_dim_ != (int)value[i].size()) {
          throw "JSON Parsing error: required key has incorrectly "
                "formatted data; incorrect column dimension found in "
                "STRING_2DARRAY";
        }

        if(value[i].isArray() && !(value[i][j].isString())) {
          throw "JSON Parsing error: required key has incorrectly "
                "formatted data; expected STRING_2DARRAY";
        }
      }
    }
  }

  for(unsigned int i = 0; i < value.size(); i++) {
    std::vector<std::string> row;
    if(value[i].isArray()) {
      for(unsigned int j = 0; j < value[i].size(); j++) {
        row.push_back(value[i][j].asString());
      }
    } else if(value[i].isString()) {
      row.push_back(value[i].asString());
    }
    std::any_cast<std::vector<std::vector<std::string>>*>(data_)->push_back(
      row);
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_DOUBLE(Json::Value& json_value) {
  if(!json_value.isDouble()) {
    throw "JSON Parsing error, required key has incorrectly formatted "
          "data; expected DOUBLE";
  }
  *std::any_cast<double*>(data_) = json_value.asDouble();
  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_DOUBLE_ARRAY(Json::Value& value) {
  if(!value.isArray()) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected DOUBLE_ARRAY";
  }
  if(row_dim_ > 0 && row_dim_ != static_cast<int>(value.size())) {
    throw "JSON Parsing error: required key has incorrect number of elements";
  }
  if(value.size() == 0) {
    throw "JSON Parsing error: array has zero elements";
  }

  if(row_dim_ > -1 && col_dim_ > -1 &&
     static_cast<int>(value.size()) != row_dim_ * col_dim_) {
    throw "JSON Parsing error: DOUBLE_ARRAY has incorrect dimension ";
  }

  for(unsigned int i = 0; i < value.size(); i++) {
    if(value[i].isArray()) {
    }
    if(!value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected DOUBLE_ARRAY";
    }
    (*std::any_cast<std::vector<double>*>(data_))[i] = value[i].asDouble();
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_KSL_VEC3(Json::Value& value) {
  if(!value.isArray() || !(value.size() == 3)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_VEC3";
  }
  ksl_vec3_t* data = std::any_cast<ksl_vec3_t*>(data_);
  for(int i = 0; i < 3; i++) {
    if(!value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_VEC3";
    }
    data->at[i] = value[i].asDouble();
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_KSL_VEC4(Json::Value& value) {
  if(!value.isArray() || !(value.size() == 4)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_VEC4";
  }
  ksl_vec4_t* data = std::any_cast<ksl_vec4_t*>(data_);
  for(int i = 0; i < 4; i++) {
    if(!value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_VEC4";
      return;
    }
    data->at[i] = value[i].asDouble();
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_KSL_QUATERNION(Json::Value& value) {
  if(!value.isArray() || !(value.size() == 4)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_QUATERNION";
    return;
  }
  ksl_quaternion_t quaternion;
  for(int i = 0; i < 4; i++) {
    if(!value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_QUATERNION";
      return;
    }
    quaternion.at[i] = value[i].asDouble();
  }

  if(normalize_quaternion_) {
    ksl_quaternion_normalize(&quaternion);
  }

  if(quaternion_maps_to_mat3x3_) {
    ksl_mat3x3_t* data = std::any_cast<ksl_mat3x3_t*>(data_);
    ksl_quaternion_toMat3x3(&quaternion, data);
  } else {
    ksl_quaternion_t* data = std::any_cast<ksl_quaternion_t*>(data_);
    *data = quaternion;
  }

  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_KSL_SCREW(Json::Value& value) {
  if(!value.isArray() || !(value.size() == 6)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_SCREW";
    return;
  }
  ksl_screw_t* data = std::any_cast<ksl_screw_t*>(data_);
  for(int i = 0; i < 6; i++) {
    if(!value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_SCREW";
      return;
    }
    data->at[i] = value[i].asDouble();
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_KSL_COSCREW(Json::Value& value) {
  if(!value.isArray() || !(value.size() == 6)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_COSCREW";
    return;
  }
  ksl_coscrew_t* data = std::any_cast<ksl_coscrew_t*>(data_);
  for(int i = 0; i < 6; i++) {
    if(!value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_COSCREW";
      return;
    }
    data->at[i] = value[i].asDouble();
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_KSL_INERTIA(Json::Value& value) {
  if(!value.isObject()) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_INERTIA";
    return;
  }
  ksl_inertia_t* inertia = std::any_cast<ksl_inertia_t*>(data_);

  if(value.isMember("mass")) {
    if(!value["mass"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->mass = value["mass"].asDouble();
  }

  if(value.isMember("Ixx")) {
    if(!value["Ixx"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Ixx = value["Ixx"].asDouble();
  }

  if(value.isMember("Iyy")) {
    if(!value["Iyy"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Iyy = value["Iyy"].asDouble();
  }

  if(value.isMember("Izz")) {
    if(!value["Izz"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Izz = value["Izz"].asDouble();
  }

  if(value.isMember("Ixy")) {
    if(!value["Ixy"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Ixy = value["Ixy"].asDouble();
  }

  if(value.isMember("Iyz")) {
    if(!value["Iyz"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Iyz = value["Iyz"].asDouble();
  }

  if(value.isMember("Izx")) {
    if(!value["Izx"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Izx = value["Izx"].asDouble();
  }

  if(value.isMember("Iyx")) {
    if(!value["Iyx"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Iyx = value["Iyx"].asDouble();
  }

  if(value.isMember("Izy")) {
    if(!value["Izy"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Izy = value["Izy"].asDouble();
  }

  if(value.isMember("Ixz")) {
    if(!value["Ixz"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Ixz = value["Ixz"].asDouble();
  }

  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_KSL_MAT3X3(Json::Value& value) {
  if(!value.isArray() || !(value.size() == 3)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_MAT3X3";
    return;
  }
  for(int i = 0; i < 3; i++) {
    if(!value[i].isArray() || !(value[i].size() == 3)) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_MAT3X3";
      return;
    }
  }

  ksl_mat3x3_t* data = std::any_cast<ksl_mat3x3_t*>(data_);
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      if(!value[i][j].isDouble()) {
        throw "JSON Parsing error: required key has incorrectly formatted "
              "data; expected KSL_MAT3X3";
      }
      data->at[j][i] = value[i][j].asDouble();
    }
  }
  if(normalize_mat3x3_) {
    ksl_mat3x3_normalize(data);
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_KSL_MAT4X4(Json::Value& value) {
  if(!value.isArray() || !(value.size() == 4)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_MAT3X3";
    return;
  }
  for(int i = 0; i < 4; i++) {
    if(!value[i].isArray() || !(value[i].size() == 4)) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_MAT3X3";
      return;
    }
  }

  ksl_mat4x4_t mat4x4;
  ksl_mat4x4_setIdentity(&mat4x4);
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      if(!value[i][j].isDouble()) {
        throw "JSON Parsing error: required key has incorrectly formatted "
              "data; expected KSL_MAT3X3";
      }
      mat4x4.at[j][i] = value[i][j].asDouble();
    }
  }

  if(mat4x4_maps_to_SE3_) {
    ksl_SE3_t* data = std::any_cast<ksl_SE3_t*>(data_);
    ksl_mat4x4_toSE3(&mat4x4, data);
    if(normalize_mat3x3_) {
      ksl_mat3x3_normalize(&data->R);
    }
  } else {
    ksl_mat4x4_t* data = std::any_cast<ksl_mat4x4_t*>(data_);
    *data = mat4x4;
  }

  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_EULERANGLES(Json::Value& value) {
  if(!value.isObject()) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_EULERANGLES";
    return;
  }

  ksl_axis_enum_t sequence = KSL_AXIS_XYZ;
  ksl_vec3_t angles;
  if(value.isMember("angles") && value["angles"].size() == 3) {
    for(int i = 0; i < 3; i++) {
      if(!value["angles"][i].isDouble()) {
        throw "JSON Parsing error: angles object is incorrectly formatted";
      }
      angles.at[i] = value["angles"][i].asDouble();
    }
  } else {
    throw "JSON Parsing error: \"euler_angles\" object has no key \"angles\"";
    return;
  }

  if(value.isMember("axes")) {
    if(value["axes"].isArray() && value["axes"].size() == 3) {
      for(int i = 0; i < 3; i++) {
        if(value["angles"][i].isString()) {
          if(value["angles"][i].asString() == "x") {
            axes_.at[i] = 0;
          } else if(value["angles"][i].asString() == "y") {
            axes_.at[i] = 1;
          } else if(value["angles"][i].asString() == "z") {
            axes_.at[i] = 2;
          }
        } else if(value["angles"][i].isInt()) {
          axes_.at[i] = value["angles"][i].asInt();
        } else {
          throw "JSON Parsing error: angles object is incorrectly formatted, "
                "expected integer values 0, 1, or 2 - or string values x, y, "
                "or z";
          return;
        }
      }
      sequence = ksl_axis_getEulerAngles(axes_);
      if(sequence == KSL_AXIS_INVALID) {
        throw "JSON Parsing error: \"euler_angles\" axes object is incorrectly "
              "formatted, expected three integer values {0, 1, 2} or three "
              "string values {x, y, z}";
        return;
      } else {
      }
    } else {
      throw "JSON Parsing error: \"euler_angles\" axes object is incorrectly "
            "formatted, expected three integer values or three "
            "string values following the format [0, 1, 2] or "
            "[\"x\", \"y\", \"z\"], where the axes follow one of the twelve "
            "Euler angle conventions (XYZ, YZX, ZXY, XZY, YXZ, ZYX, XYX, XZX, "
            "YXY, YZY, ZXZ, or ZYZ).";
      return;
    }
  }

  if(euler_angles_maps_to_mat3x3_) {
    ksl_mat3x3_t* data = std::any_cast<ksl_mat3x3_t*>(data_);
    ksl_mat3x3_setFromEulerAngles(data, sequence, &angles);
  } else {
    ksl_vec3_t* data = std::any_cast<ksl_vec3_t*>(data_);
    *data = angles;
  }

  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_BOOLEAN(Json::Value& value) {
  if(!value.isBool()) {
    throw "JSON Parsing error, required key %s has incorrectly formatted "
          "data; expected BOOLEAN";
  }
  bool* data = std::any_cast<bool*>(data_);
  *data = value.asBool();
  loaded_from_file_ = true;
}

void Shift::Parameter::KeyValue::readJSON_POSE(Json::Value& value) {
  if(!value.isObject()) {
    throw "Error: Pose is not a valid object";
    return;
  }

  if(value.isMember("quaternion")) {
    readJSON_KSL_QUATERNION(value["quaternion"]);
  } else if(value.isMember("q")) {
    readJSON_KSL_QUATERNION(value["q"]);
  }

  if(value.isMember("mat3x3")) {
    readJSON_KSL_MAT3X3(value["mat3x3"]);
  } else if(value.isMember("R")) {
    readJSON_KSL_MAT3X3(value["R"]);
  }

  if(value.isMember("mat4x4")) {
    readJSON_KSL_MAT4X4(value["mat4x4"]);
  }

  if(value.isMember("euler_angles")) {
    readJSON_EULERANGLES(value["euler_angles"]);
  }

  loaded_from_file_ = true;
}

Shift::Parameter::KeyValue&
Shift::Parameter::List::addParameter(Shift::Parameter::KeyValue key_value) {
  parameters_[key_value.key()] = key_value;
  return parameters_[key_value.key()];
}

void Shift::Parameter::KeyValue::print() const {
  print("");
}

void Shift::Parameter::KeyValue::print(std::string prefix) const {

  /* check if value exists */
  switch(parameter_type_) {
    case Parameter::Type::STRING: {
      std::cout << prefix << key_ << ": " << value<std::string>() << std::endl;
      break;
    }
    case Parameter::Type::STRING_ARRAY: {
      std::cout << prefix << key_ << ": [ ";
      for(auto& i : value<std::vector<std::string>>()) {
        std::cout << "\"" << i << "\" ";
      }
      std::cout << "]" << std::endl;
      break;
    }
    case Parameter::Type::STRING_2DARRAY: {
      std::cout << prefix << key_ << ": [ " << std::endl;
      for(auto& row : value<std::vector<std::vector<std::string>>>()) {
        std::cout << prefix << "  "
                  << "[ ";
        for(auto& column : row) {
          std::cout << "\"" << column << "\" ";
        }
        std::cout << "] " << std::endl;
      }
      std::cout << prefix << "]" << std::endl;
      break;
    }
    case Parameter::Type::DOUBLE: {
      std::cout << prefix << key_ << ": " << value<double>() << std::endl;
      break;
    }
    case Parameter::Type::DOUBLE_ARRAY: {
      std::cout << prefix << key_ << ": [ ";
      for(auto& i : value<std::vector<double>>()) {
        std::cout << i << " ";
      }
      std::cout << " ]" << std::endl;
      break;
    }
    case Parameter::Type::KSL_VEC3: {
      prefix += key_;
      prefix += ": ";
      ksl_vec3_print(stdout, data<ksl_vec3_t>(), prefix.c_str());
      break;
    }
    case Parameter::Type::KSL_VEC4: {
      prefix += key_;
      prefix += ": ";
      ksl_vec4_print(stdout, data<ksl_vec4_t>(), prefix.c_str());
      break;
    }
    case Parameter::Type::KSL_QUATERNION: {
      prefix += key_;
      prefix += ": ";
      ksl_quaternion_print(stdout, data<ksl_quaternion_t>(), prefix.c_str());
      break;
    }
    case Parameter::Type::KSL_SCREW: {
      prefix += key_;
      prefix += ": ";
      ksl_screw_print(stdout, data<ksl_screw_t>(), prefix.c_str());
      break;
    }
    case Parameter::Type::KSL_COSCREW: {
      prefix += key_;
      prefix += ": ";
      ksl_coscrew_print(stdout, data<ksl_coscrew_t>(), prefix.c_str());
      break;
    }
    case Parameter::Type::KSL_INERTIA: {
      prefix += key_;
      prefix += ": ";
      ksl_inertia_print(stdout, data<ksl_inertia_t>(), prefix.c_str());
      break;
    }
    case Parameter::Type::KSL_MAT3X3: {
      prefix += key_;
      prefix += ": ";
      ksl_mat3x3_print(stdout, data<ksl_mat3x3_t>(), prefix.c_str());
      break;
    }
    case Parameter::Type::KSL_MAT4X4: {
      prefix += key_;
      prefix += ": ";
      ksl_mat4x4_print(stdout, data<ksl_mat4x4_t>(), prefix.c_str());
      break;
    }
    case Parameter::Type::KSL_EULERANGLES: {
      prefix += key_;
      prefix += ": ";
      ksl_eulerangles_print(stdout, data<ksl_eulerangles_t>(), prefix.c_str());
      break;
    }
    case Parameter::Type::BOOLEAN: {
      std::cout << prefix << key_ << ": " << (value<bool>() ? "true" : "false")
                << std::endl;
      break;
    }
    case Parameter::Type::KSL_SE3: {
      prefix += key_;
      prefix += ": ";
      ksl_SE3_print(stdout, data<ksl_SE3_t>(), prefix.c_str());
      break;
    }
    case Parameter::Type::PARAMETER_LIST: {
      std::cout << prefix << key_ << ": ";
      value<Shift::Parameter::List>().print(prefix);
      break;
    }
    case Parameter::Type::PARAMETER_ARRAY: {
      std::cout << prefix << key_ << ": ";
      value<Shift::Parameter::ArrayBase>().print(prefix);
      break;
    }
  }
}

void Shift::Parameter::KeyValue::setParent(Context* context,
                                           Assembly* parent_assembly,
                                           Body::Base* parent_body) {
  if(is_list_) {
    std::any_cast<Parameter::List*>(data_)->setParent(context, parent_assembly,
                                                      parent_body);
  } else if(is_array_) {
    std::any_cast<Parameter::ArrayBase*>(data_)->setParent(
      context, parent_assembly, parent_body);
  }
}

void Shift::Parameter::List::print(std::string prefix) const {
  std::cout << prefix << std::endl << prefix << "{" << std::endl;
  for(auto& parameter : parameters_) {
    parameter.second.print(prefix + "  ");
  }
  std::cout << prefix << "}" << std::endl;
}

void Shift::Parameter::List::print() const {
  print("");
}

void Shift::Parameter::ArrayBase::print(std::string prefix) const {
  std::cout << prefix << "[";
  for(auto& parameter : objects_) {
    parameter->print(prefix + "  ");
  }
  std::cout << prefix << "]" << std::endl;
}

void Shift::Parameter::ArrayBase::print() const {
  print("");
}

void Shift::Parameter::List::removeParameter(std::string key) {
  parameters_.erase(key);
}

void Shift::Parameter::List::readJSON(Json::Value& value) {
  std::cout << ": ";
  for(auto& parameter : parameters_) {
    std::cout << parameter.first << ", ";
  }
  std::cout << std::endl;

  for(auto& parameter : parameters_) {
    if(!value.isMember(parameter.first) && parameter.second.required()) {
      std::cout << "  error: " << parameter.first << std::endl;
      throw "JSON parsing error, required key not found";
    }
    if(value.isMember(parameter.first)) {
      parameter.second.readJSON(value);
    }
  }
}

void Shift::Parameter::List::setParent(Shift::Context* context,
                                       Shift::Assembly* parent_assembly,
                                       Body::Base* parent_body) {
  context_ = context;
  parent_assembly_ptr_ = parent_assembly;
  parent_body_ptr_ = parent_body;

  Shift::Assembly* assembly = nullptr;
  is_assembly_ ? assembly = dynamic_cast<Shift::Assembly*>(this)
               : assembly = parent_assembly_ptr_;
  Shift::Body::Base* body = nullptr;
  is_body_ ? body = dynamic_cast<Shift::Body::Base*>(this)
           : body = parent_body_ptr_;
  for(auto& parameter : parameters_) {
    parameter.second.setParent(context_, assembly, body);
  }
}

template <class T>
void Shift::Parameter::Array<T>::readJSON(Json::Value& value) {
  std::string array_key = T::arrayKey();
  if(value.isMember(array_key) && value[array_key].isArray()) {
    for(auto& i : value[array_key]) {
      auto object = std::make_unique<T>();
      if(object) {
        try {
          object->readJSON(i);
          object->setIsBody(array_key == "bodies");
          object->setIsAssembly(array_key == "assemblies");
        } catch(const char* msg) {
          std::cerr << msg << std::endl;
        }
        object_specializations_.push_back(std::move(object));
        objects_.push_back(dynamic_cast<Shift::Parameter::List*>(
          object_specializations_.back().get()));
      }
    }
  }
}

template <class T>
void Shift::Parameter::Array<T>::push_back(std::unique_ptr<T> object) {
  object_specializations_.push_back(std::move(object));
  objects_.push_back(dynamic_cast<Shift::Parameter::List*>(
    object_specializations_.back().get()));
}

template <class T>
void Shift::Parameter::FactoryArray<T>::push_back(std::unique_ptr<T> object) {
  object_specializations_.push_back(std::move(object));
  objects_.push_back(dynamic_cast<Shift::Parameter::List*>(
    object_specializations_.back().get()));
}

template <class T>
void Shift::Parameter::FactoryArray<T>::readJSON(Json::Value& value) {
  std::string array_key = T::arrayKey();
  if(value.isMember(array_key) && value[array_key].isArray()) {
    for(auto& i : value[array_key]) {
      std::string objectType(T::defaultType());
      if(i.isMember("type") && i["type"].isString()) {
        objectType.assign(i["type"].asString());
      }
      /* if there is no "type" specified in the object and
      there is no default type available, then skip over this object
      since Shift does not know which class specialization to use */
      if(!objectType.empty()) {
        auto object = Shift::Factory<T>::get()->create(objectType);
        if(object) {
          try {
            object->readJSON(i);
          } catch(const char* msg) {
            std::cerr << array_key << std::endl;
            std::cerr << msg << std::endl;
          }
          object_specializations_.push_back(std::move(object));
          objects_.push_back(dynamic_cast<Shift::Parameter::List*>(
            object_specializations_.back().get()));
        }
      }
    }
  }
}

/* define specializations of Array class */
template class Shift::Parameter::Array<Shift::Assembly>;
template class Shift::Parameter::Array<Shift::Frame>;

/* define specializations of FactoryArray class */
template class Shift::Parameter::FactoryArray<Shift::Behavior::Base>;
template class Shift::Parameter::FactoryArray<Shift::Body::Base>;
template class Shift::Parameter::FactoryArray<Shift::CollisionGeometry::Base>;
template class Shift::Parameter::FactoryArray<Shift::Constraint::Base>;
template class Shift::Parameter::FactoryArray<Shift::ForceModule::Base>;
template class Shift::Parameter::FactoryArray<Shift::Geometry::Base>;
template class Shift::Parameter::FactoryArray<Shift::Integrator::Base>;
template class Shift::Parameter::FactoryArray<Shift::Joint::Base>;
template class Shift::Parameter::FactoryArray<Shift::Light::Base>;
template class Shift::Parameter::FactoryArray<Shift::Logger::Base>;
template class Shift::Parameter::FactoryArray<Shift::Sensor::Base>;
template class Shift::Parameter::FactoryArray<Shift::Solver::Base>;
template class Shift::Parameter::FactoryArray<Shift::Terrain::Base>;

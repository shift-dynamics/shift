#include "shift/core/parameter/value.h"
#include "shift/core/assembly.h"
#include "shift/core/behavior/base.h"
#include "shift/core/body/base.h"
#include "shift/core/integrator/base.h"
#include "shift/core/logger/base.h"
#include "shift/core/modelparameters.h"
#include "shift/core/parameter/arraybase.h"
#include "shift/core/sensor/base.h"
#include "shift/core/terrain/base.h"

#include "shift/core/frame.h"

#include "ksl/print.h"

// if user requests Shift object, up cast Parameter::List to requested
// type
template <>
const Shift::Frame& Shift::Parameter::Value::get<Shift::Frame>() const {
  return *dynamic_cast<Shift::Frame*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Assembly& Shift::Parameter::Value::get<Shift::Assembly>() const {
  return *dynamic_cast<Shift::Assembly*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Behavior::Base&
Shift::Parameter::Value::get<Shift::Behavior::Base>() const {
  return *dynamic_cast<Shift::Behavior::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Body::Base&
Shift::Parameter::Value::get<Shift::Body::Base>() const {
  return *dynamic_cast<Shift::Body::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::CollisionGeometry::Base&
Shift::Parameter::Value::get<Shift::CollisionGeometry::Base>() const {
  return *dynamic_cast<Shift::CollisionGeometry::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Constraint::Base&
Shift::Parameter::Value::get<Shift::Constraint::Base>() const {
  return *dynamic_cast<Shift::Constraint::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::ForceModule::Base&
Shift::Parameter::Value::get<Shift::ForceModule::Base>() const {
  return *dynamic_cast<Shift::ForceModule::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Geometry::Base&
Shift::Parameter::Value::get<Shift::Geometry::Base>() const {
  return *dynamic_cast<Shift::Geometry::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Integrator::Base&
Shift::Parameter::Value::get<Shift::Integrator::Base>() const {
  return *dynamic_cast<Shift::Integrator::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Joint::Base&
Shift::Parameter::Value::get<Shift::Joint::Base>() const {
  return *dynamic_cast<Shift::Joint::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Light::Base&
Shift::Parameter::Value::get<Shift::Light::Base>() const {
  return *dynamic_cast<Shift::Light::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Logger::Base&
Shift::Parameter::Value::get<Shift::Logger::Base>() const {
  return *dynamic_cast<Shift::Logger::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Sensor::Base&
Shift::Parameter::Value::get<Shift::Sensor::Base>() const {
  return *dynamic_cast<Shift::Sensor::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Solver::Base&
Shift::Parameter::Value::get<Shift::Solver::Base>() const {
  return *dynamic_cast<Shift::Solver::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
const Shift::Terrain::Base&
Shift::Parameter::Value::get<Shift::Terrain::Base>() const {
  return *dynamic_cast<Shift::Terrain::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Frame* Shift::Parameter::Value::data<Shift::Frame>() const {
  return dynamic_cast<Shift::Frame*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Assembly* Shift::Parameter::Value::data<Shift::Assembly>() const {
  return dynamic_cast<Shift::Assembly*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Behavior::Base*
Shift::Parameter::Value::data<Shift::Behavior::Base>() const {
  return dynamic_cast<Shift::Behavior::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Body::Base* Shift::Parameter::Value::data<Shift::Body::Base>() const {
  return dynamic_cast<Shift::Body::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::CollisionGeometry::Base*
Shift::Parameter::Value::data<Shift::CollisionGeometry::Base>() const {
  return dynamic_cast<Shift::CollisionGeometry::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Constraint::Base*
Shift::Parameter::Value::data<Shift::Constraint::Base>() const {
  return dynamic_cast<Shift::Constraint::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::ForceModule::Base*
Shift::Parameter::Value::data<Shift::ForceModule::Base>() const {
  return dynamic_cast<Shift::ForceModule::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Geometry::Base*
Shift::Parameter::Value::data<Shift::Geometry::Base>() const {
  return dynamic_cast<Shift::Geometry::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Integrator::Base*
Shift::Parameter::Value::data<Shift::Integrator::Base>() const {
  return dynamic_cast<Shift::Integrator::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Joint::Base* Shift::Parameter::Value::data<Shift::Joint::Base>() const {
  return dynamic_cast<Shift::Joint::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Light::Base* Shift::Parameter::Value::data<Shift::Light::Base>() const {
  return dynamic_cast<Shift::Light::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Logger::Base*
Shift::Parameter::Value::data<Shift::Logger::Base>() const {
  return dynamic_cast<Shift::Logger::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Sensor::Base*
Shift::Parameter::Value::data<Shift::Sensor::Base>() const {
  return dynamic_cast<Shift::Sensor::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Solver::Base*
Shift::Parameter::Value::data<Shift::Solver::Base>() const {
  return dynamic_cast<Shift::Solver::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

template <>
Shift::Terrain::Base*
Shift::Parameter::Value::data<Shift::Terrain::Base>() const {
  return dynamic_cast<Shift::Terrain::Base*>(
    std::any_cast<Shift::Parameter::List*>(data_));
}

Shift::Parameter::Value& Shift::Parameter::Value::
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

Shift::Parameter::Value& Shift::Parameter::Value::
operator[](const char* key_str) {
  return operator[](std::string(key_str));
}

Shift::Parameter::Value& Shift::Parameter::Value::
operator[](unsigned int index) {
  if(is_array_) {
    return std::any_cast<Parameter::ArrayBase*>(data_)->operator[](index);
  } else {
    std::cerr << "error" << std::endl;
    throw "error: no parameter found";
    // return (std::any_cast<List>(data_))[key_str];
  }
}

void Shift::Parameter::Value::setActiveObjects(bool parent_is_active) {
  if(is_array_) {
    std::any_cast<Parameter::ArrayBase*>(data_)->setActiveObjects(
      parent_is_active);
  } else if(is_list_) {
    std::any_cast<Parameter::List*>(data_)->setActiveObjects(parent_is_active);
  }
}

void Shift::Parameter::Value::readJSON(Json::Value& object) {
  switch(parameter_type_) {
    case STRING: {
      readJSON_STRING(object);
      break;
    }
    case STRING_ARRAY: {
      readJSON_STRING_ARRAY(object);
      break;
    }
    case STRING_2DARRAY: {
      readJSON_STRING_2DARRAY(object);
      break;
    }
    case DOUBLE: {
      readJSON_DOUBLE(object);
      break;
    }
    case DOUBLE_ARRAY: {
      readJSON_DOUBLE_ARRAY(object);
      break;
    }
    case KSL_VEC3: {
      readJSON_KSL_VEC3(object);
      break;
    }
    case KSL_VEC4: {
      readJSON_KSL_VEC4(object);
      break;
    }
    case KSL_QUATERNION: {
      readJSON_KSL_QUATERNION(object);
      break;
    }
    case KSL_SCREW: {
      readJSON_KSL_SCREW(object);
      break;
    }
    case KSL_COSCREW: {
      readJSON_KSL_COSCREW(object);
      break;
    }
    case KSL_INERTIA: {
      readJSON_KSL_INERTIA(object);
      break;
    }
    case KSL_MAT3X3: {
      readJSON_KSL_MAT3X3(object);
      break;
    }
    case KSL_MAT4X4: {
      readJSON_KSL_MAT4X4(object);
      break;
    }
    case KSL_EULERANGLES: {
      readJSON_EULERANGLES(object);
      break;
    }
    case BOOLEAN: {
      readJSON_BOOLEAN(object);
      break;
    }
    case KSL_SE3: {
      readJSON_POSE(object);
      break;
    }
    case PARAMETER_LIST: {
      if(object.isObject()) {
        std::any_cast<Parameter::List*>(data_)->readJSON(object);
      }
      break;
    }
    case PARAMETER_ARRAY: {
      if(object.isArray()) {
        std::any_cast<Parameter::ArrayBase*>(data_)->readJSON(object);
      }
      break;
    }
  }
}

void Shift::Parameter::Value::readJSON_STRING(Json::Value& json_value) {
  if(!json_value.isString()) {
    if(required_) {
      throw "JSON Parsing error: required key has incorrectly formatted data; "
            "expected STRING";
    }
    return;
  }
  (*std::any_cast<std::string*>(data_)).assign(json_value.asString());
  loaded_from_file_ = true;
}

void Shift::Parameter::Value::readJSON_STRING_ARRAY(Json::Value& json_value) {
  if(!json_value.isArray()) {
    if(required_) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected STRING_ARRAY";
    }
    return;
  }
  for(unsigned int i = 0; i < json_value.size(); i++) {
    if(!(json_value[i].isString())) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected STRING_ARRAY";
    }
  }

  for(unsigned int i = 0; i < json_value.size(); i++) {
    (*std::any_cast<std::vector<std::string>*>(data_))
      .push_back(json_value[i].asString());
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::Value::readJSON_STRING_2DARRAY(Json::Value& json_value) {
  if(json_value.isArray()) {
    for(unsigned int i = 0; i < json_value.size(); i++) {
      if(row_dim_ > 0 && row_dim_ != (int)json_value.size()) {
        throw "JSON Parsing error: required key has incorrectly "
              "formatted data; incorrect row dimension found in "
              "STRING_2DARRAY";
      }
      if(!(json_value[i].isArray()) && !(json_value[i].isString())) {
        throw "JSON Parsing error: required key has incorrectly formatted "
              "data; expected STRING_2DARRAY";
      }
      for(unsigned int j = 0; j < json_value[i].size(); j++) {
        if(col_dim_ > 0 && col_dim_ != (int)json_value[i].size()) {
          throw "JSON Parsing error: required key has incorrectly "
                "formatted data; incorrect column dimension found in "
                "STRING_2DARRAY";
        }

        if(json_value[i].isArray() && !(json_value[i][j].isString())) {
          throw "JSON Parsing error: required key has incorrectly "
                "formatted data; expected STRING_2DARRAY";
        }
      }
    }
  }

  for(unsigned int i = 0; i < json_value.size(); i++) {
    std::vector<std::string> row;
    if(json_value[i].isArray()) {
      for(unsigned int j = 0; j < json_value[i].size(); j++) {
        row.push_back(json_value[i][j].asString());
      }
    } else if(json_value[i].isString()) {
      row.push_back(json_value[i].asString());
    }
    std::any_cast<std::vector<std::vector<std::string>>*>(data_)->push_back(
      row);
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::Value::readJSON_DOUBLE(Json::Value& json_value) {
  if(!json_value.isDouble()) {
    throw "JSON Parsing error, required key has incorrectly formatted "
          "data; expected DOUBLE";
  }
  *std::any_cast<double*>(data_) = json_value.asDouble();
  loaded_from_file_ = true;
}

void Shift::Parameter::Value::readJSON_DOUBLE_ARRAY(Json::Value& json_value) {
  if(!json_value.isArray()) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected DOUBLE_ARRAY";
  }
  if(row_dim_ > 0 && row_dim_ != static_cast<int>(json_value.size())) {
    throw "JSON Parsing error: required key has incorrect number of elements";
  }
  if(json_value.size() == 0) {
    throw "JSON Parsing error: array has zero elements";
  }

  if(row_dim_ > -1 && col_dim_ > -1 &&
     static_cast<int>(json_value.size()) != row_dim_ * col_dim_) {
    throw "JSON Parsing error: DOUBLE_ARRAY has incorrect dimension ";
  }

  std::any_cast<std::vector<double>*>(data_)->clear();
  for(unsigned int i = 0; i < json_value.size(); i++) {
    if(!json_value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected DOUBLE_ARRAY";
    }
    std::any_cast<std::vector<double>*>(data_)->push_back(
      json_value[i].asDouble());
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::Value::readJSON_KSL_VEC3(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 3)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_VEC3";
  }
  ksl_vec3_t* data = std::any_cast<ksl_vec3_t*>(data_);
  for(int i = 0; i < 3; i++) {
    if(!json_value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_VEC3";
    }
    data->at[i] = json_value[i].asDouble();
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::Value::readJSON_KSL_VEC4(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 4)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_VEC4";
  }
  ksl_vec4_t* data = std::any_cast<ksl_vec4_t*>(data_);
  for(int i = 0; i < 4; i++) {
    if(!json_value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_VEC4";
      return;
    }
    data->at[i] = json_value[i].asDouble();
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::Value::readJSON_KSL_QUATERNION(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 4)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_QUATERNION";
    return;
  }
  ksl_quaternion_t quaternion;
  for(int i = 0; i < 4; i++) {
    if(!json_value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_QUATERNION";
      return;
    }
    quaternion.at[i] = json_value[i].asDouble();
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

void Shift::Parameter::Value::readJSON_KSL_SCREW(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 6)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_SCREW";
    return;
  }
  ksl_screw_t* data = std::any_cast<ksl_screw_t*>(data_);
  for(int i = 0; i < 6; i++) {
    if(!json_value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_SCREW";
      return;
    }
    data->at[i] = json_value[i].asDouble();
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::Value::readJSON_KSL_COSCREW(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 6)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_COSCREW";
    return;
  }
  ksl_coscrew_t* data = std::any_cast<ksl_coscrew_t*>(data_);
  for(int i = 0; i < 6; i++) {
    if(!json_value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_COSCREW";
      return;
    }
    data->at[i] = json_value[i].asDouble();
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::Value::readJSON_KSL_INERTIA(Json::Value& json_value) {
  if(!json_value.isObject()) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_INERTIA";
    return;
  }
  ksl_inertia_t* inertia = std::any_cast<ksl_inertia_t*>(data_);

  if(json_value.isMember("mass")) {
    if(!json_value["mass"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->mass = json_value["mass"].asDouble();
  }

  if(json_value.isMember("Ixx")) {
    if(!json_value["Ixx"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Ixx = json_value["Ixx"].asDouble();
  }

  if(json_value.isMember("Iyy")) {
    if(!json_value["Iyy"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Iyy = json_value["Iyy"].asDouble();
  }

  if(json_value.isMember("Izz")) {
    if(!json_value["Izz"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Izz = json_value["Izz"].asDouble();
  }

  if(json_value.isMember("Ixy")) {
    if(!json_value["Ixy"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Ixy = json_value["Ixy"].asDouble();
  }

  if(json_value.isMember("Iyz")) {
    if(!json_value["Iyz"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Iyz = json_value["Iyz"].asDouble();
  }

  if(json_value.isMember("Izx")) {
    if(!json_value["Izx"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Izx = json_value["Izx"].asDouble();
  }

  if(json_value.isMember("Iyx")) {
    if(!json_value["Iyx"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Iyx = json_value["Iyx"].asDouble();
  }

  if(json_value.isMember("Izy")) {
    if(!json_value["Izy"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Izy = json_value["Izy"].asDouble();
  }

  if(json_value.isMember("Ixz")) {
    if(!json_value["Ixz"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Ixz = json_value["Ixz"].asDouble();
  }

  loaded_from_file_ = true;
}

void Shift::Parameter::Value::readJSON_KSL_MAT3X3(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 3)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_MAT3X3";
    return;
  }
  for(int i = 0; i < 3; i++) {
    if(!json_value[i].isArray() || !(json_value[i].size() == 3)) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_MAT3X3";
      return;
    }
  }

  ksl_mat3x3_t* data = std::any_cast<ksl_mat3x3_t*>(data_);
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      if(!json_value[i][j].isDouble()) {
        throw "JSON Parsing error: required key has incorrectly formatted "
              "data; expected KSL_MAT3X3";
      }
      data->at[j][i] = json_value[i][j].asDouble();
    }
  }
  if(normalize_mat3x3_) {
    ksl_mat3x3_normalize(data);
  }
  loaded_from_file_ = true;
}

void Shift::Parameter::Value::readJSON_KSL_MAT4X4(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 4)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_MAT3X3";
    return;
  }
  for(int i = 0; i < 4; i++) {
    if(!json_value[i].isArray() || !(json_value[i].size() == 4)) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_MAT3X3";
      return;
    }
  }

  ksl_mat4x4_t mat4x4;
  ksl_mat4x4_setIdentity(&mat4x4);
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      if(!json_value[i][j].isDouble()) {
        throw "JSON Parsing error: required key has incorrectly formatted "
              "data; expected KSL_MAT3X3";
      }
      mat4x4.at[j][i] = json_value[i][j].asDouble();
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

void Shift::Parameter::Value::readJSON_EULERANGLES(Json::Value& json_value) {
  if(!json_value.isObject()) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_EULERANGLES";
    return;
  }

  ksl_axis_enum_t sequence = KSL_AXIS_XYZ;
  ksl_vec3_t angles;
  if(json_value.isMember("angles") && json_value["angles"].size() == 3) {
    for(int i = 0; i < 3; i++) {
      if(!json_value["angles"][i].isDouble()) {
        throw "JSON Parsing error: angles object is incorrectly formatted";
      }
      angles.at[i] = json_value["angles"][i].asDouble();
    }
  } else {
    throw "JSON Parsing error: \"euler_angles\" object has no key \"angles\"";
    return;
  }

  if(json_value.isMember("axes")) {
    if(json_value["axes"].isArray() && json_value["axes"].size() == 3) {
      for(int i = 0; i < 3; i++) {
        if(json_value["angles"][i].isString()) {
          if(json_value["angles"][i].asString() == "x") {
            axes_.at[i] = 0;
          } else if(json_value["angles"][i].asString() == "y") {
            axes_.at[i] = 1;
          } else if(json_value["angles"][i].asString() == "z") {
            axes_.at[i] = 2;
          }
        } else if(json_value["angles"][i].isInt()) {
          axes_.at[i] = json_value["angles"][i].asInt();
        } else {
          throw "JSON Parsing error: angles object is incorrectly formatted, "
                "expected integer gets 0, 1, or 2 - or string gets x, y, "
                "or z";
          return;
        }
      }
      sequence = ksl_axis_getEulerAngles(axes_);
      if(sequence == KSL_AXIS_INVALID) {
        throw "JSON Parsing error: \"euler_angles\" axes object is incorrectly "
              "formatted, expected three integer gets {0, 1, 2} or three "
              "string gets {x, y, z}";
        return;
      } else {
      }
    } else {
      throw "JSON Parsing error: \"euler_angles\" axes object is incorrectly "
            "formatted, expected three integer gets or three "
            "string gets following the format [0, 1, 2] or "
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

void Shift::Parameter::Value::readJSON_BOOLEAN(Json::Value& json_value) {
  if(!json_value.isBool()) {
    throw "JSON Parsing error, required key %s has incorrectly formatted "
          "data; expected BOOLEAN";
  }
  bool* data = std::any_cast<bool*>(data_);
  *data = json_value.asBool();
  loaded_from_file_ = true;
}

void Shift::Parameter::Value::readJSON_POSE(Json::Value& json_value) {
  if(!json_value.isObject()) {
    throw "Error: Pose is not a valid object";
    return;
  }

  if(json_value.isMember("quaternion")) {
    readJSON_KSL_QUATERNION(json_value["quaternion"]);
  } else if(json_value.isMember("q")) {
    readJSON_KSL_QUATERNION(json_value["q"]);
  }

  if(json_value.isMember("mat3x3")) {
    readJSON_KSL_MAT3X3(json_value["mat3x3"]);
  } else if(json_value.isMember("R")) {
    readJSON_KSL_MAT3X3(json_value["R"]);
  }

  if(json_value.isMember("mat4x4")) {
    readJSON_KSL_MAT4X4(json_value["mat4x4"]);
  }

  if(json_value.isMember("euler_angles")) {
    readJSON_EULERANGLES(json_value["euler_angles"]);
  }

  loaded_from_file_ = true;
}

void Shift::Parameter::Value::print() const {
  print("");
}

void Shift::Parameter::Value::print(std::string prefix) const {

  /* check if get exists */
  switch(parameter_type_) {
    case STRING: {
      std::cout << get<std::string>() << std::endl;
      break;
    }
    case STRING_ARRAY: {
      std::cout << "[ ";
      for(auto& i : get<std::vector<std::string>>()) {
        std::cout << "\"" << i << "\" ";
      }
      std::cout << "]" << std::endl;
      break;
    }
    case STRING_2DARRAY: {
      std::cout << "[ " << std::endl;
      for(auto& row : get<std::vector<std::vector<std::string>>>()) {
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
    case DOUBLE: {
      std::cout << get<double>() << std::endl;
      break;
    }
    case DOUBLE_ARRAY: {
      std::cout << "[ ";
      for(auto& i : get<std::vector<double>>()) {
        std::cout << i << " ";
      }
      std::cout << " ]" << std::endl;
      break;
    }
    case KSL_VEC3: {
      ksl_vec3_print(stdout, data<ksl_vec3_t>());
      break;
    }
    case KSL_VEC4: {
      ksl_vec4_print(stdout, data<ksl_vec4_t>());
      break;
    }
    case KSL_QUATERNION: {
      ksl_quaternion_print(stdout, data<ksl_quaternion_t>());
      break;
    }
    case KSL_SCREW: {
      ksl_screw_print(stdout, data<ksl_screw_t>());
      break;
    }
    case KSL_COSCREW: {
      ksl_coscrew_print(stdout, data<ksl_coscrew_t>());
      break;
    }
    case KSL_INERTIA: {
      std::cout << std::endl;
      ksl_inertia_print(stdout, data<ksl_inertia_t>(), prefix.c_str());
      break;
    }
    case KSL_MAT3X3: {
      std::cout << std::endl;
      ksl_mat3x3_print(stdout, data<ksl_mat3x3_t>(), prefix.c_str());
      break;
    }
    case KSL_MAT4X4: {
      std::cout << std::endl;
      ksl_mat4x4_print(stdout, data<ksl_mat4x4_t>(), prefix.c_str());
      break;
    }
    case KSL_EULERANGLES: {
      std::cout << std::endl;
      ksl_eulerangles_print(stdout, data<ksl_eulerangles_t>(), prefix.c_str());
      break;
    }
    case BOOLEAN: {
      std::cout << (get<bool>() ? "true" : "false") << std::endl;
      break;
    }
    case KSL_SE3: {
      std::cout << std::endl;
      ksl_SE3_print(stdout, data<ksl_SE3_t>(), prefix.c_str());
      break;
    }
    case PARAMETER_LIST: {
      get<Shift::Parameter::List>().print(prefix);
      break;
    }
    case PARAMETER_ARRAY: {
      get<Shift::Parameter::ArrayBase>().print(prefix);
      break;
    }
  }
}

void Shift::Parameter::Value::setParent(Context* context,
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

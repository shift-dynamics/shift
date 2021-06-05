#include "shift/core/parameter/Value.h"
#include "shift/core/Assembly.h"
#include "shift/core/behavior/Base.h"
#include "shift/core/body/Base.h"
#include "shift/core/integrator/Base.h"
#include "shift/core/logger/Base.h"
#include "shift/core/ModelParameters.h"
#include "shift/core/parameter/ArrayBase.h"
#include "shift/core/sensor/Base.h"
#include "shift/core/terrain/Base.h"
#include "shift/core/Exception.h"

#include "shift/core/Frame.h"

#include "ksl/print.h"

// if user requests Shift object, up cast parameter::List to requested
// type
template <>
const shift::core::Frame& shift::core::parameter::Value::get<shift::core::Frame>() const {
  return *dynamic_cast<shift::core::Frame*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::Assembly& shift::core::parameter::Value::get<shift::core::Assembly>() const {
  return *dynamic_cast<shift::core::Assembly*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::behavior::Base&
shift::core::parameter::Value::get<shift::core::behavior::Base>() const {
  return *dynamic_cast<shift::core::behavior::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::body::Base&
shift::core::parameter::Value::get<shift::core::body::Base>() const {
  return *dynamic_cast<shift::core::body::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::collisiongeometry::Base&
shift::core::parameter::Value::get<shift::core::collisiongeometry::Base>() const {
  return *dynamic_cast<shift::core::collisiongeometry::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::constraint::Base&
shift::core::parameter::Value::get<shift::core::constraint::Base>() const {
  return *dynamic_cast<shift::core::constraint::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::forcemodule::Base&
shift::core::parameter::Value::get<shift::core::forcemodule::Base>() const {
  return *dynamic_cast<shift::core::forcemodule::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::geometry::Base&
shift::core::parameter::Value::get<shift::core::geometry::Base>() const {
  return *dynamic_cast<shift::core::geometry::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::integrator::Base&
shift::core::parameter::Value::get<shift::core::integrator::Base>() const {
  return *dynamic_cast<shift::core::integrator::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::joint::Base&
shift::core::parameter::Value::get<shift::core::joint::Base>() const {
  return *dynamic_cast<shift::core::joint::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::light::Base&
shift::core::parameter::Value::get<shift::core::light::Base>() const {
  return *dynamic_cast<shift::core::light::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::logger::Base&
shift::core::parameter::Value::get<shift::core::logger::Base>() const {
  return *dynamic_cast<shift::core::logger::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::sensor::Base&
shift::core::parameter::Value::get<shift::core::sensor::Base>() const {
  return *dynamic_cast<shift::core::sensor::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::solver::Base&
shift::core::parameter::Value::get<shift::core::solver::Base>() const {
  return *dynamic_cast<shift::core::solver::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
const shift::core::terrain::Base&
shift::core::parameter::Value::get<shift::core::terrain::Base>() const {
  return *dynamic_cast<shift::core::terrain::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::Frame* shift::core::parameter::Value::get_data<shift::core::Frame>() const {
  return dynamic_cast<shift::core::Frame*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::Assembly* shift::core::parameter::Value::get_data<shift::core::Assembly>() const {
  return dynamic_cast<shift::core::Assembly*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::behavior::Base*
shift::core::parameter::Value::get_data<shift::core::behavior::Base>() const {
  return dynamic_cast<shift::core::behavior::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::body::Base* shift::core::parameter::Value::get_data<shift::core::body::Base>() const {
  return dynamic_cast<shift::core::body::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::collisiongeometry::Base*
shift::core::parameter::Value::get_data<shift::core::collisiongeometry::Base>() const {
  return dynamic_cast<shift::core::collisiongeometry::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::constraint::Base*
shift::core::parameter::Value::get_data<shift::core::constraint::Base>() const {
  return dynamic_cast<shift::core::constraint::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::forcemodule::Base*
shift::core::parameter::Value::get_data<shift::core::forcemodule::Base>() const {
  return dynamic_cast<shift::core::forcemodule::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::geometry::Base*
shift::core::parameter::Value::get_data<shift::core::geometry::Base>() const {
  return dynamic_cast<shift::core::geometry::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::integrator::Base*
shift::core::parameter::Value::get_data<shift::core::integrator::Base>() const {
  return dynamic_cast<shift::core::integrator::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::joint::Base* shift::core::parameter::Value::get_data<shift::core::joint::Base>() const {
  return dynamic_cast<shift::core::joint::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::light::Base* shift::core::parameter::Value::get_data<shift::core::light::Base>() const {
  return dynamic_cast<shift::core::light::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::logger::Base*
shift::core::parameter::Value::get_data<shift::core::logger::Base>() const {
  return dynamic_cast<shift::core::logger::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::sensor::Base*
shift::core::parameter::Value::get_data<shift::core::sensor::Base>() const {
  return dynamic_cast<shift::core::sensor::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::solver::Base*
shift::core::parameter::Value::get_data<shift::core::solver::Base>() const {
  return dynamic_cast<shift::core::solver::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

template <>
shift::core::terrain::Base*
shift::core::parameter::Value::get_data<shift::core::terrain::Base>() const {
  return dynamic_cast<shift::core::terrain::Base*>(
    std::any_cast<shift::core::parameter::List*>(data_));
}

shift::core::parameter::Value& shift::core::parameter::Value::
operator[](const std::string key_str) {
  if(is_list_) {
    return std::any_cast<parameter::List*>(data_)->operator[](key_str);
  } else if(is_array_) {
    return std::any_cast<parameter::ArrayBase*>(data_)->operator[](key_str);
  } else {
    throw shift::core::Exception(
      "error: no parameter found for key '" + key_str + "'");
    return std::any_cast<parameter::List*>(data_)->operator[](key_str);
  }
}

shift::core::parameter::Value& shift::core::parameter::Value::
operator[](const char* key_str) {
  return operator[](std::string(key_str));
}

shift::core::parameter::Value& shift::core::parameter::Value::
operator[](unsigned int index) {
  if(is_array_) {
    return std::any_cast<parameter::ArrayBase*>(data_)->operator[](index);
  } else {
   throw shift::core::Exception(
      "error: no parameter found for index '" + std::to_string(index) + "'");
  }
}

void shift::core::parameter::Value::ActivateObjects(bool parent_is_active) {
  if(is_array_) {
    std::any_cast<parameter::ArrayBase*>(data_)->ActivateObjects(
      parent_is_active);
  } else if(is_list_) {
    std::any_cast<parameter::List*>(data_)->ActivateObjects(parent_is_active);
  }
}

void shift::core::parameter::Value::ReadJSON(Json::Value& object) {
  switch(parameter_type_) {
    case STRING: {
      ReadJSON_STRING(object);
      break;
    }
    case STRING_ARRAY: {
      ReadJSON_STRING_ARRAY(object);
      break;
    }
    case STRING_2DARRAY: {
      ReadJSON_STRING_2DARRAY(object);
      break;
    }
    case DOUBLE: {
      ReadJSON_DOUBLE(object);
      break;
    }
    case INT: {
      ReadJSON_INT(object);
      break;
    }
    case INT_ARRAY: {
      ReadJSON_INT_ARRAY(object);
      break;
    }
    case DOUBLE_ARRAY: {
      ReadJSON_DOUBLE_ARRAY(object);
      break;
    }
    case KSL_VEC3: {
      ReadJSON_KSL_VEC3(object);
      break;
    }
    case KSL_VEC4: {
      ReadJSON_KSL_VEC4(object);
      break;
    }
    case KSL_QUATERNION: {
      ReadJSON_KSL_QUATERNION(object);
      break;
    }
    case KSL_SCREW: {
      ReadJSON_KSL_SCREW(object);
      break;
    }
    case KSL_COSCREW: {
      ReadJSON_KSL_COSCREW(object);
      break;
    }
    case KSL_INERTIA: {
      ReadJSON_KSL_INERTIA(object);
      break;
    }
    case KSL_MAT3X3: {
      ReadJSON_KSL_MAT3X3(object);
      break;
    }
    case KSL_MAT4X4: {
      ReadJSON_KSL_MAT4X4(object);
      break;
    }
    case KSL_EULERANGLES: {
      ReadJSON_EULERANGLES(object);
      break;
    }
    case BOOLEAN: {
      ReadJSON_BOOLEAN(object);
      break;
    }
    case KSL_SE3: {
      ReadJSON_POSE(object);
      break;
    }
    case PARAMETER_LIST: {
      if(object.isObject()) {
        std::any_cast<parameter::List*>(data_)->ReadJSON(object);
      }
      break;
    }
    case PARAMETER_ARRAY: {
      if(object.isArray()) {
        std::any_cast<parameter::ArrayBase*>(data_)->ReadJSON(object);
      }
      break;
    }
  }
}

void shift::core::parameter::Value::ReadJSON_STRING(Json::Value& json_value) {
  if(!json_value.isString()) {
    if(required_) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted data; " 
        "expected STRING");
    }
    return;
  }
  (*std::any_cast<std::string*>(data_)).assign(json_value.asString());
  loaded_from_file_ = true;
}

void shift::core::parameter::Value::ReadJSON_STRING_ARRAY(Json::Value& json_value) {
  if(!json_value.isArray()) {
    if(required_) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected STRING_ARRAY");
    }
    return;
  }
  for(unsigned int i = 0; i < json_value.size(); i++) {
    if(!(json_value[i].isString())) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted " 
        "data; expected STRING_ARRAY");
    }
  }

  for(unsigned int i = 0; i < json_value.size(); i++) {
    (*std::any_cast<std::vector<std::string>*>(data_))
      .push_back(json_value[i].asString());
  }
  loaded_from_file_ = true;
}

void shift::core::parameter::Value::ReadJSON_STRING_2DARRAY(Json::Value& json_value) {
  if(json_value.isArray()) {
    for(unsigned int i = 0; i < json_value.size(); i++) {
      if(row_dim_ > 0 && row_dim_ != (int)json_value.size()) {
        throw shift::core::Exception(
          "JSON Parsing error: required key has incorrectly "
          "formatted data; incorrect row dimension found in "
          "STRING_2DARRAY");
      }
      if(!(json_value[i].isArray()) && !(json_value[i].isString())) {
        throw shift::core::Exception(
          "JSON Parsing error: required key has incorrectly formatted "
          "data; expected STRING_2DARRAY");
      }
      for(unsigned int j = 0; j < json_value[i].size(); j++) {
        if(col_dim_ > 0 && col_dim_ != (int)json_value[i].size()) {
          throw shift::core::Exception(
            "JSON Parsing error: required key has incorrectly "
            "formatted data; incorrect column dimension found in "
            "STRING_2DARRAY");
        }

        if(json_value[i].isArray() && !(json_value[i][j].isString())) {
          throw shift::core::Exception(
            "JSON Parsing error: required key has incorrectly "
            "formatted data; expected STRING_2DARRAY");
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

void shift::core::parameter::Value::ReadJSON_INT(Json::Value& json_value) {
  if(!json_value.isInt()) {
    throw shift::core::Exception(
      "JSON Parsing error, required key has incorrectly formatted " 
      "data; expected DOUBLE");
  }
  *std::any_cast<int*>(data_) = json_value.asInt();
  loaded_from_file_ = true;
}

void shift::core::parameter::Value::ReadJSON_INT_ARRAY(Json::Value& json_value) {
  if(!json_value.isArray()) {
    throw shift::core::Exception(
      "JSON Parsing error: required key has incorrectly formatted " 
      "data; expected DOUBLE_ARRAY");
  }
  if(row_dim_ > 0 && row_dim_ != static_cast<int>(json_value.size())) {
    throw shift::core::Exception(
      "JSON Parsing error: required key has incorrect number of elements");
  }
  if(json_value.size() == 0) {
    throw shift::core::Exception(
      "JSON Parsing error: array has zero elements");
  }

  if(row_dim_ > -1 && col_dim_ > -1 &&
     static_cast<int>(json_value.size()) != row_dim_ * col_dim_) {
    throw shift::core::Exception(
      "JSON Parsing error: DOUBLE_ARRAY has incorrect dimension");
  }

  std::any_cast<std::vector<int>*>(data_)->clear();
  for(unsigned int i = 0; i < json_value.size(); i++) {
    if(!json_value[i].isInt()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted " 
        "data; expected DOUBLE_ARRAY");
    }
    std::any_cast<std::vector<int>*>(data_)->push_back(
      json_value[i].asInt());
  }
  loaded_from_file_ = true;
}

void shift::core::parameter::Value::ReadJSON_DOUBLE(Json::Value& json_value) {
  if(!json_value.isDouble()) {
    throw shift::core::Exception(
      "JSON Parsing error, required key has incorrectly formatted " 
      "data; expected DOUBLE");
  }
  *std::any_cast<double*>(data_) = json_value.asDouble();
  loaded_from_file_ = true;
}

void shift::core::parameter::Value::ReadJSON_DOUBLE_ARRAY(Json::Value& json_value) {
  if(!json_value.isArray()) {
    throw shift::core::Exception(
      "JSON Parsing error: required key has incorrectly formatted " 
      "data; expected DOUBLE_ARRAY");
  }
  if(row_dim_ > 0 && row_dim_ != static_cast<int>(json_value.size())) {
    throw shift::core::Exception(
      "JSON Parsing error: required key has incorrect number of elements");
  }
  if(json_value.size() == 0) {
    throw shift::core::Exception(
      "JSON Parsing error: array has zero elements");
  }

  if(row_dim_ > -1 && col_dim_ > -1 &&
     static_cast<int>(json_value.size()) != row_dim_ * col_dim_) {
    throw shift::core::Exception(
      "JSON Parsing error: DOUBLE_ARRAY has incorrect dimension");
  }

  std::any_cast<std::vector<double>*>(data_)->clear();
  for(unsigned int i = 0; i < json_value.size(); i++) {
    if(!json_value[i].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted " 
        "data; expected DOUBLE_ARRAY");
    }
    std::any_cast<std::vector<double>*>(data_)->push_back(
      json_value[i].asDouble());
  }
  loaded_from_file_ = true;
}

void shift::core::parameter::Value::ReadJSON_KSL_VEC3(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 3)) {
    throw "JSON Parsing error: required key has incorrectly formatted " 
          "data; expected KSL_VEC3";
  }
  ksl_vec3_t* data = std::any_cast<ksl_vec3_t*>(data_);
  for(int i = 0; i < 3; i++) {
    if(!json_value[i].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted " 
        "data; expected KSL_VEC3");
    }
    data->at[i] = json_value[i].asDouble();
  }
  loaded_from_file_ = true;
}

void shift::core::parameter::Value::ReadJSON_KSL_VEC4(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 4)) {
    throw shift::core::Exception(
      "JSON Parsing error: required key has incorrectly formatted "
      "data; expected KSL_VEC4");
  }
  ksl_vec4_t* data = std::any_cast<ksl_vec4_t*>(data_);
  for(int i = 0; i < 4; i++) {
    if(!json_value[i].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_VEC4");
      return;
    }
    data->at[i] = json_value[i].asDouble();
  }
  loaded_from_file_ = true;
}

void shift::core::parameter::Value::ReadJSON_KSL_QUATERNION(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 4)) {
    throw shift::core::Exception(
      "JSON Parsing error: required key has incorrectly formatted "
      "data; expected KSL_QUATERNION");
    return;
  }
  ksl_quaternion_t quaternion;
  for(int i = 0; i < 4; i++) {
    if(!json_value[i].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_QUATERNION");
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

void shift::core::parameter::Value::ReadJSON_KSL_SCREW(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 6)) {
    throw shift::core::Exception(
      "JSON Parsing error: required key has incorrectly formatted "
      "data; expected KSL_SCREW");
    return;
  }
  ksl_screw_t* data = std::any_cast<ksl_screw_t*>(data_);
  for(int i = 0; i < 6; i++) {
    if(!json_value[i].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_SCREW");
      return;
    }
    data->at[i] = json_value[i].asDouble();
  }
  loaded_from_file_ = true;
}

void shift::core::parameter::Value::ReadJSON_KSL_COSCREW(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 6)) {
    throw shift::core::Exception(
      "JSON Parsing error: required key has incorrectly formatted "
      "data; expected KSL_COSCREW");
    return;
  }
  ksl_coscrew_t* data = std::any_cast<ksl_coscrew_t*>(data_);
  for(int i = 0; i < 6; i++) {
    if(!json_value[i].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_COSCREW");
      return;
    }
    data->at[i] = json_value[i].asDouble();
  }
  loaded_from_file_ = true;
}

void shift::core::parameter::Value::ReadJSON_KSL_INERTIA(Json::Value& json_value) {
  if(!json_value.isObject()) {
    throw shift::core::Exception(
      "JSON Parsing error: required key has incorrectly formatted "
      "data; expected KSL_INERTIA");
    return;
  }
  ksl_inertia_t* inertia = std::any_cast<ksl_inertia_t*>(data_);

  if(json_value.isMember("mass")) {
    if(!json_value["mass"].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_INERTIA");
      return;
    }
    inertia->mass = json_value["mass"].asDouble();
  }

  if(json_value.isMember("Ixx")) {
    if(!json_value["Ixx"].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_INERTIA");
      return;
    }
    inertia->Ixx = json_value["Ixx"].asDouble();
  }

  if(json_value.isMember("Iyy")) {
    if(!json_value["Iyy"].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_INERTIA");
      return;
    }
    inertia->Iyy = json_value["Iyy"].asDouble();
  }

  if(json_value.isMember("Izz")) {
    if(!json_value["Izz"].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_INERTIA");
      return;
    }
    inertia->Izz = json_value["Izz"].asDouble();
  }

  if(json_value.isMember("Ixy")) {
    if(!json_value["Ixy"].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_INERTIA");
      return;
    }
    inertia->Ixy = json_value["Ixy"].asDouble();
  }

  if(json_value.isMember("Iyz")) {
    if(!json_value["Iyz"].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_INERTIA");
      return;
    }
    inertia->Iyz = json_value["Iyz"].asDouble();
  }

  if(json_value.isMember("Izx")) {
    if(!json_value["Izx"].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_INERTIA");
      return;
    }
    inertia->Izx = json_value["Izx"].asDouble();
  }

  if(json_value.isMember("Iyx")) {
    if(!json_value["Iyx"].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_INERTIA");
      return;
    }
    inertia->Iyx = json_value["Iyx"].asDouble();
  }

  if(json_value.isMember("Izy")) {
    if(!json_value["Izy"].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_INERTIA");
      return;
    }
    inertia->Izy = json_value["Izy"].asDouble();
  }

  if(json_value.isMember("Ixz")) {
    if(!json_value["Ixz"].isDouble()) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_INERTIA");
      return;
    }
    inertia->Ixz = json_value["Ixz"].asDouble();
  }

  loaded_from_file_ = true;
}

void shift::core::parameter::Value::ReadJSON_KSL_MAT3X3(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 3)) {
    throw shift::core::Exception(
      "JSON Parsing error: required key has incorrectly formatted "
      "data; expected KSL_MAT3X3");
    return;
  }
  for(int i = 0; i < 3; i++) {
    if(!json_value[i].isArray() || !(json_value[i].size() == 3)) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_MAT3X3");
      return;
    }
  }

  ksl_mat3x3_t* data = std::any_cast<ksl_mat3x3_t*>(data_);
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      if(!json_value[i][j].isDouble()) {
        throw shift::core::Exception(
          "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_MAT3X3");
      }
      data->at[j][i] = json_value[i][j].asDouble();
    }
  }
  if(normalize_mat3x3_) {
    ksl_mat3x3_normalize(data);
  }
  loaded_from_file_ = true;
}

void shift::core::parameter::Value::ReadJSON_KSL_MAT4X4(Json::Value& json_value) {
  if(!json_value.isArray() || !(json_value.size() == 4)) {
    throw shift::core::Exception(
      "JSON Parsing error: required key has incorrectly formatted "
      "data; expected KSL_MAT3X3");
    return;
  }
  for(int i = 0; i < 4; i++) {
    if(!json_value[i].isArray() || !(json_value[i].size() == 4)) {
      throw shift::core::Exception(
        "JSON Parsing error: required key has incorrectly formatted "
        "data; expected KSL_MAT3X3");
      return;
    }
  }

  ksl_mat4x4_t mat4x4;
  ksl_mat4x4_setIdentity(&mat4x4);
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      if(!json_value[i][j].isDouble()) {
        throw shift::core::Exception(
          "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_MAT3X3");
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

void shift::core::parameter::Value::ReadJSON_EULERANGLES(Json::Value& json_value) {
  if(!json_value.isObject()) {
    throw shift::core::Exception(
      "JSON Parsing error: required key has incorrectly formatted "
      "data; expected KSL_EULERANGLES");
    return;
  }

  ksl_axis_enum_t sequence = KSL_AXIS_XYZ;
  ksl_vec3_t angles;
  if(json_value.isMember("angles") && json_value["angles"].size() == 3) {
    for(int i = 0; i < 3; i++) {
      if(!json_value["angles"][i].isDouble()) {
        throw shift::core::Exception(
          "JSON Parsing error: angles object is incorrectly formatted");
      }
      angles.at[i] = json_value["angles"][i].asDouble();
    }
  } else {
    throw shift::core::Exception(
      "JSON Parsing error: \"euler_angles\" object has no key \"angles\"");
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
          throw shift::core::Exception(
            "JSON Parsing error: angles object is incorrectly formatted, "
            "expected integer gets 0, 1, or 2 - or string gets x, y, "
            "or z");
          return;
        }
      }
      sequence = ksl_axis_getEulerAngles(axes_);
      if(sequence == KSL_AXIS_INVALID) {
        throw shift::core::Exception(
          "JSON Parsing error: \"euler_angles\" axes object is incorrectly "
          "formatted, expected three integer gets {0, 1, 2} or three "
          "string gets {x, y, z}");
        return;
      } else {
      }
    } else {
      throw shift::core::Exception(
        "JSON Parsing error: \"euler_angles\" axes object is incorrectly "
        "formatted, expected three integer gets or three "
        "string gets following the format [0, 1, 2] or "
        "[\"x\", \"y\", \"z\"], where the axes follow one of the twelve "
        "Euler angle conventions (XYZ, YZX, ZXY, XZY, YXZ, ZYX, XYX, XZX, "
        "YXY, YZY, ZXZ, or ZYZ).");
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

void shift::core::parameter::Value::ReadJSON_BOOLEAN(Json::Value& json_value) {
  if(!json_value.isBool()) {
    throw shift::core::Exception(
      "JSON Parsing error, required key %s has incorrectly formatted "
      "data; expected BOOLEAN");
  }
  bool* data = std::any_cast<bool*>(data_);
  *data = json_value.asBool();
  loaded_from_file_ = true;
}

void shift::core::parameter::Value::ReadJSON_POSE(Json::Value& json_value) {
  if(!json_value.isObject()) {
    throw shift::core::Exception(
      "Error: Pose is not a valid object");
    return;
  }

  if(json_value.isMember("quaternion")) {
    ReadJSON_KSL_QUATERNION(json_value["quaternion"]);
  } else if(json_value.isMember("q")) {
    ReadJSON_KSL_QUATERNION(json_value["q"]);
  }

  if(json_value.isMember("mat3x3")) {
    ReadJSON_KSL_MAT3X3(json_value["mat3x3"]);
  } else if(json_value.isMember("R")) {
    ReadJSON_KSL_MAT3X3(json_value["R"]);
  }

  if(json_value.isMember("mat4x4")) {
    ReadJSON_KSL_MAT4X4(json_value["mat4x4"]);
  }

  if(json_value.isMember("euler_angles")) {
    ReadJSON_EULERANGLES(json_value["euler_angles"]);
  }

  loaded_from_file_ = true;
}

void shift::core::parameter::Value::Print() const {
  Print("");
}

void shift::core::parameter::Value::Print(std::string prefix) const {

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
    case INT: {
      std::cout << get<int>() << std::endl;
      break;
    }
    case INT_ARRAY: {
      std::cout << "[ ";
      for(auto& i : get<std::vector<int>>()) {
        std::cout << i << " ";
      }
      std::cout << " ]" << std::endl;
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
      ksl_vec3_print(stdout, get_data<ksl_vec3_t>());
      break;
    }
    case KSL_VEC4: {
      ksl_vec4_print(stdout, get_data<ksl_vec4_t>());
      break;
    }
    case KSL_QUATERNION: {
      ksl_quaternion_print(stdout, get_data<ksl_quaternion_t>());
      break;
    }
    case KSL_SCREW: {
      ksl_screw_print(stdout, get_data<ksl_screw_t>());
      break;
    }
    case KSL_COSCREW: {
      ksl_coscrew_print(stdout, get_data<ksl_coscrew_t>());
      break;
    }
    case KSL_INERTIA: {
      std::cout << std::endl;
      ksl_inertia_print(stdout, get_data<ksl_inertia_t>(), prefix.c_str());
      break;
    }
    case KSL_MAT3X3: {
      std::cout << std::endl;
      ksl_mat3x3_print(stdout, get_data<ksl_mat3x3_t>(), prefix.c_str());
      break;
    }
    case KSL_MAT4X4: {
      std::cout << std::endl;
      ksl_mat4x4_print(stdout, get_data<ksl_mat4x4_t>(), prefix.c_str());
      break;
    }
    case KSL_EULERANGLES: {
      std::cout << std::endl;
      ksl_eulerangles_print(stdout, get_data<ksl_eulerangles_t>(), prefix.c_str());
      break;
    }
    case BOOLEAN: {
      std::cout << (get_data<bool>() ? "true" : "false") << std::endl;
      break;
    }
    case KSL_SE3: {
      std::cout << std::endl;
      ksl_SE3_print(stdout, get_data<ksl_SE3_t>(), prefix.c_str());
      break;
    }
    case PARAMETER_LIST: {
      get<shift::core::parameter::List>().Print(prefix);
      break;
    }
    case PARAMETER_ARRAY: {
      get<shift::core::parameter::ArrayBase>().Print(prefix);
      break;
    }
  }
}

void shift::core::parameter::Value::set_parent(Context* context,
                                        Assembly* parent_assembly,
                                        shift::core::body::Base* parent_body) {
  if(is_list_) {
    std::any_cast<parameter::List*>(data_)->set_parent(context, parent_assembly,
                                                      parent_body);
  } else if(is_array_) {
    std::any_cast<parameter::ArrayBase*>(data_)->set_parent(
      context, parent_assembly, parent_body);
  }
}

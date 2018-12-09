#include "shift-core/serialize.h"
#include "ksl/mat4x4.h"

#include <iostream>

Shift::SerialEntry::SerialEntry(const std::string key,
                                const Shift::SerialType serialType, void* data,
                                const int rowDim, const int colDim) {
  key_ = key;
  type_ = serialType;
  data_ = data;
  rowDim_ = rowDim;
  colDim_ = colDim;
}

Shift::SerialEntry::~SerialEntry() {
  switch(type_) {
    case SerialType::STRING: {
      std::string* data = static_cast<std::string*>(defaultData_);
      if(data) {
        delete data;
      }
      break;
    }
    case SerialType::STRING_ARRAY: {
      std::vector<std::string>* data =
        static_cast<std::vector<std::string>*>(defaultData_);
      if(data) {
        delete data;
      }
      break;
    }
    case SerialType::STRING_2DARRAY: {
      std::vector<std::vector<std::string>>* data =
        static_cast<std::vector<std::vector<std::string>>*>(defaultData_);
      if(data) {
        delete data;
      }
      break;
    }
    case SerialType::DOUBLE:
    case SerialType::DOUBLE_ARRAY: {
      double* data = static_cast<double*>(defaultData_);
      if(data) {
        free(data);
      }
      break;
    }
    case SerialType::KSL_VEC3: {
      ksl_vec3_t* data = static_cast<ksl_vec3_t*>(defaultData_);
      if(data) {
        free(data);
      }
      break;
    }
    case SerialType::KSL_VEC4: {
      ksl_vec4_t* data = static_cast<ksl_vec4_t*>(defaultData_);
      if(data) {
        free(data);
      }
      break;
    }
    case SerialType::KSL_QUATERNION: {
      ksl_quaternion_t* data = static_cast<ksl_quaternion_t*>(defaultData_);
      if(data) {
        free(data);
      }
      break;
    }
    case SerialType::KSL_SCREW: {
      ksl_screw_t* data = static_cast<ksl_screw_t*>(defaultData_);
      if(data) {
        free(data);
      }
      break;
    }
    case SerialType::KSL_COSCREW: {
      ksl_coscrew_t* data = static_cast<ksl_coscrew_t*>(defaultData_);
      if(data) {
        free(data);
      }
      break;
    }
    case SerialType::KSL_INERTIA: {
      ksl_inertia_t* data = static_cast<ksl_inertia_t*>(defaultData_);
      if(data) {
        free(data);
      }
      break;
    }
    case SerialType::KSL_MAT3X3: {
      ksl_mat3x3_t* data = static_cast<ksl_mat3x3_t*>(defaultData_);
      if(data) {
        free(data);
      }
      break;
    }
    case SerialType::KSL_MAT4X4: {
      ksl_mat4x4_t* data = static_cast<ksl_mat4x4_t*>(defaultData_);
      if(data) {
        free(data);
      }
      break;
    }
    case SerialType::BOOLEAN: {
      bool* data = static_cast<bool*>(defaultData_);
      if(data) {
        free(data);
      }
      break;
    }
    default: { break; }
  }
}

void Shift::SerialEntry::readJSON(Json::Value& object) {

  /* check if value exists */
  if(!object.isMember(key_)) {
    if(required_) {
      std::cout << key_ << std::endl;
      throw "JSON Parsing error: required key not found";
    } else {
      return;
    }
  }

  switch(type_) {
    case SerialType::STRING: {
      readJSON_STRING(object[key_]);
      break;
    }
    case SerialType::STRING_ARRAY: {
      readJSON_STRING_ARRAY(object[key_]);
      break;
    }
    case SerialType::STRING_2DARRAY: {
      readJSON_STRING_2DARRAY(object[key_]);
      break;
    }
    case SerialType::DOUBLE: {
      readJSON_DOUBLE(object[key_]);
      break;
    }
    case SerialType::DOUBLE_ARRAY: {
      readJSON_DOUBLE_ARRAY(object[key_]);
      break;
    }
    case SerialType::KSL_VEC3: {
      readJSON_KSL_VEC3(object[key_]);
      break;
    }
    case SerialType::KSL_VEC4: {
      readJSON_KSL_VEC4(object[key_]);
      break;
    }
    case SerialType::KSL_QUATERNION: {
      readJSON_KSL_QUATERNION(object[key_]);
      break;
    }
    case SerialType::KSL_SCREW: {
      readJSON_KSL_SCREW(object[key_]);
      break;
    }
    case SerialType::KSL_COSCREW: {
      readJSON_KSL_COSCREW(object[key_]);
      break;
    }
    case SerialType::KSL_INERTIA: {
      readJSON_KSL_INERTIA(object[key_]);
      break;
    }
    case SerialType::KSL_MAT3X3: {
      readJSON_KSL_MAT3X3(object[key_]);
      break;
    }
    case SerialType::KSL_MAT4X4: {
      readJSON_KSL_MAT4X4(object[key_]);
      break;
    }
    case SerialType::EULERANGLES: {
      readJSON_EULERANGLES(object[key_]);
      break;
    }
    case SerialType::BOOLEAN: {
      readJSON_BOOLEAN(object[key_]);
      break;
    }
  }
}

void Shift::SerialEntry::readJSON_STRING(Json::Value& value) {
  if(!value.isString()) {
    if(required_) {
      throw "JSON Parsing error: required key has incorrectly formatted data; "
            "expected STRING";
    }
    return;
  }
  std::string* data = (std::string*)data_;
  data->assign(value.asString());
  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_STRING_ARRAY(Json::Value& value) {
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

  std::vector<std::string>* data = (std::vector<std::string>*)data_;
  for(unsigned int i = 0; i < value.size(); i++) {
    data->push_back(value[i].asString());
  }
  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_STRING_2DARRAY(Json::Value& value) {
  if(!value.isArray()) {
    for(unsigned int i = 0; i < value.size(); i++) {
      if(rowDim_ > 0 && rowDim_ != (int)value.size()) {
        throw "JSON Parsing error: required key has incorrectly "
              "formatted data; incorrect row dimension found in "
              "STRING_2DARRAY";
      }
      if(!(value[i].isArray())) {
        throw "JSON Parsing error: required key has incorrectly formatted "
              "data; expected STRING_2DARRAY";
      }
      for(unsigned int j = 0; j < value[i].size(); j++) {
        if(colDim_ > 0 && colDim_ != (int)value[i].size()) {
          throw "JSON Parsing error: required key has incorrectly "
                "formatted data; incorrect column dimension found in "
                "STRING_2DARRAY";
        }
        if(!(value[i][j].isString())) {
          throw "JSON Parsing error: required key has incorrectly "
                "formatted data; expected STRING_2DARRAY";
        }
      }
    }
  }
  std::vector<std::vector<std::string>>* data =
    (std::vector<std::vector<std::string>>*)data_;
  for(unsigned int i = 0; i < value.size(); i++) {
    std::vector<std::string> row;
    for(unsigned int j = 0; j < value[i].size(); j++) {
      row.push_back(value[i][j].asString());
    }
    data->push_back(row);
  }
  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_DOUBLE(Json::Value& value) {
  if(!value.isDouble()) {
    throw "JSON Parsing error, required key has incorrectly formatted "
          "data; expected DOUBLE";
  }
  double* data = (double*)data_;
  *data = value.asDouble();
  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_DOUBLE_ARRAY(Json::Value& value) {
  if(!value.isArray()) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected DOUBLE_ARRAY";
  }
  if(rowDim_ > 0 && rowDim_ != (int)value.size()) {
    throw "JSON Parsing error: required key has incorrect number of elements";
  }
  if(value.size() == 0) {
    throw "JSON Parsing error: array has zero elements";
  }
  double* data = (double*)data_;
  if(rowDim_ == -1) {
    /* dynamic memory allocation requested, the module must free the
    data in its destructor */
    data = (double*)malloc(value.size() * sizeof(double));
  }

  for(unsigned int i = 0; i < value.size(); i++) {
    if(!value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected DOUBLE_ARRAY";
    }
    data[i] = value[i].asDouble();
  }
  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_KSL_VEC3(Json::Value& value) {
  if(!value.isArray() || !(value.size() == 3)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_VEC3";
  }
  ksl_vec3_t* data = (ksl_vec3_t*)data_;
  for(int i = 0; i < 3; i++) {
    if(!value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_VEC3";
    }
    data->at[i] = value[i].asDouble();
  }
  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_KSL_VEC4(Json::Value& value) {
  if(!value.isArray() || !(value.size() == 4)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_VEC4";
  }
  ksl_vec4_t* data = (ksl_vec4_t*)data_;
  for(int i = 0; i < 4; i++) {
    if(!value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_VEC4";
      return;
    }
    data->at[i] = value[i].asDouble();
  }
  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_KSL_QUATERNION(Json::Value& value) {
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

  if(normalizeQuaternion_) {
    ksl_quaternion_normalize(&quaternion);
  }

  if(quaternionMapsToMat3x3_) {
    ksl_mat3x3_t* data = (ksl_mat3x3_t*)data_;
    ksl_quaternion_toMat3x3(&quaternion, data);
  } else {
    ksl_quaternion_t* data = (ksl_quaternion_t*)data_;
    *data = quaternion;
  }

  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_KSL_SCREW(Json::Value& value) {
  if(!value.isArray() || !(value.size() == 6)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_SCREW";
    return;
  }
  ksl_screw_t* data = (ksl_screw_t*)data_;
  for(int i = 0; i < 6; i++) {
    if(!value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_SCREW";
      return;
    }
    data->at[i] = value[i].asDouble();
  }
  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_KSL_COSCREW(Json::Value& value) {
  if(!value.isArray() || !(value.size() == 6)) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_COSCREW";
    return;
  }
  ksl_coscrew_t* data = (ksl_coscrew_t*)data_;
  for(int i = 0; i < 6; i++) {
    if(!value[i].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_COSCREW";
      return;
    }
    data->at[i] = value[i].asDouble();
  }
  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_KSL_INERTIA(Json::Value& value) {
  if(!value.isObject()) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_INERTIA";
    return;
  }
  ksl_inertia_t* inertia = (ksl_inertia_t*)data_;

  if(value["mass"]) {
    if(!value["mass"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->mass = value["mass"].asDouble();
  }

  if(value["Ixx"]) {
    if(!value["Ixx"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Ixx = value["Ixx"].asDouble();
  }

  if(value["Iyy"]) {
    if(!value["Iyy"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Iyy = value["Iyy"].asDouble();
  }

  if(value["Izz"]) {
    if(!value["Izz"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Izz = value["Izz"].asDouble();
  }

  if(value["Ixy"]) {
    if(!value["Ixy"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Ixy = value["Ixy"].asDouble();
  }

  if(value["Iyz"]) {
    if(!value["Iyz"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Iyz = value["Iyz"].asDouble();
  }

  if(value["Izx"]) {
    if(!value["Izx"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Izx = value["Izx"].asDouble();
  }

  if(value["Iyx"]) {
    if(!value["Iyx"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Iyx = value["Iyx"].asDouble();
  }

  if(value["Izy"]) {
    if(!value["Izy"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Izy = value["Izy"].asDouble();
  }

  if(value["Ixz"]) {
    if(!value["Ixz"].isDouble()) {
      throw "JSON Parsing error: required key has incorrectly formatted "
            "data; expected KSL_INERTIA";
      return;
    }
    inertia->Ixz = value["Ixz"].asDouble();
  }

  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_KSL_MAT3X3(Json::Value& value) {
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

  ksl_mat3x3_t* data = (ksl_mat3x3_t*)data_;
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      if(!value[i][j].isDouble()) {
        throw "JSON Parsing error: required key has incorrectly formatted "
              "data; expected KSL_MAT3X3";
      }
      data->at[j][i] = value[i][j].asDouble();
    }
  }
  if(normalizeMat3x3_) {
    ksl_mat3x3_normalize(data);
  }
  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_KSL_MAT4X4(Json::Value& value) {
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

  if(mat4x4MapsToSE3_) {
    ksl_SE3_t* data = (ksl_SE3_t*)data_;
    ksl_mat4x4_toSE3(&mat4x4, data);
    if(normalizeMat3x3_) {
      ksl_mat3x3_normalize(&data->R);
    }
  } else {
    ksl_mat4x4_t* data = (ksl_mat4x4_t*)data_;
    *data = mat4x4;
  }

  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_EULERANGLES(Json::Value& value) {
  if(!value.isObject()) {
    throw "JSON Parsing error: required key has incorrectly formatted "
          "data; expected KSL_EULERANGLES";
    return;
  }

  ksl_axis_enum_t sequence = KSL_AXIS_XYZ;
  ksl_vec3_t angles;
  if(value["angles"] && value["angles"].size() == 3) {
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

  if(value["axes"]) {
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
            "formatted, expected three integer values {0, 1, 2} or three "
            "string values {x, y, z}";
      return;
    }
  }

  if(eulerAnglesMapToMat3x3_) {
    ksl_mat3x3_t* data = (ksl_mat3x3_t*)data_;
    ksl_mat3x3_setFromEulerAngles(data, sequence, &angles);
  } else {
    ksl_vec3_t* data = (ksl_vec3_t*)data_;
    *data = angles;
  }

  loadedFromFile_ = true;
}

void Shift::SerialEntry::readJSON_BOOLEAN(Json::Value& value) {

  std::cout << value.asString() << std::endl;
  if(!value.isBool()) {
    throw "JSON Parsing error, required key %s has incorrectly formatted "
          "data; expected BOOLEAN";
  }
  double* data = (double*)data_;
  *data = value.asBool();
  loadedFromFile_ = true;
}

void Shift::Serialize::addSerialEntry(Shift::SerialEntry& serialEntry) {
  serialEntries_.insert(serialEntries_.end(), serialEntry);
}

void Shift::Serialize::readJSON(Json::Value& value) {
  for(unsigned int i = 0; i < serialEntries_.size(); i++) {
    serialEntries_[i].readJSON(value);
  }
}

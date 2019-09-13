#ifndef SHIFT_PARAMETER_VALUE_H_
#define SHIFT_PARAMETER_VALUE_H_

#include "ksl/SE3.h"
#include "ksl/coscrew.h"
#include "ksl/eulerangles.h"
#include "ksl/inertia.h"
#include "ksl/mat4x4.h"
#include "ksl/quaternion.h"
#include "ksl/screw.h"
#include "ksl/vec3.h"
#include "ksl/vec4.h"

#include <any>
#include <assert.h>
#include <cxxabi.h>
#include <exception>
#include <iostream>
#include <json/json.h>
#include <memory>
#include <string>
#include <typeindex>
#include <vector>

namespace Shift {

class Context;
class Assembly;
namespace Body {
class Base;
}

namespace Parameter {
class ArrayBase;
class List;

class Value {
public:
  Value() {}

  Value(std::string* data, const std::string description, bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(STRING) {}

  Value(std::vector<std::string>* data, const std::string description,
        bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(STRING_ARRAY) {}

  Value(std::vector<std::vector<std::string>>* data,
        const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(STRING_2DARRAY) {}

  Value(double* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(DOUBLE) {}

  Value(std::vector<double>* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(DOUBLE_ARRAY) {}

  Value(ksl_vec3_t* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_VEC3) {}

  Value(ksl_vec4_t* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_VEC4) {}

  Value(ksl_quaternion_t* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_QUATERNION) {}

  Value(ksl_screw_t* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_SCREW) {}

  Value(ksl_coscrew_t* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_COSCREW) {}

  Value(ksl_inertia_t* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_INERTIA) {}

  Value(ksl_mat3x3_t* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_MAT3X3) {}

  Value(ksl_mat4x4_t* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_MAT4X4) {}

  Value(ksl_eulerangles_t* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_EULERANGLES) {}

  Value(bool* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(BOOLEAN) {}

  Value(ksl_SE3_t* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_SE3)
    , quaternion_maps_to_mat3x3_(true)
    , mat3x3_maps_to_SE3_(true)
    , vec3_maps_to_mat3x3_(true)
    , mat4x4_maps_to_SE3_(true)
    , euler_angles_maps_to_mat3x3_(true)
    , normalize_quaternion_(true)
    , normalize_mat3x3_(true) {}

  Value(List* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(PARAMETER_LIST)
    , quaternion_maps_to_mat3x3_(false)
    , mat3x3_maps_to_SE3_(false)
    , vec3_maps_to_mat3x3_(false)
    , mat4x4_maps_to_SE3_(false)
    , euler_angles_maps_to_mat3x3_(false)
    , normalize_quaternion_(false)
    , normalize_mat3x3_(false)
    , is_list_(true) {}

  Value(ArrayBase* data, const std::string description, bool required)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(PARAMETER_ARRAY)
    , quaternion_maps_to_mat3x3_(false)
    , mat3x3_maps_to_SE3_(false)
    , vec3_maps_to_mat3x3_(false)
    , mat4x4_maps_to_SE3_(false)
    , euler_angles_maps_to_mat3x3_(false)
    , normalize_quaternion_(false)
    , normalize_mat3x3_(false)
    , is_array_(true) {}

  ~Value() {}

  enum ValueType {
    STRING,
    STRING_ARRAY,
    STRING_2DARRAY,
    DOUBLE,
    DOUBLE_ARRAY,
    KSL_VEC3,
    KSL_VEC4,
    KSL_QUATERNION,
    KSL_SCREW,
    KSL_COSCREW,
    KSL_INERTIA,
    KSL_MAT3X3,
    KSL_MAT4X4,
    KSL_EULERANGLES,
    KSL_SE3,
    BOOLEAN,
    PARAMETER_LIST,
    PARAMETER_ARRAY
  };

  Value& operator[](int index) {
    return operator[]((unsigned int)(index));
  }

  Value& operator[](unsigned int index);

  virtual Value& operator[](const std::string key_str);

  virtual Value& operator[](const char* key_str);

  void setRequired(bool required) {
    required_ = required;
  }

  bool required() {
    return required_;
  }

  const std::string& description() {
    return description_;
  }

  template <typename T>
  T* data() const {
    return std::any_cast<T*>(data_);
  }

  template <typename T>
  const T& get() const {
    return *std::any_cast<T*>(data_);
  }

  const std::type_info& typeInfo() {
    return data_.type();
  }

  const std::type_index typeIndex() {
    return std::type_index(data_.type());
  }

  const std::string typeName() {
    int status;
    std::string type_str = data_.type().name();
    char* demangled_name =
      abi::__cxa_demangle(type_str.c_str(), nullptr, nullptr, &status);
    if(status == 0) {
      type_str = demangled_name;
      std::free(demangled_name);
      if(type_str.substr(type_str.length() - 1, type_str.length()) ==
         std::string("*")) {
        type_str = type_str.substr(0, type_str.length() - 1);
      }
    }
    return type_str;
  }

  void setActiveObjects(bool parent_is_active);

  Value& setQuaternionMapsToMat3x3(bool enabled) {
    quaternion_maps_to_mat3x3_ = enabled;
    return *this;
  }

  Value& setMat3x3MapsToSE3(bool enabled) {
    mat3x3_maps_to_SE3_ = enabled;
    return *this;
  }

  Value& setMat4x4MapsToSE3(bool enabled) {
    mat4x4_maps_to_SE3_ = enabled;
    return *this;
  }

  Value& setVec3MapsToMat3x3(bool enabled) {
    vec3_maps_to_mat3x3_ = enabled;
    return *this;
  }

  Value& setEulerAnglesMapToMat3x3(bool enabled) {
    euler_angles_maps_to_mat3x3_ = enabled;
    return *this;
  }

  Value& setNormalizeQuaternion(bool enabled) {
    normalize_quaternion_ = enabled;
    return *this;
  }

  Value& setNormalizeMat3x3(bool enabled) {
    normalize_mat3x3_ = enabled;
    return *this;
  }

  Value& addValidStringValue(std::string value) {
    valid_string_values_.insert(valid_string_values_.end(), value);
    return *this;
  }

  Value& setCreateDefaultInstanceIfNotSpecified(bool enabled) {
    create_default_instance_if_not_specified_ = enabled;
    return *this;
  }

  Value& setRowDim(int row_dim) {
    row_dim_ = row_dim;
    return *this;
  }

  Value& setColumnDim(int col_dim) {
    col_dim_ = col_dim;
    return *this;
  }

  void readJSON(Json::Value&);
  void print(std::string) const;
  void print() const;

  void setParent(Context* context, Assembly* parent_assembly = nullptr,
                 Body::Base* parent_body = nullptr);

private:
  std::any data_; //!< contains pointer to value
  std::string description_;
  bool required_;
  ValueType parameter_type_;

  int row_dim_ = 1; /*!< -1 indicates dynamic sizing requested */
  int col_dim_ = 1; /*!< -1 indicates dynamic sizing requested */

  std::vector<std::string> valid_string_values_;
  bool quaternion_maps_to_mat3x3_ = false;
  bool mat3x3_maps_to_SE3_ = false;
  bool vec3_maps_to_mat3x3_ = false;
  bool mat4x4_maps_to_SE3_ = false;
  bool euler_angles_maps_to_mat3x3_ = false;
  bool normalize_quaternion_ = true;
  bool normalize_mat3x3_ = true;

  bool loaded_from_file_ = false;

  ksl_vec3i_t axes_ = {{0, 1, 2}};
  bool is_list_ = false;
  bool is_array_ = false;

  /*! create default instance of object with default configuration if no key :
   * value is provided  */
  bool create_default_instance_if_not_specified_ = false;

  void readJSON_STRING(Json::Value&);
  void readJSON_STRING_ARRAY(Json::Value&);
  void readJSON_STRING_2DARRAY(Json::Value&);
  void readJSON_DOUBLE(Json::Value&);
  void readJSON_DOUBLE_ARRAY(Json::Value&);
  void readJSON_KSL_VEC3(Json::Value&);
  void readJSON_KSL_VEC4(Json::Value&);
  void readJSON_KSL_QUATERNION(Json::Value&);
  void readJSON_KSL_SCREW(Json::Value&);
  void readJSON_KSL_COSCREW(Json::Value&);
  void readJSON_KSL_INERTIA(Json::Value&);
  void readJSON_KSL_MAT3X3(Json::Value&);
  void readJSON_KSL_MAT4X4(Json::Value&);
  void readJSON_EULERANGLES(Json::Value&);
  void readJSON_BOOLEAN(Json::Value&);
  void readJSON_POSE(Json::Value&);
};

} // namespace Parameter
} // namespace Shift

#endif

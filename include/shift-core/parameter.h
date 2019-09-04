#ifndef SHIFT_PARAMETER_H_
#define SHIFT_PARAMETER_H_

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

class List;
class ArrayBase;

enum class Type {
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

class KeyValue {
public:
  KeyValue() {}

  KeyValue(const std::string key, std::string* data,
           const std::string description, bool required = true)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::STRING) {}

  KeyValue(const std::string key, std::vector<std::string>* data,
           const std::string description, bool required = true)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::STRING_ARRAY) {}

  KeyValue(const std::string key, std::vector<std::vector<std::string>>* data,
           const std::string description, bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::STRING_2DARRAY) {}

  KeyValue(const std::string key, double* data, const std::string description,
           bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::DOUBLE) {}

  KeyValue(const std::string key, std::vector<double>* data,
           const std::string description, bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::DOUBLE_ARRAY) {}

  KeyValue(const std::string key, ksl_vec3_t* data,
           const std::string description, bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Parameter::Type::KSL_VEC3) {}

  KeyValue(const std::string key, ksl_vec4_t* data,
           const std::string description, bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::KSL_VEC4) {}

  KeyValue(const std::string key, ksl_quaternion_t* data,
           const std::string description, bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::KSL_QUATERNION) {}

  KeyValue(const std::string key, ksl_screw_t* data,
           const std::string description, bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::KSL_SCREW) {}

  KeyValue(const std::string key, ksl_coscrew_t* data,
           const std::string description, bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::KSL_COSCREW) {}

  KeyValue(const std::string key, ksl_inertia_t* data,
           const std::string description, bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::KSL_INERTIA) {}

  KeyValue(const std::string key, ksl_mat3x3_t* data,
           const std::string description, bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Parameter::Type::KSL_MAT3X3) {}

  KeyValue(const std::string key, ksl_mat4x4_t* data,
           const std::string description, bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::KSL_MAT4X4) {}

  KeyValue(const std::string key, ksl_eulerangles_t* data,
           const std::string description, bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::KSL_EULERANGLES) {}

  KeyValue(const std::string key, bool* data, const std::string description,
           bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::BOOLEAN) {}

  KeyValue(const std::string key, ksl_SE3_t* data,
           const std::string description, bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::KSL_SE3)
    , quaternion_maps_to_mat3x3_(true)
    , mat3x3_maps_to_SE3_(true)
    , vec3_maps_to_mat3x3_(true)
    , mat4x4_maps_to_SE3_(true)
    , euler_angles_maps_to_mat3x3_(true)
    , normalize_quaternion_(true)
    , normalize_mat3x3_(true) {}

  KeyValue(const std::string key, List* data, const std::string description,
           bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::PARAMETER_LIST)
    , quaternion_maps_to_mat3x3_(false)
    , mat3x3_maps_to_SE3_(false)
    , vec3_maps_to_mat3x3_(false)
    , mat4x4_maps_to_SE3_(false)
    , euler_angles_maps_to_mat3x3_(false)
    , normalize_quaternion_(false)
    , normalize_mat3x3_(false)
    , is_list_(true) {}

  KeyValue(const std::string key, ArrayBase* data,
           const std::string description, bool required)
    : key_(key)
    , data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(Shift::Parameter::Type::PARAMETER_ARRAY)
    , quaternion_maps_to_mat3x3_(false)
    , mat3x3_maps_to_SE3_(false)
    , vec3_maps_to_mat3x3_(false)
    , mat4x4_maps_to_SE3_(false)
    , euler_angles_maps_to_mat3x3_(false)
    , normalize_quaternion_(false)
    , normalize_mat3x3_(false)
    , is_array_(true) {}

  ~KeyValue() {}

  KeyValue& operator[](int index) {
    return operator[]((unsigned int)(index));
  }

  KeyValue& operator[](unsigned int index);

  virtual KeyValue& operator[](const std::string key_str);

  virtual KeyValue& operator[](const char* key_str);

  const std::string& key() {
    return key_;
  }

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
  const T data() const {
    return std::any_cast<T>(data_);
  }

  template <typename T>
  const T& value() const {
    return *std::any_cast<T*>(data_);
  }

  const std::type_info& type_info() {
    return data_.type();
  }

  const std::string type_name() {
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

  KeyValue& setQuaternionMapsToMat3x3(bool enabled) {
    quaternion_maps_to_mat3x3_ = enabled;
    return *this;
  }

  KeyValue& setMat3x3MapsToSE3(bool enabled) {
    mat3x3_maps_to_SE3_ = enabled;
    return *this;
  }

  KeyValue& setMat4x4MapsToSE3(bool enabled) {
    mat4x4_maps_to_SE3_ = enabled;
    return *this;
  }

  KeyValue& setVec3MapsToMat3x3(bool enabled) {
    vec3_maps_to_mat3x3_ = enabled;
    return *this;
  }

  KeyValue& setEulerAnglesMapToMat3x3(bool enabled) {
    euler_angles_maps_to_mat3x3_ = enabled;
    return *this;
  }

  KeyValue& setNormalizeQuaternion(bool enabled) {
    normalize_quaternion_ = enabled;
    return *this;
  }

  KeyValue& setNormalizeMat3x3(bool enabled) {
    normalize_mat3x3_ = enabled;
    return *this;
  }

  KeyValue& addValidStringValue(std::string value) {
    valid_string_values_.insert(valid_string_values_.end(), value);
    return *this;
  }

  KeyValue& setCreateDefaultInstanceIfNotSpecified(bool enabled) {
    create_default_instance_if_not_specified_ = enabled;
    return *this;
  }

  KeyValue& setRowDim(int row_dim) {
    row_dim_ = row_dim;
    return *this;
  }

  KeyValue& setColumnDim(int col_dim) {
    col_dim_ = col_dim;
    return *this;
  }

  void readJSON(Json::Value&);
  void print(std::string) const;
  void print() const;

private:
  std::string key_;
  std::any data_; //!< contains pointer to value
  std::string description_;
  bool required_;
  Parameter::Type parameter_type_;

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

/*!
@brief Shift Parameter::List class
*/
class List {

public:
  List() {}
  List(Context* context, Assembly* assembly = nullptr,
       Body::Base* body = nullptr)
    : context_(context)
    , parent_assembly_(assembly)
    , parent_body_(body) {}
  virtual ~List() {
    std::cout << "virtual ~List(): " << std::endl;
  }
  bool active() {
    return active_;
  }
  virtual void addParameters() {}
  void removeParameter(std::string key);

  virtual void setActiveObjects(bool) {}

  KeyValue& addParameter(KeyValue parameter);
  KeyValue& addParameter(const std::string key, std::string* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, std::vector<std::string>* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }
  KeyValue& addParameter(const std::string key,
                         std::vector<std::vector<std::string>>* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, double* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, std::vector<double>* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, ksl_vec3_t* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, ksl_vec4_t* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, ksl_quaternion_t* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, ksl_screw_t* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, ksl_coscrew_t* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, ksl_inertia_t* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, ksl_mat3x3_t* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, ksl_mat4x4_t* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, ksl_eulerangles_t* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, bool* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, ksl_SE3_t* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, List* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  KeyValue& addParameter(const std::string key, ArrayBase* data,
                         const std::string description, bool required = true) {
    KeyValue key_value(key, data, description, required);
    return addParameter(key_value);
  }

  void setContext(Context* context) {
    context_ = context;
  }

  void setParentAssembly(Assembly* assembly) {
    parent_assembly_ = assembly;
  }

  void setParentBody(Body::Base* body) {
    parent_body_ = body;
  }

  virtual void readJSON(Json::Value&);

  virtual KeyValue& operator[](const char* key) {
    return parameters_[key];
  }

  virtual KeyValue& operator[](const std::string key) {
    return parameters_[key];
  }

  bool has_key(const std::string key) const {
    return parameters_.count(key);
  }

  auto& parameters() const {
    return parameters_;
  }

  void print() const;
  void print(std::string) const;

  Assembly* parentAssembly() {
    return parent_assembly_;
  }

  Body::Base* parentBody() {
    return parent_body_;
  }

protected:
  std::map<std::string, Parameter::KeyValue> parameters_;
  Context* context_ = nullptr;
  Assembly* parent_assembly_ = nullptr;
  Body::Base* parent_body_ = nullptr;
  bool active_ = true;
};

class ArrayBase {
public:
  ArrayBase() {}
  virtual ~ArrayBase() {
    std::cout << "virtual ~ArrayBase()" << std::endl;
  }

  bool active() {
    return active_;
  }

  virtual void setActiveObjects(bool) {}

  virtual KeyValue& operator[](const std::string key_str) {
    for(auto& object : objects_) {
      if(object->operator[]("name").value<std::string>() == key_str) {
        void_ = KeyValue("key_str", object, "description", true);
        return void_;
      }
    }
    throw "JSON Parsing error, parameter not found";
    return void_;
  }

  virtual KeyValue& operator[](const char* key_str) {
    return operator[](std::string(key_str));
  }

  virtual KeyValue& operator[](unsigned int index) {
    try {
      assert(index < objects_.size());
      auto& object = objects_[index];
      void_ = KeyValue("key_str", object, "description", true);
    } catch(const char* msg) {
      std::cout << msg << std::endl;
    }
    return void_;
  }

  size_t size() {
    return objects_.size();
  }

  virtual void readJSON(Json::Value&) {}

  void print() const;
  void print(std::string) const;

private:
  KeyValue void_;

protected:
  std::vector<Shift::Parameter::List*> objects_;
  bool active_ = true;
};

template <class T>
class Array : public Shift::Parameter::ArrayBase {
public:
  Array() {}
  Array(Context* context, Assembly* parent_assembly = nullptr,
        Shift::Body::Base* parent_body = nullptr)
    : context_(context)
    , parent_assembly_(parent_assembly)
    , parent_body_(parent_body) {}
  ~Array() {}

  virtual void readJSON(Json::Value& value) override;
  virtual void setActiveObjects(bool parent_is_active) override {
    for(auto& object : object_specializations_) {
      object.get()->setActiveObjects(active_ && parent_is_active);
    }
  }

  T* get(unsigned int i) {
    if(i < objects_.size()) {
      return object_specializations_[i].get();
    } else {
      return nullptr;
    }
  }
  std::vector<std::unique_ptr<T>>& data() {
    return object_specializations_;
  }
  void setContext(Context* context) {
    context_ = context;
  }
  void setParentAssembly(Assembly* assembly) {
    parent_assembly_ = assembly;
  }
  void setParentBody(Body::Base* body) {
    parent_body_ = body;
  }
  void push_back(std::unique_ptr<T>);

private:
  Context* context_ = nullptr;
  Assembly* parent_assembly_ = nullptr;
  Shift::Body::Base* parent_body_ = nullptr;
  std::vector<std::unique_ptr<T>> object_specializations_;
};

template <class T>
class FactoryArray : public Shift::Parameter::ArrayBase {
public:
  FactoryArray() {}
  FactoryArray(Context* context, Assembly* parent_assembly = nullptr,
               Shift::Body::Base* parent_body = nullptr)
    : context_(context)
    , parent_assembly_(parent_assembly)
    , parent_body_(parent_body) {}
  ~FactoryArray() {}

  virtual void readJSON(Json::Value& value) override;
  virtual void setActiveObjects(bool parent_is_active) override {
    for(auto& object : object_specializations_) {
      object.get()->setActiveObjects(active_ && parent_is_active);
    }
  }

  // T* get(unsigned int i) {
  //   if(i < objects_.size()) {
  //     return object_specializations_[i].get();
  //   } else {
  //     return nullptr;
  //   }
  // }

  std::vector<std::unique_ptr<T>>& data() {
    return object_specializations_;
  }

  void setContext(Context* context) {
    context_ = context;
  }
  void setParentAssembly(Assembly* assembly) {
    parent_assembly_ = assembly;
  }
  void setParentBody(Body::Base* body) {
    parent_body_ = body;
  }
  void push_back(std::unique_ptr<T>);

private:
  Context* context_ = nullptr;
  Assembly* parent_assembly_ = nullptr;
  Shift::Body::Base* parent_body_ = nullptr;
  std::vector<std::unique_ptr<T>> object_specializations_;
};

} // namespace Parameter
} // namespace Shift

#endif

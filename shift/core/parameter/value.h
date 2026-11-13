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

#include <assert.h>
#include <cxxabi.h>

#include <any>
#include <exception>
#include <iostream>
#include <json/json.h>
#include <memory>
#include <string>
#include <typeindex>
#include <vector>

namespace shift {
namespace core {

class Context;
class Assembly;
class Frame;
namespace controller {
class Base;
}
namespace body {
class Base;
}
namespace collision_geometry {
class Base;
}
namespace constraint {
class Base;
}
namespace force_module {
class Base;
}
namespace geometry {
class Base;
}
namespace integrator {
class Base;
}
namespace joint {
class Base;
}
namespace light {
class Base;
}
namespace logger {
class Base;
}
namespace sensor {
class Base;
}
namespace solver {
class Base;
}
namespace terrain {
class Base;
}

namespace parameter {
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
        const std::string description, bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(STRING_2DARRAY) {}

  Value(int* data, const std::string description, bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(INT) {}

  Value(std::vector<int>* data, const std::string description,
        bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(INT_ARRAY) {}

  Value(double* data, const std::string description, bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(DOUBLE) {}

  Value(std::vector<double>* data, const std::string description,
        bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(DOUBLE_ARRAY) {}

  Value(ksl_vec3_t* data, const std::string description, bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_VEC3) {}

  Value(ksl_vec4_t* data, const std::string description, bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_VEC4) {}

  Value(ksl_quaternion_t* data, const std::string description,
        bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_QUATERNION) {}

  Value(ksl_screw_t* data, const std::string description, bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_SCREW) {}

  Value(ksl_coscrew_t* data, const std::string description,
        bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_COSCREW) {}

  Value(ksl_inertia_t* data, const std::string description,
        bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_INERTIA) {}

  Value(ksl_mat3x3_t* data, const std::string description, bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_MAT3X3)
    , normalize_mat3x3_(true) {}

  Value(ksl_mat4x4_t* data, const std::string description, bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_MAT4X4) {}

  Value(ksl_eulerangles_t* data, const std::string description,
        bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(KSL_EULERANGLES) {}

  Value(bool* data, const std::string description, bool required = true)
    : data_(data)
    , description_(description)
    , required_(required)
    , parameter_type_(BOOLEAN) {}

  Value(ksl_SE3_t* data, const std::string description, bool required = true)
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

  Value(List* data, const std::string description, bool required = true)
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

  Value(ArrayBase* data, const std::string description, bool required = true)
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
    INT,
    INT_ARRAY,
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

  enum LogLevel { NONE = 0, ATTRIBUTE = 1, INTERVAL = 3, ALWAYS = 4 };

  /*! the role of a value in its component (see List::DeclareInput and
   * List::DeclareOutput):
   * - PARAMETER: read from the model file;
   * - INPUT: a number that the component reads each time it is evaluated,
   *   and that another component (a controller) sets as the model runs; the
   *   model file may give its value until something sets it;
   * - OUTPUT: a number that the component computes, for other components to
   *   read; not read from the model file. */
  enum Role { PARAMETER, INPUT, OUTPUT };

  Value& operator[](int index) {
    return operator[]((unsigned int)(index));
  }

  Value& operator[](unsigned int index);

  virtual Value& operator[](const std::string key_str);

  virtual Value& operator[](const char* key_str);

  bool is_required() {
    return required_;
  }
  ValueType get_value_type() const {
    return parameter_type_;
  }
  /*! \brief whether the value is an object with parameters of its own (a
   * list), or a collection of named objects (an array) */
  bool is_list() const {
    return is_list_;
  }
  bool is_array() const {
    return is_array_;
  }

  const std::string& get_description() const {
    return description_;
  }

  template <typename T>
  T* get_data() const {
    return std::any_cast<T*>(data_);
  }

  template <typename T>
  const T& get() const {
    return *std::any_cast<T*>(data_);
  }

  const std::type_info& get_type_info() {
    return data_.type();
  }

  inline const std::type_index get_type_index() {
    return std::type_index(data_.type());
  }

  const std::string get_type_name() {
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

  inline const std::string get_key() {
    return key_;
  }

  const std::vector<std::string> get_fully_qualified_name() {
    // std::list<std::string>(key_);
    std::vector<std::string> val;
    return val;
    // while(
  }

  inline void set_required(bool required = true) {
    required_ = required;
  }

  Value& set_log_level(LogLevel log_level) {
    log_level_ = log_level;
    return *this;
  }

  Value& set_load_from_json(bool load) {
    load_from_json_ = load;
    return *this;
  }
  /*! \brief mark the parameter as one that may be changed while the model
   * runs (by a schedule controller, a script or the viewer): the module reads
   * it each time it evaluates its forces, and does not compute anything from
   * it only when it loads. A parameter is not changeable at run time unless
   * its module says so. */
  Value& set_runtime(bool runtime = true) {
    runtime_ = runtime;
    return *this;
  }
  bool is_runtime() const {
    return runtime_;
  }
  Value& set_role(Role role) {
    role_ = role;
    return *this;
  }
  Role get_role() const {
    return role_;
  }
  bool is_input() const {
    return role_ == INPUT;
  }
  bool is_output() const {
    return role_ == OUTPUT;
  }
  /*! \brief the fully qualified name of the component that sets this input,
   * empty if none does */
  const std::string& get_driver() const {
    return driver_;
  }
  /*! \brief record driver (a fully qualified name) as the component that sets
   * this input; throws if the value is not an input, or if another component
   * already sets it. Connecting the same driver again does nothing. */
  void ConnectDriver(const std::string& driver);
  /*! \brief whether the value was read from the model file, rather than
   * left at its default */
  bool is_loaded_from_file() const {
    return loaded_from_file_;
  }
  /*! \brief whether the parameter points at data; a parameter added by
   * List::operator[] without being declared does not */
  bool has_data() const {
    return data_.has_value();
  }
  /*! \brief the address of the data; parameters that share it are aliases,
   * other keys for the same value (as "R" and "rotation" of a frame) */
  const void* get_address() const;
  /*! \brief whether the parameter is read in another form than its data's
   * (as a quaternion into a rotation matrix): an alias of another key */
  bool maps_to_other_type() const {
    return parameter_type_ != KSL_SE3 &&
           (quaternion_maps_to_mat3x3_ || mat3x3_maps_to_SE3_ ||
            vec3_maps_to_mat3x3_ || mat4x4_maps_to_SE3_ ||
            euler_angles_maps_to_mat3x3_);
  }
  /*! \brief the values a string parameter may take; empty if any */
  const std::vector<std::string>& get_valid_string_values() const {
    return valid_string_values_;
  }

  Value& set_quaternion_maps_to_mat3x3(bool enabled) {
    quaternion_maps_to_mat3x3_ = enabled;
    return *this;
  }

  Value& set_mat3x3_maps_to_SE3(bool enabled) {
    mat3x3_maps_to_SE3_ = enabled;
    return *this;
  }

  Value& set_mat4x4_maps_to_SE3(bool enabled) {
    mat4x4_maps_to_SE3_ = enabled;
    return *this;
  }

  Value& set_vec3_maps_to_mat3x3(bool enabled) {
    vec3_maps_to_mat3x3_ = enabled;
    return *this;
  }

  Value& set_euler_angles_map_to_mat3x3(bool enabled) {
    euler_angles_maps_to_mat3x3_ = enabled;
    return *this;
  }

  Value& set_normalize_quaternion(bool enabled) {
    normalize_quaternion_ = enabled;
    return *this;
  }

  Value& set_normalize_mat3x3(bool enabled) {
    normalize_mat3x3_ = enabled;
    return *this;
  }

  Value& set_create_default_instance_if_not_specified(bool enabled) {
    create_default_instance_if_not_specified_ = enabled;
    return *this;
  }

  Value& set_row_dim(int row_dim) {
    row_dim_ = row_dim;
    return *this;
  }

  Value& set_column_dim(int col_dim) {
    col_dim_ = col_dim;
    return *this;
  }

  Value& set_key(const std::string key) {
    key_ = key;
    return *this;
  }

  void set_parent(shift::core::Context* context,
                  shift::core::Assembly* parent_assembly = nullptr,
                  shift::core::body::Base* parent_body = nullptr);

  void set_parent(shift::core::parameter::List* parent) {
    parent_ = parent;
  }

  Value& AddValidStringValue(std::string value) {
    valid_string_values_.insert(valid_string_values_.end(), value);
    return *this;
  }

  void ActivateObjects(bool parent_is_active);
  void ReadJSON(Json::Value&);
  void Print(std::string) const;
  void Print() const;

private:
  shift::core::Context* context_ = nullptr;
  std::string key_;
  shift::core::parameter::List* parent_ = nullptr;
  LogLevel log_level_ = LogLevel::NONE; //!< indicates whether this
                                        // Value should be logged
                                        // during simulation
  bool load_from_json_ = true;          //!< load from json input file
  bool runtime_ = false;                //!< may be changed as the model runs
  Role role_ = PARAMETER;               //!< parameter, input or output
  std::string driver_; //!< the component that sets this input, if any
  std::any data_;                       //!< contains pointer to value
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

  void ReadJSON_STRING(Json::Value&);
  void ReadJSON_STRING_ARRAY(Json::Value&);
  void ReadJSON_STRING_2DARRAY(Json::Value&);
  void ReadJSON_INT(Json::Value&);
  void ReadJSON_INT_ARRAY(Json::Value&);
  void ReadJSON_DOUBLE(Json::Value&);
  void ReadJSON_DOUBLE_ARRAY(Json::Value&);
  void ReadJSON_KSL_VEC3(Json::Value&);
  void ReadJSON_KSL_VEC4(Json::Value&);
  void ReadJSON_KSL_QUATERNION(Json::Value&);
  void ReadJSON_KSL_SCREW(Json::Value&);
  void ReadJSON_KSL_COSCREW(Json::Value&);
  void ReadJSON_KSL_INERTIA(Json::Value&);
  void ReadJSON_KSL_MAT3X3(Json::Value&);
  void ReadJSON_KSL_MAT4X4(Json::Value&);
  void ReadJSON_EULERANGLES(Json::Value&);
  void ReadJSON_BOOLEAN(Json::Value&);
  void ReadJSON_POSE(Json::Value&);
};

// Lists are stored as List*; these specializations (defined in value.cc)
// dynamic_cast to the requested type. They must be declared here, before any
// use: otherwise callers instantiate the generic any_cast versions above,
// which throw std::bad_any_cast once inlined (-O1 and up).
template <>
const Frame& Value::get<Frame>() const;
template <>
const Assembly& Value::get<Assembly>() const;
template <>
const controller::Base& Value::get<controller::Base>() const;
template <>
const body::Base& Value::get<body::Base>() const;
template <>
const collision_geometry::Base& Value::get<collision_geometry::Base>() const;
template <>
const constraint::Base& Value::get<constraint::Base>() const;
template <>
const force_module::Base& Value::get<force_module::Base>() const;
template <>
const geometry::Base& Value::get<geometry::Base>() const;
template <>
const integrator::Base& Value::get<integrator::Base>() const;
template <>
const joint::Base& Value::get<joint::Base>() const;
template <>
const light::Base& Value::get<light::Base>() const;
template <>
const logger::Base& Value::get<logger::Base>() const;
template <>
const sensor::Base& Value::get<sensor::Base>() const;
template <>
const solver::Base& Value::get<solver::Base>() const;
template <>
const terrain::Base& Value::get<terrain::Base>() const;
template <>
Frame* Value::get_data<Frame>() const;
template <>
Assembly* Value::get_data<Assembly>() const;
template <>
controller::Base* Value::get_data<controller::Base>() const;
template <>
body::Base* Value::get_data<body::Base>() const;
template <>
collision_geometry::Base* Value::get_data<collision_geometry::Base>() const;
template <>
constraint::Base* Value::get_data<constraint::Base>() const;
template <>
force_module::Base* Value::get_data<force_module::Base>() const;
template <>
geometry::Base* Value::get_data<geometry::Base>() const;
template <>
integrator::Base* Value::get_data<integrator::Base>() const;
template <>
joint::Base* Value::get_data<joint::Base>() const;
template <>
light::Base* Value::get_data<light::Base>() const;
template <>
logger::Base* Value::get_data<logger::Base>() const;
template <>
sensor::Base* Value::get_data<sensor::Base>() const;
template <>
solver::Base* Value::get_data<solver::Base>() const;
template <>
terrain::Base* Value::get_data<terrain::Base>() const;

} // namespace parameter
} // namespace core
} // namespace shift

#endif

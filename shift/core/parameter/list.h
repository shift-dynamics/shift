#ifndef SHIFT_PARAMETER_LIST_H_
#define SHIFT_PARAMETER_LIST_H_

#include "shift/core/parameter/value.h"

#include <map>
#include <string>

namespace shift {
namespace core {

class Context;
class Assembly;
namespace body {
class Base;
}

namespace parameter {

/*!
@brief Shift parameter::List class
*/
class List {

public:
  List() {}
  List(Context* context, Assembly* assembly = nullptr,
       shift::core::body::Base* body = nullptr)
    : context_(context)
    , parent_assembly_ptr_(assembly)
    , parent_body_ptr_(body) {}

  virtual ~List() {}

  virtual Value& operator[](const char* key) {
    return parameters_[key];
  }

  virtual Value& operator[](const std::string key) {
    return parameters_[key];
  }

  bool is_active() {
    return active_;
  }

  Assembly* get_parent_assembly() {
    return parent_assembly_ptr_;
  }

  shift::core::body::Base* get_parent_body() {
    return parent_body_ptr_;
  }

  void set_parent(Context* context, Assembly* parent_assembly = nullptr,
                  shift::core::body::Base* parent_body = nullptr);

  auto& get_parameters() const {
    return parameters_;
  }

  void set_context(Context* context) {
    context_ = context;
  }

  void set_parent_assembly(Assembly* assembly) {
    parent_assembly_ptr_ = assembly;
  }

  void set_parent_body(shift::core::body::Base* body) {
    parent_body_ptr_ = body;
  }

  void set_is_body(bool is_body) {
    is_body_ = is_body;
  }

  void set_is_assembly(bool is_assembly) {
    is_assembly_ = is_assembly;
  }

  bool has_key(const std::string key) const {
    return parameters_.count(key);
  }

  virtual void ActivateObjects(bool parent_is_active) {
    for(auto& parameter : parameters_) {
      parameter.second.ActivateObjects(active_ && parent_is_active);
    }
  }

  virtual void DeclareParameters() = 0;

  void RemoveParameter(std::string key);

  Value& DeclareParameter(const std::string key, Value parameter);

  Value& DeclareParameter(const std::string key, std::string* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);
    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, std::vector<std::string>* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key,
                          std::vector<std::vector<std::string>>* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, int* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, std::vector<int>* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, double* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, std::vector<double>* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_vec3_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_vec4_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_quaternion_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_screw_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_coscrew_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_inertia_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_mat3x3_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_mat4x4_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_eulerangles_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, bool* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);
    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_SE3_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);
    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, List* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);
    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ArrayBase* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);
    return DeclareParameter(key, value);
  }

  virtual void ReadJSON(Json::Value&);

  void Print() const;
  void Print(std::string, bool break_line = true) const;
  std::vector<std::string> DescribeHeaders() {
    std::vector<std::string> headers;
    return headers;
  }

protected:
  std::map<std::string, parameter::Value> parameters_;
  Context* context_ = nullptr;
  Assembly* parent_assembly_ptr_ = nullptr;
  shift::core::body::Base* parent_body_ptr_ = nullptr;
  bool active_ = true;
  bool is_body_ = false;
  bool is_assembly_ = false;
};

} // namespace parameter
} // namespace core
} // namespace shift
#endif

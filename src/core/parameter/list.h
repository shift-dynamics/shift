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
  bool active() {
    return active_;
  }
  virtual void addParameters() = 0;
  void removeParameter(std::string key);

  virtual void setActiveObjects(bool parent_is_active) {
    for(auto& parameter : parameters_) {
      parameter.second.setActiveObjects(active_ && parent_is_active);
    }
  }

  Value& addParameter(const std::string key, Value parameter);
  Value& addParameter(const std::string key, std::string* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, std::vector<std::string>* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }
  Value& addParameter(const std::string key,
                      std::vector<std::vector<std::string>>* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, double* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, std::vector<double>* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, ksl_vec3_t* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, ksl_vec4_t* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, ksl_quaternion_t* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, ksl_screw_t* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, ksl_coscrew_t* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, ksl_inertia_t* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, ksl_mat3x3_t* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, ksl_mat4x4_t* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, ksl_eulerangles_t* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, bool* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, ksl_SE3_t* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, List* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  Value& addParameter(const std::string key, ArrayBase* data,
                      const std::string description, bool required = true) {
    Value value(data, description, required);
    return addParameter(key, value);
  }

  void setContext(Context* context) {
    context_ = context;
  }

  void setParentAssembly(Assembly* assembly) {
    parent_assembly_ptr_ = assembly;
  }

  void setParentBody(shift::core::body::Base* body) {
    parent_body_ptr_ = body;
  }

  virtual void readJSON(Json::Value&);

  void setParent(Context* context, Assembly* parent_assembly = nullptr,
                 shift::core::body::Base* parent_body = nullptr);

  virtual Value& operator[](const char* key) {
    return parameters_[key];
  }

  virtual Value& operator[](const std::string key) {
    return parameters_[key];
  }

  bool has_key(const std::string key) const {
    return parameters_.count(key);
  }

  auto& parameters() const {
    return parameters_;
  }

  void print() const;
  void print(std::string, bool break_line = true) const;

  Assembly* parentAssembly() {
    return parent_assembly_ptr_;
  }

  shift::core::body::Base* parentBody() {
    return parent_body_ptr_;
  }

  void setIsBody(bool is_body) {
    is_body_ = is_body;
  }

  void setIsAssembly(bool is_assembly) {
    is_assembly_ = is_assembly;
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

} // namespace Parameter
}
} // namespace shift
#endif

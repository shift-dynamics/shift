#ifndef SHIFT_PARAMETER_ARRAYBASE_H_
#define SHIFT_PARAMETER_ARRAYBASE_H_

#include "shift/core/parameter/Value.h"

namespace shift {
namespace core {

class Context;
class Assembly;

namespace body {
class Base;
}

namespace parameter {

class List;

/*!
\brief Abstract base class for named parameter arrays
*/
class ArrayBase {
public:
  ArrayBase() {}
  ArrayBase(Context* context, Assembly* parent_assembly = nullptr,
            shift::core::body::Base* parent_body = nullptr)
    : context_(context)
    , parent_assembly_ptr_(parent_assembly)
    , parent_body_ptr_(parent_body) {}
  virtual ~ArrayBase() {}

  bool active() {
    return active_;
  }

  virtual void setActiveObjects(bool) {}

  virtual Value& operator[](const std::string key_str);

  virtual Value& operator[](const char* key_str) {
    return operator[](std::string(key_str));
  }

  virtual Value& operator[](unsigned int index) {
    assert(index < objects_.size());
    auto& object = objects_[index];
    void_ = Value(object, "description", true);
    return void_;
  }

  virtual void setParent(Context* context, Assembly* parent_assembly = nullptr,
                         shift::core::body::Base* parent_body = nullptr) {
    context_ = context;
    parent_assembly_ptr_ = parent_assembly;
    parent_body_ptr_ = parent_body;
  }

  size_t size() {
    return objects_.size();
  }

  virtual void readJSON(Json::Value&) {}

  void print(std::string prefix) const;

  void print() const {
    print("");
  }

private:
  Value void_;

protected:
  std::vector<shift::core::parameter::List*> objects_;
  bool active_ = true;
  Context* context_ = nullptr;
  Assembly* parent_assembly_ptr_ = nullptr;
  shift::core::body::Base* parent_body_ptr_ = nullptr;
};

} // namespace parameter
} // namespace core
} // namespace shift
#endif

#ifndef SHIFT_PARAMETER_ARRAY_H_
#define SHIFT_PARAMETER_ARRAY_H_

#include "shift/core/parameter/ArrayBase.h"

namespace shift {
namespace core {
namespace parameter {

template <class T>
class Array : public shift::core::parameter::ArrayBase {
public:
  Array() {}
  Array(Context* context, Assembly* parent_assembly = nullptr,
        shift::core::body::Base* parent_body = nullptr)
    : ArrayBase(context, parent_assembly, parent_body) {}
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
    parent_assembly_ptr_ = assembly;
  }
  void setParentBody(shift::core::body::Base* body) {
    parent_body_ptr_ = body;
  }
  void push_back(std::unique_ptr<T>);

  virtual void setParent(Context* context, Assembly* parent_assembly = nullptr,
                         shift::core::body::Base* parent_body = nullptr) override {
    shift::core::parameter::ArrayBase::setParent(context, parent_assembly,
                                           parent_body);
    for(auto& object : object_specializations_) {
      object->setParent(context_, parent_assembly_ptr_, parent_body_ptr_);
    }
  }

private:
  std::vector<std::unique_ptr<T>> object_specializations_;
};

} // namespace Parameter
}
} // namespace shift
#endif

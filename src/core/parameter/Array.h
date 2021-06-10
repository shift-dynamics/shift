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

  T* get(unsigned int i) {
    if(i < objects_.size()) {
      return object_specializations_[i].get();
    } else {
      return nullptr;
    }
  }

  std::vector<std::unique_ptr<T>>& get_data() {
    return object_specializations_;
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

  virtual void set_parent(Context* context, Assembly* parent_assembly = nullptr,
                         shift::core::body::Base* parent_body = nullptr) override {
    shift::core::parameter::ArrayBase::set_parent(context, parent_assembly,
                                           parent_body);
    for(auto& object : object_specializations_) {
      object->set_parent(context_, parent_assembly_ptr_, parent_body_ptr_);
    }
  }

  virtual void ActivateObjects(bool parent_is_active) override {
    for(auto& object : object_specializations_) {
      object.get()->ActivateObjects(active_ && parent_is_active);
    }
  }

  void PushBack(std::unique_ptr<T>);
  virtual void ReadJSON(Json::Value& value) override;

private:
  std::vector<std::unique_ptr<T>> object_specializations_;
};

} // namespace Parameter
}
} // namespace shift
#endif

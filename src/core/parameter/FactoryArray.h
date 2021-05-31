#ifndef SHIFT_PARAMETER_FACTORYARRAY_H_
#define SHIFT_PARAMETER_FACTORYARRAY_H_

#include "shift/core/parameter/ArrayBase.h"

#include <vector>

namespace shift {
namespace core {
namespace parameter {

template <class T>
class FactoryArray : public shift::core::parameter::ArrayBase {
public:
  FactoryArray() {}
  FactoryArray(Context* context, Assembly* parent_assembly = nullptr,
               shift::core::body::Base* parent_body = nullptr)
    : ArrayBase(context, parent_assembly, parent_body) {}
  ~FactoryArray() {}

  virtual void set_objects_active(bool parent_is_active) override {
    for(auto& object : object_specializations_) {
      object.get()->set_objects_active(active_ && parent_is_active);
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

  void PushBack(std::unique_ptr<T>);
  virtual void ReadJSON(Json::Value& value) override;

private:
  std::vector<std::unique_ptr<T>> object_specializations_;
};

} // namespace Parameter
}
} // namespace shift
#endif

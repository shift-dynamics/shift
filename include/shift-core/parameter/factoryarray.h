#ifndef SHIFT_PARAMETER_FACTORYARRAY_H_
#define SHIFT_PARAMETER_FACTORYARRAY_H_

#include "shift-core/parameter/arraybase.h"

#include <vector>

namespace Shift {

namespace Parameter {

template <class T>
class FactoryArray : public Shift::Parameter::ArrayBase {
public:
  FactoryArray() {}
  FactoryArray(Context* context, Assembly* parent_assembly = nullptr,
               Shift::Body::Base* parent_body = nullptr)
    : ArrayBase(context, parent_assembly, parent_body) {}
  ~FactoryArray() {}

  virtual void readJSON(Json::Value& value) override;
  virtual void setActiveObjects(bool parent_is_active) override {
    for(auto& object : object_specializations_) {
      object.get()->setActiveObjects(active_ && parent_is_active);
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
  void setParentBody(Body::Base* body) {
    parent_body_ptr_ = body;
  }

  void push_back(std::unique_ptr<T>);

  virtual void setParent(Context* context, Assembly* parent_assembly = nullptr,
                         Body::Base* parent_body = nullptr) override {
    Shift::Parameter::ArrayBase::setParent(context, parent_assembly,
                                           parent_body);
    for(auto& object : object_specializations_) {
      object->setParent(context_, parent_assembly_ptr_, parent_body_ptr_);
    }
  }

private:
  std::vector<std::unique_ptr<T>> object_specializations_;
};

} // namespace Parameter
} // namespace Shift
#endif

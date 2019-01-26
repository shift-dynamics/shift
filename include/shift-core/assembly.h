#ifndef SHIFT_ASSEMBLY_H_
#define SHIFT_ASSEMBLY_H_

#include <json/json.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "shift-core/base.h"
#include "shift-core/behavior/base.h"
#include "shift-core/body/base.h"
#include "shift-core/constraint/base.h"
#include "shift-core/forcemodule/base.h"
#include "shift-core/joint/base.h"
#include "shift-core/light/base.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Behavior {
class Base;
}
namespace Body {
class Base;
}
namespace Constraint {
class Base;
}
namespace ForceModule {
class Base;
}
namespace Joint {
class Base;
}
namespace Light {
class Base;
}

class Context;
class SerialEntry;

/*!
@brief Shift Assembly datastructure
*/
class Assembly
  : public Base
  , public Serialize {

public:
  Assembly(std::string name)
    : Base(name)
    , Serialize() {
    addKeys();
  }

  Assembly()
    : Base("")
    , Serialize() {
    addKeys();
  }

  Assembly(Assembly&& a)
    : Base(a.name_) {
    for(auto& i : a.behaviors_) {
      behaviors_.push_back(std::move(i));
    }
    for(auto& i : a.bodies_) {
      bodies_.push_back(std::move(i));
    }
    for(auto& i : a.constraints_) {
      constraints_.push_back(std::move(i));
    }
    for(auto& i : a.forcemodules_) {
      forcemodules_.push_back(std::move(i));
    }
    for(auto& i : a.joints_) {
      joints_.push_back(std::move(i));
    }
  }

  Assembly& operator=(Assembly& a) {
    // check for self-assignment
    if(&a == this) {
      return *this;
    }

    for(auto& i : a.behaviors_) {
      behaviors_.push_back(std::move(i));
    }
    for(auto& i : a.bodies_) {
      bodies_.push_back(std::move(i));
    }
    for(auto& i : a.constraints_) {
      constraints_.push_back(std::move(i));
    }
    for(auto& i : a.forcemodules_) {
      forcemodules_.push_back(std::move(i));
    }
    for(auto& i : a.joints_) {
      joints_.push_back(std::move(i));
    }
    return *this;
  }

  ~Assembly() {}

  void readJSON(Json::Value& value, bool fixed = false);
  void printBehaviors();
  void printBodies();
  void printConstraints();
  void printForceModules();
  void printJoints();
  void print();
  void setFullyQualifiedNames(std::vector<std::string>&);
  void setActiveObjects(bool);
  void setContext(Context* context) {
    context_ = context;
  }
  Body::Base* getBody(unsigned int i) {
    if(i < bodies_.size()) {
      return bodies_[i].get();
    } else {
      return NULL;
    }
  }

private:
  Context* context_ = NULL;
  void addKeys();

  std::vector<std::unique_ptr<Assembly>> subassemblies_;

  std::vector<std::unique_ptr<Behavior::Base>> behaviors_;
  std::vector<std::unique_ptr<Body::Base>> bodies_;
  std::vector<std::unique_ptr<Constraint::Base>> constraints_;
  std::vector<std::unique_ptr<ForceModule::Base>> forcemodules_;
  std::vector<std::unique_ptr<Joint::Base>> joints_;
};

} // namespace Shift

#endif

#ifndef _SHIFT_CONSTRAINT_H_
#define _SHIFT_CONSTRAINT_H_

#include <functional>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Body {
class BodyBase;
}

class Frame;
class Substructure;

namespace Constraint {

/*!
@brief Abstract Shift::Constraint base class
*/
class ConstraintBase
  : public Base
  , public Serialize {

public:
  ConstraintBase(std::string name)
    : Base(name) {
    addKeys();
  }
  ConstraintBase()
    : Base("") {
    addKeys();
  }
  ~ConstraintBase() {}

  const std::string& getType() {
    return type_;
  }

protected:
  void addKeys();

  std::string type_; //!< descriptive name of Constraint type

  std::vector<std::vector<std::string>> bodyFramePairs_;

  std::vector<std::string> parentBody_; //!< descriptive name of parent Body
  BodyBase* parentBodyPointer_ = NULL;  //!< pointer to parent Body

  std::vector<std::string> parentFrame_; //!< descriptive name of parent Frame
  Frame* parentFramePointer_ = NULL;     //!< pointer to parent Frame

  std::vector<std::string> childBody_; //!< descriptive name of child Body
  BodyBase* childBodyPointer = NULL;   //!< pointer to child Body

  std::vector<std::string> childFrame_; //!< descriptive name of child frame
  Frame* childFramePointer = NULL;      //!< pointer to child Frame
};

class Factory {

public:
  Factory();
  Factory(const Factory&) {}
  Factory& operator=(const Factory&) {
    return *this;
  }

  ~Factory() {
    factoryMap_.clear();
  }

  typedef std::function<ConstraintBase*()> CreateFunction;

  static Factory* get() {
    static Factory instance;
    return &instance;
  }

  static void registerModule(const std::string& name, CreateFunction pfnCreate);
  static ConstraintBase* create(const std::string& name);

private:
  static std::map<std::string, CreateFunction> factoryMap_;
};

class Registration {
public:
  Registration(std::string name, Factory::CreateFunction fcn) {
    Shift::Factory::registerModule(name, fcn);
  }
};

} // namespace Constraint

} // namespace Shift

#endif

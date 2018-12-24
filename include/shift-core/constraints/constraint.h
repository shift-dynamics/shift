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
class Context;

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

  void setContext(Context* context) {
    context_ = context;
  }

  virtual void load();
  virtual void connect();
  virtual void readJSON(Json::Value&);

protected:
  void addKeys();

  Context* context_;
  std::string type_; //!< descriptive name of Constraint type

  std::vector<std::vector<std::string>> bodyFramePairs_;

  std::vector<std::string> parentBody_; //!< descriptive name of parent Body
  Body::BodyBase* parentBodyPointer_ = NULL; //!< pointer to parent Body

  std::vector<std::string> parentFrame_; //!< descriptive name of parent Frame
  Frame* parentFramePointer_ = NULL;     //!< pointer to parent Frame

  std::vector<std::string> childBody_;     //!< descriptive name of child Body
  Body::BodyBase* childBodyPointer = NULL; //!< pointer to child Body

  std::vector<std::string> childFrame_; //!< descriptive name of child frame
  Frame* childFramePointer = NULL;      //!< pointer to child Frame
};

class Modules {
public:
  void list();
  static bool initialized();

private:
  static std::vector<std::string> modules_;
  static bool initialized_;
};

} // namespace Constraint

} // namespace Shift

#endif

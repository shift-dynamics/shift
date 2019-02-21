#ifndef SHIFT_CONSTRAINT_BASE_H_
#define SHIFT_CONSTRAINT_BASE_H_

#include <functional>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Body {
class Base;
}

class Frame;
class Substructure;
class Context;

namespace Constraint {

/*!
@brief Abstract Shift::Constraint base class
*/
class Base
  : public Shift::Base
  , public Serialize {

public:
  Base(std::string name)
    : Shift::Base(name) {
    addKeys();
  }
  Base()
    : Shift::Base("") {
    addKeys();
  }
  virtual ~Base() {}

  const std::string& getType() {
    return type_;
  }

  void setContext(Context* context) {
    context_ = context;
  }

  virtual void load();
  virtual void connect();
  virtual void readJSON(Json::Value&);

  void setParentBody(Shift::Body::Base*);

  void setParentFrame(Frame*);

  void setChildBody(Shift::Body::Base*);

  void setChildFrame(Frame*);

  Body::Base* getParentBody() {
    return parentBodyPointer_;
  }

  Frame* getParentFrame() {
    return parentFramePointer_;
  }

  Body::Base* getChildBody() {
    return childBodyPointer_;
  }

  Frame* getChildFrame() {
    return childFramePointer_;
  }

protected:
  void addKeys();

  Context* context_;
  std::string type_; //!< descriptive name of Constraint type

  std::vector<std::vector<std::string>> bodyFramePairs_;

  std::vector<std::string> parentBody_; //!< descriptive name of parent Body
  Shift::Body::Base* parentBodyPointer_ = NULL; //!< pointer to parent Body

  std::vector<std::string> parentFrame_; //!< descriptive name of parent Frame
  Frame* parentFramePointer_ = NULL;     //!< pointer to parent Frame

  std::vector<std::string> childBody_; //!< descriptive name of child Body
  Shift::Body::Base* childBodyPointer_ = NULL; //!< pointer to child Body

  std::vector<std::string> childFrame_; //!< descriptive name of child frame
  Frame* childFramePointer_ = NULL;     //!< pointer to child Frame
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

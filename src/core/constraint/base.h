#ifndef SHIFT_CONSTRAINT_BASE_H_
#define SHIFT_CONSTRAINT_BASE_H_

#include <functional>
#include <map>
#include <string>

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace shift {

namespace Body {
class Base;
}

class Frame;
class Substructure;
class Context;

namespace Constraint {

/*!
@brief Abstract shift::Constraint base class
*/
class Base
  : public shift::Base
  , public Parameter::List {

public:
  Base(std::string name)
    : shift::Base(name) {
    addParameters();
  }
  Base()
    : shift::Base("") {
    addParameters();
  }
  virtual ~Base() override {}

  const std::string& type() {
    return type_;
  }

  virtual void setActiveObjects(bool parent_is_active) override;

  virtual void load();
  virtual void connect();

  static const std::string arrayKey() {
    return "constraints";
  }

  static const std::string defaultType() {
    return "";
  }

  void setParentBody(shift::Body::Base*);

  void setParentFrame(Frame*);

  void setChildBody(shift::Body::Base*);

  void setChildFrame(Frame*);

  Body::Base* parentBody() {
    return parent_body_ptr_;
  }

  Frame* parentFrame() {
    return parent_frame_ptr_;
  }

  Body::Base* childBody() {
    return child_body_ptr_;
  }

  Frame* childFrame() {
    return child_frame_ptr_;
  }

protected:
  void addParameters() override;

  std::string type_; //!< descriptive name of Constraint type

  std::vector<std::vector<std::string>> body_frame_pairs_;

  std::vector<std::string> parent_body_; //!< descriptive name of parent Body
  shift::Body::Base* parent_body_ptr_ = nullptr; //!< pointer to parent Body

  std::vector<std::string> parent_frame_; //!< descriptive name of parent Frame
  Frame* parent_frame_ptr_ = nullptr;     //!< pointer to parent Frame

  std::vector<std::string> child_body_; //!< descriptive name of child Body
  shift::Body::Base* child_body_ptr_ = nullptr; //!< pointer to child Body

  std::vector<std::string> child_frame_; //!< descriptive name of child frame
  Frame* child_frame_ptr_ = nullptr;     //!< pointer to child Frame
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

} // namespace shift

#endif

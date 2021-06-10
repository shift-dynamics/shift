#ifndef SHIFT_CONSTRAINT_BASE_H_
#define SHIFT_CONSTRAINT_BASE_H_

#include <functional>
#include <map>
#include <string>

#include "shift/core/Base.h"
#include "shift/core/parameter/List.h"

namespace shift {
namespace core {

namespace body {
class Base;
}

class Frame;
class Substructure;
class Context;

namespace constraint {

/*!
@brief Abstract shift::Constraint base class
*/
class Base
  : public shift::core::Base
  , public parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name) {
    DeclareParameters();
  }
  Base()
    : shift::core::Base("") {
    DeclareParameters();
  }
  virtual ~Base() override {}

  const std::string& get_type() {
    return type_;
  }

  static const std::string get_array_key() {
    return "constraints";
  }

  static const std::string get_default_type() {
    return "";
  }

  shift::core::body::Base* get_parent_body() {
    return parent_body_ptr_;
  }

  shift::core::Frame* get_parent_frame() {
    return parent_frame_ptr_;
  }

  shift::core::body::Base* get_child_body() {
    return child_body_ptr_;
  }

  Frame* get_child_frame() {
    return child_frame_ptr_;
  }

  void set_parent_body(shift::core::body::Base*);

  void set_parent_frame(Frame*);

  void set_child_body(shift::core::body::Base*);

  void set_child_frame(Frame*);

  virtual void ActivateObjects(bool parent_is_active) override;
  virtual void Load();
  virtual void Connect();


protected:
  void DeclareParameters() override;

  std::string type_; //!< descriptive name of Constraint type

  std::vector<std::vector<std::string>> body_frame_pairs_;

  std::vector<std::string> parent_body_; //!< descriptive name of parent Body
  shift::core::body::Base* parent_body_ptr_ = nullptr; //!< pointer to parent Body

  std::vector<std::string> parent_frame_; //!< descriptive name of parent Frame
  Frame* parent_frame_ptr_ = nullptr;     //!< pointer to parent Frame

  std::vector<std::string> child_body_; //!< descriptive name of child Body
  shift::core::body::Base* child_body_ptr_ = nullptr; //!< pointer to child Body

  std::vector<std::string> child_frame_; //!< descriptive name of child frame
  shift::core::Frame* child_frame_ptr_ = nullptr;     //!< pointer to child Frame
};

class Modules {
public:
  void list();
  static bool is_initialized();

private:
  static std::vector<std::string> modules_;
  static bool initialized_;
};

} // namespace constraint
}
} // namespace shift

#endif

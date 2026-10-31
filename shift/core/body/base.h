#ifndef SHIFT_BODY_BASE_H_
#define SHIFT_BODY_BASE_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/inertia.h"

#include "shift/core/base.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/frame.h"
#include "shift/core/geometry/base.h"
#include "shift/core/light/base.h"
#include "shift/core/parameter/array.h"
#include "shift/core/parameter/factory_array.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

namespace joint {
class Base;
}

class Assembly;
class Context;

namespace body {

/*!
@brief Parameters that describe a shift::Body
*/
class Base
  : public shift::core::Base
  , public shift::core::parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name)
    , shift::core::parameter::List() {
    auto origin = std::make_unique<Frame>("origin");
    origin->set_is_origin(true);
    origin->set_is_reference(true);
    origin->set_parent_body(this);
    assert(origin->get_parent_body());

    frames_.PushBack(std::move(origin));
    DeclareParameters();
  }
  Base()
    : shift::core::Base("")
    , shift::core::parameter::List() {
    auto origin = std::make_unique<Frame>("origin");
    origin->set_is_origin(true);
    origin->set_is_reference(true);
    origin->set_parent_body(this);
    frames_.PushBack(std::move(origin));
    DeclareParameters();
  }
  Base(Base&& b)
    : shift::core::Base(b.name_) {
    // for(auto& i : b.lights_) {
    //   lights_.push_back(std::move(i));
    // }
    // for(auto& i : b.geometry_) {
    //   geometry_.push_back(std::move(i));
    // }
    // for(auto& i : b.collision_geometry_) {
    //   collision_geometry_.push_back(std::move(i));
    // }
  }

  virtual ~Base() override {}
 
  static const std::string get_array_key() {
    return "bodies";
  }

  static const std::string get_default_type() {
    return "rigid";
  }

  const std::string& get_type() {
    return type_;
  }
  
  Base* get_parent_body();
  Frame* get_origin();
  Frame* get_reference_frame();

  unsigned int get_frame_count() {
    return frames_.get_size();
  }

  /*!
  return number of frames in spanning tree, context->ProcessFrames must be
  called to set frames in spanning tree.
  */
  unsigned int get_frame_in_tree_count();

  int get_index() {
    return index_;
  }

  int get_level() {
    return level_;
  }

  std::vector<std::unique_ptr<Frame>>& get_frames() {
    return frames_.get_data();
  }

  std::vector<std::unique_ptr<shift::core::geometry::Base>>& get_geometry() {
    return geometry_.get_data();
  }

  const joint::Base* get_parent_joint() {
    return parent_joint_;
  }

  int get_va_index() {
    return va_index_;
  }

  virtual const ksl_inertia_t& get_inertia() const {
    return inertia_;
  }

  virtual ksl_inertia_t get_inertia_wrt_reference() {
    return inertia_;
  }

  const ksl_vec3_t get_center_of_mass_wrt_reference();


  bool has_mass() {
    return (inertia_.m >= 1e-9);
  }

  bool is_articulated() {
    return is_articulated_;
  }

  bool is_virtual() {
    return is_virtual_;
  }

  void set_reference_frame(shift::core::Frame*);

  void set_index(int index) {
    index_ = index;
  }

  void set_level(int level) {
    level_ = level;
  }

  void set_parent_joint(joint::Base*);

  void set_va_index(int index) {
    va_index_ = index;
  }

  void SetupDWrtReference(ksl_SE3_t);
  virtual void ActivateObjects(bool parent_is_active) override;
  void ClearInTreeFlags();
  void SetupReferenceFrame();
  virtual void Load();
  virtual void Connect();


protected:
  virtual void DeclareParameters() override;
  void SetupDWrtReference();

  std::string type_;
  bool is_articulated_ = true; /*!< indicates whether body may have joints
                                 attached to it, i.e. that the body comprises a
                                 link of an articulated assembly of bodies */

  shift::core::parameter::Array<shift::core::Frame> frames_;

  shift::core::parameter::FactoryArray<shift::core::light::Base> lights_;
  shift::core::parameter::FactoryArray<shift::core::geometry::Base> geometry_;
  shift::core::parameter::FactoryArray<shift::core::collision_geometry::Base> collision_geometry_;

  bool draw_enabled_ = true;
  bool draw_collision_geometry_enabled_ = false;
  int index_ = -1;
  int level_ = -1;
  int va_index_ = -1;
  joint::Base* parent_joint_ = nullptr;
  shift::core::Frame* reference_frame_ =
    nullptr; /*!< pointer to unique reference frame on body, in an articulated
                multibody chain, this is the *proximal* or upstream
                frame in the spanning tree */
  ksl_inertia_t inertia_ = {}; /*!< centroidal mass and moment of inertia
                                    properties expressed in the body's origin
                                    coordinate system */
  ksl_vec3_t center_of_mass_wrt_origin_ = {};    /*!< vector from body origin to
                                                center of mass */
  ksl_vec3_t center_of_mass_wrt_reference_ = {}; /*!< vector from body reference
                                                frame to center of mass */
  bool is_virtual_ = false;
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace body
}
} // namespace shift

#endif

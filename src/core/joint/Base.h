#ifndef SHIFT_JOINT_BASE_H_
#define SHIFT_JOINT_BASE_H_

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <string.h>

#include "shift/core/Base.h"
#include "shift/core/parameter/List.h"
#include "shift/core/Topology.h"

#include "ksl/SE3.h"
#include "ksl/screw.h"

namespace shift {
namespace core {

class Substructure;

namespace body {
class Base;
}

class Frame;
class Substructure;
class Context;
class Assembly;

namespace joint {

/*!%
@brief Abstract shift::Joint base class
*/
class Base
  : public shift::core::Base
  , public shift::core::parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name)
    , parameter::List() {
    DeclareParameters();
  }
  Base()
    : shift::core::Base("")
    , parameter::List() {
    DeclareParameters();
  }
  virtual ~Base() {}

  static const std::string get_array_key() {
    return "joints";
  }

  static const std::string get_default_type() {
    return "";
  }

  const std::string& get_type() {
    return type_;
  }

  unsigned int get_dc_count() {
    return dc_count_;
  };

  unsigned int get_q_count() const {
    return q_count_;
  };

  unsigned int get_dq_count() const {
    return dq_count_;
  };

  const double* get_q0() {
    return q_0_;
  }

  const double* get_dq0() {
    return dq_0_;
  }

  shift::core::body::Base* get_parent_body() {
    return parent_body_ptr_;
  }

  shift::core::body::Base* get_system_parent_body() {
    if(sense_ >= 0) {
      return parent_body_ptr_;
    } else {
      return child_body_ptr_;
    }
  }

  shift::core::body::Base* get_child_body() {
    return child_body_ptr_;
  }

  shift::core::body::Base* get_system_child_body() const {
    if(system_child_body_ptr_) {
      return system_child_body_ptr_;
    }
    if(sense_ >= 0) {
      return child_body_ptr_;
    } else {
      return parent_body_ptr_;
    }
  }

  Frame* get_parent_frame() {
    return parent_frame_ptr_;
  }

  Frame* get_system_parent_frame() {
    if(sense_ >= 0) {
      return parent_frame_ptr_;
    } else {
      return child_frame_ptr_;
    }
  }

  shift::core::Frame* get_child_frame() {
    return child_frame_ptr_;
  }

  shift::core::Frame* get_system_child_frame() {
    if(system_child_frame_ptr_) {
      return system_child_frame_ptr_;
    }
    if(sense_ >= 0) {
      return child_frame_ptr_;
    } else {
      return parent_frame_ptr_;
    }
  }

  const Substructure* get_substructure() const {
    return substructure_;
  }

  const std::vector<int>& get_rotation_indices() const {
    return rotation_indices_;
  }

  int get_sense() {
    return sense_;
  }

  int get_level() {
    return level_;
  }

  virtual unsigned int get_data_count() {
    return data_count_;
  }

  virtual const double* get_data() {
    return nullptr;
  }

  virtual ksl_SE3_t get_transform() {
    ksl_SE3_t temp;
    ksl_SE3_setIdentity(&temp);
    return temp;
  }

  int get_q_index() const {
    return q_index_;
  }

  int get_dq_index() const {
    return dq_index_;
  }

  int get_dc_index() const {
    return dc_index_;
  }

  int get_data_index() const {
    return data_index_;
  }

  const std::vector<shift::core::joint::Base*>& get_child_joints() const {
    return child_joints_;
  }

  void set_objects_active(bool parent_is_active) override;
  void set_initial_displacement(const double* q);
  void set_initial_velocity(const double* dq);

  void set_system_child_body(shift::core::body::Base* body) {
    system_child_body_ptr_ = body;
  }

  void set_system_child_frame(shift::core::Frame* frame) {
    system_child_frame_ptr_ = frame;
  }

  void set_sense(int sense) {
    sense_ = sense;
  }

  void set_level(int level) {
    level_ = level;
  }

  void set_q_index(int index) {
    q_index_ = index;
  }

  void set_dq_index(int index) {
    dq_index_ = index;
  }

  void set_dc_index(int index) {
    dc_index_ = index;
  }

  void set_data_index(int index) {
    data_index_ = index;
  }
  
  void set_substructure(Substructure* substructure) {
    substructure_ = substructure;
  }

  virtual topology::dh_fptr Dfptr() = 0;
  virtual topology::dh_fptr DHfptr() = 0;
  virtual topology::dhva_fptr DHVAfptr() = 0;
  
  virtual void Load();
  void Connect();

  void ClearChildJoints() {
    child_joints_.clear();
  }

  void PushChildJoint(shift::core::joint::Base* child) {
    child_joints_.push_back(child);
  }

  void ClearSubstructure() {
    substructure_ = nullptr;
  }
 
  virtual void Service(const std::vector<double>&) {}

protected:
  virtual void DeclareParameters() override;
  std::string type_;          //!< descriptive name of joint type
  unsigned int dof_ = 0;      //!< number of degrees of freedom
  unsigned int dc_count_ = 0; /*!< number of direction cosines used by joint, 1
                                  indicates a (sin, cos) pair */
  unsigned int q_count_ = 0;  //!< number of generalized coordinates
  unsigned int dq_count_ = 0; //!< number of generalized coordinate derivatives
  unsigned int data_count_ = 0; //!< additional quantities used to process joint
  int index_ = -1;              //!< index in joint array
  int q_index_ = -1;            //!< index into q array
  int dq_index_ = -1;           //!< index into dq array
  int dc_index_ = -1;           //!< index into dc array
  int index_substructure_ = -1; //!< local index into substructure
  int sense_ = 0; /*!< indicates whether joint is positively aligned with
               spanning tree (+1), or negatively aligned with spanning tree
               (-1). Automatically set by topology::sort() */
  int level_ = 0; /*!< Indicates Joint's level in the spanning tree,
                set by ProcessTopology */
  int data_index_ = -1;

  Substructure* substructure_ = nullptr; /*!< substructure the joint is
    a member of, nullptr if the joint is not a member of any substructure */

  std::vector<int> rotation_indices_;
  bool is_rotation_ = false;
  /*!
  The `parent` descriptive name follows the format:

    [["fully", "qualified", "body_name"], "frame_name"]

  or if the body is in the same assembly as the joint currently being
  processed it is sufficient to use:

    [["body_name"], "frame_name"]

  To attach a joint to the fixed inertial reference frame, use the following
  syntax:

    [["fixed"], "frame_name"]

  */
  std::vector<std::vector<std::string>> parent_;

  /*!
  The `child` descriptive name follows the format:

    [["fully", "qualified", "body_name"], "frame_name"]

  or if the body is in the same assembly as the joint currently being
  processed it is sufficient to use:

    [["body_name"], "frame_name"]

  To attach a joint to the fixed inertial reference frame, use the following
  syntax:

    [["fixed"], "frame_name"]
  */
  std::vector<std::vector<std::string>> child_;

  shift::core::body::Base* parent_body_ptr_ = nullptr; //!< pointer to parent Body
  Frame* parent_frame_ptr_ = nullptr;            //!< pointer to parent Frame
  shift::core::body::Base* child_body_ptr_ = nullptr;  //!< pointer to child Body
  Frame* child_frame_ptr_ = nullptr;             //!< pointer to child Frame
  shift::core::body::Base* system_child_body_ptr_ =
    nullptr; //!< pointer to a system defined virtual body
  shift::core::Frame* system_child_frame_ptr_ =
    nullptr; //!< pointer to a system defined virtual body

  std::vector<shift::core::joint::Base*> child_joints_;

  double* q_0_ = nullptr;  /*!< initial primitive displacement
                                              specified by user */
  double* dq_0_ = nullptr; /*!< initial primitive velocity specified
                                         by user */

  double* mobility_number_ = nullptr;
  bool independent_ = true; /*!< boolean to indicate whether joint is
                         independent (true) or dependent (false) as
                         determined by fullFactor_LU algorithm */

  Base* parent_joint_ = nullptr; /*!< pointer to unique parentJoint */

  Substructure* substructure_ptr_ =
    nullptr; /*!< pointer to substructure assignment */

  std::vector<double>
    damping_coefficients_; /*!< damping coefficients, c0 + c1*x + c2 * x**2
+ ... + cn * x**n */
  std::vector<double>
    stiffness_coefficients_; /*!< spring coefficients, k0 + k1 * x
                 + k2 * x**2 + ... + kn * x**n */
  bool visible_ = false;
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace joint
}
} // namespace shift

#endif

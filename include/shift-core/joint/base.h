#ifndef SHIFT_JOINT_BASE_H_
#define SHIFT_JOINT_BASE_H_

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <string.h>

#include "shift-core/base.h"
#include "shift-core/parameter.h"
#include "shift-core/topology.h"

#include "ksl/SE3.h"
#include "ksl/screw.h"

namespace Shift {

class Substructure;

namespace Body {
class Base;
}

class Frame;
class Substructure;
class Context;
class Assembly;

namespace Joint {

/*!
@brief Abstract Shift::Joint base class
*/
class Base
  : public Shift::Base
  , public Parameter::List {

public:
  Base(std::string name)
    : Shift::Base(name)
    , Parameter::List() {
    addParameters();
  }
  Base()
    : Shift::Base("")
    , Parameter::List() {
    addParameters();
  }
  virtual ~Base() {}

  const std::string& type() {
    return type_;
  }

  unsigned int dcCount() {
    return dcCount_;
  };

  unsigned int qCount() const {
    return qCount_;
  };

  unsigned int dqCount() const {
    return dqCount_;
  };

  const double* q0() {
    return q_0_;
  }

  const double* dq0() {
    return dq_0_;
  }

  void setInitialDisplacement(const double* q);
  void setInitialVelocity(const double* dq);

  virtual Topology::dh_fptr Dfptr() = 0;
  virtual Topology::dh_fptr DHfptr() = 0;
  virtual Topology::dhva_fptr DHVAfptr() = 0;

  static const std::string arrayKey() {
    return "joints";
  }
  static const std::string defaultType() {
    return "";
  }

  void setContext(Context* context) {
    context_ = context;
  }

  void setAssembly(Assembly* assembly) {
    assembly_ = assembly;
  }

  virtual void load();

  void connect();

  Shift::Body::Base* parentBody() {
    return parentBodyPointer_;
  }

  Shift::Body::Base* systemParentBody() {
    if(sense_ >= 0) {
      return parentBodyPointer_;
    } else {
      return childBodyPointer_;
    }
  }

  Shift::Body::Base* childBody() {
    return childBodyPointer_;
  }

  Shift::Body::Base* systemChildBody() const {
    if(systemChildBodyPointer_) {
      return systemChildBodyPointer_;
    }
    if(sense_ >= 0) {
      return childBodyPointer_;
    } else {
      return parentBodyPointer_;
    }
  }

  Shift::Frame* parentFrame() {
    return parentFramePointer_;
  }

  Shift::Frame* systemParentFrame() {
    if(sense_ >= 0) {
      return parentFramePointer_;
    } else {
      return childFramePointer_;
    }
  }

  Shift::Frame* childFrame() {
    return childFramePointer_;
  }

  Shift::Frame* systemChildFrame() {
    if(systemChildFramePointer_) {
      return systemChildFramePointer_;
    }
    if(sense_ >= 0) {
      return childFramePointer_;
    } else {
      return parentFramePointer_;
    }
  }

  void setSystemChildBody(Shift::Body::Base* body) {
    systemChildBodyPointer_ = body;
  }

  void setSystemChildFrame(Shift::Frame* frame) {
    systemChildFramePointer_ = frame;
  }

  int sense() {
    return sense_;
  }

  void setSense(int sense) {
    sense_ = sense;
  }

  int getLevel() {
    return level_;
  }

  void setLevel(int level) {
    level_ = level;
  }

  virtual unsigned int dataCount() {
    return dataCount_;
  }

  virtual const double* data() {
    return NULL;
  }

  virtual ksl_SE3_t transform() {
    ksl_SE3_t temp;
    ksl_SE3_setIdentity(&temp);
    return temp;
  }

  void setqIndex(int index) {
    qIndex_ = index;
  }

  void setdqIndex(int index) {
    dqIndex_ = index;
  }

  void setdcIndex(int index) {
    dcIndex_ = index;
  }

  int qIndex() const {
    return qIndex_;
  }

  int dqIndex() const {
    return dqIndex_;
  }

  int dcIndex() const {
    return dcIndex_;
  }

  int dataIndex() const {
    return dataIndex_;
  }

  void setDataIndex(int index) {
    dataIndex_ = index;
  }

  void clearChildJoints() {
    childJoints_.clear();
  }

  void pushChildJoint(Shift::Joint::Base* child) {
    childJoints_.push_back(child);
  }

  const std::vector<Shift::Joint::Base*>& childJoints() const {
    return childJoints_;
  }

  void setSubstructure(Substructure* substructure) {
    substructure_ = substructure;
  }

  void clearSubstructure() {
    substructure_ = NULL;
  }

  const Substructure* substructure() const {
    return substructure_;
  }

protected:
  Context* context_ = NULL;
  Assembly* assembly_ = NULL;

  virtual void addParameters();
  std::string type_;           //!< descriptive name of joint type
  unsigned int dof_ = 0;       //!< number of degrees of freedom
  unsigned int dcCount_ = 0;   /*!< number of direction cosines used by joint, 1
                                    indicates a (sin, cos) pair */
  unsigned int qCount_ = 0;    //!< number of generalized coordinates
  unsigned int dqCount_ = 0;   //!< number of generalized coordinate derivatives
  unsigned int dataCount_ = 0; //!< additional quantities used to process joint
  int index_ = -1;             //!< index in joint array
  int qIndex_ = -1;            //!< index into q array
  int dqIndex_ = -1;           //!< index into dq array
  int dcIndex_ = -1;           //!< index into dc array
  int indexSubstructure_ = -1; //!< local index into substructure
  int sense_ = 0; /*!< indicates whether joint is positively aligned with
               spanning tree (+1), or negatively aligned with spanning tree
               (-1). Automatically set by Topology::sort() */
  int level_ = 0; /*!< Indicates Joint's level in the spanning tree,
                set by ProcessTopology */
  int dataIndex_ = -1;

  Substructure* substructure_ = NULL; /*!< substructure the joint is
    a member of, NULL if the joint is not a member of any substructure */

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

  Shift::Body::Base* parentBodyPointer_ = NULL; //!< pointer to parent Body
  Frame* parentFramePointer_ = NULL;            //!< pointer to parent Frame
  Shift::Body::Base* childBodyPointer_ = NULL;  //!< pointer to child Body
  Frame* childFramePointer_ = NULL;             //!< pointer to child Frame
  Shift::Body::Base* systemChildBodyPointer_ =
    NULL; //!< pointer to a system defined virtual body
  Shift::Frame* systemChildFramePointer_ =
    NULL; //!< pointer to a system defined virtual body

  std::vector<Shift::Joint::Base*> childJoints_;

  double* q_0_ = NULL;  /*!< initial primitive displacement
                                              specified by user */
  double* dq_0_ = NULL; /*!< initial primitive velocity specified
                                         by user */

  double* mobilityNumber_ = NULL;
  bool independent_ = true; /*!< boolean to indicate whether joint is
                         independent (true) or dependent (false) as
                         determined by fullFactor_LU algorithm */

  Base* parentJoint_ = NULL; /*!< pointer to unique parentJoint */

  Substructure* substructurePointer_ =
    NULL; /*!< pointer to substructure assignment */

  std::vector<double>
    dampingCoefficients_; /*!< damping coefficients, c0 + c1*x + c2 * x**2
+ ... + cn * x**n */
  std::vector<double>
    stiffnessCoefficients_; /*!< spring coefficients, k0 + k1 * x
                 + k2 * x**2 + ... + kn * x**n */
  bool isVisible_ = false;
};

class Modules {
public:
  static bool initialized();
};

} // namespace Joint

} // namespace Shift

#endif

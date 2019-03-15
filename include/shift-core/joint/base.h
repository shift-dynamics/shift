#ifndef SHIFT_JOINT_BASE_H_
#define SHIFT_JOINT_BASE_H_

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <string.h>

#include "shift-core/base.h"
#include "shift-core/serialize.h"
#include "shift-core/topology.h"

#include "ksl/SE3.h"
#include "ksl/screw.h"

namespace Shift {

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
  , public Serialize {

public:
  Base(std::string name)
    : Shift::Base(name)
    , Serialize() {
    addKeys();
  }
  Base()
    : Shift::Base("")
    , Serialize() {
    addKeys();
  }
  virtual ~Base() {}

  const std::string& getType() {
    return type_;
  }

  unsigned int getdcCount() {
    return dcCount_;
  };

  unsigned int getqCount() {
    return qCount_;
  };

  unsigned int getdqCount() {
    return dqCount_;
  };

  const double* getq0() {
    return q_0_;
  }

  const double* getdq0() {
    return dq_0_;
  }

  void setInitialDisplacement(const double* q);
  void setInitialVelocity(const double* dq);

  virtual Topology::dh_fptr getDfptr() = 0;
  virtual Topology::dh_fptr getDHfptr() = 0;
  virtual Topology::dhva_fptr getDHVAfptr() = 0;

  virtual void readJSON(Json::Value&);

  void print();

  void setContext(Context* context) {
    context_ = context;
  }

  void setAssembly(Assembly* assembly) {
    assembly_ = assembly;
  }

  virtual void load();

  void connect();

  Shift::Body::Base* getParentBody() {
    return parentBodyPointer_;
  }

  Shift::Body::Base* getSystemParentBody() {
    if(sense_ >= 0) {
      return parentBodyPointer_;
    } else {
      return childBodyPointer_;
    }
  }

  Shift::Body::Base* getChildBody() {
    return childBodyPointer_;
  }

  Shift::Body::Base* getSystemChildBody() {
    if(systemChildBodyPointer_) {
      return systemChildBodyPointer_;
    }
    if(sense_ >= 0) {
      return childBodyPointer_;
    } else {
      return parentBodyPointer_;
    }
  }

  Shift::Frame* getParentFrame() {
    return parentFramePointer_;
  }

  Shift::Frame* getSystemParentFrame() {
    if(sense_ >= 0) {
      return parentFramePointer_;
    } else {
      return childFramePointer_;
    }
  }

  Shift::Frame* getChildFrame() {
    return childFramePointer_;
  }

  Shift::Frame* getSystemChildFrame() {
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

  int getSense() {
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

  virtual unsigned int getDataCount() {
    return dataCount_;
  }

  virtual const double* getData() {
    return NULL;
  }

  virtual ksl_SE3_t getTransform() {
    ksl_SE3_t temp;
    ksl_SE3_setIdentity(&temp);
    return temp;
  }

protected:
  Context* context_ = NULL;
  Assembly* assembly_ = NULL;

  void addKeys();
  std::string type_;           //!< descriptive name of joint type
  unsigned int dof_ = 0;       //!< number of degrees of freedom
  unsigned int dcCount_ = 0;   /*!< number of direction cosines used by joint, 1
                                    indicates a (sin, cos) pair */
  unsigned int qCount_ = 0;    //!< number of generalized coordinates
  unsigned int dqCount_ = 0;   //!< number of generalized coordinate derivatives
  unsigned int dataCount_ = 0; //!< additional quantities used to process joint
  int index_;                  //!< index in joint array
  int qIndex_;                 //!< index into q array
  int dqIndex_;                //!< index into dq array
  int dcIndex_;                //!< index into dc array
  int indexSubstructure_;      //!< local index into substructure
  int sense_ = 0; /*!< indicates whether joint is positively aligned with
               spanning tree (+1), or negatively aligned with spanning tree
               (-1). Automatically set by Topology::sort() */
  int level_ = 0; /*!< Indicates Joint's level in the spanning tree,
                set by ProcessTopology */

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

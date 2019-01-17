#ifndef _SHIFT_JOINT_H_
#define _SHIFT_JOINT_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

#include "ksl/screw.h"

namespace Shift {

namespace Body {
class BodyBase;
}

class Frame;
class Substructure;
class Context;
class Assembly;

namespace Joint {

/*!
@brief Abstract Shift::Joint base class
*/
class JointBase
  : public Base
  , public Serialize {

public:
  JointBase(std::string name)
    : Base(name)
    , Serialize() {
    addKeys();
  }
  JointBase()
    : Base("")
    , Serialize() {
    addKeys();
  }
  ~JointBase() {}

  const std::string& getType() {
    return type_;
  }

  virtual void updateD() = 0;
  virtual void updateDH() = 0;
  virtual void updateDHVA() = 0;

  void readJSON(Json::Value&);

  void print();

  void setContext(Context* context) {
    context_ = context;
  }

  void setAssembly(Assembly* assembly) {
    assembly_ = assembly;
  }

  virtual void load();

  void connect();

  Shift::Body::BodyBase* getParentBody() {
    return parentBodyPointer_;
  }

  Shift::Body::BodyBase* getChildBody() {
    return childBodyPointer_;
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

protected:
  Context* context_ = NULL;
  Assembly* assembly_ = NULL;

  void addKeys();
  std::string type_;      //!< descriptive name of joint type
  int dof_;               //!< number of degrees of freedom
  int index_;             //!< index in joint array
  int qIndex_;            //!< index into q array
  int dqIndex_;           //!< index into dq array
  int dcIndex_;           //!< index into dc array
  int indexSubstructure_; //!< local index into substructure
  int sense_ = 0; /*!< indicates whether joint is positively aligned with
               spanning tree (+1), or negatively aligned with spanning tree
               (-1). Automatically set by Topology::sort() */
  int level_ = 0; /*!< Indicates Joint's level in the spanning tree,
                set by ProcessTopology */

  /*!
  The `parent` descriptive name follows the format:

    ["fully", "qualified", "body_name"], ["frame_name"]
  */
  std::vector<std::vector<std::string>> parent_;

  /*! The `child` descriptive name follows the format:

    ["fully", "qualified", "body_name"], ["frame_name"]
  */
  std::vector<std::vector<std::string>> child_;

  Body::BodyBase* parentBodyPointer_ = NULL; //!< pointer to parent Body
  Frame* parentFramePointer_ = NULL;         //!< pointer to parent Frame
  Body::BodyBase* childBodyPointer_ = NULL;  //!< pointer to child Body
  Frame* childFramePointer_ = NULL;          //!< pointer to child Frame

  double* q_0_ = NULL;  /*!< initial primitive displacement
                                              specified by user */
  double* dq_0_ = NULL; /*!< initial primitive velocity specified
                                         by user */

  double* q_ = NULL;  /*!< current primitive displacement in
                                model, for spherical joints consists of
                                quaternion following x,y,z,w
                                convention, for float joints consists of
                                x,y,z translation, followed by
                                quaternion specified using x,y,z,w convention */
  double* dq_ = NULL; /*!< current generalized velocity in model */

  ksl_screw_t* h_ = NULL;

  double* mobilityNumber_ = NULL;
  bool independent_ = true; /*!< boolean to indicate whether joint is
                         independent (true) or dependent (false) as
                         determined by fullFactor_LU algorithm */

  JointBase* parentJoint_ = NULL; /*!< pointer to unique parentJoint */

  Substructure* substructurePointer_ =
    NULL; /*!< pointer to substructure assignment */

  double* dampingCoefficients_ =
    NULL; /*!< damping coefficients, c0 + c1*x + c2 * x**2
        + ... + cn * x**n */
  int dampingCoefficientCount_ = 0; /*! number of damping coefficients */

  double* stiffnessCoefficients_ = NULL; /*!< spring coefficients, k0 + k1 * x +
                                   k2 * x**2 + ... + kn * x**n */
  int stiffnessCoefficientCount_ = 0;    /*!< number of spring coefficients */
  bool isVisible_ = false;
};

class Modules {
public:
  static bool initialized();
};

} // namespace Joint

} // namespace Shift

#endif

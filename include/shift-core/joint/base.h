#ifndef SHIFT_JOINT_BASE_H_
#define SHIFT_JOINT_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

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

struct DHVA {
  ksl_SE3_t* d;    /*!< array of displacements equal
                        in size to number of frames */
  double* dc;      /*!< direction cosine arrays */
  ksl_screw_t* h;  /*!< an array of influence coefficient matrices equal to the
                        6 * number of variables in the datastructure */
  ksl_screw_t* v;  /*!< an array of body velocities equal to the
                        number of variables in the datastructure */
  ksl_screw_t* aq; /*!< an array of body accelerations equal to the
                        number of variables in the datastructure */
};

struct Offsets {
  union {
    unsigned int dc;
    unsigned int q;
  };
  unsigned int dq;
  struct {
    unsigned int d;
    unsigned int v;
  } parent;
  struct {
    unsigned int d;
    unsigned int v;
  } child;
  unsigned int data;
};

typedef void (*dh_fptr)(const Shift::Context&, DHVA&, const Offsets&,
                        const double*);

typedef void (*dhva_fptr)(const Shift::Context&, DHVA&, const Offsets&,
                          const double*, const double*);

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
  ~Base() {}

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

  virtual dh_fptr getDH() = 0;
  virtual dhva_fptr getDHVA() = 0;

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
    if(sense_ >= 0) {
      return childFramePointer_;
    } else {
      return parentFramePointer_;
    }
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

protected:
  Context* context_ = NULL;
  Assembly* assembly_ = NULL;

  void addKeys();
  std::string type_;         //!< descriptive name of joint type
  unsigned int dof_;         //!< number of degrees of freedom
  unsigned int dcCount_ = 0; /*!< number of direction cosines used by joint, 1
                                  indicates a (sin, cos) pair */
  unsigned int qCount_ = 0;  //!< number of generalized coordinates
  unsigned int dqCount_ = 0; //!< number of generalized coordinate derivatives
  unsigned int dataCount_ =
    0;                    //!< additional fixed quantities used to process joint
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

    [["fully", "qualified", "body_name"], "frame_name"]

  or if the body is in the same assembly as the joint currently being processed
  it is sufficient to use:

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

  Base* parentJoint_ = NULL; /*!< pointer to unique parentJoint */

  Substructure* substructurePointer_ =
    NULL; /*!< pointer to substructure assignment */

  double* dampingCoefficients_ =
    NULL; /*!< damping coefficients, c0 + c1*x + c2 * x**2
        + ... + cn * x**n */
  int dampingCoefficientCount_ = 0; /*! number of damping coefficients */

  double* stiffnessCoefficients_ = NULL; /*!< spring coefficients, k0 + k1 * x
                                   + k2 * x**2 + ... + kn * x**n */
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

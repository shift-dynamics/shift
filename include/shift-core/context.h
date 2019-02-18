#ifndef SHIFT_CONTEXT_H_
#define SHIFT_CONTEXT_H_

#include <iostream>
#include <json/json.h>
#include <memory>
#include <string>

#include "shift-core/assembly.h"
#include "shift-core/base.h"
#include "shift-core/joint/base.h"
#include "shift-core/modelparameters.h"

namespace Shift {

namespace Behavior {
class Base;
}

namespace CollisionGeometry {
class Base;
}

namespace Geometry {
class Base;
}

namespace Sensor {
class Base;
}

namespace Terrain {
class Base;
}

namespace Constraint {
class Chord;
}

/*!
@brief Shift Context datastructure
*/
class Context : Base {

public:
  Context(std::string name)
    : Base(name)
    , modelParameters_(name)
    , fixed_("fixed") {
    fixed_.setContext(this);
    initialize();
  }
  Context()
    : Base("")
    , modelParameters_("")
    , fixed_("fixed") {
    initialize();
  }
  ~Context() {

    if(dWRTReference_) {
      free(dWRTReference_);
    }

    if(jointData_) {
      delete jointData_;
    }

    if(q_) {
      delete q_;
    }

    if(ddq_) {
      delete ddq_;
    }
  }

  Context& operator=(Context&&) {
    return *this;
  }
  void readJSON(const std::string&);
  void writeJSON();
  void writeJSON(std::ostream&);
  void writeJSON(const std::string&);
  void print();
  void setFullyQualifiedNames();
  void setActiveObjects();
  void printJSON();
  Body::Base* getFixedBody();

  /*! @brief sort active bodies and joints by level increasing order in spanning
   * tree
   */
  void levelOrderSort();

  /* @brief add a virtual body for each kinematic chord constraint */
  void processVirtualBodies();
  void appendActive(Behavior::Base*);
  void appendActive(Body::Base*);
  void appendActive(CollisionGeometry::Base*);
  void appendActive(Constraint::Base*);
  void appendActive(ForceModule::Base*);
  void appendActive(Geometry::Base*);
  void appendActive(Joint::Base*);
  void appendActive(Light::Base*);
  void appendActive(Sensor::Base*);
  void appendActive(Terrain::Base*);
  void removeActive(Behavior::Base*);
  void removeActive(Body::Base*);
  void removeActive(CollisionGeometry::Base*);
  void removeActive(Constraint::Base*);
  void removeActive(ForceModule::Base*);
  void removeActive(Geometry::Base*);
  void removeActive(Joint::Base*);
  void removeActive(Light::Base*);
  void removeActive(Sensor::Base*);
  void removeActive(Terrain::Base*);
  void setActive(Behavior::Base*);
  void setActive(Body::Base*);
  void setActive(CollisionGeometry::Base*);
  void setActive(Constraint::Base*);
  void setActive(ForceModule::Base*);
  void setActive(Geometry::Base*);
  void setActive(Joint::Base*);
  void setActive(Light::Base*);
  void setActive(Sensor::Base*);
  void setActive(Terrain::Base*);
  void clearActiveBehavior();
  void clearActiveArticulatedBodies();
  void clearActiveFreeBodies();
  void clearActiveCollisionGeometry();
  void clearActiveConstraints();
  void clearActiveForceModules();
  void clearActiveGeometry();
  void clearActiveJoints();
  void clearActiveLights();
  void clearActiveSensors();
  void clearActiveTerrain();
  void clearActiveArrays();
  void connectActiveObjects();
  std::vector<Shift::Joint::Base*>* getActiveJoints() {
    return &activeJoints_;
  }
  /*! get Shift::Behavior::Base by its descriptive name specified in model
   * definition file. This function is intended to be used only to set pointers
   * in models during initial set up.
   * It is not recommended to use this function at run-time */
  Behavior::Base* getBehavior(std::vector<std::string>&);

  /*! get Shift::Body::Base by its descriptive name specified in model
   * definition file. This function is intended to be used only to set pointers
   * in models during initial set up. It is not recommended to use this function
   * at run-time */
  Body::Base* getBody(std::vector<std::string>&);

  /*! get Shift::CollisionGeometry::Base by its descriptive name specified in
   * model definition file. This function is intended to be used only to set
   * pointers in models during initial set up. It is not recommended to use this
   * function at run-time */
  CollisionGeometry::Base* getCollisionGeometry(std::vector<std::string>&);

  /*! get Shift::Constraint::Base by its descriptive name specified in model
   * definition file. This function is intended to be used only to set pointers
   * in models during initial set up. It is not recommended to use this function
   * at run-time */
  Constraint::Base* getConstraint(std::vector<std::string>&);
  ForceModule::Base* getForceModule(std::vector<std::string>&);
  Geometry::Base* getGeometry(std::vector<std::string>&);
  Joint::Base* getJoint(std::vector<std::string>&);
  Light::Base* getLight(std::vector<std::string>&);
  Sensor::Base* getSensor(std::vector<std::string>&);
  Terrain::Base* getTerrain(std::vector<std::string>&);
  inline const ksl_SE3_t* getDWRTReference() const {
    return dWRTReference_;
  }
  inline const double* getJointData() const {
    return jointData_;
  }
  void appendFrame(Shift::Frame* frame) {
    frames_.push_back(frame);
  }
  void updateD(Shift::Joint::DHVA& dhva, const double* q);
  void updateDH(Shift::Joint::DHVA& dhva, const double* q);
  void updateDHVA(Shift::Joint::DHVA& dhva, const double* q, const double* dq);

private:
  void initialize();
  void connect();
  void loadData();
  void processPCL();
  void processLoopMembers();
  void detectSubstructures();
  unsigned int getActiveFrameCount();
  void processFrames();
  void allocateRunTimeArrays();

  Shift::Joint::DHVA dhva_; /*!< displacements, influence coefficient matrices,
                               spatial velocities and quadratic accelerations */
  std::vector<Shift::Joint::Offsets>
    dhvaOffsets_; /*!< offsets into d,dc,h,v,a and dWRTReference array to update
                     kinematics */

  std::vector<Shift::Joint::dh_fptr>
    d_fptr_; /*!< function pointers to update displacements */
  std::vector<Shift::Joint::dh_fptr>
    dh_fptr_; /*!< function pointers to update displacements and influence
                 coefficient matrices */
  std::vector<Shift::Joint::dhva_fptr> dhva_fptr_; /*!< function pointers to
  update displacements, influence coefficient matrices, velocities and quadratic
  accelerations */

  std::vector<int> dReferenceFrames_; /*!< index of frames in
framesInTree that are the reference frame for the body */

  std::vector<int>
    dcq_; /*!< index of framesInTree that require updating direction cosines */

  ksl_SE3_t* dWRTReference_ = NULL; /*!< relative displacement of a body frame
                                       with respect to its reference frame */
  double* jointData_ =
    NULL; /*!< additional data required to process data across joints */

  double* q_ = NULL;
  double* dq_ = NULL;
  double* ddq_ = NULL;
  double* qCopy_ = NULL;

  ModelParameters modelParameters_;
  Assembly fixed_;
  std::string filename_;
  Json::Value root_;
  std::string indentation_ = "  ";

  /*! struct for sorting members of a loop in monotonically level increasing
   * order */
  struct levelOrder {
    levelOrder(Context* context)
      : joints_(context->getActiveJoints()) {}
    const std::vector<Shift::Joint::Base*>* joints_;
    bool operator()(const int& a, const int& b) {
      int a_level = (a == -1) ? 0 : (*joints_)[a]->getLevel();
      int b_level = (b == -1) ? 0 : (*joints_)[b]->getLevel();
      return a_level < b_level;
    }
  };

  /*! struct for sorting loops within a substructure by level order of base node
   */
  struct substructureLoopLevelOrder {
    substructureLoopLevelOrder(Context* context,
                               const std::vector<std::vector<int>>& loopMembers)
      : joints_(context->getActiveJoints())
      , loopMembers_(loopMembers) {}
    const std::vector<Shift::Joint::Base*>* joints_;
    const std::vector<std::vector<int>> loopMembers_;
    bool operator()(const int& a, const int& b) {
      return (*joints_)[loopMembers_[a][0]]->getLevel() <
             (*joints_)[loopMembers_[b][0]]->getLevel();
    }
  };

  /*! struct for sorting substructures by level increasing order */
  struct substructureLevelOrder {
    substructureLevelOrder(Context* context,
                           const std::vector<std::vector<int>>& loopMembers)
      : joints_(context->getActiveJoints())
      , loopMembers_(loopMembers) {}
    const std::vector<Shift::Joint::Base*>* joints_;
    const std::vector<std::vector<int>> loopMembers_;
    bool operator()(const std::vector<int>& a, const std::vector<int>& b) {
      return (*joints_)[loopMembers_[a[0]][0]]->getLevel() <
             (*joints_)[loopMembers_[b[0]][0]]->getLevel();
    }
  };

  std::vector<int> pcl_; /*!< parent child list */
  std::vector<std::vector<int>> loopMembers_;
  std::vector<std::vector<int>> positiveLoopMembers_;
  std::vector<std::vector<int>> negativeLoopMembers_;
  std::vector<int> substructures_;

  std::vector<Behavior::Base*> activeBehaviors_;
  std::vector<Body::Base*> activeArticulatedBodies_ = {};
  std::vector<Body::Base*> activeFreeBodies_ = {};
  std::vector<CollisionGeometry::Base*> activeCollisionGeometry_;
  std::vector<Constraint::Base*> activeConstraints_;
  std::vector<ForceModule::Base*> activeForceModules_;
  std::vector<Geometry::Base*> activeGeometry_;
  std::vector<Joint::Base*> activeJoints_;
  std::vector<Light::Base*> activeLights_;
  std::vector<Sensor::Base*> activeSensors_;
  std::vector<Terrain::Base*> activeTerrain_;
  std::vector<Constraint::Chord*> chordConstraints_;

  std::vector<Shift::Frame*> frames_;
  std::vector<Shift::Frame*> framesInTree_;
  std::vector<Shift::Frame*> activeFrames_;

  /* Manage objects that are automatically created when processing system
   * topology; these objects are not loaded from an external file.*/
  std::vector<std::unique_ptr<Behavior::Base>> behaviors_;
  std::vector<std::unique_ptr<Body::Base>> bodies_;
  std::vector<std::unique_ptr<Constraint::Base>> constraints_;
  std::vector<std::unique_ptr<ForceModule::Base>> forcemodules_;
  std::vector<std::unique_ptr<Joint::Base>> joints_;

  void updateDC(Shift::Joint::DHVA& dhva, const double* q);
  void updateD();
  void updateDH();
  void updateDHVA();

}; // namespace Shift

} // namespace Shift

#endif

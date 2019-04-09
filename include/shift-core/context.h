#ifndef SHIFT_CONTEXT_H_
#define SHIFT_CONTEXT_H_

#include <iostream>
#include <json/json.h>
#include <memory>
#include <string>

#include "shift-core/assembly.h"
#include "shift-core/base.h"
#include "shift-core/jacobian.h"
#include "shift-core/joint/base.h"
#include "shift-core/modelparameters.h"
#include "shift-core/topology.h"

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
    , jacobian_(this)
    , modelParameters_(this)
    , fixed_("fixed") {
    fixed_.setContext(this);
    initialize();
  }
  Context()
    : Base("")
    , jacobian_(this)
    , modelParameters_(this)
    , fixed_("fixed") {
    initialize();
  }
  ~Context() {

    if(dWRTReference_) {
      free(dWRTReference_);
    }

    if(jointData_) {
      delete[] jointData_;
    }

    if(q_) {
      delete[] q_;
    }

    if(ddq_) {
      delete[] ddq_;
    }

    if(deltaq_) {
      delete[] deltaq_;
    }

    if(qCopy_) {
      delete[] qCopy_;
    }

    if(f_) {
      delete[] f_;
    }

    if(g_) {
      delete[] g_;
    }

    if(k_) {
      delete[] k_;
    }

    if(inertia_) {
      free(inertia_);
    }

    if(inertiaP_) {
      free(inertiaP_);
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
  Body::Base* fixedBody();

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
  unsigned int qCount();
  unsigned int dqCount();

  const std::vector<int>& dqLookup() {
    return dqLookup_;
  }

  const std::vector<int>& qLookup() {
    return qLookup_;
  }

  const std::vector<int>& jointLookup() {
    return jointLookup_;
  }

  const std::vector<Shift::Joint::Base*>& activeJoints() const {
    return activeJoints_;
  }

  unsigned int nonRigidJointCount() const {
    return nonRigidJointCount_;
  }

  unsigned int substructureCount() const {
    return substructureIndices_.size();
  }

  const std::vector<std::vector<int>>& substructureIndices() const {
    return substructureIndices_;
  }

  const std::vector<std::vector<int>>& substructureMembers() const {
    return substructureMembers_;
  }

  const std::vector<std::vector<int>>& loopMembers() const {
    return loopMembers_;
  }

  const std::vector<std::vector<int>>& positiveLoopMembers() const {
    return positiveLoopMembers_;
  }

  const std::vector<std::vector<int>>& negativeLoopMembers() const {
    return negativeLoopMembers_;
  }

  /*! \brief get Shift::Behavior::Base by its descriptive name specified in
   * model definition file. This function is intended to be used only to set
   * pointers in models during initial set up. It is not recommended to use this
   * function at run-time */
  Behavior::Base* getBehavior(std::vector<std::string>&);

  /*! get Shift::Body::Base by its descriptive name specified in model
   * definition file. This function is intended to be used only to set pointers
   * in models during initial set up. It is not recommended to use this function
   * at run-time */
  Body::Base* getBody(std::vector<std::string>&);

  /*! \brief get Shift::CollisionGeometry::Base by its descriptive name
   * specified in model definition file. This function is intended to be used
   * only to set pointers in models during initial set up. It is not recommended
   * to use this function at run-time */
  CollisionGeometry::Base* getCollisionGeometry(std::vector<std::string>&);

  /*! \brief get Shift::Constraint::Base by its descriptive name specified in
   * model definition file. This function is intended to be used only to set
   * pointers in models during initial set up. It is not recommended to use this
   * function at run-time */
  Constraint::Base* getConstraint(std::vector<std::string>&);

  /*!\brief  get Shift::ForceModule::Base by its descriptive name specified in
   * model definition file. This function is intended to be used only to set
   * pointers in models during initial set up. It is not recommended to use this
   * function at run-time */
  ForceModule::Base* getForceModule(std::vector<std::string>&);

  /*! \brief get Shift::Geometry::Base by its descriptive name specified in
   * model definition file. This function is intended to be used only to set
   * pointers in models during initial set up. It is not recommended to use this
   * function at run-time */
  Geometry::Base* getGeometry(std::vector<std::string>&);

  /*! \brief get Shift::Joint::Base by its descriptive name specified in model
   * definition file. This function is intended to be used only to set pointers
   * in models during initial set up. It is not recommended to use this function
   * at run-time */
  Joint::Base* getJoint(std::vector<std::string>&);

  /*! get Shift::Light::Base by its descriptive name specified in model
   * definition file. This function is intended to be used only to set pointers
   * in models during initial set up. It is not recommended to use this function
   * at run-time */
  Light::Base* getLight(std::vector<std::string>&);

  /*! \brief get Shift::Sensor::Base by its descriptive name specified in model
   * definition file. This function is intended to be used only to set pointers
   * in models during initial set up. It is not recommended to use this function
   * at run-time */
  Sensor::Base* getSensor(std::vector<std::string>&);

  /*! \brief get Shift::Terrain::Base by its descriptive name specified in model
   * definition file. This function is intended to be used only to set pointers
   * in models during initial set up. It is not recommended to use this function
   * at run-time */
  Terrain::Base* getTerrain(std::vector<std::string>&);

  /*! \brief get pointer to dWRTReference, an array of relative transformations
   * to their respective reference frames */
  inline const ksl_SE3_t* DWRTReference() const {
    return dWRTReference_;
  }

  /*! \brief get pointer to jointData, an array of additional constant data
   * required to process some joints */
  inline const double* jointData() const {
    return jointData_;
  }

  const Shift::ModelParameters& modelParameters() {
    return modelParameters_;
  }

  /*! \brief append frame to Shift::Context's list of frames
   */
  void appendFrame(Shift::Frame* frame) {
    frames_.push_back(frame);
  }

  /*! \brief get count of total number of active frames */
  unsigned int activeFrameCount() {
    return activeFrames_.size();
  }

  /*! \brief get count of total number of active joints */
  unsigned int activeJointCount() {
    return activeJoints_.size();
  }

  /*! \brief get count of active chord constraints */
  unsigned int chordConstraintCount() {
    return chordConstraints_.size();
  }

  double* dq() {
    return dq_;
  }

  double* q() {
    return q_;
  }

  double* deltaq() {
    return deltaq_;
  }

  const Topology::DHVA& dhva() const {
    return dhva_;
  }

  /*! \brief Update displacements in a Shift::Joint::DHVA data structure using a
   * user-provided array of generalized coordinates. The instructions to update
   * displacements are stored in Shift::Context */
  void updateD(Topology::DHVA& dhva, const double* q);

  /*! \brief Update displacements and influence coefficient matrices in a
   * Shift::Joint::DHVA data structure using a user-provided array of
   * generalized coordinates. The instructions to update displacements and
   * influence coefficient matrices are stored in Shift::Context */
  void updateDH(Topology::DHVA& dhva, const double* q);

  /*! \brief Update displacements, influence coefficient matrices, velocities
   * and quadratic accelerations in a Shift::Joint::DHVA data structure using a
   * user-provided array of generalized coordinates and generalized velocities.
   * The instructions to update DHVA are stored in Shift::Context */
  void updateDHVA(Topology::DHVA& dhva, const double* q, const double* dq);

  /*! \brief set q and dq arrays to intial conditions saved in each joint */
  void setToInitialConditions();

  /*! \brief extract current generalized displacements and velocities stored in
   * q and dq arrays and use to override initial conditions stored in joints */
  void setInitialConditionsToCurrentConditions();

private:
  /*! \brief Shift::Context initialization function, called in Shift::Context
   * constructor */
  void initialize();

  /*! \brief connect pointers between objects managed by Shift::Context */
  void connect();

  /*! \brief process all Shiift objectss loadData() functions, which loads any
   * additional external data not specified in JSON model definition file from
   * other data sources */
  void loadData();

  /*! \brief sort active bodies and joints by level increasing order in spanning
   * tree
   */
  void levelOrderSort();

  /*! \brief add a virtual body for each kinematic chord constraint */
  void processVirtualBodies();

  /*! \brief generate parent-child list */
  void processPCL();

  /*! \brief identify loop members */
  void processLoopMembers();

  /*! \brief detect overlapping clusters of loops (i.e. substructures) if they
   * exist */
  void detectSubstructures();

  /*! \brief identify frames in spanning tree and optimize rigid joints out of
   * model */
  void processFrames();

  /*! \brief allocate arrays used at run-time used to process kinematics and
   * store quantities for solving the equations of motion. set up function
   * pointers required to update displacements, influence coefficient matrices,
   * velocities and accelerations. */
  void allocateRunTimeArrays();

  /*! \brief set -g for the fixed frame in the array of quadratic accelerations.
   * After updating DHVA, this effectively applies gravitational forces to all
   * bodies without having to explicitly add gravitational forces to individual
   * bodies. */
  void setFixedFrameGravityVector();

  void compressRigidJoints(int i, // current body Index and parent joint index
                           unsigned int& operationCount);

  /*! \brief initialize spatial inertias for articulated bodies
   */
  void initializeInertia();

  void setLookup();

  const std::vector<Shift::Joint::Base*>* activeJointsPtr() {
    return &activeJoints_;
  }

  Shift::Topology::DHVA
    dhva_; /*!< displacements, influence coefficient matrices,
           spatial velocities and quadratic accelerations */
  std::vector<Topology::Offsets>
    dhvaOffsets_; /*!< offsets into d,dc,h,v,a and dWRTReference array to
                     update kinematics */

  std::vector<Topology::dh_fptr>
    d_fptr_; /*!< function pointers to update displacements */
  std::vector<Topology::dh_fptr>
    dh_fptr_; /*!< function pointers to update displacements and influence
                 coefficient matrices */
  std::vector<Topology::dhva_fptr> dhva_fptr_; /*!< function pointers to
  update displacements, influence coefficient matrices, velocities and
  quadratic accelerations */

  std::vector<int> dReferenceFrames_; /*!< index of frames in
framesInTree that are the reference frame for the body */

  std::vector<int> dcq_; /*!< index of framesInTree that require updating
                            direction cosines */

  ksl_SE3_t* dWRTReference_ = NULL; /*!< relative displacement of a body frame
                                       with respect to its reference frame */
  double* jointData_ =
    NULL; /*!< additional data required to process data across joints */

  double* q_ = NULL;   /*!< generalized coordinates */
  double* dq_ = NULL;  /*!< generalized coordinate first time derivative, or
                          generalized velocities */
  double* ddq_ = NULL; /*!< generalized coordinate second time derivative, or
                          generalized accelerations */
  double* deltaq_ = NULL;

  double* qCopy_ =
    NULL; /*!< copy of q array, used to restore q to a previous state */
  double* f_ = NULL; /*!< Working forces within a joint, such as from friction
                       or damping */
  ksl_screw_t* g_ =
    NULL; /*!< spatial body and inertial forces, length equal to dq count */
  ksl_screw_t* k_ = NULL;          /*!< joint reaction forces */
  ksl_inertia_t* inertia_ = NULL;  /*!< body fixed spatial  inertias */
  ksl_inertia_t* inertiaP_ = NULL; /*!< projected spatial inertias */
  std::vector<int> jointDOF_;
  std::vector<int> jointDOFCompressed_;
  std::vector<int> jointLookup_;
  std::vector<int> dqLookup_;
  std::vector<int> qLookup_;
  std::vector<int> dqLookupCompressed_;
  Shift::Jacobian jacobian_;

  ksl_screw_t spatialMomentum_ = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t barycentricSpatialMomentum_ =
    ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  double kineticEnergy_ = 0.0;
  double gravitationalPotentialEnergy_ = 0.0;

  ModelParameters modelParameters_;
  Assembly fixed_;
  std::string filename_;
  Json::Value root_;
  std::string indentation_ = "  ";

  /*! \brief struct for sorting members of a loop in monotonically level
   * increasing order */
  struct levelOrder {
    levelOrder(Context* context)
      : joints_(context->activeJointsPtr()) {}
    const std::vector<Shift::Joint::Base*>* joints_;
    bool operator()(const int& a, const int& b) {
      int a_level = (a == -1) ? 0 : (*joints_)[a]->getLevel();
      int b_level = (b == -1) ? 0 : (*joints_)[b]->getLevel();
      return a_level < b_level;
    }
  };

  /*! \brief struct for sorting loops within a substructure by level order of
   * base node
   */
  struct substructureLoopLevelOrder {
    substructureLoopLevelOrder(Context* context,
                               const std::vector<std::vector<int>>& loopMembers)
      : joints_(context->activeJointsPtr())
      , loopMembers_(loopMembers) {}
    const std::vector<Shift::Joint::Base*>* joints_;
    const std::vector<std::vector<int>> loopMembers_;
    bool operator()(const int& a, const int& b) {
      return (*joints_)[loopMembers_[a][0]]->getLevel() <
             (*joints_)[loopMembers_[b][0]]->getLevel();
    }
  };

  /*! \brief struct for sorting substructures by level increasing order */
  struct substructureLevelOrder {
    substructureLevelOrder(Context* context,
                           const std::vector<std::vector<int>>& loopMembers)
      : joints_(context->activeJointsPtr())
      , loopMembers_(loopMembers) {}
    const std::vector<Shift::Joint::Base*>* joints_;
    const std::vector<std::vector<int>> loopMembers_;
    bool operator()(const std::vector<int>& a, const std::vector<int>& b) {
      return (*joints_)[loopMembers_[a[0]][0]]->getLevel() <
             (*joints_)[loopMembers_[b[0]][0]]->getLevel();
    }
  };

  unsigned int nonRigidJointCount_ = 0;
  std::vector<int> pcl_; /*!< parent child list */
  std::vector<std::vector<int>> loopMembers_;
  std::vector<std::vector<int>> positiveLoopMembers_;
  std::vector<std::vector<int>> negativeLoopMembers_;
  std::vector<std::vector<int>> substructureIndices_;
  std::vector<std::vector<int>> substructureMembers_;

  /*! \defgroup active_objects Active objects managed by Shift::Context

  The following lists of objects contain pointers to only the objects whose
  active flag is set to true.

  @{ */
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

  std::vector<Shift::Frame*> frames_;
  std::vector<Shift::Frame*> framesInTree_;
  std::vector<Shift::Frame*> activeFrames_;

  /*! @{ */

  /*! \defgroup managed_objects Objects managed by Shift::Context

  These objects are created automatically when processing system
  topology and  are not loaded from an external file.

  @{ */

  /*! \brief System generated behaviors */
  std::vector<std::unique_ptr<Behavior::Base>> behaviors_;

  /*! \brief System generated bodies */
  std::vector<std::unique_ptr<Body::Base>> bodies_;

  /*! \brief System generated constraints */
  std::vector<std::unique_ptr<Constraint::Base>> constraints_;

  /*! \brief A subset of constraints used in model corresponding to
   * constraints imposed for kinematic chord loops */
  std::vector<Constraint::Chord*> chordConstraints_;

  /*! \brief System generated force modules */
  std::vector<std::unique_ptr<ForceModule::Base>> forcemodules_;

  /*! \brief System generated joints */
  std::vector<std::unique_ptr<Joint::Base>> joints_;

  /*! @} */

  /*! \brief Update direction cosines in model */
  void updateDC(Topology::DHVA& dhva, const double* q);
  void updateD();
  void updateDH();
  void updateDHVA();

  unsigned int qCount_ = 0;
  unsigned int dqCount_ = 0;

}; // namespace Shift

} // namespace Shift

#endif

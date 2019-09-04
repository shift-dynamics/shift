#ifndef SHIFT_CONTEXT_H_
#define SHIFT_CONTEXT_H_

#include <iostream>
#include <json/json.h>
#include <memory>
#include <string>

#include "shift-core/assembly.h"
#include "shift-core/base.h"
#include "shift-core/body/fixed.h"
#include "shift-core/factory.h"
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
class Context : public Shift::Body::Fixed {

public:
  Context(std::string)
    // : Shift::Parameter::List()
    : Shift::Body::Fixed("fixed")
    , jacobian_(this)
    , model_parameters_(this) {
    addParameters();
    initialize();
  }
  Context()
    : Shift::Body::Fixed("fixed")
    , jacobian_(this)
    , model_parameters_(this) {
    addParameters();
    initialize();
  }
  virtual ~Context() override {

    if(d_wrt_reference_) {
      free(d_wrt_reference_);
    }

    if(joint_data_) {
      delete[] joint_data_;
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

    if(q_copy_) {
      delete[] q_copy_;
    }

    if(f_) {
      delete[] f_;
    }

    if(p_) {
      delete[] p_;
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

    if(mobility_numbers_) {
      free(mobility_numbers_);
    }

    std::cout << "virtual ~Context() override" << std::endl;
  }

  Context& operator=(Context&&) {
    return *this;
  }

  void readJSON(const std::string);
  void writeJSON();
  void writeJSON(std::ostream&);
  void writeJSON(const std::string&);
  void printJSON();

  Shift::Body::Base* fixedBody() {
    return this;
  }

  void postLoad(const std::string);

  void setFullyQualifiedNames();
  virtual void setActiveObjects(bool) override;

  void setActive(Behavior::Base*, bool);
  void setActive(Body::Base*, bool);
  void setActive(CollisionGeometry::Base*, bool);
  void setActive(Constraint::Base*, bool);
  void setActive(ForceModule::Base*, bool);
  void setActive(Geometry::Base*, bool);
  void setActive(Joint::Base*, bool);
  void setActive(Light::Base*, bool);
  void setActive(Sensor::Base*, bool);
  void setActive(Terrain::Base*, bool);

  void setActive(Integrator::Base*, bool);
  void setActive(Solver::Base*, bool);
  void setActive(Logger::Base*, bool);

  void connectActiveObjects();
  unsigned int qCount();
  unsigned int dqCount();

  double gravitationalPotentialEnergy();
  double kineticEnergy();

  const std::vector<int>& dqLookup() {
    return dq_lookup_;
  }

  const std::vector<int>& qLookup() {
    return q_lookup_;
  }

  const std::vector<int>& jointLookup() {
    return joint_lookup_;
  }

  const std::vector<Shift::Joint::Base*>& activeJoints() const {
    return active_joints_;
  }

  const std::vector<int>& pcl() const {
    return pcl_;
  }

  const std::vector<Shift::Body::Base*>& activeArticulatedBodies() const {
    return active_articulated_bodies_;
  }

  const std::vector<Shift::Frame*>& framesInTree() const {
    return frames_in_tree_;
  }

  unsigned int nonRigidJointCount() const {
    return non_rigid_joint_count_;
  }

  unsigned int substructureCount() const {
    return substructure_indices_.size();
  }

  const std::vector<std::vector<int>>& substructureIndices() const {
    return substructure_indices_;
  }

  const std::vector<std::vector<int>>& substructureMembers() const {
    return substructure_members_;
  }

  const std::vector<std::vector<int>>& loopMembers() const {
    return loop_members_;
  }

  const std::vector<std::vector<int>>& positiveLoopMembers() const {
    return positive_loop_members_;
  }

  const std::vector<std::vector<int>>& negativeLoopMembers() const {
    return negative_loop_members_;
  }

  /*! \brief get pointer to dWRTReference, an array of relative
   * transformations to their respective reference frames */
  inline const ksl_SE3_t* DWRTReference() const {
    return d_wrt_reference_;
  }

  /*! \brief get pointer to jointData, an array of additional constant data
   * required to process some joints */
  inline const double* jointData() const {
    return joint_data_;
  }

  const Shift::ModelParameters& modelParameters() {
    return model_parameters_;
  }

  /*! \brief append frame to Shift::Context's list of frames
   */
  // void appendFrame(Shift::Frame* frame) {
  //   frames_.push_back(frame);
  // }

  /*! \brief get count of total number of active frames */
  unsigned int activeFrameCount() {
    return active_frames_.size();
  }

  /*! \brief get count of total number of active joints */
  unsigned int activeJointCount() const {
    return active_joints_.size();
  }

  /*! \brief get count of active chord constraints */
  unsigned int chordConstraintCount() const {
    return chord_constraints_.size();
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

  const std::vector<Topology::Offsets>& dhvaOffsets() const {
    return dhva_offsets_;
  }

  double* mobilityNumbers() {
    return mobility_numbers_;
  }

  /*! \brief Update displacements in a Shift::Joint::DHVA data structure using
   * a user-provided array of generalized coordinates. The instructions to
   * update displacements are stored in Shift::Context */
  void updateD(Topology::DHVA& dhva, const double* q);

  /*! \brief Update displacements and influence coefficient matrices in a
   * Shift::Joint::DHVA data structure using a user-provided array of
   * generalized coordinates. The instructions to update displacements and
   * influence coefficient matrices are stored in Shift::Context */
  void updateDH(Topology::DHVA& dhva, const double* q);

  /*! \brief Update displacements, influence coefficient matrices, velocities
   * and quadratic accelerations in a Shift::Joint::DHVA data structure using
   * a user-provided array of generalized coordinates and generalized
   * velocities.
   * The instructions to update DHVA are stored in Shift::Context */
  void updateDHVA(Topology::DHVA& dhva, const double* q, const double* dq);

  void updateD();
  void updateDH();
  void updateDHVA();

  /*! \brief set q and dq arrays to intial conditions saved in each joint */
  void setToInitialConditions();

  /*! \brief extract current generalized displacements and velocities stored
   * in q and dq arrays and use to override initial conditions stored in
   * joints */
  void setInitialConditionsToCurrentConditions();

  const std::vector<Constraint::Chord*>& chordConstraints() const {
    return chord_constraints_;
  }

  double globalMobility();

  int digraph(std::string filename);

  int digraphFrames(std::string filename);

private:
  virtual void addParameters() override;

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
  void appendActive(Solver::Base*);
  void appendActive(Integrator::Base*);
  void appendActive(Logger::Base*);
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
  void removeActive(Solver::Base*);
  void removeActive(Integrator::Base*);
  void removeActive(Logger::Base*);
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

  /*! \brief Shift::Context initialization function, called in Shift::Context
   * constructor */
  void initialize();

  /*! \brief connect pointers between objects managed by Shift::Context */
  void connect() override;

  /*! \brief process all Shiift objectss loadData() functions, which loads any
   * additional external data not specified in JSON model definition file from
   * other data sources */
  void loadData();

  /*! \brief sort active bodies and joints by level increasing order in
   * spanning tree
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
   * pointers required to update displacements, influence coefficient
   * matrices, velocities and accelerations. */
  void allocateRunTimeArrays();

  /*! \brief set -g for the fixed frame in the array of quadratic
   * accelerations. After updating DHVA, this effectively applies
   * gravitational forces to all bodies without having to explicitly add
   * gravitational forces to individual bodies. */
  void setFixedFrameGravityVector();

  void compressRigidJoints(int i, // current body Index and parent joint index
                           unsigned int& operationCount);

  /*! \brief initialize spatial inertias for articulated bodies
   */
  void initializeInertia();

  void setLookup();

  /*! \brief User defined terrain attached to fixed inertial reference frame
   */
  Shift::Parameter::Array<Shift::Assembly> assemblies_;

  const std::vector<Shift::Joint::Base*>* activeJointsPtr() {
    return &active_joints_;
  }

  Shift::Topology::DHVA
    dhva_; /*!< displacements, influence coefficient matrices,
           spatial velocities and quadratic accelerations */
  std::vector<Topology::Offsets>
    dhva_offsets_; /*!< offsets into d,dc,h,v,a and dWRTReference array to
                     update kinematics */

  std::vector<Topology::dh_fptr>
    d_fptr_; /*!< function pointers to update displacements */
  std::vector<Topology::dh_fptr>
    dh_fptr_; /*!< function pointers to update displacements and influence
                 coefficient matrices */
  std::vector<Topology::dhva_fptr> dhva_fptr_; /*!< function pointers to
  update displacements, influence coefficient matrices, velocities and
  quadratic accelerations */

  std::vector<int> d_reference_frames_; /*!< index of frames in
framesInTree that are the reference frame for the body */

  std::vector<int> dcq_; /*!< index of framesInTree that require updating
                            direction cosines */

  ksl_SE3_t* d_wrt_reference_ =
    nullptr; /*!< relative displacement of a body frame
             with respect to its reference frame */
  double* joint_data_ =
    nullptr; /*!< additional data required to process data across joints */

  double* q_ = nullptr;   /*!< generalized coordinates */
  double* dq_ = nullptr;  /*!< generalized coordinate first time derivative, or
                          generalized velocities */
  double* ddq_ = nullptr; /*!< generalized coordinate second time derivative,
                          or generalized accelerations */
  double* deltaq_ = nullptr;

  double* q_copy_ =
    nullptr; /*!< copy of q array, used to restore q to a previous state */
  double* f_ = nullptr;        /*!< Working forces within a joint, such as from
                              friction        or damping */
  ksl_coscrew_t* p_ = nullptr; /*< momentum */
  ksl_coscrew_t* g_ = nullptr; /*!< spatial body and inertial forces, length
                                  equal to dq count */
  ksl_coscrew_t* k_ = nullptr; /*!< joint reaction forces */
  ksl_inertia_t* inertia_ = nullptr;  /*!< body fixed spatial  inertias */
  ksl_inertia_t* inertiaP_ = nullptr; /*!< projected spatial inertias */
  std::vector<int> joint_dof_;
  std::vector<int> joint_dof_compressed_;
  std::vector<int> joint_lookup_;
  std::vector<int> dq_lookup_;
  std::vector<int> q_lookup_;
  std::vector<int> dq_lookup_compressed_;
  Shift::Jacobian jacobian_;

  ksl_screw_t spatial_momentum_ = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ksl_screw_t barycentric_spatial_momentum_ =
    ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  double kinetic_energy_ = 0.0;

  std::string filename_;
  Json::Value root_;
  std::string indentation_ = "  ";

  /*! \brief Parameters used to configure a simulation */
  ModelParameters model_parameters_;

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
    substructureLoopLevelOrder(
      Context* context, const std::vector<std::vector<int>>& loop_members)
      : joints_(context->activeJointsPtr())
      , loop_members_(loop_members) {}
    const std::vector<Shift::Joint::Base*>* joints_;
    const std::vector<std::vector<int>> loop_members_;
    bool operator()(const int& a, const int& b) {
      return (*joints_)[loop_members_[a][0]]->getLevel() <
             (*joints_)[loop_members_[b][0]]->getLevel();
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

  unsigned int non_rigid_joint_count_ = 0;
  std::vector<int> pcl_; /*!< parent child list */
  std::vector<std::vector<int>> loop_members_;
  std::vector<std::vector<int>> positive_loop_members_;
  std::vector<std::vector<int>> negative_loop_members_;
  std::vector<std::vector<int>> substructure_indices_;
  std::vector<std::vector<int>> substructure_members_;

  /*! \defgroup active_objects Active objects managed by Shift::Context

  The following lists of objects contain pointers to only the objects whose
  active flag is set to true.

  @{ */
  std::vector<Behavior::Base*> active_behaviors_ = {};
  std::vector<Body::Base*> active_articulated_bodies_ = {};
  std::vector<Body::Base*> active_free_bodies_ = {};
  std::vector<CollisionGeometry::Base*> active_collision_geometry_ = {};
  std::vector<Constraint::Base*> active_constraints_ = {};
  std::vector<ForceModule::Base*> active_force_modules_ = {};
  std::vector<Geometry::Base*> active_geometry_ = {};
  std::vector<Joint::Base*> active_joints_ = {};
  std::vector<Light::Base*> active_lights_ = {};
  std::vector<Sensor::Base*> active_sensors_ = {};
  std::vector<Terrain::Base*> active_terrain_ = {};

  std::vector<Integrator::Base*> active_integrators_ = {};
  std::vector<Logger::Base*> active_loggers_ = {};
  std::vector<Solver::Base*> active_solvers_ = {};

  std::vector<Shift::Frame*> frames_in_tree_ = {};
  std::vector<Shift::Frame*> active_frames_ = {};

  /*! @{ */

  /*! \defgroup managed_objects Objects managed by Shift::Context

  These objects are created automatically when processing system
  topology and  are not loaded from an external file.

  @{ */
  /*! \brief System generated bodies */
  std::vector<std::unique_ptr<Body::Base>> bodies_ = {};

  /*! \brief System generated behaviors */
  std::vector<std::unique_ptr<Behavior::Base>> behaviors_ = {};

  /*! \brief System generated constraints */
  std::vector<std::unique_ptr<Constraint::Base>> constraints_ = {};

  /*! \brief System generated force modules */
  std::vector<std::unique_ptr<ForceModule::Base>> force_modules_ = {};

  /*! \brief System generated joints */
  std::vector<std::unique_ptr<Joint::Base>> joints_ = {};

  /*! \brief A subset of constraints used in model corresponding to
   * constraints imposed for kinematic chord loops */
  std::vector<Constraint::Chord*> chord_constraints_ = {};

  /*! @} */

  double* mobility_numbers_ = nullptr;

  void initializeMobility();

  /*! \brief Update direction cosines in model */
  void updateDC(Topology::DHVA& dhva, const double* q);

  unsigned int q_count_ = 0;
  unsigned int dq_count_ = 0;
  double mobility_ = 0.0;

}; // namespace Shift

} // namespace Shift

#endif

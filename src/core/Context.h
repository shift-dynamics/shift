#ifndef SHIFT_CONTEXT_H_
#define SHIFT_CONTEXT_H_

#include <iostream>
#include <json/json.h>
#include <memory>
#include <string>

#include "shift/core/Assembly.h"
#include "shift/core/Base.h"
#include "shift/core/body/Fixed.h"
#include "shift/core/Factory.h"
#include "shift/core/Jacobian.h"
#include "shift/core/joint/Base.h"
#include "shift/core/ModelParameters.h"
#include "shift/core/Topology.h"

namespace shift {
namespace core {

namespace behavior {
class Base;
}

namespace collisiongeometry {
class Base;
}

namespace geometry {
class Base;
}

namespace sensor {
class Base;
}

namespace terrain {
class Base;
}

namespace constraint {
class Chord;
}

/*!
@brief Shift Context datastructure
*/
class Context : public body::Fixed {

public:
  Context(std::string)
    // : shift::core::parameter::List()
    : body::Fixed("fixed")
    , jacobian_(this)
    , model_parameters_(this) {
    addParameters();
    initialize();
  }
  Context()
    : body::Fixed("fixed")
    , jacobian_(this)
    , model_parameters_(this) {
    addParameters();
    initialize();
  }
  virtual ~Context() override {}

  Context& operator=(Context&&) {
    return *this;
  }

  void loadJSON(const std::string);
  void writeJSON();
  void writeJSON(std::ostream&);
  void writeJSON(const std::string&);
  void printJSON();

  body::Base* fixedBody() {
    return this;
  }

  void postLoad(const std::string);

  void setFullyQualifiedNames();
  virtual void setActiveObjects(bool) override;

  void setActive(behavior::Base*, bool);
  void setActive(body::Base*, bool);
  void setActive(collisiongeometry::Base*, bool);
  void setActive(constraint::Base*, bool);
  void setActive(forcemodule::Base*, bool);
  void setActive(geometry::Base*, bool);
  void setActive(joint::Base*, bool);
  void setActive(light::Base*, bool);
  void setActive(sensor::Base*, bool);
  void setActive(terrain::Base*, bool);

  void setActive(integrator::Base*, bool);
  void setActive(solver::Base*, bool);
  void setActive(logger::Base*, bool);

  void connectActiveObjects();
  unsigned int qCount();
  unsigned int dqCount();

  double gravitationalPotentialEnergy();
  double kineticEnergy();

  const std::vector<int>& dqLookup() const {
    return dq_lookup_;
  }

  const std::vector<int>& dqLookupCompressed() const {
    return dq_lookup_compressed_;
  }

  const std::vector<bool>& hasMassdq() const {
    return has_mass_dq_;
  }

  const std::vector<bool>& hasMassArray() const {
    return has_mass_;
  }

  const std::vector<int>& qLookup() const {
    return q_lookup_;
  }

  const std::vector<int>& jointLookup() const {
    return joint_lookup_;
  }

  const std::vector<shift::core::joint::Base*>& activeJoints() const {
    return active_joints_;
  }

  const std::vector<shift::core::forcemodule::Base*>& activeforcemodules() const {
    return active_force_modules_;
  }

  const std::vector<int>& pcl() const {
    return pcl_;
  }

  const std::vector<int>& pclCompressed() const {
    return pcl_compressed_;
  }

  const std::vector<shift::core::body::Base*>& activeArticulatedBodies() const {
    return active_articulated_bodies_;
  }

  const std::vector<shift::core::Frame*>& framesInTree() const {
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
  inline const std::vector<ksl_SE3_t>& DWRTReference() const {
    return d_wrt_reference_;
  }

  /*! \brief get pointer to jointData, an array of additional constant data
   * required to process some joints */
  inline const std::vector<double>& jointData() const {
    return joint_data_;
  }

  const ModelParameters& modelParameters() {
    return model_parameters_;
  }

  /*! \brief append frame to shift::core::Context's list of frames
   */
  // void appendFrame(shift::core::Frame* frame) {
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

  unsigned int constraintEquationCount() const {
    return chord_constraints_.size() * 6;
  }

  std::vector<double>& ddq() {
    return ddq_;
  }

  double* dq() {
    return dq_;
  }

  std::vector<double>& q() {
    return q_;
  }

  std::vector<double>& deltaq() {
    return delta_q_;
  }

  const topology::DHVA& dhva() const {
    return dhva_;
  }

  const std::vector<topology::Offsets>& dhvaOffsets() const {
    return dhva_offsets_;
  }

  std::vector<double>& mobilityNumbers() {
    return mobility_numbers_;
  }

  /*! \brief Update displacements in a shift::core::joint::DHVA data structure using
   * a user-provided array of generalized coordinates. The instructions to
   * update displacements are stored in shift::core::Context */
  void updateD(topology::DHVA& dhva, const double* q);

  /*! \brief Update displacements and influence coefficient matrices in a
   * shift::core::joint::DHVA data structure using a user-provided array of
   * generalized coordinates. The instructions to update displacements and
   * influence coefficient matrices are stored in shift::core::Context */
  void updateDH(topology::DHVA& dhva, const double* q);

  /*! \brief Update displacements, influence coefficient matrices, velocities
   * and quadratic accelerations in a shift::core::joint::DHVA data structure using
   * a user-provided array of generalized coordinates and generalized
   * velocities.
   * The instructions to update DHVA are stored in shift::core::Context */
  void updateDHVA(topology::DHVA& dhva, const double* q, const double* dq);

  /*! \brief Update displacements stored in a shift::core::Context data structure
   * using a user-provided array of generalized coordinates. The instructions to
   * update displacements are stored in shift::core::Context */
  void updateD(const double* q);

  /*! \brief Update displacements and influence coefficient matrices in a
   * shift::core::Context data structure using a user-provided array of
   * generalized coordinates. The instructions to update displacements and
   * influence coefficient matrices are stored in shift::core::Context */
  void updateDH(const double* q);

  /*! \brief Update displacements, influence coefficient matrices, velocities
   * and quadratic accelerations in a shift::core::Context data structure using
   * a user-provided array of generalized coordinates and generalized
   * velocities.
   * The instructions to update DHVA are stored in shift::core::Context */
  void updateDHVA(const double* q, const double* dq);

  void updateD();
  void updateDH();
  void updateDHVA();

  /*! \brief set q and dq arrays to intial conditions saved in each joint */
  void setToInitialConditions();

  /*! \brief extract current generalized displacements and velocities stored
   * in q and dq arrays and use to override initial conditions stored in
   * joints */
  void setInitialConditionsToCurrentConditions();

  const std::vector<constraint::Chord*>& chordConstraints() const {
    return chord_constraints_;
  }

  double globalMobility();

  int digraph(std::string filename);
  int digraph();

  int digraphFrames(std::string filename);
  int digraphFrames();

  unsigned int forceModuleEquationCount();

  void evaluateConstraints();
  double constraintError();

  void setForces(const double t, const std::vector<double>& y);

  /*! copies dq and ddq quantities to dy array using vectors quaternion_indices_
  and non_quaternion_indices_; for quaternion_indices_, converts angular
  velocities to time derivative of quaternions */
  void dqTody(const std::vector<double>& y, std::vector<double>& dy);

  bool newtonRaphsonFailure() {
    return jacobian_.newtonRaphsonFailure();
  }

  shift::core::Jacobian& jacobian() {
    return jacobian_;
  }

  const std::vector<int>& dReferenceFrames() const {
    return d_reference_frames_;
  }

  const std::vector<ksl_inertia_t>& inertiaArray() const {
    return inertia_;
  }

  std::vector<double>& jointWorkingForces() {
    return f_;
  }

  const std::vector<bool>& isRotation() const {
    return is_rotation_;
  }

  const std::vector<int>& pcldq() const {
    return pcldq_;
  }

  const std::vector<unsigned int>& jointDOFCompressed() const {
    return joint_dof_compressed_;
  }

  void simulate();

  /*!
  retrieve current time in numerical integration and delta_t, the time
  since the last successful function evaluation
  */
  void currentTime(double& time, double& delta_t);

  void writeStates(const double time, const std::vector<double>& y);

  std::vector<double>& qCopy() {
    return q_copy_;
  }

private:
  struct dqTodyIndex {
    unsigned int q;
    unsigned int dq;
  };
  std::vector<dqTodyIndex>
    quaternion_indices_; /*!< contains mapping of dq to dy for quaternions */
  std::vector<dqTodyIndex>
    non_quaternion_indices_; /*!< contains mapping of dq to dy for primitive
                                generalized coordinates, including any
                                coordinates used by force modules */

  virtual void addParameters() override;

  void appendActive(behavior::Base*);
  void appendActive(body::Base*);
  void appendActive(collisiongeometry::Base*);
  void appendActive(constraint::Base*);
  void appendActive(forcemodule::Base*);
  void appendActive(geometry::Base*);
  void appendActive(joint::Base*);
  void appendActive(light::Base*);
  void appendActive(sensor::Base*);
  void appendActive(terrain::Base*);
  void appendActive(solver::Base*);
  void appendActive(integrator::Base*);
  void appendActive(logger::Base*);
  void removeActive(behavior::Base*);
  void removeActive(body::Base*);
  void removeActive(collisiongeometry::Base*);
  void removeActive(constraint::Base*);
  void removeActive(forcemodule::Base*);
  void removeActive(geometry::Base*);
  void removeActive(joint::Base*);
  void removeActive(light::Base*);
  void removeActive(sensor::Base*);
  void removeActive(terrain::Base*);
  void removeActive(solver::Base*);
  void removeActive(integrator::Base*);
  void removeActive(logger::Base*);
  void clearActiveBehavior();
  void clearActiveArticulatedBodies();
  void clearActiveFreeBodies();
  void clearActiveCollisionGeometry();
  void clearActiveConstraints();
  void clearActiveforcemodules();
  void clearActiveGeometry();
  void clearActiveJoints();
  void clearActiveLights();
  void clearActiveSensors();
  void clearActiveTerrain();
  void clearActiveArrays();

  /*! \brief shift::core::Context initialization function, called in shift::core::Context
   * constructor */
  void initialize();

  /*! \brief connect pointers between objects managed by shift::core::Context */
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

  /*! \brief set mapping between positions of generalized coordinates in
  q to dq
  */
  void setQuaternionMap();

  /*! \brief User defined terrain attached to fixed inertial reference frame
   */
  shift::core::parameter::Array<shift::core::Assembly> assemblies_;

  const std::vector<shift::core::joint::Base*>* activeJointsPtr() {
    return &active_joints_;
  }

  shift::core::topology::DHVA
    dhva_; /*!< displacements, influence coefficient matrices,
           spatial velocities and quadratic accelerations */
  std::vector<topology::Offsets>
    dhva_offsets_; /*!< offsets into d,dc,h,v,a and dWRTReference array to
                     update kinematics */

  std::vector<topology::dh_fptr>
    d_fptr_; /*!< function pointers to update displacements */
  std::vector<topology::dh_fptr>
    dh_fptr_; /*!< function pointers to update displacements and influence
                 coefficient matrices */
  std::vector<topology::dhva_fptr> dhva_fptr_; /*!< function pointers to
  update displacements, influence coefficient matrices, velocities and
  quadratic accelerations */

  std::vector<int> d_reference_frames_; /*!< index of frames in
  framesInTree that are the reference frame for the body */

  std::vector<int> dcq_; /*!< index of framesInTree that require updating
                            direction cosines */
  std::vector<ksl_SE3_t> d_wrt_reference_;
  std::vector<double> joint_data_;

  std::vector<double> q_;
  double* dq_ = nullptr; /*!< generalized coordinate first time derivative, or
                         generalized velocities */
  std::vector<double> ddq_;
  std::vector<double> delta_q_;

  std::vector<double> q_copy_;
  std::vector<double> f_;
  /*!< Working forces within a joint, such as from
                        friction        or damping */
  std::vector<ksl_coscrew_t> p_; /*< momentum */
  std::vector<ksl_coscrew_t> g_; /*!< spatial body and inertial
                                  forces, length equal to dq count */
  std::vector<ksl_coscrew_t> k_;
  std::vector<ksl_inertia_t> inertia_; /*!< body fixed spatial inertias */
  std::vector<int> joint_dof_;
  std::vector<int> joint_lookup_;
  std::vector<int> dq_lookup_;
  std::vector<int> q_lookup_;
  std::vector<unsigned int> joint_dof_compressed_;
  std::vector<int> dq_lookup_compressed_;
  std::vector<int> pcldq_;
  std::vector<bool> is_rotation_;
  std::vector<bool> has_mass_;
  std::vector<bool> has_mass_dq_;

  shift::core::Jacobian jacobian_;

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
    const std::vector<shift::core::joint::Base*>* joints_;
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
    const std::vector<shift::core::joint::Base*>* joints_;
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
    const std::vector<shift::core::joint::Base*>* joints_;
    const std::vector<std::vector<int>> loopMembers_;
    bool operator()(const std::vector<int>& a, const std::vector<int>& b) {
      return (*joints_)[loopMembers_[a[0]][0]]->getLevel() <
             (*joints_)[loopMembers_[b[0]][0]]->getLevel();
    }
  };

  unsigned int non_rigid_joint_count_ = 0;
  std::vector<int> pcl_; /*!< parent child list */
  std::vector<int> pcl_compressed_;
  std::vector<std::vector<int>> loop_members_;
  std::vector<std::vector<int>> positive_loop_members_;
  std::vector<std::vector<int>> negative_loop_members_;
  std::vector<std::vector<int>> substructure_indices_;
  std::vector<std::vector<int>> substructure_members_;

  /*! \defgroup active_objects Active objects managed by shift::core::Context

  The following lists of objects contain pointers to only the objects whose
  active flag is set to true.

  @{ */
  std::vector<behavior::Base*> active_behaviors_ = {};
  std::vector<body::Base*> active_articulated_bodies_ = {};
  std::vector<body::Base*> active_free_bodies_ = {};
  std::vector<collisiongeometry::Base*> active_collision_geometry_ = {};
  std::vector<constraint::Base*> active_constraints_ = {};
  std::vector<forcemodule::Base*> active_force_modules_ = {};
  std::vector<geometry::Base*> active_geometry_ = {};
  std::vector<joint::Base*> active_joints_ = {};
  std::vector<light::Base*> active_lights_ = {};
  std::vector<sensor::Base*> active_sensors_ = {};
  std::vector<terrain::Base*> active_terrain_ = {};

  std::vector<integrator::Base*> active_integrators_ = {};
  std::vector<logger::Base*> active_loggers_ = {};
  std::vector<solver::Base*> active_solvers_ = {};

  std::vector<shift::core::Frame*> frames_in_tree_ = {};
  std::vector<shift::core::Frame*> active_frames_ = {};

  /*! @{ */

  /*! \defgroup managed_objects Objects managed by shift::core::Context

  These objects are created automatically when processing system
  topology and  are not loaded from an external file.

  @{ */
  /*! \brief System generated bodies */
  std::vector<std::unique_ptr<body::Base>> bodies_ = {};

  /*! \brief System generated behaviors */
  std::vector<std::unique_ptr<behavior::Base>> behaviors_ = {};

  /*! \brief System generated constraints */
  std::vector<std::unique_ptr<constraint::Base>> constraints_ = {};

  /*! \brief System generated force modules */
  std::vector<std::unique_ptr<forcemodule::Base>> force_modules_ = {};

  /*! \brief System generated joints */
  std::vector<std::unique_ptr<joint::Base>> joints_ = {};

  /*! \brief A subset of constraints used in model corresponding to
   * constraints imposed for kinematic chord loops */
  std::vector<constraint::Chord*> chord_constraints_ = {};

  /*! @} */
  std::vector<double> mobility_numbers_;

  void initializeMobility();

  /*! \brief Update direction cosines in model */
  void updateDC(topology::DHVA& dhva, const double* q);

  unsigned int q_count_ = 0;
  unsigned int dq_count_ = 0;
  double mobility_ = 0.0;
  double constraint_error_ = 0;

};

}
} // namespace shift

#endif

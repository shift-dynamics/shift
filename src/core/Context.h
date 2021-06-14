#ifndef SHIFT_CONTEXT_H_
#define SHIFT_CONTEXT_H_

#include <iostream>
#include <json/json.h>
#include <memory>
#include <string>

#include "shift/core/Assembly.h"
#include "shift/core/Base.h"
#include "shift/core/Factory.h"
#include "shift/core/Jacobian.h"
#include "shift/core/ModelParameters.h"
#include "shift/core/Topology.h"
#include "shift/core/body/Fixed.h"
#include "shift/core/joint/Base.h"

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
    DeclareParameters();
    Initialize();
  }
  Context()
    : body::Fixed("fixed")
    , jacobian_(this)
    , model_parameters_(this) {
    DeclareParameters();
    Initialize();
  }
  virtual ~Context() override {}

  Context& operator=(Context&&) {
    return *this;
  }

  unsigned int get_q_count();
  unsigned int get_dq_count();

  body::Base* get_fixed_body() {
    return this;
  }

  const std::vector<int>& get_dq_lookup() const {
    return dq_lookup_;
  }

  const std::vector<int>& get_dq_lookup_compressed() const {
    return dq_lookup_compressed_;
  }

  const std::vector<bool>& get_has_mass_dq() const {
    return has_mass_dq_;
  }

  const std::vector<bool>& get_has_mass_array() const {
    return has_mass_;
  }

  const std::vector<int>& get_q_lookup() const {
    return q_lookup_;
  }

  const std::vector<int>& get_joint_lookup() const {
    return joint_lookup_;
  }

  const std::vector<shift::core::joint::Base*>& get_active_joints() const {
    return active_joints_;
  }

  const std::vector<shift::core::forcemodule::Base*>&
  get_active_force_modules() const {
    return active_force_modules_;
  }

  const std::vector<int>& get_pcl() const {
    return pcl_;
  }

  const std::vector<int>& get_pcl_compressed() const {
    return pcl_compressed_;
  }

  const std::vector<shift::core::body::Base*>&
  get_active_articulated_bodies() const {
    return active_articulated_bodies_;
  }

  const std::vector<shift::core::Frame*>& get_frames_in_tree() const {
    return frames_in_tree_;
  }

  unsigned int get_non_rigid_joint_count() const {
    return non_rigid_joint_count_;
  }

  unsigned int get_substructure_count() const {
    return substructure_indices_.size();
  }

  const std::vector<std::vector<int>>& get_substructure_indices() const {
    return substructure_indices_;
  }

  const std::vector<std::vector<int>>& get_substructure_members() const {
    return substructure_members_;
  }

  const std::vector<std::vector<int>>& get_loop_members() const {
    return loop_members_;
  }

  const std::vector<std::vector<int>>& get_positive_loop_members() const {
    return positive_loop_members_;
  }

  const std::vector<std::vector<int>>& get_negative_loop_members() const {
    return negative_loop_members_;
  }

  /*! \brief get pointer to d_wrt_reference, a vector of relative
   * transformations to their respective reference frames */
  inline const std::vector<ksl_SE3_t>& get_d_wrt_reference() const {
    return d_wrt_reference_;
  }

  /*! \brief get pointer to joint_data, an array of additional constant data
   * required to process some joints */
  inline const std::vector<double>& get_joint_data() const {
    return joint_data_;
  }

  const ModelParameters& get_model_parameters() {
    return model_parameters_;
  }

  /*! \brief get count of total number of active frames */
  inline unsigned int get_active_frame_count() {
    return active_frames_.size();
  }

  /*! \brief get count of total number of active joints */
  inline unsigned int get_active_joint_count() const {
    return active_joints_.size();
  }

  /*! \brief get count of active chord constraints */
  inline unsigned int get_chord_constraint_count() const {
    return chord_constraints_.size();
  }

  inline unsigned int get_constraint_equation_count() const {
    return chord_constraints_.size() * 6;
  }

  inline std::vector<double>& get_ddq() {
    return ddq_;
  }

  inline double* get_dq() {
    return dq_;
  }

  inline std::vector<double>& get_q() {
    return q_;
  }

  inline std::vector<double>& get_delta_q() {
    return delta_q_;
  }

  inline const topology::DHVA& get_dhva() const {
    return dhva_;
  }

  inline const std::vector<topology::Offsets>& get_dhva_offsets() const {
    return dhva_offsets_;
  }

  inline std::vector<double>& get_mobility_numbers() {
    return mobility_numbers_;
  }

  inline const std::vector<constraint::Chord*>& get_chord_constraints() const {
    return chord_constraints_;
  }

  unsigned int get_force_module_equation_count();

  inline shift::core::Jacobian& get_jacobian() {
    return jacobian_;
  }

  inline const std::vector<int>& get_d_reference_frames() const {
    return d_reference_frames_;
  }

  inline const std::vector<ksl_inertia_t>& get_inertia_vector() const {
    return inertia_;
  }

  inline std::vector<double>& get_joint_working_forces() {
    return f_;
  }

  inline const std::vector<bool>& get_is_rotation() const {
    return is_rotation_;
  }

  inline const std::vector<int>& get_pcldq() const {
    return pcldq_;
  }

  inline const std::vector<unsigned int>& get_joint_dof_compressed() const {
    return joint_dof_compressed_;
  }

  inline std::vector<double>& get_q_copy() {
    return q_copy_;
  }

  inline const std::string get_filename() {
    return filename_;
  }

  inline const std::vector<shift::core::joint::Base*>* get_active_joints_ptr() {
    return &active_joints_;
  }

  bool get_newton_raphson_failure() {
    return jacobian_.get_newton_raphson_failure();
  }

  inline double get_constraint_error() {
    return constraint_error_;
  }

  /*! retrieve current time in numerical integration and delta_t, the time
  since the last successful function evaluation */
  void get_current_time(double& time, double& delta_t);

  virtual void ActivateObjects(bool) override;

  void ActivateObject(behavior::Base*, bool);
  void ActivateObject(body::Base*, bool);
  void ActivateObject(collisiongeometry::Base*, bool);
  void ActivateObject(constraint::Base*, bool);
  void ActivateObject(forcemodule::Base*, bool);
  void ActivateObject(geometry::Base*, bool);
  void ActivateObject(joint::Base*, bool);
  void ActivateObject(light::Base*, bool);
  void ActivateObject(sensor::Base*, bool);
  void ActivateObject(terrain::Base*, bool);
  void ActivateObject(integrator::Base*, bool);
  void ActivateObject(solver::Base*, bool);
  void ActivateObject(logger::Base*, bool);

  void LoadJSON(const std::string);
  void WriteJSON();
  void WriteJSON(std::ostream&);
  void WriteJSON(const std::string&);
  void PrintJSON();
  void PostLoad(const std::string);
  void SetupFullyQualifiedNames();
  void ConnectActiveObjects();
  double GravitationalPotentialEnergy();
  double KineticEnergy();

  /*! \brief Update displacements in a shift::core::joint::DHVA data structure
   * using a user-provided array of generalized coordinates. The instructions to
   * update displacements are stored in shift::core::Context */
  void UpdateD(topology::DHVA& dhva, const double* q);

  /*! \brief Update displacements and influence coefficient matrices in a
   * shift::core::joint::DHVA data structure using a user-provided array of
   * generalized coordinates. The instructions to update displacements and
   * influence coefficient matrices are stored in shift::core::Context */
  void UpdateDH(topology::DHVA& dhva, const double* q);

  /*! \brief Update displacements, influence coefficient matrices, velocities
   * and quadratic accelerations in a shift::core::joint::DHVA data structure
   * using a user-provided array of generalized coordinates and generalized
   * velocities.
   * The instructions to update DHVA are stored in shift::core::Context */
  void UpdateDHVA(topology::DHVA& dhva, const double* q, const double* dq);

  /*! \brief Update displacements stored in a shift::core::Context data
   * structure using a user-provided array of generalized coordinates. The
   * instructions to update displacements are stored in shift::core::Context */
  void UpdateD(const double* q);

  /*! \brief Update displacements and influence coefficient matrices in a
   * shift::core::Context data structure using a user-provided array of
   * generalized coordinates. The instructions to update displacements and
   * influence coefficient matrices are stored in shift::core::Context */
  void UpdateDH(const double* q);

  /*! \brief Update displacements, influence coefficient matrices, velocities
   * and quadratic accelerations in a shift::core::Context data structure using
   * a user-provided array of generalized coordinates and generalized
   * velocities.
   * The instructions to update DHVA are stored in shift::core::Context */
  void UpdateDHVA(const double* q, const double* dq);

  void UpdateD();
  void UpdateDH();
  void UpdateDHVA();

  /*! \brief set q and dq arrays to intial conditions saved in each joint */
  void SetToInitialConditions();

  /*! \brief extract current generalized displacements and velocities stored
   * in q and dq arrays and use to override initial conditions stored in
   * joints */
  void SetInitialConditionsToCurrentConditions();

  double GlobalMobility();

  int Digraph(std::string filename);
  int Digraph();

  int DigraphFrames(std::string filename);
  int DigraphFrames();

  void EvaluateConstraints();
  double EvaluateConstraintError();

  void SetForces(const double t, const std::vector<double>& y);

  /*! copies dq and ddq quantities to dy array using vectors quaternion_indices_
  and non_quaternion_indices_; for quaternion_indices_, converts angular
  velocities to time derivative of quaternions */
  void ConvertdqTody(const std::vector<double>& y, std::vector<double>& dy);

  void Simulate();

  void WriteStates(const double time, const std::vector<double>& y);

  void RegisterLogValue(shift::core::parameter::Value*);

  void UnregisterLogValue(shift::core::parameter::Value*);

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

  virtual void DeclareParameters() override;
  void ActivateLoggers();
  void AppendActiveObject(behavior::Base*);
  void AppendActiveObject(body::Base*);
  void AppendActiveObject(collisiongeometry::Base*);
  void AppendActiveObject(constraint::Base*);
  void AppendActiveObject(forcemodule::Base*);
  void AppendActiveObject(geometry::Base*);
  void AppendActiveObject(joint::Base*);
  void AppendActiveObject(light::Base*);
  void AppendActiveObject(sensor::Base*);
  void AppendActiveObject(terrain::Base*);
  void AppendActiveObject(solver::Base*);
  void AppendActiveObject(integrator::Base*);
  void AppendActiveObject(logger::Base*);
  void RemoveActiveObject(behavior::Base*);
  void RemoveActiveObject(body::Base*);
  void RemoveActiveObject(collisiongeometry::Base*);
  void RemoveActiveObject(constraint::Base*);
  void RemoveActiveObject(forcemodule::Base*);
  void RemoveActiveObject(geometry::Base*);
  void RemoveActiveObject(joint::Base*);
  void RemoveActiveObject(light::Base*);
  void RemoveActiveObject(sensor::Base*);
  void RemoveActiveObject(terrain::Base*);
  void RemoveActiveObject(solver::Base*);
  void RemoveActiveObject(integrator::Base*);
  void RemoveActiveObject(logger::Base*);
  void ClearActiveBehavior();
  void ClearActiveArticulatedBodies();
  void ClearActiveFreeBodies();
  void ClearActiveCollisionGeometry();
  void ClearActiveConstraints();
  void ClearActiveforcemodules();
  void ClearActiveGeometry();
  void ClearActiveJoints();
  void ClearActiveLights();
  void ClearActiveSensors();
  void ClearActiveTerrain();
  void ClearActiveArrays();

  /*! \brief shift::core::Context initialization function, called in
   * shift::core::Context constructor */
  void Initialize();

  /*! \brief connect pointers between objects managed by shift::core::Context */
  void Connect() override;

  /*! \brief recursively process all Shift objects' LoadData() functions, which
   * loads any additional external data not specified in JSON model definition
   * file from other data sources */
  void LoadData();

  /*! \brief sort active bodies and joints by level increasing order in
   * spanning tree
   */
  void LevelOrderSort();

  /*! \brief add a virtual body for each kinematic chord constraint */
  void ProcessVirtualBodies();

  /*! \brief generate parent-child list */
  void ProcessPCL();

  /*! \brief identify loop members */
  void ProcessLoopMembers();

  /*! \brief detect overlapping clusters of loops (i.e. substructures) if they
   * exist */
  void DetectSubstructures();

  /*! \brief identify frames in spanning tree and optimize rigid joints out of
   * model */
  void ProcessFrames();

  /*! \brief allocate arrays used at run-time used to process kinematics and
   * store quantities for solving the equations of motion. set up function
   * pointers required to update displacements, influence coefficient
   * matrices, velocities and accelerations. */
  void AllocateRunTimeArrays();

  /*! \brief set -g for the fixed frame in the array of quadratic
   * accelerations. After updating DHVA, this effectively applies
   * gravitational forces to all bodies without having to explicitly add
   * gravitational forces to individual bodies. */
  void SetupFixedFrameGravityVector();

  /*! \brief Collapse rigid joints out of the spanning tree */
  void CompressRigidJoints(int i, // current body Index and parent joint index
                           unsigned int& operationCount);

  /*! \brief initialize spatial inertias for articulated bodies
   */
  void InitializeInertia();

  void SetupLookup();

  /*! \brief set mapping between positions of generalized coordinates in
  q to dq
  */
  void SetupQuaternionMap();

  /*! \brief User defined terrain attached to fixed inertial reference frame
   */
  shift::core::parameter::Array<shift::core::Assembly> assemblies_;

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
   
  std::vector<double> q_; /*!< generalized coordinates */
  double* dq_ = nullptr; /*!< generalized coordinate first time derivative, or
                         generalized velocities */
  std::vector<double> ddq_; /*!< generalized accelerations */
  std::vector<double> delta_q_;

  std::vector<double> q_copy_;
  std::vector<double> f_;
  /*!< Working forces within a joint, such as from
                        friction        or damping */
  // std::vector<ksl_coscrew_t> g_; /*!< spatial body and inertial
  //                                 forces, length equal to dq count */
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
  struct LevelOrder {
    LevelOrder(Context* context)
      : joints_(context->get_active_joints_ptr()) {}
    const std::vector<shift::core::joint::Base*>* joints_;
    bool operator()(const int& a, const int& b) {
      int a_level = (a == -1) ? 0 : (*joints_)[a]->get_level();
      int b_level = (b == -1) ? 0 : (*joints_)[b]->get_level();
      return a_level < b_level;
    }
  };

  /*! \brief struct for sorting loops within a substructure by level order of
   * base node
   */
  struct SubstructureLoopLevelOrder {
    SubstructureLoopLevelOrder(
      Context* context, const std::vector<std::vector<int>>& loop_members)
      : joints_(context->get_active_joints_ptr())
      , loop_members_(loop_members) {}
    const std::vector<shift::core::joint::Base*>* joints_;
    const std::vector<std::vector<int>> loop_members_;
    bool operator()(const int& a, const int& b) {
      int a_level =
        (a == -1) ? -1 : (*joints_)[loop_members_[a][0]]->get_level();
      int b_level =
        (b == -1) ? -1 : (*joints_)[loop_members_[b][0]]->get_level();
      return a_level < b_level;
    }
  };

  /*! \brief struct for sorting substructures by level increasing order */
  struct SubstructureLevelOrder {
    SubstructureLevelOrder(Context* context,
                           const std::vector<std::vector<int>>& loop_members)
      : joints_(context->get_active_joints_ptr())
      , loop_members_(loop_members) {}
    const std::vector<shift::core::joint::Base*>* joints_;
    const std::vector<std::vector<int>> loop_members_;
    bool operator()(const std::vector<int>& a, const std::vector<int>& b) {
      int a_level = (loop_members_[a[0]][0] == -1)
                      ? -1
                      : (*joints_)[loop_members_[a[0]][0]]->get_level();
      int b_level = (loop_members_[b[0]][0] == -1)
                      ? -1
                      : (*joints_)[loop_members_[b[0]][0]]->get_level();
      return a_level < b_level;
    }
  };

  unsigned int non_rigid_joint_count_ = 0;
  std::vector<int> pcl_;            /*!< parent child list */
  std::vector<int> pcl_compressed_; /*!< compressed parent child list */
  std::vector<std::vector<int>> loop_members_;
  std::vector<std::vector<int>> positive_loop_members_;
  std::vector<std::vector<int>> negative_loop_members_;
  std::vector<std::vector<int>> substructure_indices_;
  std::vector<std::vector<int>> substructure_members_;

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

  void InitializeMobility();

  /*! \brief Update direction cosines in model */
  void UpdateDC(topology::DHVA& dhva, const double* q);

  unsigned int q_count_ = 0;
  unsigned int dq_count_ = 0;
  double mobility_ = 0.0;
  double constraint_error_ = 0;
};

} // namespace core
} // namespace shift

#endif

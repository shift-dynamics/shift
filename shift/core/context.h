#ifndef SHIFT_CONTEXT_H_
#define SHIFT_CONTEXT_H_

#include <functional>
#include <iostream>
#include <json/json.h>
#include <map>
#include <memory>
#include <set>

#include <string>
#include <unordered_map>

#include "shift/core/assembly.h"
#include "shift/core/base.h"
#include "shift/core/collision_geometry/gjk.h"
#include "shift/core/factory.h"
#include "shift/core/jacobian.h"
#include "shift/core/model_parameters.h"
#include "shift/core/topology.h"
#include "shift/core/body/fixed.h"
#include "shift/core/joint/base.h"

namespace shift {
namespace core {

namespace body {
class Free;
}

namespace controller {
class Base;
}

namespace collision_geometry {
class Plane;
class Base;
}

namespace geometry {
class Base;
struct MeshAsset;
}

namespace sensor {
class Base;
}

namespace terrain {
class Base;
}

namespace gpu {
class Device;
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

  /*! \brief the active body named name, null if there is none: "fixed" for the
   * fixed body, else a body of assembly, else the body of that fully qualified
   * name (<assembly>__<subassembly>__<name>), else the one body of that name in
   * the model. It is an error if several bodies have the name: the fully
   * qualified name tells them apart.
   * @param name [in] name of the body
   * @param assembly [in] assembly of the object asking, searched first
   */
  body::Base* FindActiveBody(const std::string& name,
                             const Assembly* assembly = nullptr);

  /*! \brief the active lights, connected to their frames */
  const std::vector<light::Base*>& get_active_lights() const {
    return active_lights_;
  }

  /*! \brief the GPU, made on the first call and shared by the objects of
   * the context that compute on it (e.g. terrain::Sph), which keep it while
   * they use it; throws if there is none, or shift was built without Vulkan
   * (see gpu::Device) */
  std::shared_ptr<gpu::Device> GetGpuDevice();

  /*! \brief the active terrains, see terrain::Base::Query */
  const std::vector<terrain::Base*>& get_active_terrain() const {
    return active_terrain_;
  }

  const std::vector<logger::Base*>& get_active_loggers() const {
    return active_loggers_;
  }

  /*! \brief collision geometry of the fixed body, given at the top level of
   * the model, e.g. the floor; see AddDefaultFloor */
  std::vector<std::unique_ptr<collision_geometry::Base>>&
  get_fixed_collision_geometry() {
    return collision_geometry_.get_data();
  }

  /*! \brief the assemblies at the top level of the model */
  std::vector<std::unique_ptr<Assembly>>& get_assemblies() {
    return assemblies_.get_data();
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

  const std::vector<shift::core::force_module::Base*>&
  get_active_force_modules() const {
    return active_force_modules_;
  }

  const std::vector<shift::core::sensor::Base*>& get_active_sensors() const {
    return active_sensors_;
  }

  const std::vector<shift::core::controller::Base*>&
  get_active_controllers() const {
    return active_controllers_;
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

  const std::vector<solver::Base*>& get_active_solvers() const {
    return active_solvers_;
  }
  const std::vector<integrator::Base*>& get_active_integrators() const {
    return active_integrators_;
  }

  const std::vector<shift::core::body::Base*>& get_active_free_bodies() const {
    return active_free_bodies_;
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

  inline topology::DHVA& get_dhva() {
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

  /*! \brief number of states of the bodies: q, dq and the states of the
   * free bodies (see body::Free), the leading part of the state vector
   * \f$\underline{y} = [\underline{q}, \underline{\dot{q}}, \text{free body states}, \text{auxiliary states}]\f$
   * and the size of get_q() */
  unsigned int get_state_count() const {
    return q_count_ + dq_count_ + free_state_count_;
  }

  /*! \brief offset of the free body states in the state vector */
  unsigned int get_free_state_offset() const {
    return q_count_ + dq_count_;
  }

  /*! \brief active free bodies, in the order of their states */
  const std::vector<body::Free*>& get_free_bodies() const {
    return free_bodies_;
  }

  /*! \brief number of auxiliary states (internal states of force modules
   * and controllers) that follow the body states in the state vector y */
  unsigned int get_auxiliary_state_count();

  /*! \brief assign each active force module and controller its offset in
   * the state vector \f$\underline{y} = [\underline{q}, \underline{\dot{q}}, \text{free body states},
   * \text{auxiliary states}]\f$ and copy
   * its initial values to y_auxiliary, which points to the first auxiliary
   * state; called by the integrators when they (re)initialize */
  void InitializeAuxiliaryStates(double* y_auxiliary);

  /*! \brief index of the body's group in the body load and velocity arrays:
   * the compressed index of its nearest non-rigid parent joint, -1 for a
   * body rigidly attached to the fixed body, and for a free body
   * non_rigid_joint_count plus its free index; valid once the topology is
   * initialized */
  int GetBodyLoadIndex(const body::Base* body) const;

  /*! \brief spatial velocity of a body group (see GetBodyLoadIndex) in the
   * fixed frame, referred to its origin; zero for index -1 */
  ksl_screw_t GetBodyVelocity(int load_index) const;

  /*! \brief add an external spatial force (in the fixed frame, referred to
   * its origin) to a body group (see GetBodyLoadIndex); loads on the fixed
   * body (index -1) are ignored. Loads are cleared in SetForces. */
  void AddBodyLoad(int load_index, const ksl_coscrew_t& load);

  inline const std::vector<ksl_coscrew_t>& get_body_loads() const {
    return body_loads_;
  }

  /*! \brief the external spatial force on a body group (see
   * GetBodyLoadIndex) from the force modules, in the fixed frame, referred to
   * its origin, summed in the last evaluation of the forces; zero for the
   * fixed body (index -1) */
  ksl_coscrew_t GetBodyLoad(int load_index) const;

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

  /*! \brief the path of a file named by the model, empty if it is not found
   * (see utility::FindFile); recorded with RecordFile. A name given to
   * set_file_map is found where the map puts it. */
  std::string FindFile(const std::string& name);

  /*! \brief note that the model read the file at path, such as a texture
   * that a mesh file names, so that a logger can keep a copy of every file
   * the model needs (get_files_read) */
  void RecordFile(const std::string& path);

  /*! \brief the files the model has read, absolute and without repeats, in
   * the order first read */
  const std::vector<std::string>& get_files_read() const {
    return files_read_;
  }

  /*! \brief the names by which the model asked FindFile for each file
   * read, as name -> its path in get_files_read */
  const std::map<std::string, std::string>& get_file_names() const {
    return file_names_;
  }

  /*! \brief the paths of files by the names the model gives them, which
   * FindFile takes before looking for them, such as those of a model taken
   * from a log (set before LoadJSON) */
  void set_file_map(std::map<std::string, std::string> file_map) {
    file_map_ = std::move(file_map);
  }

  /*! \brief the model as read from its file (YAML as JSON) */
  const Json::Value& get_model_json() const {
    return root_;
  }

  inline const std::vector<shift::core::joint::Base*>* get_active_joints_ptr() {
    return &active_joints_;
  }

  bool get_newton_raphson_failure() {
    return jacobian_.get_newton_raphson_failure();
  }

  inline double get_rms_constraint_error() {
    return rms_constraint_error_;
  }

  /*! retrieve current time in numerical integration and delta_t, the time
  since the last successful function evaluation */
  void get_current_time(double& time, double& delta_t);

  virtual void ActivateObjects(bool) override;

  void ActivateObject(controller::Base*, bool);
  void ActivateObject(body::Base*, bool);
  void ActivateObject(collision_geometry::Base*, bool);
  void ActivateObject(constraint::Base*, bool);
  void ActivateObject(force_module::Base*, bool);
  void ActivateObject(geometry::Base*, bool);
  void ActivateObject(joint::Base*, bool);
  void ActivateObject(light::Base*, bool);
  void ActivateObject(sensor::Base*, bool);
  void ActivateObject(terrain::Base*, bool);
  void ActivateObject(integrator::Base*, bool);
  void ActivateObject(solver::Base*, bool);
  void ActivateObject(logger::Base*, bool);

  void LoadJSON(const std::string);
  /*! \brief load a model given as JSON, as if read from filename: the
   * files it names are found relative to filename (e.g. a model edited in
   * the viewer, see get_model_json) */
  void LoadJSON(const Json::Value& model, const std::string& filename);
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

  /*! \brief evaluate the mobility number of every generalized velocity at the
   * current configuration q, and return the global mobility (their sum)

   Each substructure Jacobian is factored in full, and the mobility numbers
   are the squared row norms of an orthonormal basis of its right null space.
   Variables outside substructures keep a mobility number of one. */
  double ComputeMobility();

  int Digraph(std::string filename);
  int Digraph();

  int DigraphFrames(std::string filename);
  int DigraphFrames();

  void EvaluateConstraints();
  double EvaluateConstraintError();

  /*! \brief evaluate the run-time modules for the state y: clears the
   * joint working forces and body loads, then detects contacts and services
   * joints, sensors, controllers and force modules, in that order. The
   * modules set the time derivatives of their internal states in dy. */
  void SetForces(const double t, const std::vector<double>& y,
                 std::vector<double>& dy);
  /*! \brief the time t of the state being evaluated by SetForces, for
   * modules whose inputs are functions of time */
  double get_force_time() const {
    return force_time_;
  }

  /*! copies \f$\underline{\dot{q}}\f$ and \f$\underline{\ddot{q}}\f$ quantities to dy array using
  vectors quaternion_indices_ and non_quaternion_indices_; for
  quaternion_indices_, converts angular velocities to time derivative of
  quaternions. Spherical and float joint velocities (joint sense >= 0) are
  expressed in the child frame, so they are rotated into the parent frame by
  the joint rotation \f$R(\underline{q})\f$ first; for float joints the translational
  velocity is likewise rotated, \f$\dot{\underline{t}} = R(\underline{q}) \underline{v}\f$.
  Inverse joints (sense < 0) express velocities in the original parent frame
  and are copied directly */
  void ConvertdqTody(const std::vector<double>& y, std::vector<double>& dy);

  void Simulate();

  /*! \brief set the initial internal states of the force modules that hold
   * their joints still (force_module::Base::holds): finds the forces on the
   * held joints that give them zero acceleration at the initial state, the
   * least such forces where several share a load, and hands them to the
   * modules; Simulate calls it before integrating */
  void HoldInitialStates();

  void WriteStates(const double time, const std::vector<double>& y);

  /*! \brief Callback invoked from WriteStates after the loggers, at each
   * output time step, while DHVA is consistent with y */
  using StateObserver =
    std::function<void(const double time, const std::vector<double>& y)>;

  void set_state_observer(StateObserver observer) {
    state_observer_ = std::move(observer);
  }

  /*! \brief Pose of a body's origin with respect to the inertial frame,
   * evaluated from the displacements in dhva (by default the context's own),
   * for a free body from dhva.free */
  ksl_SE3_t get_body_origin_pose(body::Base* body);
  ksl_SE3_t get_body_origin_pose(body::Base* body,
                                 const topology::DHVA& dhva);

  /*! \brief the displacements and velocities of the free bodies in dhva.free
   * from their generalized coordinates, free_q (kStateCount per body, see
   * body::Free), e.g. y + get_free_state_offset() */
  void UpdateFreeDV(topology::DHVA& dhva, const double* free_q) const;

  void RegisterLogValue(shift::core::parameter::Value*);

  void UnregisterLogValue(shift::core::parameter::Value*);

  /*!
  @brief detect contact between the collision geometry of different bodies at
  the current displacements in dhva

  Updates the collision geometry poses and tests the candidate pairs of the
  broad phase (see UpdateContactCandidates): those whose bounding spheres
  overlap, or for a plane, whose bounding sphere reaches the plane, are tested
  with collision_geometry::TestContact. The poses and the tests of the pairs
  are computed in parallel (OpenMP; OMP_NUM_THREADS sets the number of
  threads). Called from SetForces when contact detection is enabled in the
  model parameters; the results are available from get_contacts().
  */
  void DetectContacts();

  /*! \brief called by the integrators after each accepted step, at the
   * accepted state y (the state of the last evaluation of the equations of
   * motion of the step): history that the forces depend on changes here
   * only, e.g. the contact normals the penetration depths stay near and
   * the deformation of the terrain (terrain::Base::AcceptStep) */
  void AcceptStep(const std::vector<double>& y);

  /*! \brief pairs of collision geometry found in contact by the most recent
   * call to DetectContacts; a contact force module records its data (see
   * ContactPair) in them */
  const std::vector<collision_geometry::ContactPair*>& get_contacts() const {
    return contacts_;
  }

  /*! \brief margin of each geometry of get_contact_geometry() at the last
   * broad phase, by which its bounding sphere is enlarged (0 for a plane) */
  const std::vector<double>& get_contact_margins() const {
    return contact_margins_;
  }

  /*! \brief the candidate pairs of the last broad phase, as indices into
   * get_contact_geometry() */
  std::vector<std::pair<int, int>> get_contact_candidates() const {
    std::vector<std::pair<int, int>> pairs;
    for(const auto& candidate : contact_candidates_) {
      pairs.emplace_back(candidate.a, candidate.b);
    }
    return pairs;
  }

  /*! \brief axis of the fixed reference frame (0, 1, 2 for x, y, z) of the
   * sweep of the last broad phase */
  int get_sweep_axis() const {
    return sweep_axis_;
  }

  /*! \brief active collision geometry, loaded or not, available once the
   * objects are activated */
  const std::vector<collision_geometry::Base*>&
  get_active_collision_geometry() const {
    return active_collision_geometry_;
  }

  /*! \brief never test the collision geometry of bodies a and b for contact,
   * e.g. bodies joined by a joint, whose geometry overlaps at the joint.
   * Bodies in the same rigid group (see GetBodyLoadIndex) are never tested. */
  void ExcludeContact(const body::Base* a, const body::Base* b);

  /*! \brief the number of times contact has been detected, once for each
   * evaluation of the forces */
  long get_contact_detection_calls() const {
    return contact_detection_calls_;
  }

  /*! \brief whether contact between the bodies a and b is not tested, see
   * ExcludeContact */
  bool is_contact_excluded(const body::Base* a, const body::Base* b) const {
    return excluded_contacts_.count(std::minmax(a, b)) > 0;
  }

  /*! \brief give the fixed body a floor, a plane at z = 0 facing up (-z),
   * with contact switched off, unless the model gives the fixed body a plane;
   * the viewer draws the floor at the first plane of the fixed body */
  void AddDefaultFloor();

  /*! \brief active collision geometry that was fitted to a mesh and takes part
   * in contact detection */
  const std::vector<collision_geometry::Base*>& get_contact_geometry() const {
    return contact_geometry_;
  }

  /*! \brief collision geometry already fitted under key, used to share fits
   * between instances of the same mesh, or nullptr */
  collision_geometry::Base* FindCollisionGeometryPrototype(
    const std::string& key) const;
  void AddCollisionGeometryPrototype(const std::string& key,
                                     collision_geometry::Base* geometry);

  /*! \brief mesh data already imported under key, so that bodies with the
   * same mesh files share one copy, or nullptr. The cache is filled while
   * the geometry loads and emptied afterwards; the meshes keep their data. */
  std::shared_ptr<const geometry::MeshAsset>
  FindMeshAsset(const std::string& key) const;
  void AddMeshAsset(const std::string& key,
                    std::shared_ptr<const geometry::MeshAsset> asset);

private:
  /*! \brief read the objects of root_, the model of filename, and set
   * them up */
  void ReadModel(const std::string& filename);

  /*! \brief collect the fitted collision geometry for contact detection */
  void SetupContactDetection();

  /*! \brief collect the active free bodies, check that no joint connects to
   * one, and set up their inertias */
  void SetupFreeBodies();
  /*! \brief throw if a name is repeated at one level of the model, from the
   * top level down: objects are identified by their fully qualified names */
  void CheckUniqueNames() override;
  /*! \brief the accelerations of the free bodies (dhva_.free.a) from their
   * loads and gravity, and the time derivatives of their generalized
   * coordinates at the free body offset of dy. Each body is independent, so
   * they are computed in parallel. */
  void EvaluateFreeBodies(const double* y, double* dy);

  /*!
  @brief the broad phase of contact detection: the candidate pairs

  The bounding sphere of each bounded geometry is enlarged by a margin, a
  fraction of its radius, and the pairs whose enlarged spheres overlap are
  candidates: until a geometry moves farther than its margin from where it was
  at the broad phase, no pair outside the candidates can overlap, so the
  candidates are kept and DetectContacts only tests them. Each plane (with an
  unbounded bounding sphere) is a candidate with every bounded geometry, and
  DetectContacts tests the distance of the bounding sphere from the plane.
  Pairs that cannot be in contact (the same rigid group, the same body, or
  excluded) are never candidates.
  */
  void UpdateContactCandidates();
  /*! \brief whether geometries i and j may be in contact: different rigid
   * groups and bodies, not excluded */
  bool is_contact_allowed(int i, int j) const;

  
  struct dqTodyIndex {
    unsigned int q;
    unsigned int dq;
  };

  struct QuaternionIndex {
    unsigned int q;  //!< index of the quaternion in q
    unsigned int dq; //!< index of the child frame angular velocity in dq
    int translation_q = -1; //!< float joints: index of the translation in q
    int translation_dq = -1; //!< float joints: index of the child frame
                             //!< translational velocity in dq
    bool child_frame = true; //!< velocities are expressed in the child frame
                             //!< (joint sense >= 0) and are rotated by R(q)
  };

  std::vector<QuaternionIndex>
    quaternion_indices_; /*!< contains mapping of dq to dy for quaternions */

  std::vector<dqTodyIndex>
    non_quaternion_indices_; /*!< contains mapping of dq to dy for primitive
                                generalized coordinates, including any
                                coordinates used by force modules */

  virtual void DeclareParameters() override;
  void ActivateLoggers();
  void AppendActiveObject(controller::Base*);
  void AppendActiveObject(body::Base*);
  void AppendActiveObject(collision_geometry::Base*);
  void AppendActiveObject(constraint::Base*);
  void AppendActiveObject(force_module::Base*);
  void AppendActiveObject(geometry::Base*);
  void AppendActiveObject(joint::Base*);
  void AppendActiveObject(light::Base*);
  void AppendActiveObject(sensor::Base*);
  void AppendActiveObject(terrain::Base*);
  void AppendActiveObject(solver::Base*);
  void AppendActiveObject(integrator::Base*);
  void AppendActiveObject(logger::Base*);
  void RemoveActiveObject(controller::Base*);
  void RemoveActiveObject(body::Base*);
  void RemoveActiveObject(collision_geometry::Base*);
  void RemoveActiveObject(constraint::Base*);
  void RemoveActiveObject(force_module::Base*);
  void RemoveActiveObject(geometry::Base*);
  void RemoveActiveObject(joint::Base*);
  void RemoveActiveObject(light::Base*);
  void RemoveActiveObject(sensor::Base*);
  void RemoveActiveObject(terrain::Base*);
  void RemoveActiveObject(solver::Base*);
  void RemoveActiveObject(integrator::Base*);
  void RemoveActiveObject(logger::Base*);
  void ClearActiveControllers();
  void ClearActiveArticulatedBodies();
  void ClearActiveFreeBodies();
  void ClearActiveCollisionGeometry();
  void ClearActiveConstraints();
  void ClearActiveForceModules();
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
  void LoadExactShapes();

  /*! \brief add a floating joint from the fixed body to each part of the
   * model that no path of joints joins to the fixed body */
  void AddFloatingJoints();

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
  /*! external spatial forces on each body group, length
   * non_rigid_joint_count, see AddBodyLoad */
  std::vector<ksl_coscrew_t> body_loads_;
  /* free bodies: their generalized coordinates follow q and dq, their
  displacements, velocities and accelerations are in dhva_.free; the load on
  each */
  std::vector<body::Free*> free_bodies_ = {};
  unsigned int free_state_count_ = 0;
  std::vector<ksl_coscrew_t> free_loads_ = {};
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
  std::vector<std::string> files_read_;
  std::map<std::string, std::string> file_map_;
  std::map<std::string, std::string> file_names_;
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
      // a loop whose base node is the fixed body has base member -1
      int a_level = (a == -1 || loop_members_[a][0] == -1)
                      ? -1
                      : (*joints_)[loop_members_[a][0]]->get_level();
      int b_level = (b == -1 || loop_members_[b][0] == -1)
                      ? -1
                      : (*joints_)[loop_members_[b][0]]->get_level();
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

  std::vector<controller::Base*> active_controllers_ = {};
  std::vector<body::Base*> active_articulated_bodies_ = {};
  std::vector<body::Base*> active_free_bodies_ = {};
  StateObserver state_observer_;
  std::vector<collision_geometry::Base*> active_collision_geometry_ = {};
  std::vector<collision_geometry::Base*> contact_geometry_ = {};
  std::map<std::string, collision_geometry::Base*>
    collision_geometry_prototypes_ = {};
  std::map<std::string, std::shared_ptr<const geometry::MeshAsset>>
    mesh_assets_ = {};
  /*! \brief candidate pairs from the sweep and prune pass, keyed by the
   * indices of the geometries in contact_geometry_, with their GJK history */
  std::unordered_map<uint64_t, collision_geometry::ContactPair>
    contact_pairs_ = {};
  std::vector<collision_geometry::ContactPair*> contacts_ = {};
  /*! \brief placement of a body attached by a chain of rigid joints (see
   * CompressRigidJoints): the index of the reference frame of the body above
   * the chain, and the displacement of the body's reference frame with
   * respect to it */
  std::unordered_map<const body::Base*, std::pair<int, ksl_SE3_t>>
    rigid_placements_ = {};
  /*! a candidate pair of the broad phase; a and b index contact_geometry_ */
  struct ContactCandidate {
    collision_geometry::ContactPair* pair = nullptr;
    int a = 0;
    int b = 0;
    bool was_in_contact = false; //!< before the most recent test
  };
  std::vector<ContactCandidate> contact_candidates_ = {};
  bool contact_candidates_valid_ = false;
  /*! bounded geometry (planes excluded) in the order of the sweep along
   * sweep_axis_ */
  std::vector<int> sweep_order_ = {};
  int sweep_axis_ = 0;
  /*! for each geometry in contact_geometry_: its plane if it is one (nullptr
   * otherwise), the center of its bounding sphere at the last broad phase and
   * its margin */
  std::vector<const collision_geometry::Plane*> contact_planes_ = {};
  std::vector<ksl_vec3_t> broad_phase_centers_ = {};
  std::vector<double> contact_margins_ = {};
  /* wall-clock time spent in DetectContacts during Simulate */
  double contact_detection_seconds_ = 0.0;
  long contact_detection_calls_ = 0;
  double contact_test_seconds_ = 0.0; /*!< the last round of pair tests, in
                                       * seconds of one thread, see DetectContacts */
  long broad_phase_calls_ = 0;
  /*! rigid group of each geometry in contact_geometry_: the body load index
   * of an articulated body, unique for a free body */
  std::vector<int> contact_group_ = {};
  /*! body pairs never tested for contact, ordered by address */
  std::set<std::pair<const body::Base*, const body::Base*>>
    excluded_contacts_ = {};
  std::vector<constraint::Base*> active_constraints_ = {};
  std::vector<force_module::Base*> active_force_modules_ = {};
  std::vector<geometry::Base*> active_geometry_ = {};
  std::vector<joint::Base*> active_joints_ = {};
  std::vector<light::Base*> active_lights_ = {};
  std::vector<sensor::Base*> active_sensors_ = {};
  std::vector<terrain::Base*> active_terrain_ = {};
  std::shared_ptr<gpu::Device> gpu_device_;

  std::vector<integrator::Base*> active_integrators_ = {};
  std::vector<logger::Base*> active_loggers_ = {};
  std::vector<solver::Base*> active_solvers_ = {};

  std::vector<shift::core::Frame*> frames_in_tree_ = {};
  std::vector<shift::core::Frame*> active_frames_ = {};

  /*! \brief System generated bodies */
  std::vector<std::unique_ptr<body::Base>> bodies_ = {};

  /*! \brief System generated controllers */
  std::vector<std::unique_ptr<controller::Base>> controllers_ = {};

  /*! \brief System generated constraints */
  std::vector<std::unique_ptr<constraint::Base>> constraints_ = {};

  /*! \brief System generated force modules */
  std::vector<std::unique_ptr<force_module::Base>> force_modules_ = {};

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
  double rms_constraint_error_ = 0;
  double force_time_ = 0; /*!< t of the last SetForces */
};

} // namespace core
} // namespace shift

#endif

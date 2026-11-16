#ifndef SHIFT_BODY_BASE_H_
#define SHIFT_BODY_BASE_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/coscrew.h"
#include "ksl/inertia.h"
#include "ksl/screw.h"

#include "shift/core/base.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/frame.h"
#include "shift/core/geometry/base.h"
#include "shift/core/geometry/mass_properties.h"
#include "shift/core/light/base.h"
#include "shift/core/parameter/array.h"
#include "shift/core/parameter/factory_array.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

namespace joint {
class Base;
}

namespace geometry {
struct MeshAsset;
}

class Assembly;
class Context;

namespace body {

/*!
@brief Parameters that describe a shift::Body
*/
class Base
  : public shift::core::Base
  , public shift::core::parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name)
    , shift::core::parameter::List() {
    auto origin = std::make_unique<Frame>("origin");
    origin->set_is_origin(true);
    origin->set_is_reference(true);
    origin->set_parent_body(this);
    assert(origin->get_parent_body());

    frames_.PushBack(std::move(origin));
    DeclareParameters();
  }
  Base()
    : shift::core::Base("")
    , shift::core::parameter::List() {
    auto origin = std::make_unique<Frame>("origin");
    origin->set_is_origin(true);
    origin->set_is_reference(true);
    origin->set_parent_body(this);
    frames_.PushBack(std::move(origin));
    DeclareParameters();
  }
  Base(Base&& b)
    : shift::core::Base(b.name_) {
    // for(auto& i : b.lights_) {
    //   lights_.push_back(std::move(i));
    // }
    // for(auto& i : b.geometry_) {
    //   geometry_.push_back(std::move(i));
    // }
    // for(auto& i : b.collision_geometry_) {
    //   collision_geometry_.push_back(std::move(i));
    // }
  }

  virtual ~Base() override {}
 
  static const std::string get_array_key() {
    return "bodies";
  }

  static const std::string get_default_type() {
    return "rigid";
  }

  const std::string& get_type() {
    return type_;
  }
  
  Base* get_parent_body();
  Frame* get_origin();
  Frame* get_reference_frame();

  unsigned int get_frame_count() {
    return frames_.get_size();
  }

  /*!
  return number of frames in spanning tree, context->ProcessFrames must be
  called to set frames in spanning tree.
  */
  unsigned int get_frame_in_tree_count();

  int get_index() {
    return index_;
  }

  int get_level() {
    return level_;
  }

  /*! \brief throw if a name is repeated among the frames, lights, geometry or
   * collision geometry of this body */
  virtual void CheckUniqueNames();

  std::vector<std::unique_ptr<Frame>>& get_frames() {
    return frames_.get_data();
  }

  std::vector<std::unique_ptr<light::Base>>& get_lights() {
    return lights_.get_data();
  }

  std::vector<std::unique_ptr<shift::core::geometry::Base>>& get_geometry() {
    return geometry_.get_data();
  }

  const joint::Base* get_parent_joint() {
    return parent_joint_;
  }

  int get_va_index() const {
    return va_index_;
  }

  virtual const ksl_inertia_t& get_inertia() const {
    return inertia_;
  }

  virtual ksl_inertia_t get_inertia_wrt_reference() {
    return inertia_;
  }

  const ksl_vec3_t get_center_of_mass_wrt_reference();
  /*! \brief the vector from the body's origin to its center of mass, in
   * the origin's coordinates */
  const ksl_vec3_t& get_center_of_mass_wrt_origin() const {
    return center_of_mass_wrt_origin_;
  }

  /*! \name Elastic coordinates
  A flexible body (body::Flex) adds \f$n_e\f$ elastic coordinates to the
  generalized coordinates: modal coordinates \f$\underline{\eta}\f$ for
  body::Modal, reduced nodal displacements for body::Nodal. The context
  places them in \f$\underline{q}\f$ and \f$\underline{\dot{q}}\f$ right
  after the coordinates of the body's parent joint. A rigid body has none,
  so every function below has a default for \f$n_e = 0\f$, and the solvers
  treat a rigid body as the special case of a flexible one.
  */
  ///@{

  /*! \brief number \f$n_e\f$ of elastic coordinates; 0 for a rigid body */
  virtual unsigned int get_elastic_count() const {
    return 0;
  }

  /*! \brief index of the first elastic coordinate in the array of
   * generalized coordinates, -1 if the body has none or the context has not
   * placed them */
  int get_elastic_q_index() const {
    return elastic_q_index_;
  }

  /*! \brief index of the first elastic velocity in the array of generalized
   * velocities, -1 if the body has none or the context has not placed them */
  int get_elastic_dq_index() const {
    return elastic_dq_index_;
  }

  void set_elastic_q_index(int index) {
    elastic_q_index_ = index;
  }

  void set_elastic_dq_index(int index) {
    elastic_dq_index_ = index;
  }

  /*! \brief initial values of the elastic coordinates and of their rates,
   * \f$n_e\f$ each; nullptr for a rigid body */
  virtual const double* get_initial_elastic_coordinates() const {
    return nullptr;
  }
  virtual const double* get_initial_elastic_velocities() const {
    return nullptr;
  }

  /*! \brief prepare a flexible body once its reference frame is final,
   * before the context places its coordinates: express its model in the
   * reference frame's coordinates and set its mass (its node frames are
   * found when it is connected, as the context needs them earlier);
   * the context calls it for every body with elastic coordinates. A rigid
   * body has nothing to prepare. */
  virtual void SetupElastic() {}

  /*! \brief a frame of the body that a joint or chord uses, for a flexible
   * body that builds its own model: the joint type, and for a revolute joint
   * its axis in the frame's coordinates */
  struct FrameUse {
    enum class Joint { kOther, kRevolute, kSpherical };
    Frame* frame = nullptr;
    Joint joint = Joint::kOther;
    ksl_vec3_t axis = {{0.0, 0.0, 1.0}};
  };

  /*! \brief build the elastic model of a flexible body from its own data
   * (a finite element model of its geometry) once the joints are connected
   * and before the context finds the frames of the spanning tree: \p
   * reference is the frame its parent joint attaches to, \p uses the frames
   * that joints and chords use. A rigid body, or a flexible body whose model
   * is given in code, has nothing to build. */
  virtual void BuildElastic(Frame* reference,
                            const std::vector<FrameUse>& uses) {
    (void)reference;
    (void)uses;
  }

  /*! \brief number of nodes: the frames of the body that move with its
   * deformation and that a joint, force module, sensor or chord uses; 0 for
   * a rigid body, whose frames all move with its reference frame */
  virtual unsigned int get_node_count() const {
    return 0;
  }

  /*! \brief the frame of node k, 0 <= k < get_node_count() */
  virtual Frame* get_node_frame(unsigned int k) const {
    (void)k;
    return nullptr;
  }

  /*! \brief evaluate the mass matrix blocks and the bias forces of a
   * flexible body for its elastic coordinates \p eta, their
   * rates \p eta_dot and the spatial velocity \p v_r of its reference frame,
   * all in the coordinates of the reference frame; the solvers call it once
   * per evaluation of the equations of motion, before the getters below. A
   * rigid body has nothing to evaluate. */
  virtual void EvaluateElastic(const double* eta, const double* eta_dot,
                               const ksl_screw_t& v_r) {
    (void)eta;
    (void)eta_dot;
    (void)v_r;
  }

  /*! \brief frame-elastic mass block \f$M_{be}\f$: \f$n_e\f$ coscrews in the
   * reference frame's coordinates (linear part first); nullptr for a rigid
   * body. The frame-frame block is get_inertia_wrt_reference(), which keeps
   * the ten-term form of a rigid inertia. */
  virtual const ksl_coscrew_t* get_mass_be() const {
    return nullptr;
  }

  /*! \brief elastic-elastic mass block \f$M_{ee}\f$, \f$n_e \times n_e\f$,
   * symmetric, row-major; nullptr for a rigid body */
  virtual const double* get_mass_ee() const {
    return nullptr;
  }

  /*! \brief bias wrench on the reference frame from the terms quadratic in
   * the velocities, in its coordinates (linear part first), without the
   * gyroscopic wrench \f$\mathbf{v}_r \times^* M_{bb} \mathbf{v}_r\f$ of
   * the frame-frame block, which the solvers form from
   * get_inertia_wrt_reference() as for a rigid body; zero for a rigid body */
  virtual ksl_coscrew_t get_bias_b() const {
    return ksl_coscrew(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  /*! \brief bias forces of the elastic coordinates, \f$n_e\f$: the terms
   * quadratic in the velocities and the elastic and damping forces
   * \f$-K\underline{\eta} - C\underline{\dot{\eta}}\f$; nullptr for a rigid
   * body */
  virtual const double* get_bias_e() const {
    return nullptr;
  }

  /*! \brief kinematics of node k, 0 <= k < get_node_count(), for the
   * elastic coordinates \p eta and their rates \p eta_dot, all in the
   * coordinates of the body's reference frame r and referred to its origin:
   * \p d_rk the deformed displacement of the node from r; \p h_k its
   * \f$n_e\f$ influence screws (the node's slice of the mode shapes, the
   * velocity of the node relative to r is \f$\sum_m h_{km}\dot{\eta}_m\f$);
   * \p gamma_k \f$\sum_m \dot{h}_{km}\dot{\eta}_m\f$, the part of the
   * node's acceleration relative to r that comes from the change of its
   * influence screws in r. The context adds the motion of r. \p h_k may be
   * nullptr when only the displacement is needed, and \p eta_dot and
   * \p gamma_k are nullptr when the velocities are not. A rigid body has no
   * nodes. */
  virtual void EvaluateNode(unsigned int k, const double* eta,
                            const double* eta_dot, ksl_SE3_t* d_rk,
                            ksl_screw_t* h_k, ksl_screw_t* gamma_k) const {
    (void)k;
    (void)eta;
    (void)eta_dot;
    (void)h_k;
    *d_rk = ksl_SE3_identity();
    if(gamma_k) {
      *gamma_k = ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
  }
  ///@}


  /*! \brief density of the body, 0 unless given: then its mass, center of
   * mass and inertia follow from the volume of its meshes */
  double get_density() const {
    return density_;
  }

  /*! \brief set the mass, center of mass and centroidal inertia from the
   * density and the solid bounded by the body's active meshes, once they are
   * loaded; a mesh that is not a closed surface is replaced by its convex
   * hull. Throws if the body has no meshes. The properties of unit density
   * are cached by the meshes' assets, so bodies with the same meshes compute
   * them once. */
  using MassCache =
    std::map<std::vector<const shift::core::geometry::MeshAsset*>,
             std::pair<shift::core::geometry::MassProperties, bool>>;
  void ComputeMassFromGeometry(MassCache& cache);

  bool has_mass() {
    return (inertia_.m >= 1e-9);
  }

  bool is_articulated() {
    return is_articulated_;
  }

  bool is_virtual() {
    return is_virtual_;
  }

  void set_reference_frame(shift::core::Frame*);

  void set_index(int index) {
    index_ = index;
  }

  void set_level(int level) {
    level_ = level;
  }

  void set_parent_joint(joint::Base*);

  void set_va_index(int index) {
    va_index_ = index;
  }

  void SetupDWrtReference(ksl_SE3_t);
  virtual void ActivateObjects(bool parent_is_active) override;
  void ClearInTreeFlags();
  void SetupReferenceFrame();
  virtual void Load();
  virtual void Connect();


protected:
  virtual void DeclareParameters() override;
  void SetupDWrtReference();

  std::string type_;
  bool is_articulated_ = true; /*!< indicates whether body may have joints
                                 attached to it, i.e. that the body comprises a
                                 link of an articulated assembly of bodies */

  shift::core::parameter::Array<shift::core::Frame> frames_;

  shift::core::parameter::FactoryArray<shift::core::light::Base> lights_;
  shift::core::parameter::FactoryArray<shift::core::geometry::Base> geometry_;
  shift::core::parameter::FactoryArray<shift::core::collision_geometry::Base> collision_geometry_;

  bool draw_enabled_ = true;
  bool draw_collision_geometry_enabled_ = false;
  int index_ = -1;
  int level_ = -1;
  int va_index_ = -1;
  int elastic_q_index_ = -1;  /*!< index of the first elastic coordinate */
  int elastic_dq_index_ = -1; /*!< index of the first elastic velocity */
  joint::Base* parent_joint_ = nullptr;
  shift::core::Frame* reference_frame_ =
    nullptr; /*!< pointer to unique reference frame on body, in an articulated
                multibody chain, this is the *proximal* or upstream
                frame in the spanning tree */
  ksl_inertia_t inertia_ = {}; /*!< centroidal mass and moment of inertia
                                    properties expressed in the body's origin
                                    coordinate system */
  ksl_vec3_t center_of_mass_wrt_origin_ = {};    /*!< vector from body origin to
                                                center of mass */
  ksl_vec3_t center_of_mass_wrt_reference_ = {}; /*!< vector from body reference
                                                frame to center of mass */
  bool is_virtual_ = false;
  double density_ = 0.0;
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace body
}
} // namespace shift

#endif

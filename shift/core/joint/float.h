#ifndef SHIFT_JOINT_FLOAT_H_
#define SHIFT_JOINT_FLOAT_H_

#include "shift/core/frame.h"
#include "shift/core/joint/base.h"

namespace shift {
namespace core {
namespace joint {

void float_d(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
             const double*);

void floatinv_d(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*);

void float_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
              const double*);

void floatinv_dh(const shift::core::Context&, topology::DHVA&,
                 const topology::Offsets&, const double*);

void float_dhva(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*, const double*);

void floatinv_dhva(const shift::core::Context&, topology::DHVA&,
                   const topology::Offsets&, const double*, const double*);

/*!
@brief pose of the traversal child of an inverse float joint

An inverse float joint is traversed from the joint's child (traversal parent
P) to the joint's parent (traversal child C). q = [t, quaternion] keeps the
meaning it has for a forward float joint: the pose of the joint's child with
respect to the joint's parent, t expressed in the joint parent frame, so

  R_C = R_P R^T,  t_C = t_P - R_C t

dq = [v, omega] is expressed in the joint parent frame (C), v = dt/dt and
dR/dt R^T = [omega] (see Context::ConvertdqTody and Substructure::Updateq).

@param d_parent displacement of the traversal parent P
@param q the joint's generalized coordinates [t, quaternion]
@param d_child displacement of the traversal child C, output
*/
void floatinv_pose(const ksl_SE3_t* d_parent, const double* q,
                   ksl_SE3_t* d_child);

/*!
@brief influence coefficients of an inverse float joint

Translation moves C along its own axes, h = [-a_k, 0]; rotation turns C about
the origin of P, h = [t_P x -a_k, -a_k], with a_k = R_C e_k.

@param d_parent displacement of the traversal parent P
@param d_child displacement of the traversal child C
@param h the joint's six influence coefficients, output
*/
void floatinv_h(const ksl_SE3_t* d_parent, const ksl_SE3_t* d_child,
                ksl_screw_t* h);

class Float : public Base {
public:
  Float()
    : Base() {
    dof_ = 6;
    q_count_ = 7;
    dq_count_ = 6;
    q_0_ = new double[q_count_]();
    dq_0_ = new double[dq_count_]();
    q_0_[6] = 1.0; // identity quaternion [x, y, z, w]
    rotation_indices_.push_back(3);
    rotation_indices_.push_back(4);
    rotation_indices_.push_back(5);
    is_rotation_ = true;
    DeclareParameters();
  }
  Float(std::string name)
    : Base(name) {
    dof_ = 6;
    q_count_ = 7;
    dq_count_ = 6;
    q_0_ = new double[q_count_]();
    dq_0_ = new double[dq_count_]();
    q_0_[6] = 1.0; // identity quaternion [x, y, z, w]
    rotation_indices_.push_back(3);
    rotation_indices_.push_back(4);
    rotation_indices_.push_back(5);
    is_rotation_ = true;
    DeclareParameters();
  }
  ~Float() {
    delete[] q_0_;
    delete[] dq_0_;
  }

  static std::unique_ptr<Float> create();
  static bool is_registered() {
    return registered_;
  }

  topology::dh_fptr Dfptr() override;
  topology::dh_fptr DHfptr() override;
  topology::dhva_fptr DHVAfptr() override;

  /*! \brief reads the joint, then sets the initial joint coordinates
   * q_0 = [t, quaternion] from the initial displacement */
  virtual void ReadJSON(Json::Value&) override;

private:
  virtual void DeclareParameters() override;
  static bool registered_;
  shift::core::Frame frame_;
};

} // namespace joint
}
} // namespace shift

#endif

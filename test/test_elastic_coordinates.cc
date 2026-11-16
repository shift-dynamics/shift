/* The elastic coordinates of a flexible body in the generalized coordinates:
the context places them right after the coordinates of the body's parent
joint, chains them in the parent list of the velocities, copies their initial
values, and follows the body's node with a node slot. A stub body type,
defined here, has elastic coordinates, one node and made-up mass and bias
terms that depend on them; the context transforms those to fixed
coordinates. */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "ksl/coscrew.h"
#include "ksl/matrix.h"
#include "ksl/screw.h"
#include "ksl/vec3.h"

#include "shift/core/body/rigid.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"

/*! a rigid body that reports three elastic coordinates and one node, its
frame "tip" at (1, 0, 0), which the coordinates move exactly: translated by
(eta_0, eta_1, 0) and turned about z by eta_2 */
class ElasticStubBody : public shift::core::body::Rigid {
public:
  unsigned int get_elastic_count() const override {
    return 3;
  }
  unsigned int get_node_count() const override {
    return 1;
  }
  shift::core::Frame* get_node_frame(unsigned int) const override {
    for(auto& frame : const_cast<ElasticStubBody*>(this)->get_frames()) {
      if(frame->get_name() == "tip") {
        return frame.get();
      }
    }
    return nullptr;
  }
  void EvaluateNode(unsigned int, const double* eta, const double* eta_dot,
                    ksl_SE3_t* d_rk, ksl_screw_t* h_k,
                    ksl_screw_t* gamma_k) const override {
    const double px = 1.0 + eta[0];
    const double py = eta[1];
    const double c = std::cos(eta[2]);
    const double s = std::sin(eta[2]);
    *d_rk = ksl_SE3_identity();
    d_rk->m00 = c;
    d_rk->m10 = s;
    d_rk->m01 = -s;
    d_rk->m11 = c;
    d_rk->m03 = px;
    d_rk->m13 = py;
    if(h_k) {
      /* in r, referred to its origin: the linear part is the velocity of
      the point at the origin, v_p - w x p */
      h_k[0] = ksl_screw(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      h_k[1] = ksl_screw(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
      h_k[2] = ksl_screw(py, -px, 0.0, 0.0, 0.0, 1.0);
    }
    if(gamma_k) {
      /* only h_2 changes with the coordinates */
      *gamma_k = ksl_screw(eta_dot[1] * eta_dot[2], -eta_dot[0] * eta_dot[2],
                           0.0, 0.0, 0.0, 0.0);
    }
  }
  /* mass and bias terms in r: arbitrary, but each depends on the elastic
  coordinates, so that a stale evaluation shows */
  void EvaluateElastic(const double* eta, const double* eta_dot,
                       const ksl_screw_t& v_r) override {
    for(int m = 0; m < 3; m++) {
      eta_[m] = eta[m];
      eta_dot_[m] = eta_dot[m];
      mass_be_[m] = ksl_coscrew(0.5 + m, -0.25 * m, 0.1, eta[m], 0.3, -0.2 * m);
      bias_e_[m] = -10.0 * eta[m] - eta_dot[m];
    }
    v_r_ = v_r;
    bias_b_ = ksl_coscrew(eta[0], -eta[1], 0.4, 0.2, eta_dot[2], -0.7);
    for(int k = 0; k < 9; k++) {
      mass_ee_[k] = (k % 4 == 0) ? 1.0 : 0.0;
    }
  }
  ksl_inertia_t get_inertia_wrt_reference() override {
    return ksl_inertia(2.0 + eta_[0], ksl_vec3(0.0, 0.0, 0.0), 0.3, 0.4,
                       0.5 + eta_[2], 0.0, 0.0, 0.0);
  }
  const ksl_coscrew_t* get_mass_be() const override {
    return mass_be_;
  }
  const double* get_mass_ee() const override {
    return mass_ee_;
  }
  ksl_coscrew_t get_bias_b() const override {
    return bias_b_;
  }
  const double* get_bias_e() const override {
    return bias_e_;
  }
  const double* get_initial_elastic_coordinates() const override {
    return kInitialCoordinates;
  }
  const double* get_initial_elastic_velocities() const override {
    return kInitialVelocities;
  }
  static std::unique_ptr<shift::core::body::Base> Create() {
    return std::make_unique<ElasticStubBody>();
  }

  /* what the last evaluation was given */
  double eta_[3] = {};
  double eta_dot_[3] = {};
  ksl_screw_t v_r_ = {};

  ksl_coscrew_t mass_be_[3] = {};
  double mass_ee_[9] = {};
  ksl_coscrew_t bias_b_ = {};
  double bias_e_[3] = {};

  static constexpr double kInitialCoordinates[3] = {0.1, 0.2, 0.3};
  static constexpr double kInitialVelocities[3] = {-1.0, -2.0, -3.0};
  static bool registered_;
};

bool ElasticStubBody::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "elastic_stub", ElasticStubBody::Create);

/*! a chain fixed -J1- a -J2- s -J3- c with the stub body s, joints J2 and J3
of types \p j2_type and \p j3_type, J3 at the frame \p j3_frame of s */
static std::string WriteChain(const std::string& name,
                              const std::string& j2_type,
                              const std::string& j3_type = "revolute",
                              const std::string& j3_frame = "tip") {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  std::ofstream file(path);
  const std::string inertia =
    R"("center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1.0, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1,
                "Ixy": 0, "Iyz": 0, "Izx": 0})";
  file << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0.0, 0.0, 0.0], "include_directories": ["."],
  "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": 0.1, "time_step": 0.01}]},
 "assemblies": [{"name": "chain",
  "bodies": [
   {"name": "a", "geometry": [], "frames": [{"name": "tip", "T": [1, 0, 0]}],
    )" << inertia << R"(},
   {"name": "s", "type": "elastic_stub", "geometry": [],
    "frames": [{"name": "tip", "T": [1, 0, 0]},
               {"name": "mid", "T": [0.5, 0, 0]}], )"
       << inertia << R"(},
   {"name": "c", "geometry": [], )" << inertia << R"(}],
  "joints": [
   {"name": "J1", "type": "revolute", "axis": [0, 0, 1],
    "parent": [["fixed"], "origin"], "child": [["a"], "origin"]},
   {"name": "J2", "type": ")"
       << j2_type << R"(", "axis": [0, 0, 1],
    "parent": [["a"], "tip"], "child": [["s"], "origin"]},
   {"name": "J3", "type": ")"
       << j3_type << R"(", "axis": [0, 0, 1],
    "parent": [["s"], ")"
       << j3_frame << R"("], "child": [["c"], "origin"]}]}]}
)";
  return path;
}

TEST(ElasticCoordinates, PlacedAfterParentJoint) {
  const std::string path = WriteChain("elastic_coordinates", "revolute");
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());

  /* J1, J2, the three elastic coordinates of s, J3 */
  ASSERT_EQ(context.get_q_count(), 6u);
  ASSERT_EQ(context.get_dq_count(), 6u);
  EXPECT_TRUE(context.HasElasticBodies());

  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "s") {
      EXPECT_EQ(body->get_elastic_q_index(), 2);
      EXPECT_EQ(body->get_elastic_dq_index(), 2);
    } else {
      EXPECT_EQ(body->get_elastic_q_index(), -1);
      EXPECT_EQ(body->get_elastic_dq_index(), -1);
    }
  }
  for(auto joint : context.get_active_joints()) {
    if(joint->get_name() == "J3") {
      EXPECT_EQ(joint->get_q_index(), 5);
      EXPECT_EQ(joint->get_dq_index(), 5);
    }
  }

  /* the elastic coordinates chain after J2, and J3 hangs below the last */
  const auto& pcldq = context.get_pcldq();
  ASSERT_EQ(pcldq.size(), 6u);
  const int expected[6] = {-1, 0, 1, 2, 3, 4};
  for(int k = 0; k < 6; k++) {
    EXPECT_EQ(pcldq[k], expected[k]) << "dq " << k;
  }

  /* one serial chain whose velocities include the elastic coordinates */
  const auto& trees = context.get_trees();
  ASSERT_EQ(trees.size(), 1u);
  EXPECT_EQ(trees[0].dq_count, 6u);

  /* initial values of the elastic coordinates and their rates */
  const auto& q = context.get_q();
  const double* dq = context.get_dq();
  for(int k = 0; k < 3; k++) {
    EXPECT_DOUBLE_EQ(q[2 + k], ElasticStubBody::kInitialCoordinates[k]);
    EXPECT_DOUBLE_EQ(dq[2 + k], ElasticStubBody::kInitialVelocities[k]);
  }

}

/* a flexible body on a rigid joint keeps its own entry: its elastic
coordinates follow J1's, and its reference frame moves with a */
TEST(ElasticCoordinates, FlexibleBodyOnRigidJoint) {
  const std::string path = WriteChain("elastic_rigid_joint", "rigid");
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());

  /* J1, the three elastic coordinates of s, J3 */
  ASSERT_EQ(context.get_q_count(), 5u);
  const auto& pcldq = context.get_pcldq();
  const int expected[5] = {-1, 0, 1, 2, 3};
  for(int k = 0; k < 5; k++) {
    EXPECT_EQ(pcldq[k], expected[k]) << "dq " << k;
  }
  /* compressed tree: J1 (0), s on J2 (1), its node slot (2), J3 (3) */
  const auto& pcl = context.get_pcl_compressed();
  ASSERT_EQ(pcl.size(), 4u);
  EXPECT_EQ(pcl[1], 0);
  EXPECT_EQ(pcl[2], 1);
  EXPECT_EQ(pcl[3], 2);

  /* with the elastic coordinates at rest, s and a move together: the node
  and a's tip have the same velocity */
  std::vector<double> q = {0.3, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> dq = {0.7, 0.0, 0.0, 0.0, 0.0};
  context.UpdateDHVA(q.data(), dq.data());
  const auto& dhva = context.get_dhva();
  const auto& slot = context.get_node_slots()[0];
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "a" || body->get_name() == "s") {
      EXPECT_EQ(body->get_va_index(), body->get_name() == "a" ? 0 : 1);
    }
  }
  for(int k = 0; k < 6; k++) {
    EXPECT_NEAR(dhva.v[1].at[k], dhva.v[0].at[k], 1e-14);
    EXPECT_NEAR(dhva.v[slot.index].at[k], dhva.v[0].at[k], 1e-14);
  }
  /* the tip of s, at (1, 0, 0) from s, which is at (1, 0, 0) from a */
  const ksl_SE3_t& d_tip = dhva.d[slot.frame->get_index()];
  EXPECT_NEAR(d_tip.t.x, 2.0 * std::cos(0.3), 1e-12);
  EXPECT_NEAR(d_tip.t.y, 2.0 * std::sin(0.3), 1e-12);
}

/* a body welded at a node moves with the node slot, and its mass is held by
the slot */
TEST(ElasticCoordinates, BodyWeldedAtNode) {
  const std::string path =
    WriteChain("elastic_welded", "revolute", "rigid", "tip");
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());

  /* J1, J2, the three elastic coordinates of s */
  ASSERT_EQ(context.get_q_count(), 5u);
  const auto& slot = context.get_node_slots()[0];
  EXPECT_EQ(slot.index, 2);
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "c") {
      EXPECT_EQ(body->get_va_index(), slot.index);
      EXPECT_EQ(context.GetBodyLoadIndex(body), slot.index);
    }
  }
  EXPECT_DOUBLE_EQ(context.get_inertia_vector()[slot.index].m, 1.0);

  /* c's origin is at the deformed node */
  std::vector<double> q = {0.2, 0.1, 0.05, -0.03, 0.4};
  std::vector<double> dq(5, 0.0);
  context.UpdateDHVA(q.data(), dq.data());
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "c") {
      const ksl_SE3_t pose = context.get_body_origin_pose(body);
      const ksl_SE3_t& node = context.get_dhva().d[slot.frame->get_index()];
      for(int k = 0; k < 12; k++) {
        EXPECT_NEAR(pose.as_array[k], node.as_array[k], 1e-12) << k;
      }
    }
  }
}

/* a joint on a flexible body must be at a node */
TEST(ElasticCoordinates, JointAtFrameThatIsNotANodeRejected) {
  const std::string path =
    WriteChain("elastic_not_node", "revolute", "revolute", "mid");
  shift::core::Context context;
  EXPECT_THROW(context.LoadJSON(path), shift::core::Exception);
  std::remove(path.c_str());
}

/* spatial velocity of the motion from d0 to d1 in time dt, in fixed
coordinates and referred to the fixed origin: w from R1 R0^T, and the linear
part dt/dt - w x t */
static ksl_screw_t SpatialVelocity(const ksl_SE3_t& d0, const ksl_SE3_t& d1,
                                   double dt) {
  ksl_SE3_t d10;
  ksl_product_ddinv(&d1, &d0, &d10);
  const ksl_mat3x3_t& w = d10.R;
  ksl_screw_t v;
  v.m3 = 0.5 * (w.m21 - w.m12) / dt;
  v.m4 = 0.5 * (w.m02 - w.m20) / dt;
  v.m5 = 0.5 * (w.m10 - w.m01) / dt;
  const double tx = 0.5 * (d0.t.x + d1.t.x);
  const double ty = 0.5 * (d0.t.y + d1.t.y);
  const double tz = 0.5 * (d0.t.z + d1.t.z);
  v.m0 = (d1.t.x - d0.t.x) / dt - (v.m4 * tz - v.m5 * ty);
  v.m1 = (d1.t.y - d0.t.y) / dt - (v.m5 * tx - v.m3 * tz);
  v.m2 = (d1.t.z - d0.t.z) / dt - (v.m3 * ty - v.m4 * tx);
  return v;
}

/* the node slot's velocity and quadratic acceleration against differences
of the node frame's displacement, along a motion with constant velocities
(so the acceleration is the quadratic one) */
TEST(ElasticCoordinates, NodeSlotKinematics) {
  const std::string path = WriteChain("elastic_node_slot", "revolute");
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());

  const auto& slots = context.get_node_slots();
  ASSERT_EQ(slots.size(), 1u);
  const auto& slot = slots[0];
  ASSERT_NE(slot.frame, nullptr);
  /* compressed tree: J1 (0), J2 (1), the node slot (2), J3 (3) */
  EXPECT_EQ(slot.index, 2);
  const auto& pcl = context.get_pcl_compressed();
  ASSERT_EQ(pcl.size(), 4u);
  EXPECT_EQ(pcl[2], 1);
  EXPECT_EQ(pcl[3], 2);
  EXPECT_EQ(context.GetFrameLoadIndex(slot.frame), 2);
  EXPECT_EQ(context.GetFrameVaIndex(slot.frame), 2);

  const int n = (int)context.get_q_count();
  std::vector<double> q0 = {0.4, -0.7, 0.05, -0.02, 0.3, 0.6};
  std::vector<double> dq = {1.1, -0.8, 0.3, 0.2, -0.9, 0.5};
  ASSERT_EQ(n, 6);

  auto pose_at = [&](double t, ksl_screw_t* v, ksl_screw_t* aq) {
    std::vector<double> q(n);
    for(int k = 0; k < n; k++) {
      q[k] = q0[k] + dq[k] * t;
    }
    context.UpdateDHVA(q.data(), dq.data());
    const auto& dhva = context.get_dhva();
    if(v) {
      *v = dhva.v[slot.index];
    }
    if(aq) {
      *aq = dhva.aq[slot.index];
    }
    return dhva.d[slot.frame->get_index()];
  };

  ksl_screw_t v, aq;
  pose_at(0.0, &v, &aq);
  const double dt = 1e-6;
  const ksl_screw_t v_fd =
    SpatialVelocity(pose_at(-dt, nullptr, nullptr),
                    pose_at(dt, nullptr, nullptr), 2.0 * dt);
  for(int k = 0; k < 6; k++) {
    EXPECT_NEAR(v.at[k], v_fd.at[k], 1e-7) << "v " << k;
  }

  /* the child body of J3, which does not move relative to the node, has
  the node's velocity */
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "c") {
      dq[5] = 0.0;
      pose_at(0.0, nullptr, nullptr);
      const auto& dhva = context.get_dhva();
      for(int k = 0; k < 6; k++) {
        EXPECT_NEAR(dhva.v[body->get_va_index()].at[k],
                    dhva.v[slot.index].at[k], 1e-14);
      }
      dq[5] = 0.5;
    }
  }

  /* the acceleration: the derivative of the velocity screw */
  const double da = 1e-5;
  ksl_screw_t v_minus, v_plus;
  pose_at(-da, &v_minus, nullptr);
  pose_at(da, &v_plus, nullptr);
  for(int k = 0; k < 6; k++) {
    const double a_fd = (v_plus.at[k] - v_minus.at[k]) / (2.0 * da);
    EXPECT_NEAR(aq.at[k], a_fd, 1e-6) << "aq " << k;
  }
}

/* spherical joints next to a flexible body are not supported yet, before
or after it */
/* the power of a coscrew on a screw does not depend on the coordinates */
static double Power(const ksl_coscrew_t& c, const ksl_screw_t& s) {
  return ksl_dot_vv(&c.lin, &s.lin) + ksl_dot_vv(&c.ang, &s.ang);
}

TEST(ElasticCoordinates, MassAndBiasInFixedCoordinates) {
  const std::string path = WriteChain("elastic_mass_bias", "revolute");
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  ASSERT_EQ(context.get_elastic_bodies().size(), 1u);
  ElasticStubBody* s = dynamic_cast<ElasticStubBody*>(
    context.get_active_articulated_bodies()[context.get_elastic_bodies()[0]]);
  ASSERT_NE(s, nullptr);

  /* a state where every coordinate moves: J1, J2, the elastic coordinates
  of s, J3 */
  const std::vector<double> q = {0.4, -0.7, 0.05, -0.1, 0.2, 0.3};
  const std::vector<double> dq = {1.1, -0.6, 0.3, 0.8, -0.5, 0.9};
  context.UpdateDHVA(q.data(), dq.data());
  context.EvaluateElasticBodies(context.get_dhva(), q.data(), dq.data());

  for(int m = 0; m < 3; m++) {
    EXPECT_DOUBLE_EQ(s->eta_[m], q[2 + m]);
    EXPECT_DOUBLE_EQ(s->eta_dot_[m], dq[2 + m]);
  }

  /* the velocity given to the body is that of r's origin, in r's axes */
  const int j = s->get_va_index();
  const auto& dhva = context.get_dhva();
  const ksl_SE3_t& D_0r = dhva.d[context.get_d_reference_frames()[j]];
  const ksl_screw_t& v_0 = dhva.v[j];
  ksl_vec3_t w_x_t, v_origin, expected;
  ksl_cross_vv(&v_0.ang, &D_0r.t, &w_x_t);
  ksl_add_vv(&v_0.lin, &w_x_t, &v_origin);
  ksl_product_drinvv(&D_0r.R, &v_origin, &expected);
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(s->v_r_.lin.at[k], expected.at[k], 1e-12) << k;
  }
  ksl_product_drinvv(&D_0r.R, &v_0.ang, &expected);
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(s->v_r_.ang.at[k], expected.at[k], 1e-12) << k;
  }
  EXPECT_GT(std::abs(v_0.ang.z), 0.1);

  /* the frame-frame block replaces the body's inertia, still in r */
  const ksl_inertia_t& inertia = context.get_inertia_vector()[j];
  EXPECT_DOUBLE_EQ(inertia.m, 2.0 + q[2]);

  /* the coscrews in fixed coordinates do the same work on v_0 as those in r
  on v_r */
  const auto& mass_be = context.get_elastic_mass_be();
  const auto& bias_e = context.get_elastic_bias_e();
  ASSERT_EQ(mass_be.size(), dq.size());
  const int e = s->get_elastic_dq_index();
  for(int m = 0; m < 3; m++) {
    EXPECT_NEAR(Power(mass_be[e + m], v_0), Power(s->mass_be_[m], s->v_r_),
                1e-12)
      << m;
    EXPECT_DOUBLE_EQ(bias_e[e + m], -10.0 * q[2 + m] - dq[2 + m]);
  }
  const ksl_coscrew_t& bias_b = context.get_elastic_bias_b()[j];
  EXPECT_NEAR(Power(bias_b, v_0), Power(s->bias_b_, s->v_r_), 1e-12);
  /* and so for a screw that is not the body's velocity */
  const ksl_screw_t a_0 = ksl_screw(0.3, -1.2, 0.7, 0.2, 0.9, -0.4);
  ksl_screw_t a_r;
  ksl_product_Adinvs(&D_0r, &a_0, &a_r);
  EXPECT_NEAR(Power(bias_b, a_0), Power(s->bias_b_, a_r), 1e-12);

  /* the velocities that are not elastic have no terms, and the rigid
  bodies no bias wrench */
  for(int k = 0; k < (int)dq.size(); k++) {
    if(k < e || k >= e + 3) {
      EXPECT_EQ(ksl_coscrew_norm(&mass_be[k]), 0.0) << k;
      EXPECT_EQ(bias_e[k], 0.0) << k;
    }
  }
  for(int k = 0; k < (int)context.get_elastic_bias_b().size(); k++) {
    if(k != j) {
      EXPECT_EQ(ksl_coscrew_norm(&context.get_elastic_bias_b()[k]), 0.0) << k;
    }
  }
}

TEST(ElasticCoordinates, SphericalJointRejected) {
  for(const auto& [j2, j3] : {std::pair<std::string, std::string>{
                                "spherical", "revolute"},
                              {"revolute", "spherical"}}) {
    const std::string path =
      WriteChain("elastic_spherical_" + j2, j2, j3, "tip");
    shift::core::Context context;
    EXPECT_THROW(context.LoadJSON(path), shift::core::Exception) << j2;
    std::remove(path.c_str());
  }
}

/* a float joint before a flexible body: six coordinates, then the elastic
ones */
TEST(ElasticCoordinates, FloatJointBeforeFlexibleBody) {
  const std::string path = WriteChain("elastic_float", "float");
  shift::core::Context context;
  context.LoadJSON(path);
  std::remove(path.c_str());
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "s") {
      auto joint = body->get_parent_joint();
      EXPECT_EQ(body->get_elastic_dq_index(),
                joint->get_dq_index() + (int)joint->get_dq_count());
      EXPECT_EQ(body->get_elastic_q_index(),
                joint->get_q_index() + (int)joint->get_q_count());
    }
  }
}

#include "shift/core/body/flex.h"

#include <sstream>

#include "ksl/matrix.h"
#include "ksl/vec3.h"

#include "shift/core/exception.h"
#include "shift/core/log.h"

/* a A + s (a~ b~), with a~ b~ = b a^T - (a . b) I; at[c][r] is row r of
column c */
static void AddSkewSkew(double s, const ksl_vec3_t& a, const ksl_vec3_t& b,
                        ksl_mat3x3_t* A) {
  const double ab = ksl_dot_vv(&a, &b);
  for(int c = 0; c < 3; c++) {
    for(int r = 0; r < 3; r++) {
      A->at[c][r] += s * (b.at[r] * a.at[c] - (r == c ? ab : 0.0));
    }
  }
}

static void AddScaled(double s, const ksl_mat3x3_t& B, ksl_mat3x3_t* A) {
  for(int c = 0; c < 3; c++) {
    for(int r = 0; r < 3; r++) {
      A->at[c][r] += s * B.at[c][r];
    }
  }
}

static void AddScaledVector(double s, const ksl_vec3_t& b, ksl_vec3_t* a) {
  a->x += s * b.x;
  a->y += s * b.y;
  a->z += s * b.z;
}

void shift::core::body::Flex::SetModel(
  const std::vector<ksl_vec3_t>& positions, const std::vector<double>& masses,
  const std::vector<ksl_mat3x3_t>& rotary_inertias,
  unsigned int elastic_count, const std::vector<ksl_screw_t>& shapes,
  const std::vector<double>& stiffness, const std::vector<double>& damping) {
  const size_t n = elastic_count;
  if(positions.size() != masses.size() ||
     rotary_inertias.size() != masses.size() ||
     shapes.size() != masses.size() * n || stiffness.size() != n * n ||
     !(damping.empty() || damping.size() == n * n)) {
    throw shift::core::Exception(
      "The model of flexible body '" + get_name() +
      "' has arrays of inconsistent sizes");
  }
  elastic_count_ = elastic_count;
  positions_ = positions;
  masses_ = masses;
  rotary_inertias_ = rotary_inertias;
  shapes_ = shapes;
  stiffness_ = stiffness;
  damping_ = damping.empty() ? std::vector<double>(n * n, 0.0) : damping;
  initial_eta_.assign(n, 0.0);
  initial_eta_dot_.assign(n, 0.0);
}

void shift::core::body::Flex::SetNode(const std::string& frame_name,
                                      unsigned int point) {
  node_names_.push_back(frame_name);
  node_points_.push_back(point);
  node_sliding_.push_back(-1);
}

void shift::core::body::Flex::SetSlidingNode(
  const std::string& frame_name, const std::vector<unsigned int>& stations,
  double first, double spacing, const ksl_vec3_t& axis) {
  if(stations.empty() || (stations.size() > 1 && !(spacing > 0.0))) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "': sliding node '" + frame_name +
                                 "' needs stations a positive spacing apart");
  }
  SlidingNode node;
  node.stations = stations;
  node.first = first;
  node.spacing = spacing;
  node.axis = axis;
  ksl_vec3_normalize(&node.axis);
  node_names_.push_back(frame_name);
  node_points_.push_back(stations[0]);
  node_sliding_.push_back(static_cast<int>(sliding_.size()));
  sliding_.push_back(node);
}

shift::core::body::Flex::SplineWeights shift::core::body::Flex::Weights(
  const SlidingNode& node, double s) {
  SplineWeights w;
  const size_t count = node.stations.size();
  if(count < 2) {
    return w;
  }
  const double h = node.spacing;
  const double last = node.first + h * double(count - 1);
  w.d0 = -1.0 / h;
  w.d1 = 1.0 / h;
  if(s < node.first) {
    /* Y_0 + delta Y'(x_0), Y'(x_0) = (Y_1 - Y_0) / h - h M_1 / 6 */
    const double delta = s - node.first;
    w.i = 0;
    w.i1 = 1;
    w.c0 = 1.0 - delta / h;
    w.c1 = delta / h;
    w.e1 = -delta * h / 6.0;
    w.f1 = -h / 6.0;
    return w;
  }
  if(s > last) {
    /* Y_N + delta Y'(x_N), Y'(x_N) = (Y_N - Y_N-1) / h + h M_N-1 / 6 */
    const double delta = s - last;
    w.i = count - 2;
    w.i1 = count - 1;
    w.c0 = -delta / h;
    w.c1 = 1.0 + delta / h;
    w.e0 = delta * h / 6.0;
    w.f0 = h / 6.0;
    return w;
  }
  w.i = std::min(size_t((s - node.first) / h), count - 2);
  w.i1 = w.i + 1;
  const double t = (s - node.first - h * double(w.i)) / h;
  const double r = 1.0 - t;
  w.c0 = r;
  w.c1 = t;
  w.e0 = h * h / 6.0 * (r * r * r - r);
  w.e1 = h * h / 6.0 * (t * t * t - t);
  w.f0 = -h / 6.0 * (3.0 * r * r - 1.0);
  w.f1 = h / 6.0 * (3.0 * t * t - 1.0);
  w.g0 = r;
  w.g1 = t;
  return w;
}

void shift::core::body::Flex::SetInitialElasticState(
  const std::vector<double>& eta, const std::vector<double>& eta_dot) {
  if(eta.size() != elastic_count_ || eta_dot.size() != elastic_count_) {
    throw shift::core::Exception(
      "The initial elastic state of flexible body '" + get_name() +
      "' must have " + std::to_string(elastic_count_) + " values each");
  }
  initial_eta_ = eta;
  initial_eta_dot_ = eta_dot;
}

void shift::core::body::Flex::Connect() {
  Base::Connect();
  /* a body built from a mesh has its model and nodes from BuildElastic */
  if(builds_model()) {
    return;
  }
  ResolveNodes();
}

void shift::core::body::Flex::ResolveNodes() {
  const size_t point_count = masses_.size();
  if(elastic_count_ == 0 || point_count == 0) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "' has no model");
  }
  node_frames_.assign(node_names_.size(), nullptr);
  for(size_t k = 0; k < node_names_.size(); k++) {
    for(auto& frame : get_frames()) {
      if(frame->get_name() == node_names_[k]) {
        node_frames_[k] = frame.get();
      }
    }
    if(!node_frames_[k] || node_points_[k] >= point_count) {
      throw shift::core::Exception(
        "Flexible body '" + get_name() + "' has no frame '" + node_names_[k] +
        "' or no point " + std::to_string(node_points_[k]) + " for a node");
    }
  }
}

void shift::core::body::Flex::SetupElastic() {
  const size_t n = elastic_count_;
  const size_t point_count = masses_.size();

  /* the mass properties of the undeformed body, in the origin's
  coordinates, for the rigid-body uses of the body's inertia */
  double mass = 0.0;
  ksl_vec3_t first_moment = ksl_vec3(0.0, 0.0, 0.0);
  ksl_mat3x3_t J_o = {};
  for(size_t p = 0; p < point_count; p++) {
    mass += masses_[p];
    AddScaledVector(masses_[p], positions_[p], &first_moment);
    AddScaled(1.0, rotary_inertias_[p], &J_o);
    AddSkewSkew(-masses_[p], positions_[p], positions_[p], &J_o);
  }
  if(!(mass > 0.0)) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "' has no mass");
  }
  ksl_vec3_t center;
  ksl_product_av(1.0 / mass, &first_moment, &center);
  /* centroidal: J_o = I_c - m c~ c~ */
  ksl_mat3x3_t I_c = J_o;
  AddSkewSkew(mass, center, center, &I_c);
  inertia_ = ksl_inertia(mass, center, I_c.m00, I_c.m11, I_c.m22, I_c.m01,
                         I_c.m12, I_c.m02);
  center_of_mass_wrt_origin_ = center;

  /* the model in the reference frame's coordinates */
  const ksl_SE3_t* D_or = get_reference_frame()->get_d_wrt_origin();
  positions_r_.resize(point_count);
  shapes_r_.resize(point_count * n);
  std::vector<ksl_mat3x3_t> rotary_r(point_count);
  for(size_t p = 0; p < point_count; p++) {
    ksl_product_dinvv(D_or, &positions_[p], &positions_r_[p]);
    for(size_t j = 0; j < n; j++) {
      const ksl_screw_t& phi = shapes_[p * n + j];
      ksl_screw_t& phi_r = shapes_r_[p * n + j];
      ksl_product_drinvv(&D_or->R, &phi.lin, &phi_r.lin);
      ksl_product_drinvv(&D_or->R, &phi.ang, &phi_r.ang);
    }
    ksl_mat3x3_t temp;
    ksl_product_drdr(&rotary_inertias_[p], &D_or->R, &temp);
    ksl_product_drinvdr(&D_or->R, &temp, &rotary_r[p]);
  }

  /* the sliding nodes' splines: M_0 = M_N-1 = 0 and, between,
  M_i-1 + 4 M_i + M_i+1 = 6 (Y_i+1 - 2 Y_i + Y_i-1) / h^2, by the Thomas
  algorithm for each of the 6 n_e components */
  for(SlidingNode& node : sliding_) {
    const size_t count = node.stations.size();
    for(unsigned int station : node.stations) {
      if(station >= point_count) {
        throw shift::core::Exception("Flexible body '" + get_name() +
                                     "': a sliding node has no point " +
                                     std::to_string(station));
      }
    }
    node.curvature.assign(count * n, ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    if(count < 3) {
      continue;
    }
    const double h = node.spacing;
    std::vector<double> diagonal(count), right(count);
    for(size_t j = 0; j < n; j++) {
      for(int c = 0; c < 6; c++) {
        auto y = [&](size_t i) {
          return shapes_r_[size_t(node.stations[i]) * n + j].at[c];
        };
        /* forward elimination over the interior rows 1 .. N-2 */
        for(size_t i = 1; i + 1 < count; i++) {
          diagonal[i] = 4.0;
          right[i] = 6.0 * (y(i + 1) - 2.0 * y(i) + y(i - 1)) / (h * h);
          if(i > 1) {
            const double factor = 1.0 / diagonal[i - 1];
            diagonal[i] -= factor;
            right[i] -= factor * right[i - 1];
          }
        }
        for(size_t i = count - 2; i >= 1; i--) {
          const double next =
            i + 2 < count ? node.curvature[(i + 1) * n + j].at[c] : 0.0;
          node.curvature[i * n + j].at[c] = (right[i] - next) / diagonal[i];
        }
      }
    }
  }

  /* the invariants */
  mass_ = mass;
  invariant_1_ = {};
  invariant_4_ = ksl_vec3(0.0, 0.0, 0.0);
  invariant_5_.assign(n, ksl_vec3(0.0, 0.0, 0.0));
  invariant_2s_.assign(n, ksl_mat3x3_t{});
  invariant_3_.assign(n * n, ksl_mat3x3_t{});
  invariant_6_.assign(n, ksl_vec3(0.0, 0.0, 0.0));
  invariant_7_.assign(n * n, ksl_vec3(0.0, 0.0, 0.0));
  invariant_9_.assign(n * n, 0.0);
  for(size_t p = 0; p < point_count; p++) {
    const double m_p = masses_[p];
    const ksl_vec3_t& t = positions_r_[p];
    AddScaled(1.0, rotary_r[p], &invariant_1_);
    AddSkewSkew(-m_p, t, t, &invariant_1_);
    AddScaledVector(m_p, t, &invariant_4_);
    for(size_t j = 0; j < n; j++) {
      const ksl_vec3_t& phi_j = shapes_r_[p * n + j].lin;
      const ksl_vec3_t& psi_j = shapes_r_[p * n + j].ang;
      AddScaledVector(m_p, phi_j, &invariant_5_[j]);
      /* T^2_j + T^2T_j = -m (t~ phi~ + phi~ t~) */
      AddSkewSkew(-m_p, t, phi_j, &invariant_2s_[j]);
      AddSkewSkew(-m_p, phi_j, t, &invariant_2s_[j]);
      /* T^6_j = m t~ phi + I psi */
      ksl_vec3_t temp;
      ksl_cross_vv(&t, &phi_j, &temp);
      AddScaledVector(m_p, temp, &invariant_6_[j]);
      ksl_product_drv(&rotary_r[p], &psi_j, &temp);
      AddScaledVector(1.0, temp, &invariant_6_[j]);
      for(size_t i = 0; i < n; i++) {
        const ksl_vec3_t& phi_i = shapes_r_[p * n + i].lin;
        const ksl_vec3_t& psi_i = shapes_r_[p * n + i].ang;
        AddSkewSkew(-m_p, phi_i, phi_j, &invariant_3_[i * n + j]);
        ksl_cross_vv(&phi_i, &phi_j, &temp);
        AddScaledVector(m_p, temp, &invariant_7_[i * n + j]);
        ksl_vec3_t I_psi_j;
        ksl_product_drv(&rotary_r[p], &psi_j, &I_psi_j);
        invariant_9_[i * n + j] +=
          m_p * ksl_dot_vv(&phi_i, &phi_j) + ksl_dot_vv(&psi_i, &I_psi_j);
      }
    }
  }

  mass_be_.resize(n);
  bias_e_.resize(n);
  std::vector<double> eta_dot(n, 0.0);
  EvaluateElastic(initial_eta_.data(), eta_dot.data(),
                  ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  std::stringstream ss;
  ss << "Flexible body '" << get_name() << "': " << point_count
     << " points, mass " << mass << ", " << n << " elastic coordinates, "
     << node_frames_.size() << " nodes, reference frame '"
     << get_reference_frame()->get_name() << "'";
  SHIFT_DEBUG(ss.str());
}

void shift::core::body::Flex::EvaluateMassBlocks(const double* eta,
                                                 ksl_mat3x3_t* J,
                                                 ksl_vec3_t* s,
                                                 ksl_vec3_t* G) const {
  const size_t n = elastic_count_;
  *J = invariant_1_;
  *s = invariant_4_;
  for(size_t j = 0; j < n; j++) {
    AddScaledVector(eta[j], invariant_5_[j], s);
    AddScaled(eta[j], invariant_2s_[j], J);
    G[j] = invariant_6_[j];
    for(size_t i = 0; i < n; i++) {
      AddScaled(eta[i] * eta[j], invariant_3_[i * n + j], J);
      AddScaledVector(eta[i], invariant_7_[i * n + j], &G[j]);
    }
  }
}

void shift::core::body::Flex::EvaluateElastic(const double* eta,
                                              const double* eta_dot,
                                              const ksl_screw_t& v_r) {
  const size_t n = elastic_count_;
  ksl_mat3x3_t J;
  ksl_vec3_t s;
  std::vector<ksl_vec3_t> G(n);
  EvaluateMassBlocks(eta, &J, &s, G.data());

  /* the frame-frame block in the ten-term form, with centroidal moments:
  J = I_c - (1/m) s~ s~ */
  ksl_mat3x3_t I_c = J;
  AddSkewSkew(1.0 / mass_, s, s, &I_c);
  inertia_wrt_reference_.m = mass_;
  inertia_wrt_reference_.mt = s;
  inertia_wrt_reference_.Ixx = I_c.m00;
  inertia_wrt_reference_.Iyy = I_c.m11;
  inertia_wrt_reference_.Izz = I_c.m22;
  inertia_wrt_reference_.Ixy = I_c.m01;
  inertia_wrt_reference_.Iyz = I_c.m12;
  inertia_wrt_reference_.Izx = I_c.m02;

  for(size_t j = 0; j < n; j++) {
    mass_be_[j].lin = invariant_5_[j];
    mass_be_[j].ang = G[j];
  }

  const ksl_vec3_t& v = v_r.lin;
  const ksl_vec3_t& w = v_r.ang;

  /* s_dot = T^5 eta_dot, G eta_dot, J_dot w */
  ksl_vec3_t s_dot = ksl_vec3(0.0, 0.0, 0.0);
  ksl_vec3_t G_eta_dot = ksl_vec3(0.0, 0.0, 0.0);
  ksl_mat3x3_t J_dot = {};
  for(size_t j = 0; j < n; j++) {
    AddScaledVector(eta_dot[j], invariant_5_[j], &s_dot);
    AddScaledVector(eta_dot[j], G[j], &G_eta_dot);
    AddScaled(eta_dot[j], invariant_2s_[j], &J_dot);
    for(size_t i = 0; i < n; i++) {
      AddScaled(eta_dot[i] * eta[j] + eta[i] * eta_dot[j],
                invariant_3_[i * n + j], &J_dot);
    }
  }
  ksl_vec3_t J_dot_w;
  ksl_product_drv(&J_dot, &w, &J_dot_w);

  /* b = -{2 w x s_dot; w x G eta_dot + J_dot w} */
  ksl_vec3_t temp;
  ksl_cross_vv(&w, &s_dot, &temp);
  ksl_product_av(-2.0, &temp, &bias_b_.lin);
  ksl_cross_vv(&w, &G_eta_dot, &temp);
  ksl_add_vv(&temp, &J_dot_w, &bias_b_.ang);
  ksl_vec3_invert(&bias_b_.ang);

  for(size_t k = 0; k < n; k++) {
    double b = 0.0;
    /* 2 w . sum_j T^7_kj eta_dot_j */
    ksl_vec3_t T7_eta_dot = ksl_vec3(0.0, 0.0, 0.0);
    for(size_t j = 0; j < n; j++) {
      AddScaledVector(eta_dot[j], invariant_7_[k * n + j], &T7_eta_dot);
    }
    b += 2.0 * ksl_dot_vv(&w, &T7_eta_dot);
    /* -v . (T^5_k x w) */
    ksl_cross_vv(&invariant_5_[k], &w, &temp);
    b -= ksl_dot_vv(&v, &temp);
    /* 1/2 w^T (dJ/d eta_k) w = 1/2 w^T Js_k w + sum_j eta_j w^T T^3_kj w */
    ksl_mat3x3_t dJ = {};
    AddScaled(0.5, invariant_2s_[k], &dJ);
    for(size_t j = 0; j < n; j++) {
      AddScaled(eta[j], invariant_3_[k * n + j], &dJ);
    }
    ksl_product_drv(&dJ, &w, &temp);
    b += ksl_dot_vv(&w, &temp);
    /* elastic and damping forces */
    for(size_t j = 0; j < n; j++) {
      b -= stiffness_[k * n + j] * eta[j] + damping_[k * n + j] * eta_dot[j];
    }
    bias_e_[k] = b;
  }
}

double shift::core::body::Flex::KineticEnergy(const double* eta,
                                              const double* eta_dot,
                                              const ksl_screw_t& v_r) const {
  const size_t n = elastic_count_;
  ksl_mat3x3_t J;
  ksl_vec3_t s;
  std::vector<ksl_vec3_t> G(n);
  EvaluateMassBlocks(eta, &J, &s, G.data());
  const ksl_vec3_t& v = v_r.lin;
  const ksl_vec3_t& w = v_r.ang;

  /* 2T = m v.v - 2 v.(s x w) + 2 v.(T^5 eta_dot) + w^T J w
          + 2 w.(G eta_dot) + eta_dot^T T^9 eta_dot */
  ksl_vec3_t temp;
  double two_t = mass_ * ksl_dot_vv(&v, &v);
  ksl_cross_vv(&s, &w, &temp);
  two_t -= 2.0 * ksl_dot_vv(&v, &temp);
  ksl_product_drv(&J, &w, &temp);
  two_t += ksl_dot_vv(&w, &temp);
  for(size_t j = 0; j < n; j++) {
    two_t += 2.0 * eta_dot[j] *
             (ksl_dot_vv(&v, &invariant_5_[j]) + ksl_dot_vv(&w, &G[j]));
    for(size_t i = 0; i < n; i++) {
      two_t += eta_dot[i] * invariant_9_[i * n + j] * eta_dot[j];
    }
  }
  return 0.5 * two_t;
}

double shift::core::body::Flex::ElasticEnergy(const double* eta) const {
  const size_t n = elastic_count_;
  double e = 0.0;
  for(size_t i = 0; i < n; i++) {
    for(size_t j = 0; j < n; j++) {
      e += eta[i] * stiffness_[i * n + j] * eta[j];
    }
  }
  return 0.5 * e;
}

void shift::core::body::Flex::EvaluateSlidingNode(
  unsigned int k, double s, double s_dot, const double* eta,
  const double* eta_dot, ksl_SE3_t* d_rk, ksl_screw_t* h_k,
  ksl_screw_t* gamma_k, ksl_screw_t* h_s, ksl_screw_t* h_s_dot) const {
  if(!is_sliding_node(k)) {
    EvaluateNode(k, eta, eta_dot, d_rk, h_k, gamma_k);
    return;
  }
  const size_t n = elastic_count_;
  const SlidingNode& node = sliding_[size_t(node_sliding_[k])];
  const SplineWeights w = Weights(node, s);
  const ksl_screw_t* y0 = &shapes_r_[size_t(node.stations[w.i]) * n];
  const ksl_screw_t* y1 = &shapes_r_[size_t(node.stations[w.i1]) * n];
  const ksl_screw_t* m0 = &node.curvature[w.i * n];
  const ksl_screw_t* m1 = &node.curvature[w.i1 * n];
  const bool rates = eta_dot != nullptr;

  /* the deformation at the station and its derivatives in s: u, theta (and
  ', ''), and with the rates: rho_e, Theta_dot (Upsilon eta_dot) and U',
  Theta' (Upsilon' eta_dot) */
  ksl_vec3_t u = {}, theta = {}, u1 = {}, theta1 = {}, u2 = {}, theta2 = {};
  ksl_vec3_t rho_e = {}, Theta = {}, U1 = {}, Theta1 = {};
  for(size_t j = 0; j < n; j++) {
    ksl_screw_t y, d, dd;
    for(int c = 0; c < 6; c++) {
      const double a0 = y0[j].at[c], a1 = y1[j].at[c];
      const double b0 = m0[j].at[c], b1 = m1[j].at[c];
      y.at[c] = w.c0 * a0 + w.c1 * a1 + w.e0 * b0 + w.e1 * b1;
      d.at[c] = w.d0 * a0 + w.d1 * a1 + w.f0 * b0 + w.f1 * b1;
      dd.at[c] = w.g0 * b0 + w.g1 * b1;
    }
    AddScaledVector(eta[j], y.lin, &u);
    AddScaledVector(eta[j], y.ang, &theta);
    AddScaledVector(eta[j], d.lin, &u1);
    AddScaledVector(eta[j], d.ang, &theta1);
    AddScaledVector(eta[j], dd.lin, &u2);
    AddScaledVector(eta[j], dd.ang, &theta2);
    if(rates) {
      AddScaledVector(eta_dot[j], y.lin, &rho_e);
      AddScaledVector(eta_dot[j], y.ang, &Theta);
      AddScaledVector(eta_dot[j], d.lin, &U1);
      AddScaledVector(eta_dot[j], d.ang, &Theta1);
    }
    if(h_k) {
      h_k[j] = y;
    }
  }

  /* the station p(s) = t_0 + s a, moved: rho = p + u, turned by theta */
  const ksl_SE3_t* D_0 = node_frames_[k]->get_d_wrt_reference();
  ksl_vec3_t a;
  ksl_product_drv(&D_0->R, &node.axis, &a);
  ksl_vec3_t rho = D_0->t;
  AddScaledVector(s, a, &rho);
  AddScaledVector(1.0, u, &rho);
  ksl_mat3x3_t R_theta;
  const double angle = ksl_vec3_l2norm(&theta);
  if(angle > 1e-300) {
    ksl_vec3_t direction;
    ksl_product_av(1.0 / angle, &theta, &direction);
    ksl_mat3x3_setFromAxisAngle(&R_theta, &direction, angle);
  } else {
    ksl_mat3x3_setIdentity(&R_theta);
  }
  /* the frame: offset -s a from the station, turned with it */
  ksl_vec3_t offset, offset_turned;
  ksl_product_av(-s, &a, &offset);
  ksl_product_drv(&R_theta, &offset, &offset_turned);
  ksl_product_drdr(&R_theta, &D_0->R, &d_rk->R);
  ksl_add_vv(&rho, &offset_turned, &d_rk->t);

  if(h_k) {
    for(size_t j = 0; j < n; j++) {
      ksl_vec3_t moment;
      ksl_cross_vv(&rho, &h_k[j].ang, &moment);
      AddScaledVector(1.0, moment, &h_k[j].lin);
    }
  }
  /* rho_dot = (a + u') s_dot + rho_e */
  ksl_vec3_t rho_dot = rho_e;
  AddScaledVector(s_dot, a, &rho_dot);
  AddScaledVector(s_dot, u1, &rho_dot);
  if(gamma_k) {
    /* {rho_dot x Theta_dot + s_dot (U' + rho x Theta'); s_dot Theta'} */
    ksl_vec3_t temp;
    ksl_cross_vv(&rho_dot, &Theta, &gamma_k->lin);
    AddScaledVector(s_dot, U1, &gamma_k->lin);
    ksl_cross_vv(&rho, &Theta1, &temp);
    AddScaledVector(s_dot, temp, &gamma_k->lin);
    ksl_product_av(s_dot, &Theta1, &gamma_k->ang);
  }
  if(h_s) {
    /* {a + u' + rho x theta'; theta'} */
    ksl_cross_vv(&rho, &theta1, &h_s->lin);
    AddScaledVector(1.0, a, &h_s->lin);
    AddScaledVector(1.0, u1, &h_s->lin);
    h_s->ang = theta1;
  }
  if(h_s_dot) {
    /* {s_dot u'' + U' + rho_dot x theta' + rho x (s_dot theta'' + Theta');
    s_dot theta'' + Theta'} */
    ksl_vec3_t angular = Theta1, temp;
    AddScaledVector(s_dot, theta2, &angular);
    ksl_cross_vv(&rho_dot, &theta1, &h_s_dot->lin);
    AddScaledVector(s_dot, u2, &h_s_dot->lin);
    AddScaledVector(1.0, U1, &h_s_dot->lin);
    ksl_cross_vv(&rho, &angular, &temp);
    AddScaledVector(1.0, temp, &h_s_dot->lin);
    h_s_dot->ang = angular;
  }
}

void shift::core::body::Flex::EvaluateNode(unsigned int k, const double* eta,
                                           const double* eta_dot,
                                           ksl_SE3_t* d_rk, ksl_screw_t* h_k,
                                           ksl_screw_t* gamma_k) const {
  const size_t n = elastic_count_;
  const unsigned int p = node_points_[k];
  const ksl_screw_t* phi = &shapes_r_[p * n];

  /* the point's deformed position and rotation vector */
  ksl_vec3_t rho = positions_r_[p];
  ksl_vec3_t theta = ksl_vec3(0.0, 0.0, 0.0);
  for(size_t j = 0; j < n; j++) {
    AddScaledVector(eta[j], phi[j].lin, &rho);
    AddScaledVector(eta[j], phi[j].ang, &theta);
  }

  /* the node frame is carried rigidly by the point: turned by theta, and
  its offset from the point turned with it */
  ksl_mat3x3_t R_theta;
  const double angle = ksl_vec3_l2norm(&theta);
  if(angle > 1e-300) {
    ksl_vec3_t axis;
    ksl_product_av(1.0 / angle, &theta, &axis);
    ksl_mat3x3_setFromAxisAngle(&R_theta, &axis, angle);
  } else {
    ksl_mat3x3_setIdentity(&R_theta);
  }
  const ksl_SE3_t* D_0 = node_frames_[k]->get_d_wrt_reference();
  ksl_vec3_t offset, offset_turned;
  ksl_subtract_vv(&D_0->t, &positions_r_[p], &offset);
  ksl_product_drv(&R_theta, &offset, &offset_turned);
  ksl_product_drdr(&R_theta, &D_0->R, &d_rk->R);
  ksl_add_vv(&rho, &offset_turned, &d_rk->t);

  /* in r, referred to its origin: h_j = {phi_j + rho x psi_j; psi_j} */
  if(h_k) {
    for(size_t j = 0; j < n; j++) {
      ksl_cross_vv(&rho, &phi[j].ang, &h_k[j].lin);
      AddScaledVector(1.0, phi[j].lin, &h_k[j].lin);
      h_k[j].ang = phi[j].ang;
    }
  }
  /* sum_j h_dot_j eta_dot_j = {rho_dot x theta_dot; 0} */
  if(gamma_k) {
    ksl_vec3_t rho_dot = ksl_vec3(0.0, 0.0, 0.0);
    ksl_vec3_t theta_dot = ksl_vec3(0.0, 0.0, 0.0);
    for(size_t j = 0; j < n; j++) {
      AddScaledVector(eta_dot[j], phi[j].lin, &rho_dot);
      AddScaledVector(eta_dot[j], phi[j].ang, &theta_dot);
    }
    ksl_cross_vv(&rho_dot, &theta_dot, &gamma_k->lin);
    gamma_k->ang = ksl_vec3(0.0, 0.0, 0.0);
  }
}

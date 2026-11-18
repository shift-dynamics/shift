#ifndef SHIFT_BODY_FLEX_H_
#define SHIFT_BODY_FLEX_H_

#include <array>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/coscrew.h"
#include "ksl/inertia.h"
#include "ksl/screw.h"

#include "shift/core/body/base.h"
#include "shift/core/parameter/array.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {
namespace body {

/*!
@brief an entry of a flexible body's attachments: how one of its frames is
tied to the finite element mesh, overriding the automatic choice (see
fea::AttachmentRequest)
*/
class FlexAttachment : public parameter::List {
public:
  FlexAttachment() {
    DeclareParameters();
  }
  virtual ~FlexAttachment() {}

  static const std::string get_array_key() {
    return "attachments";
  }

  std::string frame;
  std::string spider = "rbe3";
  std::vector<int> faces;
  double radius = 0.0;
  double bearing_length = 0.0;

protected:
  virtual void DeclareParameters() override;
};

/*! \brief what an attachment of a flexible body built from a mesh ties,
 * for the viewer: its frame, what uses the frame, its spider, and whether
 * it holds the body (the reference frame's attachment, which has no static
 * modes) */
struct AttachmentInfo {
  std::string frame;
  std::string user; /*!< e.g. "spherical joint 'root'", "chord" */
  bool rbe2 = false;
  bool reference = false;
  /*! of a sliding attachment: its station (from 0) of count; -1 otherwise */
  int station = -1;
  int station_count = 0;
};

/*! \brief where the child of a sliding node bears on the bore: the
 * intervals [a, b] of offsets along the axis from the child's frame
 * (none: at its frame alone), the radius of the bore, and its ends, along
 * the axis from the sliding frame, beyond which the child bears on
 * nothing */
struct SlidingBearing {
  std::vector<std::array<double, 2>> intervals;
  double radius = 0.0;
  double low = -1e300;
  double high = 1e300;
};

/*!
@brief A flexible body in the floating frame of reference: the rigid motion of
its reference frame r plus a small elastic deformation, described by
\f$n_e\f$ elastic coordinates \f$\underline{\eta}\f$

The body is a set of lumped points \f$p\f$ (the nodes of a finite element
model): the mass \f$m_p\f$, the rotary inertia \f$\bar{I}_p\f$ and the
undeformed position \f$\underline{t}_p\f$ of each, and its slice of the shape
matrix, the translations \f$\Upsilon_p^t\f$ and rotations \f$\Upsilon_p^r\f$
(\f$3 \times n_e\f$ each) of the point per unit elastic coordinate. In r, the
point is at \f$\underline{\rho}_p = \underline{t}_p +
\Upsilon_p^t\underline{\eta}\f$ and turns with the angular velocity
\f$\underline{\omega} + \Upsilon_p^r\dot{\underline{\eta}}\f$. The rotation of
the points is linear in \f$\underline{\eta}\f$ throughout: the rotary inertia
stays fixed in r, and a node's orientation is the rotation by the vector
\f$\Upsilon_p^r\underline{\eta}\f$, whose rate differs from
\f$\Upsilon_p^r\dot{\underline{\eta}}\f$ only at second order in the
deformation. The modes are linear about the undeformed state, so there is no
geometric (stress) stiffening.

The shape matrix must hold r still: modes with the reference frame's point
fixed (an attached frame, or Craig–Bampton modes with that point in the
boundary set), so that the elastic coordinates do not repeat the frame's six
motions. body::Modal and body::Nodal differ only in how the shape matrix is
made; this class takes it as given (SetModel), in the coordinates of the
body's origin, with the elastic stiffness and damping matrices
\f$K^{\eta}\f$, \f$C^{\eta}\f$, for any normalization of the shapes.

With the velocity \f$\mathbf{v}_r = \{\underline{v};\ \underline{\omega}\}\f$
of r in its own coordinates, the kinetic energy \f$\frac{1}{2}\sum_p (m_p
|\underline{v} + \underline{\omega} \times \underline{\rho}_p +
\Upsilon_p^t\dot{\underline{\eta}}|^2 + (\underline{\omega} +
\Upsilon_p^r\dot{\underline{\eta}})^T \bar{I}_p (\underline{\omega} +
\Upsilon_p^r\dot{\underline{\eta}}))\f$ gives the mass matrix

\f[
M = \begin{bmatrix}
m I & -\tilde{\underline{s}} & \Upsilon^t \\
\tilde{\underline{s}} & J & G \\
\Upsilon^{tT} & G^T & M_{ee}
\end{bmatrix}
\f]

with the invariants, sums over the points,

\f[
\begin{aligned}
m &= \sum_p m_p, \quad
\underline{s} = \underline{T}^4 + T^5\underline{\eta}, \quad
\underline{T}^4 = \sum_p m_p\underline{t}_p, \quad
T^5 = \sum_p m_p\Upsilon_p^t = \Upsilon^t \\
J &= T^1 + \sum_j (T_j^2 + T_j^{2T})\eta_j + \sum_{ij} T_{ij}^3\eta_i\eta_j,
\quad T^1 = \sum_p (\bar{I}_p - m_p\tilde{\underline{t}}_p^2), \quad
T_j^2 = -\sum_p m_p\tilde{\underline{t}}_p\tilde{\underline{\phi}}_{pj}, \quad
T_{ij}^3 = -\sum_p m_p\tilde{\underline{\phi}}_{pi}\tilde{\underline{\phi}}_{pj} \\
\underline{G}_j &= \underline{T}_j^6 + \sum_i \underline{T}_{ij}^7\eta_i,
\quad \underline{T}_j^6 = \sum_p (m_p\tilde{\underline{t}}_p
\underline{\phi}_{pj} + \bar{I}_p\underline{\psi}_{pj}), \quad
\underline{T}_{ij}^7 = \sum_p m_p\tilde{\underline{\phi}}_{pi}
\underline{\phi}_{pj} \\
M_{ee} &= T^9 = \sum_p (m_p\Upsilon_p^{tT}\Upsilon_p^t +
\Upsilon_p^{rT}\bar{I}_p\Upsilon_p^r)
\end{aligned}
\f]

where \f$\underline{\phi}_{pj}\f$ and \f$\underline{\psi}_{pj}\f$ are column j
of \f$\Upsilon_p^t\f$ and \f$\Upsilon_p^r\f$, and \f$T_{ji}^7 =
-T_{ij}^7\f$. Kirchhoff's equations for r, with the rates
\f$\dot{\mathbf{v}}_r\f$ of its velocity in its own coordinates (the spatial
acceleration the solvers use, in r), and Lagrange's equations for
\f$\underline{\eta}\f$ give

\f[
M \begin{Bmatrix} \dot{\mathbf{v}}_r \\ \ddot{\underline{\eta}}
\end{Bmatrix} = \begin{Bmatrix} \mathbf{f} - \mathbf{v}_r \times^*
M_{bb}\mathbf{v}_r + \mathbf{b} \\ \underline{Q} + \underline{b}_e
\end{Bmatrix}
\f]

where the solvers form the gyroscopic wrench of the frame-frame block
\f$M_{bb}\f$ as for a rigid body, and, with \f$\dot{\underline{s}} =
T^5\dot{\underline{\eta}}\f$,

\f[
\begin{aligned}
\mathbf{b} &= -\begin{Bmatrix} 2\underline{\omega} \times
\dot{\underline{s}} \\ \underline{\omega} \times G\dot{\underline{\eta}} +
\dot{J}\underline{\omega} \end{Bmatrix} \\
b_{e,k} &= 2\underline{\omega}^T\sum_j \underline{T}_{kj}^7\dot{\eta}_j -
\underline{v}^T(\underline{T}_k^5 \times \underline{\omega}) +
\frac{1}{2}\underline{\omega}^T \frac{\partial J}{\partial \eta_k}
\underline{\omega} - (K^{\eta}\underline{\eta})_k -
(C^{\eta}\dot{\underline{\eta}})_k
\end{aligned}
\f]

(the terms \f$\tilde{\underline{v}}\dot{\underline{s}} +
\tilde{\dot{\underline{s}}}\underline{v}\f$ cancel in the angular row, and
\f$\sum_{ij}\underline{T}_{ij}^7\dot{\eta}_i\dot{\eta}_j = 0\f$).

The nodes are the frames of the body that move with a point, set by SetNode;
a node frame is carried rigidly by its point.
*/
class Flex : public Base {
public:
  Flex()
    : Base() {}
  Flex(std::string name)
    : Base(name) {}
  virtual ~Flex() override {}

  /*! \brief set the lumped model in the coordinates of the body's origin:
   * \p positions, \p masses and \p rotary_inertias of the points (the latter
   * in the origin's axes, about the point); \p shapes, the shape matrix,
   * points.size() * \p elastic_count screws, the screw of point p and
   * coordinate j at p * elastic_count + j, its linear part the point's
   * translation and its angular part its rotation; \p stiffness and
   * \p damping, \f$K^{\eta}\f$ and \f$C^{\eta}\f$, elastic_count^2 each,
   * row-major (damping may be empty) */
  void SetModel(const std::vector<ksl_vec3_t>& positions,
                const std::vector<double>& masses,
                const std::vector<ksl_mat3x3_t>& rotary_inertias,
                unsigned int elastic_count,
                const std::vector<ksl_screw_t>& shapes,
                const std::vector<double>& stiffness,
                const std::vector<double>& damping);

  /*! \brief make the frame named \p frame_name a node, carried by \p point */
  void SetNode(const std::string& frame_name, unsigned int point);

  /*! \brief make the frame named \p frame_name a sliding node (the system
   * parent frame of a prismatic joint whose child slides along a bore of
   * the body): carried by the bore's \p stations, points at s = \p first +
   * i \p spacing (i = 0, 1, ...) along \p axis (a unit vector in the frame's
   * coordinates) from the frame, their shapes interpolated in s by a natural
   * cubic spline, continued linearly beyond the first and the last (so that
   * the interpolation has continuous first and second derivatives). The
   * child moves with the mean of the interpolated shapes over the part of
   * its \p bearing within the bore (SlidingCarry), at the middle of its
   * bearing (BearingCenter). */
  void SetSlidingNode(const std::string& frame_name,
                      const std::vector<unsigned int>& stations, double first,
                      double spacing, const ksl_vec3_t& axis,
                      const SlidingBearing& bearing = {});
  bool is_sliding_node(unsigned int k) const override {
    return k < node_sliding_.size() && node_sliding_[k] >= 0;
  }
  /*! \brief where the child of sliding node k bears on its bore, nullptr
   * for a node that does not slide */
  const SlidingBearing* get_sliding_bearing(unsigned int k) const {
    return is_sliding_node(k) ? &sliding_[size_t(node_sliding_[k])].bearing
                              : nullptr;
  }
  void EvaluateSlidingNode(unsigned int k, double s, double s_dot,
                           const double* eta, const double* eta_dot,
                           ksl_SE3_t* d_rk, ksl_screw_t* h_k,
                           ksl_screw_t* gamma_k, ksl_screw_t* h_s,
                           ksl_screw_t* h_s_dot) const override;

  /*! \brief set the initial elastic coordinates and their rates,
   * elastic_count each */
  void SetInitialElasticState(const std::vector<double>& eta,
                              const std::vector<double>& eta_dot);

  unsigned int get_point_count() const {
    return (unsigned int)masses_.size();
  }

  unsigned int get_elastic_count() const override {
    return elastic_count_;
  }
  const double* get_initial_elastic_coordinates() const override {
    return initial_eta_.data();
  }
  const double* get_initial_elastic_velocities() const override {
    return initial_eta_dot_.data();
  }
  unsigned int get_node_count() const override {
    return (unsigned int)node_points_.size();
  }
  Frame* get_node_frame(unsigned int k) const override {
    return node_frames_[k];
  }

  /*! \brief find the node frames by name */
  void Connect() override;
  /*! \brief with fe_geometry: mesh the geometry, tie the frames used to the
   * mesh, reduce it (fixed-interface modes if HasFixedInterfaceModes) and
   * set the model and its nodes */
  void BuildElastic(Frame* reference,
                    const std::vector<FrameUse>& uses) override;
  /*! \brief whether the body makes its model from fe_geometry */
  bool builds_model() const {
    return !fe_geometry_.empty();
  }
  /*! \brief the natural frequencies (Hz) of the fixed-interface modes
   * computed, ascending, kept or not (empty unless built from a mesh) */
  const std::vector<double>& get_frequencies() const {
    return frequencies_;
  }
  /*! \brief the kept fixed-interface modes, indices into get_frequencies */
  const std::vector<unsigned int>& get_modes_kept() const {
    return kept_;
  }
  /*! \brief the number of static (attachment and retained-node) modes,
   * which come first among the elastic coordinates */
  unsigned int get_static_count() const {
    return static_count_;
  }
  /*! \brief the boundary of the mesh (built from a mesh): 6-node triangles
   * of points (corners, then mid-sides of edges 01, 12, 20), outward */
  const std::vector<std::array<int, 6>>& get_mesh_boundary() const {
    return mesh_boundary_;
  }
  /*! \brief the B-rep face of each boundary triangle */
  const std::vector<int>& get_mesh_boundary_face() const {
    return mesh_boundary_face_;
  }
  /*! \brief the mesh nodes (points) each attachment ties to its frame, in
   * the order of the frames attached, the reference frame first (built
   * from a mesh) */
  const std::vector<std::vector<int>>& get_attachment_nodes() const {
    return attachment_nodes_;
  }
  /*! \brief what each attachment ties, in the order of
   * get_attachment_nodes; attachment k's frame is the point
   * get_mesh_node_count() + k */
  const std::vector<AttachmentInfo>& get_attachment_info() const {
    return attachment_info_;
  }
  /*! \brief the nodes on the mesh's surface, ascending (built from a
   * mesh) */
  const std::vector<int>& get_surface_nodes() const {
    return surface_nodes_;
  }
  /*! \brief the stress at each node of get_surface_nodes in each shape
   * (fea::NodalStresses): the elastic coordinates', then the fixed-interface
   * modes computed (get_computed_shapes), as \f$(\sigma_{xx}, \sigma_{yy},
   * \sigma_{zz}, \tau_{xy}, \tau_{yz}, \tau_{zx})\f$ of node i in shape j at
   * 6 (i (get_elastic_count() + get_frequencies().size()) + j); the stress
   * of a deformation is linear in its coordinates */
  const std::vector<double>& get_surface_stresses() const {
    return surface_stresses_;
  }
  /*! \brief the yield strength of the material, in its units of stress (0:
   * not given); the viewer draws the safety factor against it. A key that
   * may change while the model runs */
  double get_yield_strength() const {
    return yield_strength_;
  }
  /*! \brief the points that are mesh nodes, the first of the points (the
   * attachments' points follow them); 0 unless built from a mesh */
  unsigned int get_mesh_node_count() const {
    return mesh_node_count_;
  }
  /*! \brief the translations of the mesh nodes in each fixed-interface
   * mode computed, kept or not, in the origin's coordinates: x, y, z of node
   * n in mode k at 3 (n * get_frequencies().size() + k) */
  const std::vector<double>& get_computed_shapes() const {
    return computed_shapes_;
  }
  /*! \brief whether the finite element analysis was read from its cache
   * (fe_cache) rather than run */
  bool from_cache() const {
    return from_cache_;
  }
  /*! \brief the name of the geometry meshed, empty if the model is given */
  const std::string& get_fe_geometry() const {
    return fe_geometry_;
  }
  /*! \brief the points' undeformed positions and the shape matrix
   * (SetModel), in the origin's coordinates */
  const std::vector<ksl_vec3_t>& get_positions() const {
    return positions_;
  }
  const std::vector<ksl_screw_t>& get_shapes() const {
    return shapes_;
  }
  void SetupElastic() override;
  void EvaluateElastic(const double* eta, const double* eta_dot,
                       const ksl_screw_t& v_r) override;
  ksl_inertia_t get_inertia_wrt_reference() override {
    return inertia_wrt_reference_;
  }
  const ksl_coscrew_t* get_mass_be() const override {
    return mass_be_.data();
  }
  const double* get_mass_ee() const override {
    return invariant_9_.data();
  }
  ksl_coscrew_t get_bias_b() const override {
    return bias_b_;
  }
  const double* get_bias_e() const override {
    return bias_e_.data();
  }
  void EvaluateNode(unsigned int k, const double* eta, const double* eta_dot,
                    ksl_SE3_t* d_rk, ksl_screw_t* h_k,
                    ksl_screw_t* gamma_k) const override;

  /*! \brief the kinetic energy of the body for the velocity \p v_r of its
   * reference frame (in its coordinates) and the elastic state, from the
   * mass matrix above; and the elastic potential energy
   * \f$\frac{1}{2}\underline{\eta}^T K^{\eta}\underline{\eta}\f$ */
  double KineticEnergy(const double* eta, const double* eta_dot,
                       const ksl_screw_t& v_r) const;
  double ElasticEnergy(const double* eta) const;

  /*! \brief \f$K^{\eta}\f$ and \f$C^{\eta}\f$, \f$n_e^2\f$ each,
   * row-major */
  const std::vector<double>& get_stiffness() const {
    return stiffness_;
  }
  const std::vector<double>& get_damping() const {
    return damping_;
  }

protected:
  /*! the keys of a body built from a finite element model, common to Modal
   * and Nodal */
  void DeclareFiniteElementParameters();
  /*! find the node frames by name; throws if there is no model */
  void ResolveNodes();
  /*! Modal keeps fixed-interface modes; Nodal only static ones */
  virtual bool HasFixedInterfaceModes() const {
    return false;
  }
  /* finite element keys */
  double youngs_modulus_ = 0.0;
  double poisson_ratio_ = 0.0;
  double yield_strength_ = 0.0;
  std::string fe_geometry_;
  std::string fe_cache_;
  double element_size_ = 0.0;
  double minimum_element_size_ = 0.0;
  parameter::Array<FlexAttachment> attachments_;
  int mode_count_ = 0;
  double maximum_frequency_ = 0.0;
  std::vector<int> modes_kept_;
  double damping_ratio_ = 0.0;
  std::vector<double> damping_ratios_;
  std::vector<double> initial_elastic_coordinates_;
  std::vector<double> initial_elastic_velocities_;
  std::vector<double> retained_points_; /*!< x, y, z of each, row by row */
  /* results of BuildElastic */
  std::vector<double> frequencies_;
  std::vector<unsigned int> kept_;
  unsigned int static_count_ = 0;
  std::vector<std::array<int, 6>> mesh_boundary_;
  std::vector<int> mesh_boundary_face_;
  unsigned int mesh_node_count_ = 0;
  std::vector<std::vector<int>> attachment_nodes_;
  std::vector<AttachmentInfo> attachment_info_;
  std::vector<int> surface_nodes_;
  std::vector<double> surface_stresses_;
  bool from_cache_ = false;
  std::vector<double> computed_shapes_;

  /*! the rotational block J of the mass matrix about r's origin, the first
   * moment s, and the columns G_j, for \p eta */
  void EvaluateMassBlocks(const double* eta, ksl_mat3x3_t* J, ksl_vec3_t* s,
                          ksl_vec3_t* G) const;

  unsigned int elastic_count_ = 0;

  /* the model as given, in the origin's coordinates */
  std::vector<ksl_vec3_t> positions_;
  std::vector<double> masses_;
  std::vector<ksl_mat3x3_t> rotary_inertias_;
  std::vector<ksl_screw_t> shapes_;
  std::vector<double> stiffness_;
  std::vector<double> damping_;

  std::vector<std::string> node_names_;
  std::vector<unsigned int> node_points_;
  std::vector<Frame*> node_frames_;

  /*! a sliding node's stations (SetSlidingNode) and, set by SetupElastic,
   * the second derivatives in s of their shapes in r, the spline's
   * \f$M_i\f$, at i n_e + j */
  struct SlidingNode {
    std::vector<unsigned int> stations;
    double first = 0.0;
    double spacing = 0.0;
    ksl_vec3_t axis = {{0.0, 0.0, 1.0}};
    SlidingBearing bearing;
    /*! BearingCenter of the bearing */
    double center = 0.0;
    std::vector<ksl_screw_t> curvature;
  };
  /*! the weights of the spline at s: the value is c0 Y_i + c1 Y_i1 + e0 M_i
   * + e1 M_i1, its derivative d0 Y_i + d1 Y_i1 + f0 M_i + f1 M_i1, its
   * second derivative g0 M_i + g1 M_i1 */
  struct SplineWeights {
    size_t i = 0, i1 = 0;
    double c0 = 1.0, c1 = 0.0, e0 = 0.0, e1 = 0.0;
    double d0 = 0.0, d1 = 0.0, f0 = 0.0, f1 = 0.0;
    double g0 = 0.0, g1 = 0.0;
  };
  static SplineWeights Weights(const SlidingNode& node, double s);
  /*! add \p scale times the spline's value (AddValue), its derivative
   * (AddSlope), its integral from the first station (AddIntegral) or the
   * integral of x times it (AddMoment) at \p x, as weights of the
   * stations' values \p y and curvatures \p m */
  static void AddValue(const SlidingNode& node, double x, double scale,
                       std::vector<double>* y, std::vector<double>* m);
  static void AddSlope(const SlidingNode& node, double x, double scale,
                       std::vector<double>* y, std::vector<double>* m);
  static void AddIntegral(const SlidingNode& node, double x, double scale,
                          std::vector<double>* y, std::vector<double>* m);
  static void AddMoment(const SlidingNode& node, double x, double scale,
                        std::vector<double>* y, std::vector<double>* m);

  /*! what carries the child of a sliding node at s, with first and second
   * derivatives in s (k = 0, 1, 2), as weights of the stations' values
   * \f$Y_i\f$ and curvatures \f$M_i\f$ of the spline \f$Y(x)\f$. With no
   * bearing in contact (point), \f$\sum_i y_{k,i} Y_i + m_{k,i} M_i\f$ is
   * the spline at the child's point x_c = s + center and its derivatives.
   * Otherwise, over the parts of the bearing within the bore, it is the
   * integral \f$F = \int Y\,dx\f$, and with py, pm the moment \f$P = \int
   * x Y\,dx\f$; length, first and second the integrals of 1, x and
   * \f$x^2\f$ (CarryShape) */
  struct Carried {
    bool point = true;
    std::vector<double> y[3];
    std::vector<double> m[3];
    std::vector<double> py[3];
    std::vector<double> pm[3];
    double length[3] = {0.0, 0.0, 0.0};
    double first[3] = {0.0, 0.0, 0.0};
    double second[3] = {0.0, 0.0, 0.0};
    double x_c = 0.0;
  };
  static Carried Carry(const SlidingNode& node, double s);
  /*! the motion that carries the child, and its first and second
   * derivatives in s, \p out[k], of one shape with values \p y and
   * curvatures \p m at the stations, \p a the axis in their coordinates.
   * In contact, the mean motion of the bore's wall under the bearing (as
   * an averaging attachment of its nodes): a section at x of the wall, of
   * radius r, moves with the spline's translation u(x) and turns with its
   * rotation theta(x); over the bearing, of length l, middle \f$\bar{x}\f$
   * and \f$I = \int (x - \bar{x})^2 dx\f$, the mean translation is
   * \f$\bar{u} = \int u\,dx / l\f$ and the mean rotation, by least
   * squares, along the axis \f$\int \theta_a dx / l\f$ and across it
   * \f$(\underline{a} \times \int (x - \bar{x}) \underline{u}\,dx +
   * \frac{r^2}{2} \int \underline{\theta}_\perp dx) / (I + \frac{r^2}{2}
   * l)\f$; at the child's point, \f$\underline{u}_c = \bar{u} + (x_c -
   * \bar{x})\,\underline{\theta} \times \underline{a}\f$. */
  static void CarryShape(const SlidingNode& node, const Carried& carried,
                         const ksl_vec3_t& a,
                         const std::vector<ksl_screw_t>& y,
                         const std::vector<ksl_screw_t>& m,
                         ksl_screw_t out[3]);

public:
  /*! \brief the second derivatives M_i of the natural cubic spline through
   * the \p y.size() values y_i at spacing \p h (M_0 = M_N-1 = 0, and zero
   * throughout for fewer than three values) */
  static std::vector<double> SplineCurvatures(const std::vector<double>& y,
                                              double h);
  /*! \brief how a sliding node of \p count stations, the first at \p
   * first and \p spacing apart along the unit vector \p axis, carries a
   * child at \p s with \p bearing: the \p derivative-th derivative in s
   * (0, 1 or 2) of the child's motion (translation, then rotation) per
   * unit motion of each station, 6 x 6 count, row-major (CarryShape).
   * Linear beyond the ends, as EvaluateSlidingNode. */
  static std::vector<double> SlidingMap(size_t count, double first,
                                        double spacing, const ksl_vec3_t& axis,
                                        double s,
                                        const SlidingBearing& bearing = {},
                                        int derivative = 0);
  /*! \brief the middle of a bearing along the axis from the child's frame,
   * weighted by length (0 for none) */
  static double BearingCenter(const SlidingBearing& bearing);

private:
  /*! for each node, its index in sliding_, or -1 */
  std::vector<int> node_sliding_;
  std::vector<SlidingNode> sliding_;

  std::vector<double> initial_eta_;
  std::vector<double> initial_eta_dot_;

  /* in the reference frame's coordinates, set by SetupElastic */
  std::vector<ksl_vec3_t> positions_r_;
  std::vector<ksl_screw_t> shapes_r_;
  double mass_ = 0.0;
  ksl_mat3x3_t invariant_1_ = {};
  ksl_vec3_t invariant_4_ = {};
  std::vector<ksl_vec3_t> invariant_5_;     /*!< n_e */
  std::vector<ksl_mat3x3_t> invariant_2s_;  /*!< T^2_j + T^2T_j, n_e */
  std::vector<ksl_mat3x3_t> invariant_3_;   /*!< n_e^2, T^3_ij at i n_e + j */
  std::vector<ksl_vec3_t> invariant_6_;     /*!< n_e */
  std::vector<ksl_vec3_t> invariant_7_;     /*!< n_e^2, T^7_ij at i n_e + j */
  std::vector<double> invariant_9_;         /*!< n_e^2 */

  /* the last evaluation */
  ksl_inertia_t inertia_wrt_reference_ = {};
  std::vector<ksl_coscrew_t> mass_be_;
  ksl_coscrew_t bias_b_ = {};
  std::vector<double> bias_e_;
};

} // namespace body
} // namespace core
} // namespace shift

#endif

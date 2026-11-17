#ifndef SHIFT_DEM_DEM_H_
#define SHIFT_DEM_DEM_H_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "shift/core/sph/sph.h"

namespace shift {
namespace core {
namespace dem {

/*!
@brief granular material as rigid grains in contact (the discrete element
method, DEM), after Chrono::GPU and Chrono DEM-Engine; see the theory page
"Discrete Element Method"

A grain is a clump of one or more spheres, rigid together, or a convex
hull (Hull), made from a Template that holds the spheres or the hull, the
mass and the principal moments of inertia. Each grain carries the position of its center of mass, its
orientation \f$\hat{e}^{fa}\f$ (scalar last), its velocity
\f$\underline{v}_{fa}^{f}\f$ and its angular velocity
\f$\underline{\omega}_{fa}^{a}\f$ in its own coordinates, along its
principal axes.

Spheres of different grains in contact, with overlap \f$d\f$, normal
\f$\underline{n}\f$ (from the second to the first) and relative velocity
\f$\underline{u}\f$ at the contact, push with the Hertz–Mindlin law of
Chrono, with \f$r_c = \sqrt{r^* d}\f$:
\f[
f_n = \tfrac{4}{3} e^* r_c\, d - c_n u_n, \quad
c_n = -2\sqrt{\tfrac{5}{6}}\,\beta\sqrt{2 e^* r_c\, m^*}, \quad
\beta = \frac{\ln\varepsilon}{\sqrt{\ln^2\varepsilon + \pi^2}}
\f]
and rub with a spring and damper on the tangential displacement
\f$\underline{\xi}\f$ of the contact since it formed, bounded by Coulomb's
law:
\f[
\underline{f}_t = -8 g^* r_c\,\underline{\xi} - c_t\,\underline{u}_t, \quad
|\underline{f}_t| \le \mu |f_n|
\f]
Rolling resistance, the constant torque model of Chrono::GPU, is a moment
\f$\mu_r |f_n| r^*\f$ on each grain, about the tangent part of the
other's angular velocity less its own, equal and opposite on the two, once
the contact has lasted longer than a collision. The Young's modulus and
Poisson's ratio are per material; the restitution, friction and rolling
resistance of a pair are the average of the two materials' unless given
for the pair (PairProperties), as in DEM-Engine.

Walls are fixed planes: a sphere touches one when it crosses it, with the
wall's radius and mass infinite.

Bodies are triangle meshes fixed in moving bodies (Body), whose motion is
given (SetBodyPaths): a sphere touches a triangle where it overlaps it, at
the point of the triangle nearest its center, with the body's radius and
mass infinite. Where a sphere touches several triangles of a body at the
same point (an edge or a vertex they share), the contact counts once. The
forces of the spheres on each body add up to its impulse (GetImpulses).
Triangles longer than a cell of the grid are divided, so that each is in at
most 8 cells.

Each step: each sphere's center and velocity from its grain, and each
triangle's vertices from its body; when a sphere or a triangle has moved
half the margin since the last search, a new search for each
sphere's neighbors, the spheres closer than the margin (a hashed grid of
cells, sorted by cell and then sphere), keeping the tangential displacement
of each pair found again; the contact forces, each sphere over its own
neighbors in the order of their indices; and each grain's motion from the
forces on its spheres, summed in order (symplectic Euler, the quaternion by
the exponential of the rotation in the step, with the gyroscopic term).
Both spheres of a pair evaluate it with the same arithmetic, the one with
the lower index first, so their forces are exactly opposite, and no sum
depends on the order the GPU runs in: a run gives the same bits each time.

Positions are anchored: three 32-bit integers counting cells of the grid,
and a float32 offset within the cell, so that the distance between two
spheres keeps the precision of the offsets anywhere in the model.

The GPU (GpuSolver) and the CPU (CpuSolver, a reference for small problems
and tests) do the same arithmetic in the same order; they differ only where
the GPU's compiler fuses a multiplication with an addition.
*/

/*! \brief a material of the spheres */
struct Material {
  float youngs_modulus = 1.0e7f; //!< Pa
  float poisson_ratio = 0.3f;
  float restitution = 0.5f;        //!< coefficient of restitution
  float friction = 0.5f;           //!< Coulomb's coefficient
  float rolling_resistance = 0.0f; //!< dimensionless
};

/*! \brief the restitution, friction and rolling resistance of a pair of
 * materials, in place of the averages of the two */
struct PairProperties {
  uint32_t a = 0;
  uint32_t b = 0;
  float restitution = 0.5f;
  float friction = 0.5f;
  float rolling_resistance = 0.0f;
};

/*! \brief a sphere of a template: its center from the center of mass, in
 * the principal axes of the template */
struct Sphere {
  std::array<float, 3> center = {0.0f, 0.0f, 0.0f};
  float radius = 0.0f;
  uint32_t material = 0;
};

/*! \brief the most vertices and face planes of a hull */
const uint32_t kMaxHullVertices = 32;
const uint32_t kMaxHullPlanes = 64;
/*! \brief the most vertices of a face of a hull */
const uint32_t kMaxFaceVertices = 12;

/*!
@brief a convex hull of contact, in place of spheres: its vertices in the
principal axes of its template, from the center of mass, and the outward
unit normal \f$\underline{n}\f$ and offset \f$o\f$ of the plane of each
face, \f$\underline{n} \cdot \underline{x} \le o\f$ inside

Two grains of hulls, or a hull and a sphere, touch where the two overlap:
the depth and the normal of the overlap are found by Minkowski portal
refinement (Snethen 2008, as Chrono's ChNarrowphaseMPR), the point of
contact is the center of the overlap of the faces of the two that face it
(the face of one most nearly along the normal clipped against the side
planes of the other's, Sutherland and Hodgman), each point of the clipped
polygon weighted by its depth below the other's face, and the contact law is that of spheres, with
the radius of the sphere of each hull's volume. A hull touches a wall at
the mean of its vertices beyond the wall, weighted by how far, at the depth
of the deepest
*/
struct Hull {
  std::vector<std::array<float, 3>> vertices;
  std::vector<std::array<float, 4>> planes;
  /*! the radius of the sphere of the hull's volume (m): r* of the contact
   * law, and of the step and the margin */
  float radius = 0.0f;
  uint32_t material = 0;
  /*! the polygon of each plane: the indices of its vertices,
   * counterclockwise about its normal */
  std::vector<std::vector<uint32_t>> faces;
  /*! \brief the radius of the smallest sphere about the center of mass
   * that holds the hull */
  float BoundingRadius() const;
  /*! \brief the hull of points (m), in their coordinates, of material; its
   * triangles, counterclockwise seen from outside, appended to triangles
   * if given (indices of its vertices). Throws if the points have no hull
   * of volume, or more than kMaxHullVertices vertices on it, or a face of
   * more than kMaxFaceVertices */
  static Hull Of(const std::vector<std::array<float, 3>>& points,
                 uint32_t material,
                 std::vector<std::array<uint32_t, 3>>* triangles = nullptr);
};

/*! \brief the shape of grains: spheres or a convex hull (Hull), the mass
 * and the principal moments of inertia about the center of mass, and,
 * optionally, a mesh to draw in their place */
struct Template {
  std::vector<Sphere> spheres;
  Hull hull; //!< a hull of vertices, in place of spheres
  float mass = 0.0f;
  std::array<float, 3> inertia = {0.0f, 0.0f, 0.0f};
  /*! drawn in place of the spheres or the hull: triangles in the principal
   * axes from the center of mass, counterclockwise seen from outside */
  std::vector<std::array<float, 3>> mesh_vertices;
  std::vector<std::array<uint32_t, 3>> mesh_triangles;
  /*! \brief a single sphere of radius and density (kg/m^3) */
  static Template SingleSphere(float radius, float density, uint32_t material);
  /*! \brief the convex hull of points (m) of density (kg/m^3), with its mass
   * and principal moments of inertia: the points are transformed to the
   * hull's principal axes from its center of mass, and its triangles are
   * the mesh. Throws if the points have no hull of volume, or more than
   * kMaxHullVertices vertices on it */
  static Template OfHull(const std::vector<std::array<float, 3>>& points,
                         float density, uint32_t material);
  /*! \brief a clump of count spheres, of material, that fills the hull of
   * hull (a template of OfHull), with its mass, inertia and mesh: spheres
   * inscribed in the hull, centered on a lattice of points inside it, each
   * in turn the one that covers the most points of a finer lattice not yet
   * covered */
  static Template ClumpOfHull(const Template& hull, uint32_t count,
                              uint32_t material);
};

/*! \brief count points (m) of a rock of mean radius size, for
 * Template::OfHull: on an ellipsoid of random semi-axes from 0.7 to 1.3
 * times size, at directions of a Fibonacci sphere, each moved along its
 * direction by up to jitter times size, all of the generator seeded with
 * seed, so that a seed always gives the same rock */
std::vector<std::array<float, 3>> RockPoints(uint32_t seed, float size,
                                             uint32_t count = 24,
                                             float jitter = 0.15f);

/*! \brief a fixed plane: the solid is n . p <= offset, with the unit normal
 * n pointing out of it */
struct Wall {
  std::array<double, 3> normal = {0.0, 0.0, -1.0};
  double offset = 0.0;
  uint32_t material = 0;
};

/*! \brief how contacts are resolved: smooth (penalty) forces, or
 * non-smooth impulses (see Nsc) */
enum class ContactMethod { kSmc, kNsc };

/*!
@brief the non-smooth contact method (NSC) of Chrono's ChSystemNSC, after
Anitescu and Tasora: contacts are rigid, and each step solves for the
impulses of the contacts at once, a cone complementarity problem

Each step of h: each grain's velocity without contacts (gravity, and the
gyroscopic term of the angular velocity); for each contact within the
margin, with gap \f$\Phi\f$ (negative where the spheres overlap), normal
\f$\underline{n}\f$ and relative velocity \f$\underline{u}\f$ after the
step, an impulse \f$\gamma = (\gamma_n, \underline{\gamma}_t)\f$ in the
friction cone, \f$|\underline{\gamma}_t| \le \mu \gamma_n\f$, and
\f$0 \le \gamma_n \perp u_n + b \ge 0\f$ with the bias \f$b = \Phi/h\f$,
no less than \f$-\f$recovery_speed where the spheres overlap, or, for a
collision faster than min_bounce_speed, \f$b = \varepsilon u_n^-\f$ of the
approach speed (Newton's restitution); friction by Anitescu's relaxation, the
impulse that takes the most energy out of the sliding; rolling friction, a
moment impulse \f$|\underline{\rho}| \le \mu_r r^* \gamma_n\f$ against
the relative rolling. The impulses minimize
\f$\tfrac{1}{2}\gamma^T N \gamma + r^T \gamma\f$ over the cones, with
\f$N\f$ the Delassus operator of the contacts and \f$r\f$ their velocities
without impulses and biases. They are found by iterations of the
accelerated projected gradient descent of Mazhar, Heyn and Negrut (APGD, as
Chrono::Multicore's ChSolverAPGD: Nesterov's acceleration, a step of the
inverse of a Lipschitz constant L found by backtracking, kept from y to x
when \f$(x - y) \cdot (g_x - g_y) \le L |x - y|^2\f$ for the gradients g,
the objective's usual test of descent, which single precision keeps,
adaptive restart, and the best iterate kept), on the impulses each scaled by
the square root of
its step size below, which leaves the cones as they are and evens out the
masses of light grains and heavy bodies; or by iterations of projected
Gauss-Seidel (each contact in turn, its new impulse applied at once, then
its rolling impulse from the spin that leaves) or
projected Jacobi (all from the velocities of the iteration before), each a
step of relaxation over the inverse of the mean of the contact's diagonal
of the Delassus operator, then the projection onto the cone. The impulses of
the step before start each step (warm start). Then each grain moves with its
new velocity. The GPU (GpuSolver) runs APGD and Jacobi; each grain sums the
impulses of its contacts in the same order on both, and the sums over all
contacts are trees in float, the same on both.

Walls are fixed. A body's triangles are where its path puts them at the
start of the step, and move at the velocity of the path at the end of it:
the step's velocity, as for the grains, and, when the body is coupled to a
multibody model a step behind, the velocity that carries the impulses of the
steps before and not this step's, so that none is counted twice. A body
(Body) of mass takes part in the step as a free rigid body: its velocity at
a contact is that of the path and the change the impulses of the step make
to it. Its joints and the
rest of the model act on it only through the multibody step that follows,
which takes the impulse as its load. A body of no mass is not slowed: a body
pushed onto a column of rigid grains on a floor would then meet a
constraint that no finite impulse satisfies.
*/
enum class NscSolver { kApgd, kGaussSeidel, kJacobi };

/*! \brief the most steps APGD tries in an iteration, doubling its
 * Lipschitz constant after each that does not descend enough, before it
 * takes the last; bounded so that the GPU can record them in advance */
const uint32_t kApgdTries = 4;

struct Nsc {
  uint32_t iterations = 100;
  NscSolver solver = NscSolver::kApgd;
  /*! 0 for the default, 1 for Gauss-Seidel and APGD and, for Jacobi,
   * kJacobiRelaxation, or kJacobiSplitRelaxation with mass_splitting */
  float relaxation = 0.0f;
  /*! Jacobi: mass splitting (Tonge, Benevolenskiy and Voroshilov 2012):
   * each grain or body of n contacts is n parts of 1/n of its mass and
   * inertia, one at each contact, whose velocities are averaged after each
   * iteration; each contact's step size is then that of its parts, its
   * grains' and body's shares of the Delassus diagonal n times as large, so
   * that the contacts of a crowded grain do not overshoot together */
  bool mass_splitting = true;
  /*! m/s, the fastest overlaps undo. Slow, so that a body coupled a step
   * behind, which moves into the grains over a step before their impulse
   * reaches it, does not rebound from the overlap that leaves */
  float recovery_speed = 0.01f;
  float min_bounce_speed = 0.15f; //!< m/s, slower collisions don't bounce
};

/*! \brief the relaxations of Jacobi without and with mass splitting: with
 * it, Jacobi held from 0.2 to 1.9 where without it, at 0.2, it diverged
 * (clumps landing on a dense pack of them), and converged fastest at 1.3 */
const float kJacobiRelaxation = 0.2f;
const float kJacobiSplitRelaxation = 1.3f;
/*! \brief the relaxation the iterations of nsc take */
inline float Relaxation(const Nsc& nsc) {
  if(nsc.relaxation > 0.0f) {
    return nsc.relaxation;
  }
  if(nsc.solver != NscSolver::kJacobi) {
    return 1.0f;
  }
  return nsc.mass_splitting ? kJacobiSplitRelaxation : kJacobiRelaxation;
}

/*! \brief whether nsc splits the masses (Jacobi, mass_splitting) */
inline bool SplitsMasses(const Nsc& nsc) {
  return nsc.solver == NscSolver::kJacobi && nsc.mass_splitting;
}

struct Settings {
  ContactMethod method = ContactMethod::kSmc;
  Nsc nsc; //!< for ContactMethod::kNsc
  std::array<float, 3> gravity = {0.0f, 0.0f, 9.81f}; //!< m/s^2, z down
  /*! the distance within which spheres are listed as neighbors (m); 0 for
   * a fifth of the smallest radius */
  float margin = 0.0f;
  /*! the most neighbors a sphere lists: spheres within the margin; walls
   * and triangles are not counted against it */
  uint32_t max_neighbors = 16;
  /*! the most triangles of bodies within the margin a sphere lists, at
   * most 32: on a flat part a sphere is near 6 to 12 (the triangles are
   * divided to the width of a cell), at an edge or a corner up to 20 */
  uint32_t max_triangles = 24;
  std::vector<Wall> walls; //!< at most 8
  std::vector<PairProperties> pairs;
};

/*! \brief a body the grains touch: triangles fixed in it, their vertices
 * in the coordinates of the body (m), wound either way, and its pose at the
 * start */
struct Body {
  std::vector<std::array<float, 3>> vertices;
  std::vector<std::array<uint32_t, 3>> triangles;
  uint32_t material = 0;
  /*! the pose and velocity of the body in the frame of the grains */
  shift::core::sph::BodyPose pose;
  /*! for the non-smooth method: the mass (kg), 0 for a body the grains
   * can't slow within a step; its center of mass and the elements of its
   * centroidal inertia tensor (Ixx, Iyy, Izz, Ixy, Iyz, Izx, kg m^2, as
   * ksl_inertia_t holds them), in its coordinates. The body
   * then takes part in each step's contacts as a free rigid body, so that
   * its impulse is at most what stops it (see Nsc) */
  float mass = 0.0f;
  std::array<float, 3> center_of_mass = {0.0f, 0.0f, 0.0f};
  std::array<float, 6> inertia = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
};

/*! \brief the triangles of a body divided at the midpoints of their edges,
 * four for one, until no edge is longer than length */
void Subdivide(const std::vector<std::array<float, 3>>& vertices,
               const std::vector<std::array<uint32_t, 3>>& triangles,
               float length, std::vector<std::array<float, 3>>& out_vertices,
               std::vector<std::array<uint32_t, 3>>& out_triangles);

/*! \brief the grains, in the order they were given */
struct State {
  std::vector<uint32_t> template_index;
  /*! the center of mass (m) */
  std::vector<std::array<double, 3>> position;
  /*! the orientation, a unit quaternion, scalar last */
  std::vector<std::array<float, 4>> orientation;
  /*! the velocity of the center of mass (m/s) */
  std::vector<std::array<float, 3>> velocity;
  /*! the angular velocity, in the grain's principal axes (rad/s) */
  std::vector<std::array<float, 3>> angular_velocity;

  size_t size() const {
    return position.size();
  }
  void resize(size_t n);
  /*! \brief add a grain of template at position, at rest, not rotated */
  void AddGrain(uint32_t template_index, const std::array<double, 3>& position);
};

/*! \brief write grains to a text file of comma separated values: a line
 * of the names of the columns, then a line a grain: the name of its shape
 * (names[template_index]), its position x, y, z (m), orientation qx, qy, qz,
 * qw, velocity vx, vy, vz (m/s) and angular velocity wx, wy, wz (rad/s, in
 * its principal axes), with as many digits as read back exactly; lines
 * starting with # are comments. Throws if the file can't be written */
void WriteGrains(const std::string& path, const State& state,
                 const std::vector<std::string>& names);
/*! \brief append the grains of a file of WriteGrains to state, each of the
 * shape of its name among names; throws if the file can't be read, or names
 * a shape not among names */
void ReadGrains(const std::string& path, const std::vector<std::string>& names,
                State& state);

/*! \brief counts of the steps run */
struct Statistics {
  uint64_t steps = 0;
  uint64_t searches = 0;    //!< searches for neighbors
  uint32_t most_neighbors = 0; //!< of any sphere in any search
  uint32_t most_triangles = 0; //!< of any sphere in any search
  uint64_t contacts = 0; //!< NSC: of the last step
  /*! NSC: the largest change of an impulse over the last iteration of the
   * last step, over the largest impulse (N s / N s) */
  float residual = 0.0f;
};

/*! \brief the duration of a Hertz collision of the stiffest pair of the
 * smallest spheres at speed (m/s) and the Rayleigh time of the smallest
 * sphere; the step is a fraction (0.1 to 0.3) of the shorter. The density
 * is that of each template's mass over its spheres' volume */
double RayleighTime(const std::vector<Template>& templates,
                    const std::vector<Material>& materials);
double CollisionTime(const std::vector<Template>& templates,
                     const std::vector<Material>& materials, double speed);
/*! \brief fraction of the shorter of the two */
double StableStep(const std::vector<Template>& templates,
                  const std::vector<Material>& materials, double speed,
                  double fraction = 0.2);

/*! \brief the constants of each pair of materials as the solvers use them,
 * pair (a, b) at a * count + b: e*, g*, beta, mu, then mu_r, the
 * restitution and two 0 */
std::vector<std::array<float, 8>> PairTable(
  const Settings& settings, const std::vector<Material>& materials);

/*! \brief the width of the cells of the grid: twice the largest radius
 * plus the margin; and the margin of the settings */
float CellWidth(const Settings& settings,
                const std::vector<Template>& templates);
float Margin(const Settings& settings, const std::vector<Template>& templates);

/*! \brief a coordinate as a cell of width w and the offset within it, and
 * back */
void ToAnchor(double x, float w, int32_t& cell, float& offset);
double FromAnchor(int32_t cell, float offset, float w);

/*! \brief the hash of a cell of the grid, as both solvers compute it; and
 * the size of the hash table, a power of 2 at least twice count */
uint32_t CellHash(const std::array<int32_t, 3>& c, uint32_t mask);
uint32_t HashTableSize(size_t count);

/*!
@brief the solver's interface: CpuSolver and GpuSolver
*/
class Solver {
public:
  virtual ~Solver() {}
  /*! \brief advance by one step of dt */
  virtual void Step(float dt) = 0;
  /*! \brief advance by n steps of dt */
  virtual void Steps(float dt, long n) {
    for(long k = 0; k < n; k++) {
      Step(dt);
    }
  }
  /*! \brief the grains now */
  virtual void GetState(State& state) = 0;
  virtual Statistics GetStatistics() = 0;
  double get_time() const {
    return time_;
  }
  void set_time(double time) {
    time_ = time;
  }
  /*! \brief the number of bodies */
  size_t body_count() const {
    return bodies_.size();
  }
  /*! \brief the bodies' path from time t0, at start, to t1, at end (one
   * pose per body): the steps from t0 on move the triangles along it
   * (sph::Interpolate). Without one, the bodies stay where they are */
  void SetBodyPaths(const std::vector<shift::core::sph::BodyPose>& start,
                    const std::vector<shift::core::sph::BodyPose>& end, double t0, double t1);
  /*! \brief set the bodies' impulses to zero */
  virtual void ClearImpulses() = 0;
  /*! \brief each body's impulse since ClearImpulses: force (N s) and
   * moment about the origin of the body at each step (N m s), in the frame
   * of the grains */
  virtual void GetImpulses(std::vector<std::array<double, 6>>& impulses) = 0;

protected:
  Solver(const Settings& settings, const std::vector<Material>& materials,
         const std::vector<Template>& templates, const State& state,
         const std::vector<Body>& bodies);

  /*! \brief the poses of the bodies at time, on their paths, anchored as
   * the shaders read them: per body the anchor of the origin (w unused), its
   * offset, the columns of R, v and w; 7 rows of 4 */
  void AnchoredPosesAt(double time, std::vector<std::array<float, 4>>& rows)
    const;
  /*! \brief for the non-smooth method, from the bodies' anchored poses
   * (AnchoredPosesAt): per body its mass (x, the rest 0), the rows of the
   * inverse of its centroidal inertia in the frame of the grains, and its
   * center of mass from its origin in that frame; 5 rows of 4, all 0 for a
   * body of no mass */
  void BodyMassRows(const std::vector<std::array<float, 4>>& poses,
                    std::vector<std::array<float, 4>>& rows) const;

  /*! the grid and the anchored grains and walls, as both solvers keep
   * them */
  struct Layout {
    float cell = 0.0f;
    float margin = 0.0f;
    uint32_t mask = 0;
    uint32_t spheres = 0;
    /* per grain: the anchor (cell, template) and the offset in the cell */
    std::vector<std::array<int32_t, 4>> anchor;
    std::vector<std::array<float, 4>> offset;
    /* per grain: its first sphere; per sphere: its grain and its sphere of
    the template */
    std::vector<uint32_t> first_sphere;
    std::vector<std::array<uint32_t, 2>> sphere_info;
    /* per template sphere: center and radius, and material */
    std::vector<std::array<float, 4>> template_sphere;
    std::vector<uint32_t> template_material;
    /* per template sphere: its hull's first vertex and number of vertices
    (0 for a sphere), first plane and number of planes; per vertex of a
    hull, its coordinates and the hull's radius (Hull::radius); per plane,
    its normal and offset. The sphere of a hull is the hull's bounding
    sphere, centered on the center of mass */
    std::vector<std::array<uint32_t, 4>> template_hull;
    std::vector<std::array<float, 4>> hull_vertex;
    std::vector<std::array<float, 4>> hull_plane;
    /* per plane: the number of vertices of its face and their indices
    within the hull, 8 bits each, 4 in each of y, z and w */
    std::vector<std::array<uint32_t, 4>> hull_face;
    bool hulls = false; //!< a template is a hull
    /* per template: mass and the principal moments; first sphere and
    count */
    std::vector<std::array<float, 4>> template_mass;
    std::vector<std::array<uint32_t, 4>> template_spheres;
    /* per wall: its anchor (a point on it), offset and normal (material
    in w, as bits) */
    std::vector<std::array<int32_t, 4>> wall_anchor;
    std::vector<std::array<float, 4>> wall_offset;
    std::vector<std::array<float, 4>> wall_normal;
    std::vector<std::array<float, 8>> pairs;
    uint32_t materials = 0;
    /* per triangle: its vertices in its body (w: the body, the material,
    as bits, and 0) */
    std::vector<std::array<float, 4>> triangle_local;
    uint32_t triangles = 0;
    uint32_t triangle_mask = 0; //!< of the triangles' hash table
  };
  /*! \brief the state from the anchored grains */
  void FromLayout(const std::vector<std::array<int32_t, 4>>& anchor,
                  const std::vector<std::array<float, 4>>& offset,
                  State& state) const;

  Settings settings_;
  std::vector<Material> materials_;
  std::vector<Template> templates_;
  Layout layout_;
  double time_ = 0.0;
  std::vector<Body> bodies_;
  std::vector<shift::core::sph::BodyPose> path_start_, path_end_;
  double path_t0_ = 0.0, path_t1_ = 0.0;
};

} // namespace dem
} // namespace core
} // namespace shift

#endif

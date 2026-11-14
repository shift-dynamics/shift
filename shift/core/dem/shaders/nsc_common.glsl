// Shared by the shaders of the non-smooth method (nsc_*.comp), after
// dem_common.glsl: the contacts, the cone and the Delassus diagonal as
// CpuSolver::NscStep computes them (cpu_solver.cc).
//
// A contact is 9 vec4 of the buffer of contacts, at 9 c:
//   0: the normal n (from b to a), the bias
//   1: the tangent t1, the coefficient of friction
//   2: the tangent t2, the rolling coefficient mu_r r*
//   3: the arm la of a, a
//   4: the arm lb of b (a body: from its origin), b (kNone for a wall or a
//      body)
//   5: the arm lc from a body's center of mass, the body (kNone)
//   6: the given velocity of a wall or body at the contact, the sphere
//   7: the given angular velocity of a wall or body, where the impulse is
//      kept (Kept)
//   8: eta, eta_r, and their square roots, the scales of APGD
// An impulse (or a vector of APGD) is 2 vec4 at 2 c: the normal and
// tangent parts, and the rolling impulse.
//
// The state of the solver, written by nsc_control.comp:
struct NscControl {
  uint count;          // contacts this step
  uint accepted;       // APGD: the step of this iteration has descended
  uint restart;        // APGD: the momentum restarts
  uint improved;       // APGD: the iterate is the best yet
  float lipschitz;     // APGD
  float theta;
  float step;          // 1 / lipschitz
  float unused0;
  float beta;          // of the momentum
  float best_residual;
  float residual;      // the statistic of the step
  float unused1;
};

// the workgroup counts of the indirect dispatches, 4 words each: [0] the
// contacts, in groups of 128; [1] the contacts, in groups of 1024 (the
// reductions); [2] and [3] the same while APGD has not accepted its step,
// else 0; [4] the grains while it has not; [5] the contacts if the iterate
// is the best yet, else 0
const uint kArgsContacts = 0u;
const uint kArgsReduce = 1u;
const uint kArgsContactsTrying = 2u;
const uint kArgsReduceTrying = 3u;
const uint kArgsGrainsTrying = 4u;
const uint kArgsBest = 5u;

const uint kNone = 0xffffffffu;

// where a contact keeps its impulse: the slot of the pair's history, the
// wall's or the triangle's
const uint kKeptPair = 0u;
const uint kKeptWall = 1u;
const uint kKeptTriangle = 2u;

uint Kept(uint kind, uint slot) {
  return (kind << 30) | slot;
}

// two unit tangents of the unit normal n
void Tangents(vec3 n, out vec3 t1, out vec3 t2) {
  const vec3 axis = abs(n.x) < 0.57735 ? vec3(1.0, 0.0, 0.0)
                                       : vec3(0.0, 1.0, 0.0);
  t1 = cross(n, axis);
  t1 = t1 / length(t1);
  t2 = cross(n, t1);
}

// the impulse (gn, gt1, gt2) onto the cone gt <= mu gn, as Chrono's
// ChConstraintTwoTuplesFrictionT::Project does it
vec3 ProjectOnCone(float mu, vec3 g) {
  const float ft = sqrt(g.y * g.y + g.z * g.z);
  if(mu == 0.0) {
    return vec3(max(g.x, 0.0), 0.0, 0.0);
  }
  if(ft <= mu * g.x) {
    return g;
  }
  if(mu * ft <= -g.x) {
    return vec3(0.0);
  }
  const float fn = (ft * mu + g.x) / (mu * mu + 1.0);
  const float scale = mu * fn / ft;
  return vec3(fn, g.y * scale, g.z * scale);
}

// the rolling impulse r in the tangent plane of n, in the disk of most
vec3 ProjectOnDisk(vec3 n, float most, vec3 r) {
  r = r - dot(r, n) * n;
  const float size = length(r);
  if(size > most) {
    r = size > 0.0 ? r * (most / size) : vec3(0.0);
  }
  return r;
}

// the change of a grain's angular velocity (in its principal axes) of the
// moment impulse m (inertial frame)
vec3 AngularChange(vec4 q, vec3 inertia, vec3 m) {
  return RotateBack(q, m) / inertia;
}

// a grain's part of the diagonal of the Delassus operator: the mean over
// n, t1 and t2 of the change of the velocity at arm l for a unit impulse
float Delassus(vec4 q, float mass, vec3 inertia, vec3 l, vec3 n, vec3 t1,
               vec3 t2) {
  float sum = 3.0 / mass;
  const vec3 e[3] = vec3[3](n, t1, t2);
  for(int k = 0; k < 3; k++) {
    const vec3 c = RotateBack(q, cross(l, e[k]));
    sum += dot(c, c / inertia);
  }
  return sum;
}

// that of a body of mass and inverse inertia (inertial frame, rows)
vec3 Times(vec3 r0, vec3 r1, vec3 r2, vec3 v) {
  return vec3(dot(r0, v), dot(r1, v), dot(r2, v));
}

float BodyDelassus(float mass, vec3 r0, vec3 r1, vec3 r2, vec3 l, vec3 n,
                   vec3 t1, vec3 t2) {
  float sum = 3.0 / mass;
  const vec3 e[3] = vec3[3](n, t1, t2);
  for(int k = 0; k < 3; k++) {
    const vec3 c = cross(l, e[k]);
    sum += dot(c, Times(r0, r1, r2, c));
  }
  return sum;
}

// that of a moment along t1 and t2
float RollingDelassus(vec4 q, vec3 inertia, vec3 t1, vec3 t2) {
  const vec3 a = RotateBack(q, t1), b = RotateBack(q, t2);
  return dot(a, a / inertia) + dot(b, b / inertia);
}

// the bias of a contact of gap and normal n, of the relative velocity u
// before the step and the restitution of the pair
float Bias(vec3 n, float gap, vec3 u, float restitution) {
  const float un = dot(n, u);
  float bias = gap >= 0.0 ? gap / dt : max(gap / dt, -recovery_speed);
  if(restitution > 0.0 && un < -min_bounce_speed && gap + un * dt < 0.0) {
    bias = min(bias, restitution * un);
  }
  return bias;
}

// an APGD vector of contact (mu, n, rolling mu_r r*, and the scales) in the
// product of the cone and the disk: the rolling impulse scaled as the
// vector is, at most rolling sqrt(eta) / sqrt(eta_r) times the normal part
void ProjectScaled(float mu, vec3 n, float rolling, vec4 scales, inout vec3 z,
                   inout vec3 zr) {
  const float cr = rolling > 0.0 ? rolling * scales.z / scales.w : 0.0;
  z = ProjectOnCone(mu, z);
  zr = ProjectOnDisk(n, cr * z.x, zr);
}

// the reductions of nsc_reduce.comp and nsc_control.comp: what each mode
// sums (or, for components below max_components, takes the largest of)
const uint kReduceLipschitz = 0u; // |g_x - g_ones|^2, |x - 1|^2
const uint kReduceAtX = 2u;       // (x - y) . (g_x - g_y), |x - y|^2
const uint kReduceResidual = 3u;  // max |x - P(x - g_x)|, max |x|, g_y . (x - x0)
const uint kReduceJacobi = 4u;    // max change, max normal impulse

uint MaxComponents(uint m) {
  return m == kReduceResidual || m == kReduceJacobi ? 2u : 0u;
}

vec4 Combine(uint m, vec4 a, vec4 b) {
  return MaxComponents(m) == 2u ? vec4(max(a.xy, b.xy), a.zw + b.zw) : a + b;
}

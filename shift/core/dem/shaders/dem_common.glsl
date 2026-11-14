// Shared by the DEM shaders (shift/core/dem): the constants, the grid, the
// quaternions and the contact law, as CpuSolver computes them
// (cpu_solver.cc).

layout(push_constant) uniform Constants {
  float dt;
  float cell;   // width of the cells of the grid and of the anchors
  float margin; // of the neighbors
  uint spheres;
  uint grains;
  uint mask;          // hash table size - 1
  uint max_neighbors; // per sphere
  uint walls;
  vec4 gravity;
  uint materials;
  uint triangles;     // of the bodies
  uint triangle_mask; // their hash table size - 1
  uint max_triangles; // per sphere
  uint bodies;
  uint substep;       // of the batch, for the bodies' poses
  // the non-smooth method (nsc_*.comp): the room for contacts, what a
  // pass does and on what, the relaxation of Jacobi and the speeds of the
  // bias and of restitution
  uint capacity;
  uint mode;
  uint option;
  uint stage;
  float omega;
  float recovery_speed;
  float min_bounce_speed;
  uint splitting; // Jacobi's mass splitting
  uint unused1;
  uint unused2;
};

// the flags: [0] a search is wanted; [1] errors: 1 too many neighbors, 2 a
// triangle in more than 8 cells, 4 too many triangles, 8 more NSC contacts
// than there is room for; [2] the most neighbors; [3] the searches run; [4]
// the most triangles; [5] the most NSC contacts of a step
const uint kTooManyNeighbors = 1u;
const uint kLargeTriangle = 2u;
const uint kTooManyTriangles = 4u;
const uint kTooManyContacts = 8u;

const float kPi = 3.14159265;
const float kTiny = 1e-20;
const uint kMaxWalls = 8u;

// the cell of a point anchored at a with offset x
ivec3 Cell(ivec3 a, vec3 x) {
  return a + ivec3(floor(x / cell));
}

uint CellHashIn(ivec3 c, uint m) {
  return ((uint(c.x) * 73856093u) ^ (uint(c.y) * 19349663u) ^
          (uint(c.z) * 83492791u)) & m;
}

uint CellHash(ivec3 c) {
  return CellHashIn(c, mask);
}

// the hashes in a table of mask m of the 27 cells around c, each once, in a
// fixed order
int NeighborCellsIn(ivec3 c, uint m, out uint hashes[27]) {
  int n = 0;
  for(int dz = -1; dz <= 1; dz++) {
    for(int dy = -1; dy <= 1; dy++) {
      for(int dx = -1; dx <= 1; dx++) {
        const uint hash = CellHashIn(c + ivec3(dx, dy, dz), m);
        bool seen = false;
        for(int k = 0; k < n; k++) {
          seen = seen || hashes[k] == hash;
        }
        if(!seen) {
          hashes[n++] = hash;
        }
      }
    }
  }
  return n;
}

int NeighborCells(ivec3 c, out uint hashes[27]) {
  return NeighborCellsIn(c, mask, hashes);
}

// the point of the triangle a, b, c nearest p (Ericson, Real-Time Collision
// Detection, 5.1.5)
vec3 ClosestPointOnTriangle(vec3 p, vec3 a, vec3 b, vec3 c) {
  const vec3 ab = b - a, ac = c - a, ap = p - a;
  const float d1 = dot(ab, ap), d2 = dot(ac, ap);
  if(d1 <= 0.0 && d2 <= 0.0) {
    return a;
  }
  const vec3 bp = p - b;
  const float d3 = dot(ab, bp), d4 = dot(ac, bp);
  if(d3 >= 0.0 && d4 <= d3) {
    return b;
  }
  const float vc = d1 * d4 - d3 * d2;
  if(vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    return a + (d1 / (d1 - d3)) * ab;
  }
  const vec3 cp = p - c;
  const float d5 = dot(ab, cp), d6 = dot(ac, cp);
  if(d6 >= 0.0 && d5 <= d6) {
    return c;
  }
  const float vb = d5 * d2 - d1 * d6;
  if(vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    return a + (d2 / (d2 - d6)) * ac;
  }
  const float va = d3 * d6 - d5 * d4;
  if(va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);
  }
  const float denominator = 1.0 / (va + vb + vc);
  return a + ab * (vb * denominator) + ac * (vc * denominator);
}

// the vector from the point anchored at (b, y) to the one at (a, x)
vec3 Relative(ivec3 a, vec3 x, ivec3 b, vec3 y) {
  return vec3(a - b) * cell + (x - y);
}

// quaternions, scalar last: the product, and the rotation of a vector
vec4 QuaternionProduct(vec4 a, vec4 b) {
  return vec4(a.w * b.xyz + b.w * a.xyz + cross(a.xyz, b.xyz),
              a.w * b.w - dot(a.xyz, b.xyz));
}

vec3 Rotate(vec4 q, vec3 v) {
  const vec3 t = 2.0 * cross(q.xyz, v);
  return v + q.w * t + cross(q.xyz, t);
}

vec3 RotateBack(vec4 q, vec3 v) {
  return Rotate(vec4(-q.xyz, q.w), v);
}

// the damping coefficient c_n of Law at overlap d
float NormalDamping(vec4 p0, float r, float d, float m) {
  return -2.0 * sqrt(5.0 / 6.0) * p0.z * sqrt(2.0 * p0.x * sqrt(r * d) * m);
}

// the moment on the first of a contact over a patch of spread rho^2 (the
// mean square distance of its points from the point of contact, weighted as
// they are): the normal damping spread over the patch resists the first's
// rocking on the second, w being the second's angular velocity less the
// first's
vec3 Rocking(vec3 n, float d, float r, float m, vec3 w, vec4 p0,
             float spread) {
  return (NormalDamping(p0, r, d, m) * spread) * (w - dot(w, n) * n);
}

// the contact law of a pair with overlap d > 0 along the unit normal n (on
// the first), effective radius r and mass m, relative velocity u of the
// first at the contact and angular velocity w of the second less the
// first's, of the constants p0 (e*, g*, beta, mu) and p1 (mu_r): the force
// on the first, the moment of rolling resistance on the first (the second
// takes the opposite), and the tangential displacement and duration in xi,
// updated
void Law(vec3 n, float d, float r, float m, vec3 u, vec3 w, vec4 p0,
         vec4 p1, inout vec4 xi, out vec3 force, out vec3 rolling) {
  const float e = p0.x, g = p0.y, beta = p0.z, mu = p0.w, mu_r = p1.x;
  const float rc = sqrt(r * d);
  const float un = dot(n, u);
  const vec3 ut = u - un * n;
  /* the history, in the tangent plane */
  vec3 history = xi.xyz + dt * ut;
  history = history - dot(history, n) * n;
  xi.w = xi.w + dt;
  /* normal */
  const float kn = (4.0 / 3.0) * e * rc;
  const float cn = NormalDamping(p0, r, d, m);
  const float fn = kn * d - cn * un;
  force = fn * n;
  /* rolling resistance, once the contact has outlasted a collision */
  rolling = vec3(0.0);
  if(mu_r > 0.0) {
    const float kns = (4.0 / 3.0) * e * sqrt(r);
    const float gns = -2.0 * sqrt((5.0 / 3.0) * m * e) * beta * pow(r, 0.25);
    const float ratio = gns / (2.0 * sqrt(kns * m));
    bool apply = true;
    if(ratio < 1.0) {
      const float collision =
        kPi * sqrt(m / (kns * (1.0 - ratio * ratio)));
      apply = xi.w > collision;
    }
    const vec3 rolling_w = w - dot(w, n) * n;
    const float speed = length(rolling_w);
    if(apply && speed > 1e-12) {
      rolling = (rolling_w / speed) * (mu_r * abs(fn) * r);
    }
  }
  /* tangential, bounded by Coulomb's law */
  if(mu > 0.0) {
    const float kt = 8.0 * g * rc;
    const float ct = -2.0 * sqrt(5.0 / 6.0) * beta * sqrt(m * kt);
    vec3 ft = -kt * history - ct * ut;
    const float size = length(ft);
    const float most = mu * abs(fn);
    if(size > most) {
      ft = ft * (most / size);
      history = -(ft + ct * ut) / kt;
    }
    force = force + ft;
  }
  xi.xyz = history;
}

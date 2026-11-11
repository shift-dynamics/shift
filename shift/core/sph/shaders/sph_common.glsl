// Shared by the SPH shaders (shift/core/sph): the kernel, the grid hash and
// the neighbours' cells, as CpuSolver computes them (sph.cc, cpu_solver.cc).

const uint kMarker = 0x80000000u;

layout(push_constant) uniform Constants {
  float dt;
  float h;
  float cell;
  uint count;      // particles and markers
  uint mask;       // hash table size - 1
  float viscosity; // Monaghan's alpha
  uint unused0;
  uint unused1;
  vec4 gravity;
  // the bodies: the id of their first marker, the number of their markers,
  // the number of bodies, and the substep of the batch (sph_bodies)
  uvec4 bodies;
};

// the body of a marker: 0 for a wall, 1 + the index of the body
uint MarkerBody(uint k) {
  return k & 0x7fffffffu;
}

float Kernel(float r) {
  const float q = r / h;
  if(q >= 2.0) {
    return 0.0;
  }
  const float a = 1.0 - 0.5 * q;
  const float a2 = a * a;
  return 21.0 / (16.0 * 3.14159265 * h * h * h) * a2 * a2 * (2.0 * q + 1.0);
}

float KernelDerivative(float r) {
  const float q = r / h;
  if(q >= 2.0) {
    return 0.0;
  }
  const float a = 1.0 - 0.5 * q;
  return 21.0 / (16.0 * 3.14159265 * h * h * h) * (-5.0 * q) * a * a * a / h;
}

ivec3 Cell(vec3 x) {
  return ivec3(floor(x / cell));
}

uint CellHash(ivec3 c) {
  return ((uint(c.x) * 73856093u) ^ (uint(c.y) * 19349663u) ^
          (uint(c.z) * 83492791u)) & mask;
}

// the hashes of the 27 cells around x, each once, in a fixed order
int NeighborCells(vec3 x, out uint hashes[27]) {
  const ivec3 c = Cell(x);
  int n = 0;
  for(int dz = -1; dz <= 1; dz++) {
    for(int dy = -1; dy <= 1; dy++) {
      for(int dx = -1; dx <= 1; dx++) {
        const uint hash = CellHash(c + ivec3(dx, dy, dz));
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

// shared by terrain.tesc and terrain.tese: the terrain surface evaluated
// from its control points, as terrain::Basic::LocalZ does for the physics

layout(std140, binding = 0) uniform Frame {
  mat4 view_projection;
  mat4 inverse_view_projection;
  vec4 camera_position;
  vec4 sun_direction;
  vec4 sun_color;
  vec4 sky_zenith;
  vec4 sky_horizon;
  vec4 floor_params;
} frame;

layout(std140, binding = 6) uniform Terrain {
  mat4 pose;         // terrain frame to world
  vec4 extent;       // x, y extent (m), patches along x, y
  vec4 tessellation; // x = pixels per unit angle, y = pixels per triangle
                     // edge, z = maximum level
  vec4 scale;        // x, y spacing of the control points (m)
  ivec4 samples;     // control points along x, y; z = 1 for the cubic
                     // B-spline, 0 for bilinear, 2 for Catmull-Rom
} terrain;

// the control points: z (down) of the terrain frame, read exactly
layout(binding = 7) uniform sampler2D control_points;

float Control(int i, int j) {
  return texelFetch(control_points, ivec2(i, j), 0).x;
}

// the indices and weights of the control points of the map that give index
// k along an axis of n, as in terrain::Basic::ControlZ
void Along(int k, int n, bool bicubic, out ivec2 index, out vec2 weight) {
  if(!bicubic || n < 2 || (k >= 0 && k < n)) {
    index = ivec2(clamp(k, 0, n - 1));
    weight = vec2(1.0, 0.0);
  } else if(k < 0) {
    index = ivec2(0, 1);
    weight = vec2(2.0, -1.0);
  } else {
    index = ivec2(n - 1, n - 2);
    weight = vec2(2.0, -1.0);
  }
}

// control point (i, j): beyond the map, the sample of its edge for the
// bilinear surface, and for the bicubic the control point extrapolated
// linearly by one more along each axis (the natural end condition)
float ControlZ(int i, int j) {
  bool bicubic = terrain.samples.z == 1;
  ivec2 ii, jj;
  vec2 wi, wj;
  Along(i, terrain.samples.x, bicubic, ii, wi);
  Along(j, terrain.samples.y, bicubic, jj, wj);
  float z = wi.x * wj.x * Control(ii.x, jj.x);
  if(wi.y != 0.0) {
    z += wi.y * wj.x * Control(ii.y, jj.x);
  }
  if(wj.y != 0.0) {
    z += wi.x * wj.y * Control(ii.x, jj.y);
    if(wi.y != 0.0) {
      z += wi.y * wj.y * Control(ii.y, jj.y);
    }
  }
  return z;
}

// uniform cubic B-spline basis b and its derivative db at a in [0, 1)
void Basis(float a, out vec4 b, out vec4 db) {
  float a2 = a * a;
  float a3 = a2 * a;
  b = vec4(1.0 - 3.0 * a + 3.0 * a2 - a3, 4.0 - 6.0 * a2 + 3.0 * a3,
           1.0 + 3.0 * a + 3.0 * a2 - 3.0 * a3, a3) / 6.0;
  db = vec4(-3.0 + 6.0 * a - 3.0 * a2, -12.0 * a + 9.0 * a2,
            3.0 + 6.0 * a - 9.0 * a2, 3.0 * a2) / 6.0;
}

// Catmull-Rom basis b and its derivative db at a in [0, 1): the cubic that
// passes through the control points, with the slopes of their neighbours
void CatmullRomBasis(float a, out vec4 b, out vec4 db) {
  float a2 = a * a;
  float a3 = a2 * a;
  b = vec4(-a + 2.0 * a2 - a3, 2.0 - 5.0 * a2 + 3.0 * a3,
           a + 4.0 * a2 - 3.0 * a3, -a2 + a3) * 0.5;
  db = vec4(-1.0 + 4.0 * a - 3.0 * a2, -10.0 * a + 9.0 * a2,
            1.0 + 8.0 * a - 9.0 * a2, -2.0 * a + 3.0 * a2) * 0.5;
}

// z (down) of the surface at xy of the terrain frame and its slopes
// dz/dx, dz/dy
float LocalZ(vec2 xy, out vec2 slope) {
  vec2 u = xy / terrain.scale.xy;
  ivec2 i = ivec2(floor(u));
  vec2 f = u - vec2(i);
  if(terrain.samples.z == 0) { // bilinear
    float z00 = ControlZ(i.x, i.y), z10 = ControlZ(i.x + 1, i.y);
    float z01 = ControlZ(i.x, i.y + 1), z11 = ControlZ(i.x + 1, i.y + 1);
    slope = vec2(mix(z10 - z00, z11 - z01, f.y),
                 mix(z01 - z00, z11 - z10, f.x)) / terrain.scale.xy;
    return mix(mix(z00, z10, f.x), mix(z01, z11, f.x), f.y);
  }
  // the 4 x 4 control points around the cell
  vec4 bs, dbs, bt, dbt;
  if(terrain.samples.z == 2) { // through the samples (deformable terrain)
    CatmullRomBasis(f.x, bs, dbs);
    CatmullRomBasis(f.y, bt, dbt);
  } else {
    Basis(f.x, bs, dbs);
    Basis(f.y, bt, dbt);
  }
  float z = 0.0;
  vec2 dz = vec2(0.0);
  for(int b = 0; b < 4; b++) {
    for(int a = 0; a < 4; a++) {
      float c = ControlZ(i.x - 1 + a, i.y - 1 + b);
      z += bs[a] * bt[b] * c;
      dz.x += dbs[a] * bt[b] * c;
      dz.y += bs[a] * dbt[b] * c;
    }
  }
  slope = dz / terrain.scale.xy;
  return z;
}

// a point of the surface over xy of the terrain frame, in world coordinates
vec3 SurfacePoint(vec2 xy) {
  vec2 slope;
  float z = LocalZ(xy, slope);
  return (terrain.pose * vec4(xy, z, 1.0)).xyz;
}

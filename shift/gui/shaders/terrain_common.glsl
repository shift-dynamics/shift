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
                     // B-spline, 0 for bilinear
} terrain;

// the control points: z (down) of the terrain frame, read exactly
layout(binding = 7) uniform sampler2D control_points;

// control point (i, j), clamped to the map: beyond it the surface continues
// with the control points of its edges
float ControlZ(int i, int j) {
  ivec2 k = clamp(ivec2(i, j), ivec2(0), terrain.samples.xy - 1);
  return texelFetch(control_points, k, 0).x;
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
  Basis(f.x, bs, dbs);
  Basis(f.y, bt, dbt);
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

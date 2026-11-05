// shared by the CAD shaders: the frame (as in mesh.vert) and the CAD
// tessellation and edge parameters (CadUniforms in viewport.cc)

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

layout(std140, binding = 6) uniform Cad {
  vec4 tessellation; // x = pixels per unit angle, y = pixels per triangle
                     // edge, z = maximum level, w = largest distance of the
                     // triangles from the surface (pixels)
  vec4 edges;        // x = half width of edge lines (pixels), y = their
                     // darkening (0 to 1), z = widening of the trims
                     // (pixels), closing cracks between faces
} cad;

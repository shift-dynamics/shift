#version 450

layout(location = 0) out vec3 world_position;
layout(location = 1) out vec2 plane_position; // in the floor's axes

layout(std140, binding = 0) uniform Frame {
  mat4 view_projection;
  mat4 inverse_view_projection;
  vec4 camera_position;
  vec4 sun_direction;
  vec4 sun_color;
  vec4 sky_zenith;
  vec4 sky_horizon;
    vec4 floor_params; // y = grid spacing, z = enabled, w = extent
  mat4 floor_frame;  // columns: in-plane axes u and v, normal up (out of the
                     // solid), origin
} frame;

void main() {
  const vec2 corners[6] = vec2[](vec2(-1.0, -1.0), vec2(1.0, -1.0),
                                 vec2(-1.0, 1.0), vec2(-1.0, 1.0),
                                 vec2(1.0, -1.0), vec2(1.0, 1.0));
    // a square of the floor plane around the point under the camera
  vec3 axis_u = frame.floor_frame[0].xyz;
  vec3 axis_v = frame.floor_frame[1].xyz;
  vec3 origin = frame.floor_frame[3].xyz;
  vec3 camera = frame.camera_position.xyz - origin;
  vec2 center = vec2(dot(camera, axis_u), dot(camera, axis_v));
  plane_position = center + corners[gl_VertexIndex] * frame.floor_params.w;
  world_position = origin + axis_u * plane_position.x + axis_v * plane_position.y;
  gl_Position = frame.view_projection * vec4(world_position, 1.0);
}

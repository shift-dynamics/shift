#version 450

layout(location = 0) in vec3 world_position;
layout(location = 1) in vec3 world_normal;
layout(location = 2) in vec3 color;
layout(location = 0) out vec4 out_color;

layout(std140, binding = 0) uniform Frame {
  mat4 view_projection;
  mat4 inverse_view_projection;
  vec4 camera_position;
} frame;

void main() {
  // lit from the camera, so that the arrows keep their colors from any side
  vec3 v = normalize(frame.camera_position.xyz - world_position);
  float n_dot_v = abs(dot(normalize(world_normal), v));
  out_color = vec4(color * (0.45 + 0.55 * n_dot_v), 1.0);
  // in front of the model, so that frames inside a body show, keeping the
  // order of the arrows among themselves
  gl_FragDepth = gl_FragCoord.z * 0.02;
}

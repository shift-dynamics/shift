#version 450

layout(location = 0) out vec3 world_position;

layout(std140, binding = 0) uniform Frame {
  mat4 view_projection;
  mat4 inverse_view_projection;
  vec4 camera_position;
  vec4 sun_direction;
  vec4 sun_color;
  vec4 sky_zenith;
  vec4 sky_horizon;
  vec4 floor_params; // x = floor z, y = grid spacing, z = enabled, w = extent
} frame;

void main() {
  const vec2 corners[6] = vec2[](vec2(-1.0, -1.0), vec2(1.0, -1.0),
                                 vec2(-1.0, 1.0), vec2(-1.0, 1.0),
                                 vec2(1.0, -1.0), vec2(1.0, 1.0));
  vec2 xy = frame.camera_position.xy + corners[gl_VertexIndex] * frame.floor_params.w;
  world_position = vec3(xy, frame.floor_params.x);
  gl_Position = frame.view_projection * vec4(world_position, 1.0);
}

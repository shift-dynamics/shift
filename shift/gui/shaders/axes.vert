#version 450

// the arrows of a coordinate frame (Viewport::set_frame_axes): x red, y
// green, z blue, one unit long, drawn with the pose of the frame times the
// scale; and the axes of joints (Viewport::set_joint_axes) as arrows along
// z, white, colored by the instance
layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec3 in_color;
// per instance: the frame to world, times the scale, column by column
layout(location = 3) in vec4 in_model_0;
layout(location = 4) in vec4 in_model_1;
layout(location = 5) in vec4 in_model_2;
layout(location = 6) in vec4 in_model_3;
layout(location = 7) in vec4 in_tint; // multiplies the color

layout(location = 0) out vec3 world_position;
layout(location = 1) out vec3 world_normal;
layout(location = 2) out vec3 color;

layout(std140, binding = 0) uniform Frame {
  mat4 view_projection;
  mat4 inverse_view_projection;
  vec4 camera_position;
} frame;

void main() {
  mat4 model = mat4(in_model_0, in_model_1, in_model_2, in_model_3);
  vec4 world = model * vec4(in_position, 1.0);
  world_position = world.xyz;
  // a joint arrow is scaled by its radius across z and its length along
  // it: the normals transform by the inverse transpose (the fragment shader
  // normalizes them)
  world_normal = transpose(inverse(mat3(model))) * in_normal;
  color = in_color * in_tint.rgb;
  gl_Position = frame.view_projection * world;
}

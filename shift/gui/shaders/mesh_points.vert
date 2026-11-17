#version 450

// the nodes a flexible body's attachments tie to their frames
// (SceneMesh::constrained_vertices), drawn as points over its surface, as
// mesh_edges.vert draws the edges of its elements

layout(location = 0) in vec3 in_position;
// per instance: the body origin pose, column by column
layout(location = 3) in vec4 in_model_0;
layout(location = 4) in vec4 in_model_1;
layout(location = 5) in vec4 in_model_2;
layout(location = 6) in vec4 in_model_3;

layout(std140, binding = 0) uniform Frame {
  mat4 view_projection;
  mat4 inverse_view_projection;
  vec4 camera_position;
} frame;

// as mesh_edges.vert: nearer the camera by this fraction of the distance,
// so that the points show over the surface they lie on
const float kPull = 0.003;

void main() {
  mat4 model = mat4(in_model_0, in_model_1, in_model_2, in_model_3);
  vec3 world = (model * vec4(in_position, 1.0)).xyz;
  vec3 camera = frame.camera_position.xyz;
  world = camera + (world - camera) * (1.0 - kPull);
  gl_Position = frame.view_projection * vec4(world, 1.0);
  gl_PointSize = 6.0;
}

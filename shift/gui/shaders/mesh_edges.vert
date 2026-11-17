#version 450

// the edges of a flexible body's finite elements (SceneMesh::edge_indices),
// drawn as lines over its surface, the mesh's own or its CAD's

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

// the fraction of its distance from the camera an edge is drawn nearer
// it, so that the edges show over the surface they lie on: the straight
// sides of the elements cut inside a curved surface by their sagitta, and
// the CAD's surface follows the nodes only approximately
const float kPull = 0.002;

void main() {
  mat4 model = mat4(in_model_0, in_model_1, in_model_2, in_model_3);
  vec3 world = (model * vec4(in_position, 1.0)).xyz;
  vec3 camera = frame.camera_position.xyz;
  world = camera + (world - camera) * (1.0 - kPull);
  gl_Position = frame.view_projection * vec4(world, 1.0);
}

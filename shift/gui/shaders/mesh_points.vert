#version 450

// the nodes a flexible body's attachments tie to their frames, and the
// frames (SceneMesh::constrained_vertices), drawn as points over its
// surface, as mesh_edges.vert draws the edges of its elements; also the
// spiders between them (SceneMesh::spider_indices, spider.frag). Each
// vertex's attachment color is in its uv.y: 1 + its index into
// shift::gui::kAttachmentColors, which this table repeats

layout(location = 0) in vec3 in_position;
layout(location = 2) in vec2 in_uv;
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

layout(location = 0) out vec3 color;

const vec3 kColors[8] = vec3[8](
  vec3(0.90, 0.10, 0.10), // the reference: held
  vec3(1.00, 0.55, 0.10), vec3(0.20, 0.55, 1.00), vec3(0.25, 0.80, 0.30),
  vec3(0.70, 0.40, 0.95), vec3(0.95, 0.85, 0.20), vec3(0.20, 0.85, 0.85),
  vec3(1.00, 0.45, 0.75));

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
  int k = int(in_uv.y + 0.5) - 1;
  color = k >= 0 && k < 8 ? kColors[k] : vec3(1.0, 0.45, 0.1);
}

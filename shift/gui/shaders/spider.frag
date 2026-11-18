#version 450

// the spiders of a flexible body's attachments (mesh_points.vert): thin
// lines in their attachment's color, partly transparent where they crowd

layout(location = 0) in vec3 color;
layout(location = 0) out vec4 out_color;

void main() {
  out_color = vec4(color, 0.45);
}

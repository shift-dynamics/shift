#version 450

layout(location = 0) out vec4 out_color;

layout(std140, binding = 1) uniform Material {
  vec4 albedo;
} mesh;

void main() {
  // the color of the body, darkened, so that the edges read on it lit or
  // in shadow
  out_color = vec4(0.25 * mesh.albedo.rgb, 1.0);
}

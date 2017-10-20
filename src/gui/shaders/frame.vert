#version 330

/* frame.vert */

uniform mat4 mvp_u;
uniform float scale_u = 1.f;

layout (location = 0) in vec4 position_a;

flat out vec4 color;

void main() {
  vec4 position = vec4(scale_u * position_a.xyz, 1.f);
  gl_Position = mvp_u * position;
  color = position;
}

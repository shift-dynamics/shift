#version 330

/* frame.frag */

uniform mat4 mvp_u;
uniform float scale_u = 1.f;

flat in vec4 color;
out vec4 fragColor;

void main() {
  fragColor = color;
}

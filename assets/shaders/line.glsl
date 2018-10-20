#version 330
/* line.vert */

uniform mat4 mvp_u;
uniform vec4 color_u;
uniform float scale_u;

#ifdef VERTEX_SHADER
layout (location = 0) in vec4 position_a;
out vec4 color;

void main() {
  color = color_u;
  gl_Position = mvp_u * vec4(scale_u * position.xyz, 1.f);
}
#endif


#ifdef FRAGMENT_SHADER
in vec4 color;
out vec4 fragColor;

void main() {
  fragColor = color;
}
#endif

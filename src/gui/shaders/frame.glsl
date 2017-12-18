#version 330
/* frame.glsl */

uniform mat4 mvp_u;
uniform float scale_u = 1.f;

#ifdef VERTEX_SHADER
layout (location = 0) in vec4 position_a;

flat out vec4 color;

void main() {
  vec4 position = vec4(scale_u * position_a.xyz, 1.f);
  gl_Position = mvp_u * position;
  color = position;
}
#endif


#ifdef FRAGMENT_SHADER
flat in vec4 color;
out vec4 fragColor;

void main() {
  fragColor = color;
}
#endif

#version 420
/* frame.glsl */

layout (std140, binding = 0) uniform UniformBlock {
  mat4 mvp_u;
  float scale_u;
};


#ifdef VERTEX_SHADER
layout (location = 0) in vec4 position_a;
layout (location = 0) flat out vec4 color;

void main() {
  vec4 position = vec4(scale_u * position_a.xyz, 1.f);
  gl_Position = mvp_u * position;
  color = position;
}
#endif


#ifdef FRAGMENT_SHADER
layout (location = 0) flat in vec4 color;
layout (location = 0) out vec4 fragColor;

void main() {
  fragColor = color;
}
#endif

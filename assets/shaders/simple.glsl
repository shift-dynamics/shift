#version 420

layout (binding = 0) uniform UniformBlock {
  mat4 mvp_u;
  vec4 color_u;
  float scale_u;
};


#ifdef VERTEX_SHADER
layout (location = 0) in vec4 position_a;

layout (location = 0) out vec4 color;

void main() {
  color = color_u;
  gl_Position = mvp_u * vec4(scale_u * position_a.xyz, 1.f);
}
#endif


#ifdef FRAGMENT_SHADER
layout (location = 0) in vec4 color;

layout (location = 0) out vec4 fragColor;

void main() {
  fragColor = color;
}
#endif

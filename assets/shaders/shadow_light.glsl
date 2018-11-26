#version 420 core
/* shadow_light.vert */

layout (std140, binding = 0) uniform BufferBlock {
  mat4 mvp_u;
};

#ifdef VERTEX_SHADER
layout (location = 0) in vec4 position_a;
layout (location = 1) in vec3 normal_a;

void main() {
  gl_Position = mvp_u * position_a;
}
#endif


#ifdef FRAGMENT_SHADER
layout (location = 0) out float fragmentDepth;

void main() {
  fragmentDepth = gl_FragCoord.z;
}
#endif

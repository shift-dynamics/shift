#version 330 core
/* shadow_light.vert */

uniform mat4 mvp_u;

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

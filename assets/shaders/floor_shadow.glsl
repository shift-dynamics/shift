#version 420 core
/* floor_shadow.glsl */

struct matrices_t {
  mat4 mvp;
};

layout(std140, binding=0) uniform UniformBlock {
  matrices_t matrices_u;
};

#ifdef VERTEX_SHADER
void main() {
  vec3[4] vertices = vec3[4](
    vec3(-1.0, -1.0, 0.0),
    vec3(-1.0, 1.0, 0.0),
    vec3(1.0, -1.0, 0.0),
    vec3(1.0, 1.0, 0.0));
  gl_Position = matrices_u.mvp * vec4(vertices[gl_VertexIndex], 1.0);
}
#endif

#ifdef FRAGMENT_SHADER
layout(location = 0) out float fragmentDepth;

void main(void) {
  fragmentDepth = gl_FragCoord.z;
}
#endif

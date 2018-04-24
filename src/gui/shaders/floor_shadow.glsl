#version 420 core
/* floor.glsl */

struct matrices_t {
  mat4 mvp;
};
uniform matrices_t matrices_u;


#ifdef VERTEX_SHADER
out vec3 tcFrag;
void main() {
  vec3[4] vertices = vec3[4](
    vec3(-1.0, -1.0, 0.0),
    vec3(1.0, -1.0, 0.0),
    vec3(-1.0, 1.0, 0.0),
    vec3(1.0, 1.0, 0.0));
  gl_Position = matrices_u.mvp * vec4(vertices[gl_VertexID], 1.0);
}
#endif

#ifdef FRAGMENT_SHADER
out float fragmentDepth;

void main(void) {
  fragmentDepth = gl_FragCoord.z;
}
#endif

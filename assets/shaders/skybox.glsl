#version 420 core
/* cube.glsl */

layout (binding = 0) uniform BufferBlock {
  mat4 view_u;
};
layout (binding = 1) uniform samplerCube texture_cube_u;


#ifdef VERTEX_SHADER
layout (location = 0) out vec3 tcFrag;

void main() {
  vec3[4] vertices = vec3[4](
    vec3(-1.0, -1.0, 1.0),
    vec3(1.0, -1.0, 1.0),
    vec3(-1.0, 1.0, 1.0),
    vec3(1.0, 1.0, 1.0));
  mat3 r;
  r[0] = vec3(1, 0, 0);
  r[1] = vec3(0, 0, -1);
  r[2] = vec3(0, 1, 0);
	tcFrag = mat3(view_u) * r * vertices[gl_VertexIndex];
  gl_Position = vec4(vertices[gl_VertexIndex], 1.0);
}
#endif

#ifdef FRAGMENT_SHADER
layout(location = 0) in vec3 tcFrag;

layout(location = 0) out vec4 color;

void main() {
  color = texture(texture_cube_u, tcFrag);
}
#endif

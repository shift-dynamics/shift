#version 420 core
/* floor.glsl */

layout (binding = 0) uniform sampler2DArrayShadow texture_shadow_u;
layout (binding = 1) uniform samplerCubeArrayShadow texture_shadowCube_u;
layout (binding = 2) uniform sampler2D texture_diffuse_u;
layout (binding = 3) uniform sampler2D texture_normal_u;
layout (binding = 4) uniform sampler2D texture_specular_u;
layout (binding = 5) uniform sampler2D texture_bump_u;

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat3 nm;
};
uniform matrices_t matrices_u;

struct material_t {
  vec4 Ka;
  vec4 Kd;
  vec4 Ks;
  vec4 Ke;
  int Ns;
};
uniform material_t material_u;

struct lights_t {
  vec3 position[10];
  vec3 Ka[10];
  vec3 Kd[10];
  vec3 Ks[10];
  int directional[10];
  int count;
  int drawShadows;
  mat4 shadowMatrix[10];
};
uniform lights_t lights_u;

struct floor_t {
  vec2 tcScale;
  int enableNormalMap;
  int enableSpecularMap;
  int enableBumpMap;
};
uniform floor_t floor_u;

#ifdef VERTEX_SHADER
out vec3 tcFrag;
void main() {
  vec3[4] vertices = vec3[4](
    vec3(-1.0, -1.0, 0.0),
    vec3(1.0, -1.0, 0.0),
    vec3(-1.0, 1.0, 0.0),
    vec3(1.0, 1.0, 0.0));
  vec3[4] tc = vec2[4](
    vec2(0.0, 0.0),
    vec2(1.0, 0.0),
    vec2(0.0, 1.0),
    vec2(1.0, 1.0));
	tcFrag = floor_u.tcScale * tc[gl_VertexID];
  gl_Position = mvp_u * vec4(vertices[gl_VertexID], 1.0);
}
#endif

#ifdef FRAGMENT_SHADER
in vec3 tcFrag;
out vec4 color;
void main() {
  color = texture(texture_u, tcFrag);
}
#endif

#version 420 core

/* terrain.vert */
layout (binding = 0) uniform sampler2DArrayShadow texture_shadow_u;
layout (binding = 1) uniform samplerCubeArrayShadow texture_shadowCube_u;
layout (binding = 2) uniform sampler2D texture_elevation_u;
layout (binding = 3) uniform sampler2D texture_diffuse_u;
layout (binding = 4) uniform sampler2D texture_normal_u;
layout (binding = 5) uniform sampler2D texture_specular_u;
layout (binding = 6) uniform sampler2D texture_bump_u;

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat4 p;
  mat3 nm;
};
uniform matrices_t matrices_u;

struct material_t {
  vec4 Kd;
  float KaOverKd;
  float KsOverKd;
  int Ns;
};
uniform material_t material_u;

struct lights_t {
  vec3 position[10];
  vec3 direction[10];
  vec3 Ka[10];
  vec3 Kd[10];
  vec3 Ks[10];
  int type[10]; /*!< 0 = point, 1 = directional */
  int count;
  int drawShadows; /*!< 0 = false, 1 = true */
  mat4 shadowMatrix[10];
  mat4 shadowMatrixCube[10][6];
};
uniform lights_t lights_u;

struct spotlights_t {
  vec3 position[10];
  vec3 direction[10];
  vec3 Ka[10];
  vec3 Kd[10];
  vec3 Ks[10];
  int count;
  int drawShadows;
  mat4 shadowMatrix[10];
}
uniform spotlights_t spotlights_u;

struct terrain_t {
  float zMin;
  float zMax;
  vec2 scale;
  vec2 tcScale;
  vec2 viewport;
  int enableDiffuseMap;
  int enableNormalMap;
  int enableSpecularMap;
  int enableBumpMap;
};
uniform terrain_t terrain_u;

out VS_OUT {
  vec2 tc;
} vs_out;

void main(void) {
  const vec4 vertices[] = vec4[](
    vec4(0.0, 0.0, 0.0, 1.0),
    vec4(1.0, 0.0, 0.0, 1.0),
    vec4(0.0, 1.0, 0.0, 1.0),
    vec4(1.0, 1.0, 0.0, 1.0));

  ivec2 texture_size = textureSize(texture_elevation_u, 0);

  int x = gl_InstanceID % texture_size.x; // row index into elevation map
  int y = gl_InstanceID / texture_size.x; // column index into elevation map

  vs_out.tc = (vertices[gl_VertexID].xy + vec2(x, y)) / texture_size;

  gl_Position =
    vertices[gl_VertexID] +
    vec4(
      float(x),
      float(y),
      0.0,
      0.0);


}

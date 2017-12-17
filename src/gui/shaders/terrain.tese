#version 420 core
/* terrain.tese */

layout (quads, fractional_odd_spacing) in;

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

in TCS_OUT {
  vec2 tc;
} tes_in[];

out TES_OUT {
  vec2 tc;
  vec3 normal_cameraSpace;
  vec3 position_cameraSpace;
  vec3 view_direction_tangentSpace;
  vec3 light_direction_tangentSpace[10];
} tes_out;

out vec4 shadowCoord[10];

void main(void) {

  vec2 tc1 =
    mix(
      tes_in[0].tc,
      tes_in[1].tc,
      gl_TessCoord.x);

  vec2 tc2 =
    mix(
      tes_in[2].tc,
      tes_in[3].tc,
      gl_TessCoord.x);

  vec2 tc =
    mix(
      tc2,
      tc1,
      gl_TessCoord.y);

  vec4 p1 =
    mix(
      gl_in[0].gl_Position,
      gl_in[1].gl_Position,
      gl_TessCoord.x);

  vec4 p2 =
    mix(
      gl_in[2].gl_Position,
      gl_in[3].gl_Position,
      gl_TessCoord.x);

  float f11 = gl_in[0].gl_Position.z;
  float f21 = gl_in[1].gl_Position.z;
  float f12 = gl_in[2].gl_Position.z;
  float f22 = gl_in[3].gl_Position.z;

  float a11 = f11;
  float a21 = f21 - f11;
  float a12 = f12 - f11;
  float a22 = f11 - f21 - f12 + f22;

  float x1 = gl_in[0].gl_Position.x;
  float x2 = gl_in[1].gl_Position.x;

  float y1 = gl_in[0].gl_Position.y;
  float y2 = gl_in[2].gl_Position.y;

  float tx = (a21 + a22 * gl_TessCoord.y) / (x2 - x1);
  float ty = (a12 + a22 * gl_TessCoord.x) / (y2 - y1);

  vec4 position =
    mix(
      p2,
      p1,
      gl_TessCoord.y);

  position.x *= terrain_u.scale.x;
  position.y *= terrain_u.scale.y;
  position.z = (texture(texture_elevation_u, tc).r * (terrain_u.zMax - terrain_u.zMin)) + terrain_u.zMin;

  vec4 position_cameraSpace = matrices_u.mv * position;
  vec3 normal = normalize(vec3(tx, ty, -1.0));
  vec3 normal_cameraSpace = matrices_u.nm * normal;

  if(terrain_u.enableNormalMap == 1) {
    /* tangent and bitangent in camera space */
    vec3 tangent = mat3(matrices_u.nm) * normalize(vec3(1.0, 0.0, tx));
    vec3 bitangent = mat3(matrices_u.nm) * normalize(vec3(0.0, -1.0, -ty));
    for(int i = 0; i < lights_u.count; i++) {
      vec3 L;
      switch(lights_u.type[i]) {
        case 0:    /* point light */
        case 1: {  /* spot light */
          vec3 temp = (matrices_u.v * vec4(lights_u.position[i], 1.0)).xyz;
          temp -= position_cameraSpace.xyz;
          L = normalize(temp);
        } break;
        case 2: { /* directional light */
          L = normalize(mat3(matrices_u.v) * lights_u.direction[i]);
        } break;
      }
      tes_out.light_direction_tangentSpace[i] =
        normalize(vec3(dot(tangent, L), dot(bitangent, L), dot(normal_cameraSpace, L)));
    }
    vec3 V = -position_cameraSpace.xyz;
    tes_out.view_direction_tangentSpace =
      normalize(vec3(dot(tangent, V), dot(bitangent, V), dot(normal_cameraSpace, V)));
  }

  tes_out.tc = tc * terrain_u.scale / terrain_u.tcScale;
  tes_out.normal_cameraSpace = normal_cameraSpace;
  tes_out.position_cameraSpace = position_cameraSpace.xyz;

  /* compute shadow coordinates */
  if(lights_u.drawShadows == 1) {
    for(int i = 0; i < lights_u.count; i++) {
      shadowCoord[i] = lights_u.shadowMatrix[i] * position;
    }
  }

  /* calculate position in clip coordinates */
  gl_Position = matrices_u.mvp * position;

}

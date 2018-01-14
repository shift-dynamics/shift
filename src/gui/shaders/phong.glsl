#version 420 core

/* phong.vert */

layout (binding = 0) uniform sampler2DArrayShadow texture_shadow_u;
layout (binding = 1) uniform samplerCubeArrayShadow texture_shadowCube_u;

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

#ifdef VERTEX_SHADER
layout (location = 0) in vec4 position_a;
layout (location = 1) in vec3 normal_a;

out vec3 normalFrag;
out vec4 positionFrag;
out vec4 shadowCoord[10];
out vec4 eyeDirectionCameraSpace;


void main() {
  normalFrag = normalize(matrices_u.nm * normal_a);
  positionFrag = position_a;
  gl_Position = matrices_u.mvp * position_a;
  if(lights_u.drawShadows == 1) {
    for(int i = 0; i < lights_u.count; i++) {
      shadowCoord[i] =
        lights_u.shadowMatrix[i] * position_a;
    }
    eyeDirectionCameraSpace = -matrices_u.mv * position_a;
  }
}
#endif


#ifdef FRAGMENT_SHADER
in vec4 positionFrag;
in vec3 normalFrag;
in vec4 eyeDirectionCameraSpace;
in vec4 shadowCoord[10];

vec2 poissonDisk[16] = vec2[](
   vec2(-0.94201624, -0.39906216),
   vec2(0.94558609, -0.76890725),
   vec2(-0.094184101, -0.92938870),
   vec2(0.34495938, 0.29387760),
   vec2(-0.91588581, 0.45771432),
   vec2(-0.81544232, -0.87912464),
   vec2(-0.38277543, 0.27676845),
   vec2(0.97484398, 0.75648379),
   vec2(0.44323325, -0.97511554),
   vec2(0.53742981, -0.47373420),
   vec2(-0.26496911, -0.41893023),
   vec2(0.79197514, 0.19090188),
   vec2(-0.24188840, 0.99706507),
   vec2(-0.81409955, 0.91437590),
   vec2(0.19984126, 0.78641367),
   vec2(0.14383161, -0.1410079 )
);

out vec4 colorFrag;

float offset_lookup(
  sampler2DArrayShadow texture_shadow,
  int index,
  vec4 location,
  vec2 offset) {
  return texture(
    texture_shadow,
    vec4(
      location.x / location.w + offset.x,
      location.y / location.w + offset.y,
      float(index),
      location.z / location.w
    )
  );
}


void main() {
  vec4 color = vec4(0.0, 0.0, 0.0, 1.0);
  vec3 V = vec3(0.0, 0.0, 1.0);
  for(int i = 0; i < lights_u.count; i++) {
    vec3 L;
    switch(lights_u.directional[i]) {
      case 0: {
        /* 0= point light
          light.position indicates position of light source
          used to compute direction of incoming light */
        vec4 temp = matrices_u.v * vec4(
          lights_u.position[i].x,
          lights_u.position[i].y,
          lights_u.position[i].z,
          1.0)
          - matrices_u.mv * positionFrag;
        L = normalize(temp.xyz);
        break;
      }
      case 1: {
        /* 1 = spot light
          light.position indicates position of light source
          used to compute direction of incoming light */
        vec4 temp = matrices_u.v * vec4(
          lights_u.position[i].x,
          lights_u.position[i].y,
          lights_u.position[i].z,
          1.0)
          - matrices_u.mv * positionFrag;
        L = normalize(temp.xyz);
        break;
      }
      case 2: {
        /* 2 = directional light
          light.position indicates direction of light source
          used to compute direction of incoming light */
        vec4 temp = matrices_u.v * vec4(
          lights_u.position[i].x,
          lights_u.position[i].y,
          lights_u.position[i].z,
          1.0);
        L = normalize(temp.xyz);
        break;
      }
      default:
        break;
    }

    float shadowTest = 1.f;
    float LdotN =  max(0.0, dot(L, normalFrag));
    if(lights_u.drawShadows == 1) {
      shadowTest = 0.0;
      for(float y = -1.5; y <= 1.5; y += 1.0) {
        for(float x = -1.5; x <= 1.5; x += 1.0) {
          shadowTest += offset_lookup(
            texture_shadow_u, i, shadowCoord[i], vec2(x, y) / 4096.0);
        }
      }
      shadowTest /= 16.0;
      // shadowTest = texture(
      //   texture_shadow_u,
      //   vec4(
      //     shadowCoord[i].x / shadowCoord[i].w,
      //     shadowCoord[i].y / shadowCoord[i].w,
      //     float(i),
      //     shadowCoord[i].z / shadowCoord[i].w)
      //     );
      //
      // float cosTheta = clamp(LdotN, 0.0, 1.0);
      // float bias = 0.005 * tan(acos(cosTheta));
      // bias = clamp(bias, 0.0, 0.01);
      //
      // shadowTest = 0;
      // for(int j = 0; j < 4; j++) {
      //   shadowTest -=
      //     0.2 * (1.0 - texture(
      //       texture_shadow_u,
      //       vec4(
      //         (shadowCoord[i].x + poissonDisk[j].x / 700.0) / shadowCoord[i].w,
      //         (shadowCoord[i].y + poissonDisk[j].y / 700.0) / shadowCoord[i].w,
      //         float(i),
      //         (shadowCoord[i].z) / shadowCoord[i].w)));
      // }

    }

    /* diffuse term */
    color.rgb += material_u.Kd.rgb * lights_u.Kd[i] * LdotN * shadowTest;

    /* specular term */
    vec3 R = -normalize(reflect(L, normalFrag));
    color.rgb +=
      material_u.Ks.rgb * lights_u.Ks[i] * pow(max(0, dot(R, V)), material_u.Ns) * shadowTest;

  }
  /* ambient term */
  color.rgb += material_u.Ka.rgb;
  colorFrag = min(color, vec4(1.0, 1.0, 1.0, 1.0));

  /* opacity */
  colorFrag.a = material_u.Kd.a;
}
#endif

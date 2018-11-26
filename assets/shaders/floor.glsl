#version 440 core
/* floor.glsl */

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat3 nm;
};

struct lights_t {
  vec3 position[10];
  vec3 direction[10];
  int type[10];
  vec3 Ka[10];
  vec3 Kd[10];
  vec3 Ks[10];
  int count;
  int drawShadows;
  mat4 shadowMatrix[10];
};

struct floor_t {
  float tcScale;
  int enableNormalMap;
  int enableSpecularMap;
  int enableBumpMap;
};

layout (std140, binding = 0) uniform UniformBlock {
  matrices_t matrices_u;
  lights_t lights_u;
  floor_t floor_u;
};
layout (binding = 1) uniform sampler2DArrayShadow texture_shadow_u;
layout (binding = 2) uniform samplerCubeArrayShadow texture_shadowCube_u;
layout (binding = 3) uniform sampler2D texture_diffuse_u;
layout (binding = 4) uniform sampler2D texture_normal_u;
layout (binding = 5) uniform sampler2D texture_specular_u;
layout (binding = 6) uniform sampler2D texture_bump_u;


#ifdef VERTEX_SHADER

layout (location = 0) out VS_OUT {
  vec2 tc;
  vec3 normal_cameraSpace;
  vec3 position_cameraSpace;
  vec3 view_direction_tangentSpace;
  vec3 light_direction_tangentSpace[10];
  vec4 shadowCoord[10];
} vs_out;

void main() {
  vec3[4] vertices = vec3[4](
    vec3(-1.0, -1.0, 0.0),
    vec3(-1.0, 1.0, 0.0),
    vec3(1.0, -1.0, 0.0),
    vec3(1.0, 1.0, 0.0));
  vec2[4] tc = vec2[4](
    vec2(0.0, 0.0),
    vec2(1.0, 0.0),
    vec2(0.0, 1.0),
    vec2(1.0, 1.0));

  vec4 position = vec4(vertices[gl_VertexIndex], 1.0);
  vs_out.tc = floor_u.tcScale * tc[gl_VertexIndex];

  vec4 position_cameraSpace = matrices_u.mv * position;
  vec3 normal = vec3(0.0, 0.0, -1.0);
  vec3 normal_cameraSpace = matrices_u.nm * normal;

  if(floor_u.enableNormalMap == 1) {
    /* tangent and bitangent in camera space */
    vec3 tangent = mat3(matrices_u.nm) * vec3(1.0, 0.0, 0.0);
    vec3 bitangent = mat3(matrices_u.nm) * vec3(0.0, -1.0, 0.0);
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
          L = normalize(mat3(matrices_u.v) * lights_u.position[i]);
        } break;
      }
      vs_out.light_direction_tangentSpace[i] =
        normalize(vec3(dot(tangent, L), dot(bitangent, L), dot(normal_cameraSpace, L)));
    }
    vec3 V = -position_cameraSpace.xyz;
    vs_out.view_direction_tangentSpace =
      normalize(vec3(dot(tangent, V), dot(bitangent, V), dot(normal_cameraSpace, V)));
  }

  vs_out.normal_cameraSpace = normal_cameraSpace;
  vs_out.position_cameraSpace = position_cameraSpace.xyz;

  /* compute shadow coordinates */
  if(lights_u.drawShadows == 1) {
    for(int i = 0; i < lights_u.count; i++) {
      vs_out.shadowCoord[i] = lights_u.shadowMatrix[i] * position;
    }
  }
  gl_Position = matrices_u.mvp * position;
}
#endif

#ifdef FRAGMENT_SHADER

layout (location = 0) in VS_OUT {
  vec2 tc;
  vec3 normal_cameraSpace;
  vec3 position_cameraSpace;
  vec3 view_direction_tangentSpace;
  vec3 light_direction_tangentSpace[10]; /* light direction in texture space */
  vec4 shadowCoord[10];
} fs_in;

layout (location = 0) out vec4 colorFrag;

float offset_lookup(sampler2DArrayShadow texture_shadow, int index,
                    vec4 location, vec2 offset) {
  return texture(
    texture_shadow, vec4(
      location.x / location.w + offset.x, location.y / location.w + offset.y,
      float(index), location.z / location.w
    )
  );
}

void main() {
  vec4 color = vec4(0.0, 0.0, 0.0, 1.0);
  vec4 KdMaterial = texture(texture_diffuse_u, fs_in.tc);
  vec3 KaMaterial = KdMaterial.rgb * 0.2;
  vec3 KsMaterial;
  if(floor_u.enableSpecularMap == 1) {
    KsMaterial = texture(texture_specular_u, fs_in.tc).rgb;
  } else {
    /* if no specular map supplied, set uniform specularity */
    KsMaterial = KdMaterial.rgb;
  }

  vec3 V = vec3(0.0, 0.0, 1.0);
  int j = 0; /* index into shadowmap array */
  int k = 0; /* index into shadowcubemap array */
  for(int i = 0; i < lights_u.count; i++) {
    vec3 L; /* light direction */
    vec3 N; /* normal direction */
    float attenuation = 1.0;
    if(floor_u.enableNormalMap == 1) {
      /* transform L and V to tangent space, retrieve normal from normal map */
      L = fs_in.light_direction_tangentSpace[i];
      V = fs_in.view_direction_tangentSpace;
      N = texture(texture_normal_u, fs_in.tc).xyz;
      N = normalize(N * 2.0 - 1.0);
    } else {
      switch(lights_u.type[i]) {
        case 0:    /* point light */
        case 1: {  /* spot light */
          vec3 temp = (matrices_u.v * vec4(lights_u.position[i], 1.0)).xyz;
          temp -= fs_in.position_cameraSpace;
          L = normalize(temp);
        } break;
        case 2: { /* directional light */
          vec3 temp = mat3(matrices_u.v) * lights_u.position[i];
          L = normalize(temp);
        } break;
      }
      N = fs_in.normal_cameraSpace;
    }

    float shadowTest = 1.f;
    if(lights_u.drawShadows == 1) {
      // switch(lights_u.type[i]) {
      //   case 0:   /* point @TODO use shadowcube array for this case*/
      //   case 1:   /* spot */
      //   case 2: { /* directional */
      shadowTest = 0.0;
      for(float y = -1.5; y <= 1.5; y += 1.0) {
        for(float x = -1.5; x <= 1.5; x += 1.0) {
          shadowTest +=
            offset_lookup(texture_shadow_u, i,
                          fs_in.shadowCoord[i], vec2(x, y) / 4096.0);
        }
      }
      // j++;
      shadowTest /= 16.0;
      //   } break;
      // }
    }

    if(floor_u.enableBumpMap == 1) {
      shadowTest *= texture(texture_bump_u, fs_in.tc).r;
    }

    /* diffuse term */
    float LdotN =  max(0.0, dot(L, N));
    color.rgb += KdMaterial.rgb * lights_u.Kd[i] * LdotN * shadowTest;

    /* specular term */
    vec3 R = -normalize(reflect(L, N));
    color.rgb += KsMaterial * lights_u.Ks[i] * pow(max(0, dot(R, V)), 10.0) * shadowTest;
  }

  /* ambient term */
  // color.rgb += KaMaterial;

  colorFrag = min(color, vec4(1.0, 1.0, 1.0, 1.0));
  colorFrag.a = 0.9;

  // if(enable_fog) {
  //   color = fog(landscape);
  // }
}
#endif

#version 450 core
/* bezier.glsl */

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

struct bezier_t {
  int enableDiffuseMap;
  int enableNormalMap;
  int enableSpecularMap;
  int enableBumpMap;
};
uniform bezier_t bezier_u;


#ifdef VERTEX_SHADER

layout (location = 0) in vec4 position_a;
layout (location = 1) in vec3 normal_a;
layout (location = 1) in vec2 tc_a;

out VS_OUT {
  vec2 tc;
} vs_out;

void main(void) {
  gl_Position = matrices_u.mv * position_a;
  vs_out.tc = tc_a;
}
#endif


#ifdef TESSELATION_CONTROL_SHADER
layout (vertices = 16) out;

in VS_OUT {
  vec2 tc;
} tcs_in[];

out TCS_OUT {
  vec2 tc;
} tcs_out[];

void main(void) {

  if(gl_InvocationID == 0) {
    gl_TessLevelInner[0] = 16.0;
	  gl_TessLevelInner[1] = 16.0;
    gl_TessLevelOuter[0] = 16.0;
    gl_TessLevelOuter[1] = 16.0;
    gl_TessLevelOuter[2] = 16.0;
    gl_TessLevelOuter[3] = 16.0;
  }

  gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
  tcs_out[gl_InvocationID].tc = tcs_in[gl_InvocationID].tc;
}
#endif


#ifdef TESSELATION_EVALUATION_SHADER
layout(quads, equal_spacing, cw) in;
in TCS_OUT {
  vec2 tc;
} tes_in[];

out TES_OUT {
  vec2 tc;
  vec3 normal_cameraSpace;
  vec3 position_cameraSpace;
  vec3 view_direction_tangentSpace;
  vec3 light_direction_tangentSpace[10];
  vec4 shadowCoord[10];
} tes_out;

vec4 quadratic_bezier(vec4 a, vec4 b, vec4, c, float t) {
  vec4 d = mix(a, b, t);
  vec4 e = mix(b, c, t);
  return mix(d, e, t);
}

vec4 cubic_bezier(vec4 a, vec4 b, vec4 c, vec4 d, float t) {
  vec4 e = mix(a, b, t);
  vec4 f = mix(b, c, t);
  vec4 g = mix(c, d, t);
  return quadratic_bezier(e, f, g, t);
}

vec4 evaluate_patch(vec2 at) {
  vec4 p[4];
  for(int i = 0; i < 4; i++) {
    p[i] = cubic_bezier(gl_in[i + 0].gl_Position,
                        gl_in[i + 4].gl_Position,
                        gl_in[i + 8].gl_Position,
                        gl_in[i + 12].gl_Position,
                        at.y]);
  }
  return cubic_bezier(p[0], p[1], p[2], p[3], at.x);
}

const float epsilon = 0.001;

void main(void) {
  vec4 position = evaluate_patch(gl_TessCoord.xy);
  vec4 p2 = evaluate_patch(gl_TessCoord.xy + vec2(0.0, epsilon));
  vec4 p3 = evaluate_patch(gl_TessCoord.xy + vec2(epsilon, 0.0));
  vec3 v1 = normalize(p2.xyz - position.xyz);
  vec3 v2 = normalize(p3.xyz - position.xyz);
  vec4 normal = cross(v1, v2);
  vec4 position_cameraSpace = matrices_u.mv * position;
  gl_Position = matrices_u.mvp * position;

  vec4 position = mix(p2, p1, gl_TessCoord.y);
  vec3 normal_cameraSpace = matrices_u.nm * normal;

  if(bezier_u.enableNormalMap == 1) {
    /* tangent and bitangent in camera space */
    vec3 tangent = mat3(matrices_u.nm) * v1;
    vec3 bitangent = mat3(matrices_u.nm) * v2;
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

  tes_out.tc = tc;
  tes_out.normal_cameraSpace = normal_cameraSpace;
  tes_out.position_cameraSpace = position_cameraSpace.xyz;

  /* compute shadow coordinates */
  if(lights_u.drawShadows == 1) {
    for(int i = 0; i < lights_u.count; i++) {
      tes_out.shadowCoord[i] = lights_u.shadowMatrix[i] * position;
    }
  }
  gl_Position = matrices_u.mvp * position;
}
#endif


#ifdef FRAGMENT_SHADER
in TES_OUT {
  vec2 tc;
  vec3 normal_cameraSpace;
  vec3 position_cameraSpace;
  vec3 view_direction_tangentSpace;
  vec3 light_direction_tangentSpace[10]; /* light direction in texture space */
  vec4 shadowCoord[10];
} fs_in;
out vec4 colorFrag;

float offset_lookup(sampler2DArrayShadow texture_shadow, int index,
                    vec4 location, vec2 offset) {
  return texture(
    texture_shadow, vec4(
      location.x / location.w + offset.x, location.y / location.w + offset.y,
      float(index), location.z / location.w
    )
  );
}

// uniform bool enable_fog = true;
// uniform vec4 fog_color = vec4(0.7, 0.8, 0.9, 0.0);
// vec4 fog(vec4 c) {
//   float z = length(fs_in.eye_coord);
//
//   float de = 0.025 * smoothstep(0.0, 6.0, 10.0 - fs_in.world_coord.z);
//   float di = 0.045 * (smoothstep(0.0, 40.0, 20.0 - fs_in.world_coord.z));
//
//   float extinction   = exp(-z * de);
//   float inscattering = exp(-z * di);
//
//   return c * extinction + fog_color * (1.0 - inscattering);
// }

void main(void) {
  vec4 color = vec4(0.0, 0.0, 0.0, 1.0);
  vec4 KdMaterial;
  if(bezier_u.enableDiffuseMap == 1) {
    /* get diffuse color from texture */
    KdMaterial = texture2D(texture_diffuse_u, fs_in.tc);
  } else {
    /* if no color map supplied, then use uniform material color */
    KdMaterial = material_u.Kd;
  }
  vec3 KaMaterial = KdMaterial.rgb * material_u.KaOverKd;
  vec3 KsMaterial;
  if(bezier_u.enableSpecularMap == 1) {
    KsMaterial = texture(texture_specular_u, fs_in.tc).rgb;
  } else {
    /* if no specular map supplied, set uniform specularity */
    KsMaterial = vec3(1.0, 1.0, 1.0) * material_u.KsOverKd;
  }

  vec3 V = vec3(0.0, 0.0, 1.0);
  int j = 0; /* index into shadowmap array */
  int k = 0; /* index into shadowcubemap array */
  for(int i = 0; i < lights_u.count; i++) {
    vec3 L; /* light direction */
    vec3 N; /* normal direction */
    float attenuation = 1.0;
    if(bezier_u.enableNormalMap == 1) {
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
          vec3 temp = mat3(matrices_u.v) * lights_u.direction[i];
          L = normalize(temp);
        } break;
      }
      N = fs_in.normal_cameraSpace;
    }

    float shadowTest = 1.0;
    if(lights_u.drawShadows == 1) {
      switch(lights_u.type[i]) {
        case 0: { /* point */

        } break;
        case 1:   /* spot */
        case 2: { /* directional */
          shadowTest = 0.0;
          for(float y = -1.5; y <= 1.5; y += 1.0) {
            for(float x = -1.5; x <= 1.5; x += 1.0) {
              // vec2 offset = vec2(x, y) / float(textureSize(texture_shadow_u, 0).xy);
              vec2 offset = vec2(x, y) / 4096.0;
              shadowTest +=
                offset_lookup(texture_shadow_u, j,
                              fs_in.shadowCoord[j], offset);
            }
          }
          j++;
          shadowTest /= 16.0;
        } break;
      }
    }

    float bumpTest = 1.f;
    if(bezier_u.enableBumpMap == 1) {
      bumpTest = texture(texture_bump_u, fs_in.tc).r;
    }

    /* diffuse term */
    float LdotN =  max(0.0, dot(L, N));
    color.rgb += KdMaterial.rgb * lights_u.Kd[i] * LdotN * shadowTest * bumpTest;

    /* specular term */
    vec3 R = -normalize(reflect(L, N));
    color.rgb += KsMaterial * lights_u.Ks[i] * pow(max(0, dot(R, V)), material_u.Ns) * shadowTest * bumpTest;
  }

  /* ambient term */
  color.rgb += KaMaterial;
  colorFrag = min(color, vec4(1.0, 1.0, 1.0, 1.0));
  colorFrag.a = KdMaterial.a;

  // if(enable_fog) {
  //   color = fog(landscape);
  // }
}
#endif

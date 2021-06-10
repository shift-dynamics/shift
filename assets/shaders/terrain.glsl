#version 440 core
/* terrain.vert */

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat4 p;
  mat3 nm;
};

struct material_t {
  vec4 Kd;
  float KaOverKd;
  float KsOverKd;
  int Ns;
};

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
};

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

layout (std140, binding = 0) uniform BufferBlock {
  matrices_t matrices_u;
  material_t material_u;
  lights_t lights_u;
  terrain_t terrain_u;
};
layout (binding = 1) uniform sampler2DArrayShadow texture_shadow_u;
layout (binding = 2) uniform samplerCubeArrayShadow texture_shadowCube_u;
layout (binding = 3) uniform sampler2D texture_elevation_u;
layout (binding = 4) uniform sampler2D texture_diffuse_u;
layout (binding = 5) uniform sampler2D texture_normal_u;
layout (binding = 6) uniform sampler2D texture_specular_u;
layout (binding = 7) uniform sampler2D texture_bump_u;

#ifdef VERTEX_SHADER
layout(location = 0) out VS_OUT {
  vec2 tc;
} vs_out;

void main(void) {
  const vec4 vertices[] = vec4[](
    vec4(0.0, 0.0, 0.0, 1.0),
    vec4(1.0, 0.0, 0.0, 1.0),
    vec4(0.0, 1.0, 0.0, 1.0),
    vec4(1.0, 1.0, 0.0, 1.0));

  ivec2 texture_size = textureSize(texture_elevation_u, 0);
  int x = gl_InstanceIndex % texture_size.x; // row index into elevation map
  int y = gl_InstanceIndex / texture_size.x; // column index into elevation map
  vs_out.tc = (vertices[gl_VertexIndex].xy + vec2(x, y)) / texture_size;
  gl_Position = vertices[gl_VertexIndex] + vec4(float(x), float(y), 0.0, 0.0);
}
#endif


#ifdef TESSELLATION_CONTROL_SHADER
layout (vertices = 4) out;

layout(location = 0) in VS_OUT {
  vec2 tc;
} tcs_in[];

layout(location = 0) out TCS_OUT {
  vec2 tc;
} tcs_out[];

float distTess(vec4 p0, vec4 p1, vec2 t0, vec2 t1) {
	// g_tessellatedTriWidth is desired pixels per tri edge
	float g_tessellatedTriWidth = 150.0;

  p0.z += (texture(texture_elevation_u, t0).r * (terrain_u.zMax - terrain_u.zMin)) + terrain_u.zMin;
  p1.z += (texture(texture_elevation_u, t1).r * (terrain_u.zMax - terrain_u.zMin)) + terrain_u.zMin;

	vec4 center = 0.5 * (p0 + p1);
	float radius = distance(p0, p1) / 2.0;

	vec4 sc0 = matrices_u.mv * center;
	vec4 sc1 = sc0;
	sc0.x -= radius;
	sc1.x += radius;

	vec4 clip0 = matrices_u.p * sc0;
	vec4 clip1 = matrices_u.p * sc1;

	clip0 /= clip0.w;
	clip1 /= clip1.w;

	clip0.xy *= terrain_u.viewport;
	clip1.xy *= terrain_u.viewport;

	float d = distance(clip0, clip1);

	return clamp(d / g_tessellatedTriWidth, 0, 64);
}

void main(void) {

  gl_TessLevelOuter[0] =
    distTess(gl_in[3].gl_Position, gl_in[0].gl_Position,
             tcs_in[3].tc, tcs_in[0].tc);

  gl_TessLevelOuter[1] =
    distTess(gl_in[0].gl_Position, gl_in[1].gl_Position,
             tcs_in[0].tc, tcs_in[1].tc);

  gl_TessLevelOuter[2] =
    distTess(gl_in[1].gl_Position, gl_in[2].gl_Position,
             tcs_in[1].tc, tcs_in[2].tc);

  gl_TessLevelOuter[3] =
    distTess(gl_in[2].gl_Position, gl_in[3].gl_Position,
             tcs_in[2].tc, tcs_in[3].tc);

  gl_TessLevelInner[0] = 0.5 * (gl_TessLevelOuter[0] + gl_TessLevelOuter[3]);
	gl_TessLevelInner[1] = 0.5 * (gl_TessLevelOuter[2] + gl_TessLevelOuter[1]);
  gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
  tcs_out[gl_InvocationID].tc = tcs_in[gl_InvocationID].tc;
}
#endif


#ifdef TESSELLATION_EVALUATION_SHADER
layout(quads, fractional_odd_spacing) in;
layout(location = 0) in TCS_OUT {
  vec2 tc;
} tes_in[];

layout(location = 0) out TES_OUT {
  vec2 tc;
  vec3 normal_cameraSpace;
  vec3 position_cameraSpace;
  vec3 view_direction_tangentSpace;
  vec3 light_direction_tangentSpace[10];
  vec4 shadowCoord[10];
} tes_out;

void main(void) {

  vec2 tc1 = mix(tes_in[0].tc, tes_in[1].tc, gl_TessCoord.x);
  vec2 tc2 = mix(tes_in[2].tc, tes_in[3].tc, gl_TessCoord.x);
  vec2 tc = mix(tc2, tc1, gl_TessCoord.y);

  vec4 p1 = mix(gl_in[0].gl_Position, gl_in[1].gl_Position, gl_TessCoord.x);
  vec4 p2 = mix(gl_in[2].gl_Position, gl_in[3].gl_Position, gl_TessCoord.x);

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

  vec4 position = mix(p2, p1, gl_TessCoord.y);

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
      tes_out.shadowCoord[i] = lights_u.shadowMatrix[i] * position;
    }
  }
  gl_Position = matrices_u.mvp * position;
}
#endif


#ifdef FRAGMENT_SHADER
layout(location = 0) in TES_OUT {
  vec2 tc;
  vec3 normal_cameraSpace;
  vec3 position_cameraSpace;
  vec3 view_direction_tangentSpace;
  vec3 light_direction_tangentSpace[10]; /* light direction in texture space */
  vec4 shadowCoord[10];
} fs_in;

layout(location = 0) out vec4 colorFrag;

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
  if(terrain_u.enableDiffuseMap == 1) {
    /* get diffuse color from texture */
    KdMaterial = texture(texture_diffuse_u, fs_in.tc);
  } else {
    /* if no color map supplied, then use uniform material color */
    KdMaterial = material_u.Kd;
  }
  vec3 KaMaterial = KdMaterial.rgb * material_u.KaOverKd;
  vec3 KsMaterial;
  if(terrain_u.enableSpecularMap == 1) {
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
    if(terrain_u.enableNormalMap == 1) {
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
    if(terrain_u.enableBumpMap == 1) {
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

#version 420 core

/* terrain.frag */
layout (binding = 0) uniform sampler2DArrayShadow texture_shadow_u;
layout (binding = 1) uniform sampler2D texture_displacement_u;
layout (binding = 2) uniform sampler2D texture_color_u;

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
  vec3 Ka[10];
  vec3 Kd[10];
  vec3 Ks[10];
  int directional[10];
  int count;
  int drawShadows; /*!< 0 = false, 1 = true */
  mat4 shadowMatrix[10];
};

uniform lights_t lights_u;

struct terrain_t {
  float zMin;
  float zMax;
  float xScale;
  float yScale;
  float xRange;
  float yRange;
  vec2 viewport;
};

uniform terrain_t terrain_u;

in vec3 normalFrag;
in vec4 positionFrag;
in vec4 shadowCoord[10];

in TES_OUT {
  vec2 tc;
  vec3 normal;
  vec3 world_coord;
  vec3 eye_coord;
} fs_in;

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
  vec3 V = vec3(0.0, 0.0, 1.0);

  /* get diffuse color from texture */
  vec4 KdMaterial = texture2D(texture_color_u, fs_in.tc);
  vec3 KaMaterial = KdMaterial.rgb * material_u.KaOverKd;
  vec3 KsMaterial = KdMaterial.rgb * material_u.KsOverKd;

  for(int i = 0; i < lights_u.count; i++) {
    vec3 L;
    switch(lights_u.directional[i]) {
      case 0: {
        // 0= point light, posLight indicates position of light source
        //need to compute direction of incoming light

        vec4 temp = matrices_u.v * vec4(lights_u.position[i], 1) - vec4(fs_in.eye_coord, 1);
        L = normalize(temp.xyz);
      } break;
      case 1: {
        // 1=directional light, posLight indicates direction of incoming light

        vec4 temp = matrices_u.v * vec4(
          lights_u.position[i].x,
          lights_u.position[i].y,
          lights_u.position[i].z,
          1.0);
        L = normalize(temp.xyz);
      } break;
    }

    float shadowTest = 1.f;
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
    }

    /* ambient term */
    float LdotN =  max(0.0, dot(L, fs_in.normal));

    /* diffuse term */
    color.rgb += KdMaterial.rgb * lights_u.Kd[i] * LdotN * shadowTest;

    /* specular term */
    vec3 R = -normalize(reflect(L, fs_in.normal));
    color.rgb += KsMaterial * lights_u.Ks[i] * pow(max(0, dot(R, V)), material_u.Ns) * shadowTest;

    color.rgb += KaMaterial;

  }

  colorFrag = min(color, vec4(1.0, 1.0, 1.0, 1.0));
  // colorFrag = vec4(0.3, 0.3, 0.3, 1.0);
  colorFrag.a = KdMaterial.a;

  // vec4 landscape = texture(tex_color, fs_in.tc);

  // if (enable_fog) {
  //   color = fog(landscape);
  // } else {
  //   color = landscape;
  // }

  // color = vec4(0.4, 0.2, 0.5, 1.0);
  // float temp = clamp(texture(tex_displacement, fs_in.tc).r, 0.0, 1.0);
  // color = vec4(temp, temp, temp, 1.0);

  // color = vec4((texture(tex_color, fs_in.tc)).rgb, 1.0);

  // vec4 color = vec4(0.0, 0.0, 0.0, 1.0);
  // vec3 V = vec3(0.0, 0.0, 1.0);
  //
  // for(int i = 0; i < lightCount; i++) {
  //   vec3 L;
  //   switch(directional[i]) {
  //     case 0: {
  //       // 0= point light, posLight indicates position of light source
  //       //need to compute direction of incoming light
  //
  //       vec4 temp = v * vec4(
  //         posLight[i].x,
  //         posLight[i].y,
  //         posLight[i].z,
  //         1.0)
  //         - mv * positionFrag;
  //       L = normalize(temp.xyz);
  //     }
  //     break;
  //     case 1: {
  //       // 1=directional light, posLight indicates direction of incoming light
  //
  //       vec4 temp = v * vec4(
  //         posLight[i].x,
  //         posLight[i].y,
  //         posLight[i].z,
  //         1.0);
  //       L = normalize(temp.xyz);
  //     }
  //     break;
  //   }
  //
  //   color.rgb += KaMaterial * Ka[i];          //ambient term
  //   float LdotN =  max(0.0, dot(L, normalFrag));
  //
  //   color.rgb += KdMaterial * Kd[i] * LdotN; //diffuse term
  //
  //   vec3 R = -normalize(reflect(L, normalFrag));       //specular term
  //   color.rgb += KsMaterial * Ks[i] * pow(max(0, dot(R, V)), Ns);
  // }
  // colorFrag = min(color, vec4(1.0, 1.0, 1.0, 1.0));

}

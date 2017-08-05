#version 420 core

layout (binding = 0) uniform sampler2D tex_displacement;
layout (binding = 1) uniform sampler2D tex_color;

/* light parameters */
uniform int lightCount;
uniform vec3 posLight[10];
uniform vec3 Ka[10];
uniform vec3 Kd[10];
uniform vec3 Ks[10];
uniform int directional[10];

/* body parameters */
uniform mat4 mvp, mv, v;
uniform mat3 nm;

/* material parameters */
uniform vec3 KaOverKd;
uniform vec3 KsOverKd;
uniform int Ns;

in vec3 normalFrag;
in vec4 positionFrag;

in TES_OUT {
  vec2 tc;
  vec3 normal;
  vec3 world_coord;
  vec3 eye_coord;
} fs_in;

out vec4 colorFrag;

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

  for(int i = 0; i < lightCount; i++) {
    vec3 L;
    switch(directional[i]) {
      case 0: {
        // 0= point light, posLight indicates position of light source
        //need to compute direction of incoming light

        vec4 temp = v * vec4(posLight[i], 1) - vec4(fs_in.eye_coord, 1);
        L = normalize(temp.xyz);
      } break;
      case 1: {
        // 1=directional light, posLight indicates direction of incoming light

        vec4 temp = v * vec4(
          posLight[i].x,
          posLight[i].y,
          posLight[i].z,
          1.0);
        L = normalize(temp.xyz);
      } break;
    }

    /* get diffuse color from texture */
    vec4 KdMaterial = texture2D(tex_color, fs_in.tc);

    /* ambient term */
    color.rgb += KaOverKd * KdMaterial.rgb * Ka[i];
    float LdotN =  max(0.0, dot(L, fs_in.normal));

    /* diffuse term */
    color.rgb += KdMaterial.rgb * Kd[i] * LdotN;

    /* specular term */
    vec3 R = -normalize(reflect(L, fs_in.normal));
    color.rgb += KsOverKd * KdMaterial.rgb * Ks[i] * pow(max(0, dot(R, V)), Ns);
  }
  colorFrag = min(color, vec4(1.0, 1.0, 1.0, 1.0));

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

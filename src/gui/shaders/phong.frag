#version 330

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
  int drawShadows; /*!< 0 = false, 1 = true */
  mat4 shadowMatrix[10];
};

uniform lights_t lights_u;

in vec3 normalFrag;
in vec4 positionFrag;

out vec4 colorFrag;

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
      } break;
      case 1: {
        /* 1 = directional light
          light.position indicates position of light source
          used to compute direction of incoming light */

        vec4 temp = matrices_u.v * vec4(
          lights_u.position[i].x,
          lights_u.position[i].y,
          lights_u.position[i].z,
          1.0);
        L = normalize(temp.xyz);
      } break;
      default: {

      } break;
    }

    /* ambient term */
    color.rgb += material_u.Ka.rgb * lights_u.Ka[i];

    /* diffuse term */
    float LdotN =  max(0.0, dot(L, normalFrag));
    color.rgb += material_u.Kd.rgb * lights_u.Kd[i] * LdotN;

    /* specular term */
    vec3 R = -normalize(reflect(L, normalFrag));
    color.rgb +=
      material_u.Ks.rgb * lights_u.Ks[i] * pow(max(0, dot(R, V)), material_u.Ns);
  }

  colorFrag = min(color, vec4(1.0, 1.0, 1.0, 1.0));
  colorFrag.a = material_u.Ka.a;

}

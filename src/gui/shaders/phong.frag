#version 330

layout (std140) uniform matrices {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat3 nm;
};

layout (std140) uniform material {
  vec4 Ka;
  vec4 Kd;
  vec4 Ks;
  int Ns;
} material;

layout (std140) uniform lights {
  int count;
  int directional[10];
  vec3 position[10];
  vec3 Ka[10];
  vec3 Kd[10];
  vec3 Ks[10];
} lights;

in vec3 normalFrag;
in vec4 positionFrag;

out vec4 colorFrag;

void main() {

  vec4 color = vec4(0.0, 0.0, 0.0, 1.0);
  vec3 V = vec3(0.0, 0.0, 1.0);

  for(int i = 0; i < lights.count; i++) {
    vec3 L;
    switch(lights.directional[i]) {
      case 0: {
        /* 0= point light
          light.position indicates position of light source
          used to compute direction of incoming light */
        vec4 temp = v * vec4(
          lights.position[i].x,
          lights.position[i].y,
          lights.position[i].z,
          1.0)
          - mv * positionFrag;
        L = normalize(temp.xyz);
      }
      break;
      case 1: {
        /* 1 = directional light
          light.position indicates position of light source
          used to compute direction of incoming light */

        vec4 temp = v * vec4(
          lights.position[i].x,
          lights.position[i].y,
          lights.position[i].z,
          1.0);
        L = normalize(temp.xyz);
      }
      break;
    }

    /* ambient term */
    color.rgb += material.Ka * lights.Ka[i];

    /* diffuse term */
    float LdotN =  max(0.0, dot(L, normalFrag));
    color.rgb += material.Kd * lights.Kd[i] * LdotN;

    /* specular term */
    vec3 R = -normalize(reflect(L, normalFrag));
    color.rgb += material.Ks * lights.Ks[i] * pow(max(0, dot(R, V)), Ns);
  }

  colorFrag = min(color, vec4(1.0, 1.0, 1.0, 1.0));
  colorFrag.a = material.Ka.a;

}

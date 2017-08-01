#version 330
/*
  light parameters
*/
uniform int lightCount;
uniform vec3 posLight[10];
uniform vec3 Ka[10];
uniform vec3 Kd[10];
uniform vec3 Ks[10];
uniform int directional[10];

/*
  body parameters
*/
uniform mat4 mvp, mv, v;
uniform mat3 nm;

/*
  material parameters
*/
uniform vec3 KaMaterial;
uniform vec3 KdMaterial;
uniform vec3 KsMaterial;
uniform int Ns;

in vec3 normalFrag;
in vec4 positionFrag;

out vec4 colorFrag;

void main() {

  vec4 color = vec4(0.0, 0.0, 0.0, 1.0);
  vec3 V = vec3(0.0, 0.0, 1.0);

  for(int i = 0; i < lightCount; i++) {
    vec3 L;
    switch(directional[i]) {
      case 0: {
        // 0= point light, posLight indicates position of light source
        //need to compute direction of incoming light

        vec4 temp = v * vec4(
          posLight[i].x,
          posLight[i].y,
          posLight[i].z,
          1.0)
          - mv * positionFrag;
        L = normalize(temp.xyz);
      }
      break;
      case 1: {
        // 1=directional light, posLight indicates direction of incoming light

        vec4 temp = v * vec4(
          posLight[i].x,
          posLight[i].y,
          posLight[i].z,
          1.0);
        L = normalize(temp.xyz);
      }
      break;
    }

    color.rgb += KaMaterial * Ka[i];          //ambient term
    float LdotN =  max(0.0, dot(L, normalFrag));

    color.rgb += KdMaterial * Kd[i] * LdotN; //diffuse term

    vec3 R = -normalize(reflect(L, normalFrag));       //specular term
    color.rgb += KsMaterial * Ks[i] * pow(max(0, dot(R, V)), Ns);
  }
  colorFrag = min(color, vec4(1.0, 1.0, 1.0, 1.0));

}

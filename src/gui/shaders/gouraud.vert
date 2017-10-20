#version 330

/* gouraud.vert */

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


layout(location = 0) in vec4 position;
layout(location = 1) in vec3 normal;

out vec4 color;

void main(){
  color = vec4(0.0, 0.0, 0.0, 1.0);

  vec3 normal = normalize(nm * normal);
  vec3 V = vec3(0.0, 0.0, 1.0);

  for(int i = 0; i < lightCount; i++) {
    vec3 L;
    switch(directional[i]) {
      case 0: {
        // point light, posLight indicates position of light source
        // need to compute direction of incoming light

          vec4 temp = v * vec4(
            posLight[i].x,
            posLight[i].y,
            posLight[i].z,
            1.0)
            - mv * position;
          L = normalize(temp.xyz);
      }
      break;
      case 1: {
        //directional light, posLight indicates direction of incoming light
        vec4 temp = v * vec4(
          posLight[i].x,
          posLight[i].y,
          posLight[i].z,
          1.0);
        L = normalize(temp.xyz);
      }
      break;
    }

    color.rgb += KaMaterial * Ka[i];                  //ambient term
    float LdotN =  max(0.0, dot(L, normal));
    color.rgb += KdMaterial * Kd[i] * LdotN;       //diffuse term

    vec3 R = -normalize(reflect(L, normal));          //specular term
    color.rgb += KsMaterial * Ks[i] * pow(max(0, dot(R, V)), Ns);
  }
  gl_Position = mvp * position;
}

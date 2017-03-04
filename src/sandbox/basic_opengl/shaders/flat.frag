#version 330
// flat.frag

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

in vec4 vertexFrag;
in vec3 normalFrag;

out vec4 fragColor;

void main() {

  vec4 color = vec4(0.0, 0.0, 0.0, 1.0);
  vec3 V = vec3(0.0, 0.0, 1.0);

  //int i = 0;
  for(int i = 0; i < lightCount; i++) {
    vec3 L;
    switch(directional[i]) {
      case 0: {
        // point light, posLight indicates position of light source
        //need to compute direction of incoming light
        vec4 temp = v * vec4(
          posLight[i].x,
          posLight[i].y,
          posLight[i].z,
          1.0) - mv * vertexFrag;
        L = normalize(vec3(temp.x, temp.y, temp.z));
      }
      break;
      case 1: {
        //directional light, posLight indicates direction of incoming light
        L = normalize(mat3(v) * posLight[i]);
      }
      break;
    }

    color.xyz += KaMaterial * Ka[i];          //ambient term
    float LdotN =  max(0.0, dot(L, normalFrag));
    color.xyz += KdMaterial * Kd[i] * LdotN; //diffuse term
    vec3 R = -normalize(reflect(L, normalFrag));       //specular term
      color.xyz += KsMaterial * Ks[i] * pow(max(0, dot(R, V)), Ns);

  }
  //fragColor = vec4(1.0);
  fragColor = min(color, vec4(1.0, 1.0, 1.0, 1.0));
  //fragColor = vertexFrag;
  //fragColor = vec4(normalFrag.x, normalFrag.y, normalFrag.z, 1);

}

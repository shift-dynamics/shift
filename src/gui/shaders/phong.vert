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

in vec4 posAttr;
in vec3 normAttr;

out vec3 normalFrag;
out vec4 positionFrag;

void main(){
  normalFrag = normalize(nm * normAttr);
  positionFrag = posAttr;
  gl_Position = mvp * posAttr;
}

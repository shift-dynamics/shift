#version 330
//flat.vert

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

out vec4 vertexGeom;

void main() {
  vertexGeom = position;
  gl_Position = mvp * position;
}

#version 330

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
uniform vec4 KaMaterial;
uniform vec4 KdMaterial;
uniform vec4 KsMaterial;
uniform int Ns;

/* attributes */
in vec4 posAttr;
in vec3 normAttr;

out vec4 vertexGeom;

void main() {
  vertexGeom = posAttr;
  gl_Position = mvp * posAttr;
}

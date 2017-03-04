#version 330
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

/*  light parameters */
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

in vec4 vertexGeom[];

out vec4 vertexFrag;
out vec3 normalFrag;

void main() {
  vec4 a = vertexGeom[0];
  vec4 b = vertexGeom[1];
  vec4 c = vertexGeom[2];
  vec3 normal = nm * normalize(cross(b.xyz - a.xyz, c.xyz - b.xyz));

  for (int i = 0; i < gl_in.length(); i++) {
    vertexFrag = vertexGeom[i];
    normalFrag = normal;
    gl_Position = gl_in[i].gl_Position;
    EmitVertex();
  }
  EndPrimitive();
}

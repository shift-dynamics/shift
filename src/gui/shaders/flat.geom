#version 330
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

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

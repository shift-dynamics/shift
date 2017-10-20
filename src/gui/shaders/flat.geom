#version 330
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

/* flat.geom */

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
};

uniform lights_t lights_u;

in vec4 vertexGeom[];

out vec4 vertexFrag;
out vec3 normalFrag;

void main() {
  vec4 a = vertexGeom[0];
  vec4 b = vertexGeom[1];
  vec4 c = vertexGeom[2];
  vec3 normal = matrices_u.nm * normalize(cross(b.xyz - a.xyz, c.xyz - b.xyz));

  for (int i = 0; i < gl_in.length(); i++) {
    vertexFrag = vertexGeom[i];
    normalFrag = normal;
    gl_Position = gl_in[i].gl_Position;
    EmitVertex();
  }
  EndPrimitive();
}

#version 330
/* vertex_normal.geom */

layout(triangles) in;

// Three lines will be generated: 6 vertices
layout(line_strip, max_vertices=6) out;

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat3 nm;
};
uniform matrices_t matrices_u;
uniform float scale_u;
uniform vec3 color_u;

in vec4 position[];
in vec3 normal[];

void main() {

  for(int i = 0; i < gl_in.length(); i++) {
    vec4 p = position[i];
    vec3 n = normal[i];

    gl_Position = matrices_u.mvp * p;
    EmitVertex();

    gl_Position = matrices_u.mvp * vec4(p.xyz + n * scale_u, 1.0);
    EmitVertex();

    EndPrimitive();
  }
}

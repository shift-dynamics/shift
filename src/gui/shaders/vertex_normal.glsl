#version 330
/* vertex_normal.frag */

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat3 nm;
};
uniform matrices_t matrices_u;
uniform float scale_u;
uniform vec3 color_u;


#ifdef VERTEX_SHADER
layout (location = 0) in vec4 position_a;
layout (location = 1) in vec3 normal_a;

out vec4 position;
out vec3 normal;

void main() {
  position = position_a;
  normal = normalize(normal_a);
}
#endif


#ifdef GEOMETRY_SHADER
layout(triangles) in;
layout(line_strip, max_vertices=6) out;

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
#endif


#ifdef FRAGMENT_SHADER
out vec4 fragColor;

void main() {
  fragColor = vec4(color_u, 1.0);
}
#endif

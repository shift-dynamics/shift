#version 450
/* grid.vert */

layout(std140, binding = 0) uniform UniformBlock {
  uniform mat4 mvp_u;
  uniform vec4 color_u;
};


#ifdef VERTEX_SHADER
layout(location = 0) in vec4 position_a;
layout(location = 0) out vec4 color;

void main() {
  color = color_u;
  gl_Position = mvp_u * position_a;
}
#endif


// ifdef GEOMETRY_SHADER
// layout (lines) in;
// layout (max_vertices=2) out;
//
// void main(void) {
//   vec4 p1 =
//     mix(
//       gl_in[0].gl_Position,
//       gl_in[1].gl_Position,
//       gl_TessCoord.x);
//   vec4 p2 =
//     mix(
//       gl_in[2].gl_Position,
//       gl_in[3].gl_Position,
//       gl_TessCoord.x);
//   gl_Position = mix(p1, p2, gl_TessCoord.y);
// }
// #endif


#ifdef FRAGMENT_SHADER
layout(location = 0) in vec4 color;
layout(location = 0) out vec4 fragColor;

void main() {
  fragColor = color;
}
#endif

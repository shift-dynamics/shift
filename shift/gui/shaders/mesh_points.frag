#version 450

layout(location = 0) in vec3 color;
layout(location = 0) out vec4 out_color;

void main() {
  // round points, in their attachment's color
  vec2 d = gl_PointCoord - vec2(0.5);
  if(dot(d, d) > 0.25) {
    discard;
  }
  out_color = vec4(color, 1.0);
}

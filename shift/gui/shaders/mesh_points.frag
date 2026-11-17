#version 450

layout(location = 0) out vec4 out_color;

void main() {
  // round points, in a color that stands out on any body
  vec2 d = gl_PointCoord - vec2(0.5);
  if(dot(d, d) > 0.25) {
    discard;
  }
  out_color = vec4(1.0, 0.45, 0.1, 1.0);
}

#version 420 core

out VS_OUT {
  vec2 tc;
} vs_out;

void main(void) {
  const vec4 vertices[] = vec4[](
    vec4(0.0, 0.0, 0.0, 1.0),
    vec4(1.0, 0.0, 0.0, 1.0),
    vec4(0.0, 1.0, 0.0, 1.0),
    vec4(1.0, 1.0, 0.0, 1.0));
  int x = gl_InstanceID & 63; // grid width
  int y = gl_InstanceID >> 6; // grid height
  vec2 offs = vec2(x, y);
  vs_out.tc = (vertices[gl_VertexID].xy + offs) / 64.0;
  gl_Position =
    vertices[gl_VertexID] +
    vec4(float(x), float(y), 0.0, 0.0);
}

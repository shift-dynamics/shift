#version 330

uniform mat4 mvp_u;
uniform float scale_u;

out vec4 color;

void main() {
  const vec3 vertices[] = vec3[](
    vec3(0.0, 0.0, 0.0),
    vec3(1.0, 0.0, 0.0),
    vec3(0.0, 0.0, 0.0),
    vec3(0.0, 1.0, 0.0),
    vec3(0.0, 0.0, 0.0),
    vec3(0.0, 0.0, 1.0));

  const vec3 colors[] = vec3[](
    vec3(1.0, 0.0, 0.0),
    vec3(1.0, 0.0, 0.0),
    vec3(0.0, 1.0, 0.0),
    vec3(0.0, 1.0, 0.0),
    vec3(0.0, 0.0, 1.0),
    vec3(0.0, 0.0, 1.0));

  vec4 position = vec4(0.0, 0.0, 0.0, 1.0);
  position.xyz = scale_u * vertices[gl_VertexID];
  gl_Position = mvp_u * position;
  color = colors[gl_VertexID]
}

#version 330

uniform mat4 mvp;
uniform vec3 colorUniform;

layout(location = 0) in vec4 position;
layout(location = 1) in vec3 normal;

out vec4 color;

void main() {
  color = colorUniform;
  gl_Position = mvp * position;
}

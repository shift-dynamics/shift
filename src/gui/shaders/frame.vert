#version 330

uniform mat4 mvp;

in vec4 position_a;
in vec4 color_a;

out vec4 color;

void main() {
  gl_Position = mvp * position_a;
  color = color_a;
}

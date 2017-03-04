#version 330

uniform mat4 mvp;
uniform vec4 colorUniform;
uniform float scale;
in vec4 posAttr;

out vec4 color;

void main() {
  color = colorUniform;
  gl_Position = mvp * scale * posAttr;
}

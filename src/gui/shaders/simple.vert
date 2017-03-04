#version 330

uniform mat4 mvp;
uniform vec4 colorUniform;

in vec4 posAttr;

out vec4 color;

void main() {
  color = colorUniform;
  // color = vec4(1.f, 1.f, 1.f, 1.f);
  gl_Position = mvp * posAttr;
}

#version 330

uniform mat4 mvp;

in vec4 posAttr;
in vec4 colorAttr;

out vec4 color;

void main() {
  gl_Position = mvp * posAttr;
  // color = vec4(1.f, 0.f, 0.f, 1.f);
  color = colorAttr;
}

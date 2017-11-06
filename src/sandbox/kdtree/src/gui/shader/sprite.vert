#version 330

uniform mat4 mvp;
uniform mat4 mv;
uniform sampler2D tex;

in vec4 posAttr;
in vec4 colAttr;

out vec4 color;

void main() {
  color = colAttr;
  gl_PointSize = max(10.0 / abs((mv * posAttr).z), 1.0);
  gl_Position = mvp * posAttr;
}

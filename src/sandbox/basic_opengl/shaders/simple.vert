#version 330

uniform mat4 mvp;

in vec4 posAttr;
in vec4 colAttr;

out vec4 color;

void main() {
    color = colAttr;
    gl_Position = mvp * posAttr;
}

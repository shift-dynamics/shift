#version 330

uniform mat4 mvp, mv;

in vec4 color;

out vec4 fragColor;

void main() 
{
    fragColor = color;
}

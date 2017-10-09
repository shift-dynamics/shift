#version 330 core

// Input vertex data, different for all executions of this shader.
in vec4 position_a;

// Values that stay constant for the whole mesh.
uniform mat4 mvp_u;

void main() {
 gl_Position = mvp_u * position_a;
}

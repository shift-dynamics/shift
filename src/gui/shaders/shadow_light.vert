#version 330 core

/* shadow_light.vert */

// Input vertex data, different for all executions of this shader.
layout (location = 0) in vec4 position_a;
layout (location = 1) in vec3 normal_a;

// Values that stay constant for the whole mesh.
uniform mat4 mvp_u;

void main() {
  gl_Position = mvp_u * position_a;
}

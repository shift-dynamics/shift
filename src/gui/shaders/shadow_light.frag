#version 330 core

/* shadow_light.frag */

out float fragmentDepth;

void main() {
  fragmentDepth = gl_FragCoord.z;
}

#version 330 core

out float depthFrag;

void main() {
  depthFrag = gl_FragCoord.z;
}

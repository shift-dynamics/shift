#version 330

/* gouraud.frag */

in vec4 color;
out vec4 fragColor;

void main(){
  /*
    for gouraud shading, interpolate colors computed
    at each vertex
  */
  fragColor = min(color, vec4(1.0, 1.0, 1.0, 1.0));
}

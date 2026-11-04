#version 450
#extension GL_GOOGLE_include_directive : require
// Patches of CAD faces (cad_scene.h): vertex i is corner (c & 1, c >> 1),
// c = i % 4, of patch i / 4, a rectangle of the parameters (u, v) of its face's
// surface; the tessellation shaders place the surface on it (cad.tesc,
// cad.tese). Instances are the bodies drawn with the geometry.

// per instance: the body origin pose, times the geometry's scale
layout(location = 3) in vec4 in_model_0;
layout(location = 4) in vec4 in_model_1;
layout(location = 5) in vec4 in_model_2;
layout(location = 6) in vec4 in_model_3;

layout(location = 0) out vec2 corner_uv;
layout(location = 1) flat out int patch_index;
layout(location = 2) out vec4 model_0;
layout(location = 3) out vec4 model_1;
layout(location = 4) out vec4 model_2;
layout(location = 5) out vec4 model_3;

#include "cad_common.glsl"

void main() {
  int p = gl_VertexIndex >> 2;
  int c = gl_VertexIndex & 3;
  vec4 r = patches[p].uv;
  corner_uv = mix(r.xy, r.zw, vec2(float(c & 1), float(c >> 1)));
  patch_index = p;
  model_0 = in_model_0;
  model_1 = in_model_1;
  model_2 = in_model_2;
  model_3 = in_model_3;
  gl_Position = vec4(0.0);
}

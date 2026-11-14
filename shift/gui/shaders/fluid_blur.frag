#version 450
#extension GL_GOOGLE_include_directive : require

// pass 2 of the soil surface: the depth of the spheres (fluid_depth.frag)
// smoothed over a radius that is fixed in the world, by levels of an
// a-trous filter: 5 x 5 samples, stride pixels apart, weighted by the B3
// spline, the stride doubling from level to level (as in
// svgf_atrous.comp), so that each level samples a depth already smoothed
// at the scale of its gaps, then fine again (kFluidBlurStrides in
// viewport.cc). A pixel takes the levels that reach no further than its
// radius, fading in the last, and keeps its depth at the others. The
// filter is bilateral: a sample whose depth differs from the pixel's own by
// more than a slant explains at its distance is weighted down by a
// Gaussian of the excess, so that a near surface is not blended with one
// far behind it, while a slanted surface, its samples on both sides, keeps
// its slope. The weights change smoothly with the depths, which keeps
// neighbouring pixels alike. Pixels without depth (0) stay empty and are
// left out.

layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 out_depth;

layout(binding = 0) uniform sampler2D depth_in;
#define FLUID_BINDING 1
#include "fluid_common.glsl"

layout(std140, binding = 2) uniform Level {
  vec4 level; // x: the stride of the samples (pixels)
} level;

// the change in depth per pixel along a surface seen this slanted to the
// line of sight, in the size of a pixel at that depth (1 / tan 18 deg)
const float kSlant = 3.0;
const float kSpline[3] = float[](3.0 / 8.0, 1.0 / 4.0, 1.0 / 16.0);

void main() {
  ivec2 p = ivec2(gl_FragCoord.xy);
  float d0 = texelFetch(depth_in, p, 0).r;
  float stride = level.level.x;
  float radius = min(fluid.filter_params.x * fluid.particles.y / d0,
                     fluid.filter_params.z);
  // 0 for a level that reaches as far as the radius, 1 for one that reaches
  // half as far
  float fade = clamp(radius / (2.0 * stride) - 1.0, 0.0, 1.0);
  if(d0 <= 0.0 || fade <= 0.0) {
    out_depth = vec4(d0, 0.0, 0.0, 0.0);
    return;
  }
  float range = 1.0 / (2.0 * fluid.filter_params.y * fluid.filter_params.y);
  float slant = kSlant * d0 / fluid.particles.y;
  ivec2 last = textureSize(depth_in, 0) - 1;
  int step = int(stride);
  float sum = 0.0;
  float weight = 0.0;
  for(int j = -2; j <= 2; j++) {
    for(int i = -2; i <= 2; i++) {
      float d = texelFetch(depth_in, clamp(p + ivec2(i, j) * step, ivec2(0),
                                           last), 0).r;
      if(d <= 0.0) {
        continue;
      }
      float excess =
        max(abs(d - d0) - slant * stride * length(vec2(i, j)), 0.0);
      float w = kSpline[abs(i)] * kSpline[abs(j)] *
                exp(-excess * excess * range);
      sum += w * d;
      weight += w;
    }
  }
  out_depth = vec4(mix(d0, sum / weight, fade), 0.0, 0.0, 0.0);
}

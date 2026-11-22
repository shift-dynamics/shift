// tone mapping of linear scene colors to display colors in [0, 1] (before
// gamma), shared by every pass that writes the image: sky.frag, floor.frag,
// mesh.frag (and cad.frag, fluid.frag), svgf_taa.comp and dlss_prepare.comp.
//
// Two curves (Viewport::ToneCurve): Reinhard's x / (1 + x), the default, and
// the ACES filmic curve, as fitted by S. Hill (the reference rendering and
// output transforms, sRGB in and out): a toe that deepens the shadows, a
// shoulder that rolls the highlights off without clipping, and the
// desaturation of very bright colors toward white, as film does.

// sRGB to the ACES working space (with the RRT's saturation), column major
const mat3 kAcesInput = mat3(0.59719, 0.07600, 0.02840,
                             0.35458, 0.90834, 0.13383,
                             0.04823, 0.01566, 0.83777);
// back to sRGB (with the ODT's saturation), column major
const mat3 kAcesOutput = mat3(1.60475, -0.10208, -0.00327,
                              -0.53108, 1.10813, -0.07276,
                              -0.07367, -0.00605, 1.07602);
// the scene scale at 0 stops, so that a mid gray (0.18) shows about as it
// did with the Reinhard curve x / (1 + x) this replaced
const float kAcesGain = 1.4;

// back from x / (1 + x), the space the ray traced image is composited and
// temporally anti-aliased in (svgf_composite.comp), to linear radiance; the
// filmic curve is applied after the anti-aliasing, since the anti-aliasing's
// clamps let more noise through where the curve is steeper (the mid tones)
vec3 FromReinhard(vec3 c) {
  c = clamp(c, vec3(0.0), vec3(0.999));
  return c / (vec3(1.0) - c);
}

vec3 AcesFit(vec3 v) {
  vec3 a = v * (v + 0.0245786) - 0.000090537;
  vec3 b = v * (0.983729 * v + 0.4329510) + 0.238081;
  return a / b;
}

// color: linear scene radiance; exposure: in stops (Viewport::set_exposure);
// curve: 0 Reinhard, 1 ACES (Viewport::set_tone_curve). The raster passes
// pass frame.sky_zenith.w and frame.sky_horizon.w, the ray traced ones
// u.tone.x and u.tone.y
vec3 ToneMap(vec3 color, float exposure, float curve) {
  vec3 x = max(color, vec3(0.0)) * exp2(exposure);
  if(curve < 0.5) {
    return x / (x + vec3(1.0));
  }
  vec3 v = kAcesInput * (x * kAcesGain);
  return clamp(kAcesOutput * AcesFit(v), 0.0, 1.0);
}

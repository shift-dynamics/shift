// the sky in the equirectangular projection (see environment.h): column u
// from azimuth -pi to pi about the vertical, from +x toward +y; row v from
// the zenith (-z, v = 0) to the nadir. Keep in sync with EnvironmentTables.

const float kEnvironmentPi = 3.14159265358979;
const float kSpecularLevels = 8.0;  // EnvironmentTables::kRoughnessCount
const float kSpecularHeight = 32.0; // EnvironmentTables::kSpecularHeight

vec2 EquirectUv(vec3 d) {
  return vec2(atan(d.y, d.x) / (2.0 * kEnvironmentPi) + 0.5,
              0.5 - asin(clamp(-d.z, -1.0, 1.0)) / kEnvironmentPi);
}

#ifdef ENVIRONMENT_TABLES
// the prefiltered radiance toward r for a roughness, between the two
// nearest levels of the stacked table, each sampled inside its own rows
vec3 SpecularLight(sampler2D table, vec3 r, float roughness) {
  vec2 uv = EquirectUv(r);
  float level = clamp(roughness, 0.0, 1.0) * (kSpecularLevels - 1.0);
  float l0 = floor(level);
  float l1 = min(l0 + 1.0, kSpecularLevels - 1.0);
  float row = clamp(uv.y * kSpecularHeight, 0.5, kSpecularHeight - 0.5);
  float rows = kSpecularLevels * kSpecularHeight;
  vec3 a = texture(table, vec2(uv.x, (l0 * kSpecularHeight + row) / rows)).rgb;
  vec3 b = texture(table, vec2(uv.x, (l1 * kSpecularHeight + row) / rows)).rgb;
  return mix(a, b, level - l0);
}

vec3 DiffuseLight(sampler2D table, vec3 n) {
  return texture(table, EquirectUv(n)).rgb;
}
#endif

// the fraction of the light from a surface at p that reaches the camera at
// c through the fog: params = density at height, height falloff (1/m,
// upward), height (m, upward) and 1 if there is fog. Heights are -z.
float FogTransmittance(vec3 c, vec3 p, vec4 params) {
  if(params.w < 0.5) {
    return 1.0;
  }
  float distance = length(p - c);
  float hc = -c.z;
  float dh = -p.z - hc;
  float density = params.x * exp(-params.y * (hc - params.z));
  // the mean of exp(-a h) along the ray, from its value at the camera
  float x = params.y * dh;
  float mean = abs(x) > 1e-4 ? (1.0 - exp(-x)) / x : 1.0 - 0.5 * x;
  return exp(-density * distance * mean);
}

// the optical depth along the segment from c to p of a medium whose density
// falls off as exp(-h / scale) with the height h (m, upward, -z) from
// density at h = 0
float ExponentialDepth(vec3 c, vec3 p, float density, float scale) {
  float hc = -c.z;
  float x = (-p.z - hc) / scale;
  // the mean of exp(-h / scale) along the segment, from its value at c
  float mean = abs(x) > 1e-4 ? (1.0 - exp(-x)) / x : 1.0 - 0.5 * x;
  return density * exp(-hc / scale) * length(p - c) * mean;
}

// the fraction of the light from p, in red, green and blue, that reaches c
// through the clear air of the physical sky: Rayleigh scattering by the air
// and scattering by the aerosols of its haze, of the optical depths overhead
// of the sky and the sun (shift::gui::SunlightColor, at 680, 550 and 440
// nm; Preetham, Shirley and Smits, 1999), the air's thinning with height
// over 8 km and the aerosols' over 1.2 km. beta is Angstrom's coefficient
// of the haze (shift::gui::AngstromBeta), negative for no air.
const float kAerosolScale = 1200.0;
const vec3 kAerosolDepth = vec3(1.651, 2.175, 2.910); // l^-1.3, l in um

vec3 AirTransmittance(vec3 c, vec3 p, float beta) {
  if(beta < 0.0) {
    return vec3(1.0);
  }
  const float kRayleighScale = 8000.0;
  const vec3 kRayleighDepth = vec3(0.04108, 0.0973, 0.2428);
  float rayleigh = ExponentialDepth(c, p, 1.0 / kRayleighScale,
                                    kRayleighScale);
  float aerosol = ExponentialDepth(c, p, beta / kAerosolScale, kAerosolScale);
  return exp(-(kRayleighDepth * rayleigh + kAerosolDepth * aerosol));
}

// the optical depth of the aerosols of the haze (as in AirTransmittance) at
// 550 nm along the segment from c to p; 0 for no air (beta < 0)
float AerosolDepth(vec3 c, vec3 p, float beta) {
  if(beta < 0.0) {
    return 0.0;
  }
  return kAerosolDepth.g *
         ExponentialDepth(c, p, beta / kAerosolScale, kAerosolScale);
}

// the direction of the horizon under the direction d (just above it); the
// sky there is the light that a long level path through the air scatters
// toward its end
vec3 HorizonDirection(vec3 d) {
  float horizontal = length(d.xy);
  return horizontal > 1e-4 ? normalize(vec3(d.xy / horizontal, -0.02))
                           : vec3(1.0, 0.0, -0.02);
}

// the fraction of the light from p that reaches the camera at c through the
// air and the fog, and haze, the light they scatter toward the camera on
// the way: the air's that of the sky at the horizon (horizon, the sky
// toward HorizonDirection), the fog's its color. fog_color.w is the air's
// beta (AirTransmittance).
vec3 Haze(vec3 c, vec3 p, vec4 fog_params, vec4 fog_color, vec3 horizon,
          out vec3 haze) {
  vec3 air = AirTransmittance(c, p, fog_color.w);
  float fog = FogTransmittance(c, p, fog_params);
  haze = fog * horizon * (1.0 - air) + (1.0 - fog) * fog_color.rgb;
  return fog * air;
}

// the light of a surface at p, color, as the camera at c sees it through
// the air and the fog (Haze)
vec3 Aerial(vec3 c, vec3 p, vec3 color, vec4 fog_params, vec4 fog_color,
            vec3 horizon) {
  vec3 haze;
  vec3 clear = Haze(c, p, fog_params, fog_color, horizon, haze);
  return color * clear + haze;
}

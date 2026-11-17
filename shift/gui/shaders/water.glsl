// a body of water (shift::core::environment::Water): a level surface with
// waves, shared by sky.frag, mesh.frag, floor.frag and raytrace.comp.
// Directions are in shift's world frame, +z down. The including shader
// provides WATER_PARAMS, WATER_WAVE, WATER_COLOR, WATER_EXTINCTION (vec4
// uniforms, see FrameUniforms in viewport.cc), the sun's SUN_DIRECTION and
// SUN_COLOR, SKY_ZENITH, and WaterSky(d), the radiance of the sky toward d
// without the sun's disk, and ValueNoise, Fbm and CellRandom
// (surfaces.glsl).

const float kWaterPi = 3.14159265358979;

// whether the model gives water
bool HasWater() {
  return WATER_PARAMS.w > 0.5;
}

// z of the surface (down)
float WaterZ() {
  return -WATER_PARAMS.x;
}

// the point where the segment from the camera c to p crosses the surface
// downward: only when c is above the water and p below it
bool WaterCrossing(vec3 c, vec3 p, out vec3 crossing) {
  crossing = p;
  float z = WaterZ();
  if(!HasWater() || c.z >= z || p.z <= z) {
    return false;
  }
  crossing = mix(c, p, (z - c.z) / (p.z - c.z));
  return true;
}

// the waves (shift::gui::OceanWaves): a layer per tile, each a grid of
// kWaterSize points of the height (m, up), the slopes along x and y and the
// sum of their squares, with levels of detail of their means
layout(binding = 20) uniform sampler2DArray water_waves;
const int kWaterTiles = 3;
const float kWaterSize = 256.0; // OceanWaves::kSize

// the side of tile c (m), as OceanWaves::TileSize: six peak wavelengths,
// 1.6 m for the ripples (or a sixteenth of the first), and their geometric
// mean
float WaterTile(int c) {
  float first = 6.0 * WATER_PARAMS.z;
  float last = min(1.6, first / 16.0);
  return c == 0 ? first : c == kWaterTiles - 1 ? last : sqrt(first * last);
}

// the waves at xy: the height of the surface above its level (m, up) and
// its slope (the gradient of the height), the sum of the tiles. Each tile
// is read at the level of detail of the footprint, the size of a pixel on
// the surface, longer along the view (along, a horizontal unit vector) than
// across it where the view grazes the water: a few points along the view,
// each read at the level of detail of the footprint across it, or of a
// share of the footprint along it (Olano and Baker, LEAN mapping, 2010).
// lost is the spread (variance) of the slopes within the footprint, from
// the mean square of the slopes less the square of their mean. Without
// ripples (for the ray tracer's march), the last tile, whose waves are a
// few centimeters high at most, is left out, its slopes all lost. shoal
// (WaterShoal) scales the waves of all but the ripples, broken in the
// shallows.
float WaterWaves(vec2 xy, vec2 along, float footprint_along,
                 float footprint_across, bool with_ripples, float shoal,
                 out vec2 slope, out float lost) {
  float taps = clamp(ceil(footprint_along / max(footprint_across, 1e-6)),
                     1.0, 4.0);
  float footprint = max(footprint_across, footprint_along / taps);
  float height = 0.0;
  slope = vec2(0.0);
  lost = 0.0;
  int tiles = with_ripples ? kWaterTiles : kWaterTiles - 1;
  for(int c = 0; c < tiles; c++) {
    float tile = WaterTile(c);
    float lod = log2(max(footprint * kWaterSize / tile, 1.0));
    vec4 sum = vec4(0.0);
    for(float t = 0.0; t < taps; t += 1.0) {
      vec2 p = xy + ((t + 0.5) / taps - 0.5) * footprint_along * along;
      sum += textureLod(water_waves,
                        vec3(p / tile + 0.5 / kWaterSize, float(c)), lod);
    }
    sum /= taps;
    float s = c < kWaterTiles - 1 ? shoal : 1.0;
    height += s * sum.x;
    slope += s * sum.yz;
    lost += s * s * max(sum.w - dot(sum.yz, sum.yz), 0.0);
  }
  if(!with_ripples) {
    lost += textureLod(water_waves, vec3(0.5, 0.5, float(kWaterTiles - 1)),
                       log2(kWaterSize)).w;
  }
  return height;
}

// the greatest height of the waves above the level of the water (m)
float WaterCrest() {
  return WATER_WAVE.w;
}

// the share of the waves left in water depth (m) deep: where the water is
// shallower than they are high they have broken, a wave at most 0.78 of
// the depth high (McCowan, 1894), so its crest and trough at most 0.39 of
// the depth from the level; what is left of them runs up the beach as the
// swash (WaterSwash). The ripples are left as they are.
float WaterShoal(float depth) {
  return clamp(0.39 * depth / max(WaterCrest(), 1e-3), 0.0, 1.0);
}

// the upward normal of the surface at xy, seen by the camera at c (+z
// down), with footprint the size of a pixel across the view there, over
// the bottom at depth (m, of the level water); roughness is the spread of
// the slopes too short to draw, for the glint and the reflection
vec3 WaterNormal(vec2 xy, vec3 c, float footprint, float depth,
                 out float roughness) {
  vec3 to_camera = vec3(c.xy - xy, c.z - WaterZ());
  float horizontal = length(to_camera.xy);
  vec2 along = horizontal > 1e-6 ? to_camera.xy / horizontal : vec2(1.0, 0.0);
  float grazing = max(-to_camera.z / max(length(to_camera), 1e-6), 0.02);
  vec2 slope;
  float lost;
  WaterWaves(xy, along, footprint / grazing, footprint, true,
             WaterShoal(depth), slope, lost);
  roughness = sqrt(0.002 + lost);
  return normalize(vec3(-slope, -1.0));
}

// the facet that the camera sees in the direction of n, toward v: a facet
// whose reflection points below the horizon faces away from the camera, and
// is hidden behind the crest in front of it, whose face toward the camera
// is seen in its place (its slope along the view mirrored)
vec3 WaterVisibleNormal(vec3 n, vec3 v) {
  if(reflect(-v, n).z < -0.02) {
    return n;
  }
  float horizontal = length(v.xy);
  if(horizontal < 1e-6) {
    return n;
  }
  vec2 along = v.xy / horizontal;
  vec2 slope = -n.xy / -n.z;
  slope -= 2.0 * dot(slope, along) * along;
  return normalize(vec3(-slope, -1.0));
}

// a standard normal pair from two uniform numbers in [0, 1)
vec2 WaterGaussian(vec2 xi) {
  float r = sqrt(-2.0 * log(max(1.0 - xi.x, 1e-7)));
  float a = 2.0 * kWaterPi * xi.y;
  return r * vec2(cos(a), sin(a));
}

// Schlick's approximation of the reflectance of water (n = 1.33)
float WaterFresnel(float cosine) {
  return 0.02 + 0.98 * pow(1.0 - clamp(cosine, 0.0, 1.0), 5.0);
}

// the period of the waves of the peak wavelength (s), from the dispersion
// of deep water waves, w^2 = g k
float WaterPeriod() {
  return sqrt(2.0 * kWaterPi * WATER_PARAMS.z / 9.81);
}

// the depth (m) at which the waves break, their height over the breaker
// index 0.78 (McCowan, 1894)
float WaterBreakingDepth() {
  return max(WATER_PARAMS.y, 1e-3) / 0.78;
}

// the time (s) a broken wave takes from depth d (m) to the shore: a bore
// moving at the speed of shallow water waves, sqrt(g d), over the
// equilibrium profile of a sandy beach, depth A x^(2/3) at a distance x from
// the shore with A = 0.1 m^(1/3) for fine sand (Dean, 1977), so 1.5 d /
// (A^(3/2) sqrt(g)); the waves reach the shore at different times along it
float WaterTravelTime(vec2 xy, float d) {
  const float a = 0.1;
  float wander = WaterPeriod() * (0.8 * ValueNoise(xy / 13.0, 73u) +
                                  0.25 * ValueNoise(xy / 3.5, 74u));
  return 1.5 * d / (a * sqrt(a) * sqrt(9.81)) + wander;
}

// the swash at xy: the depth (m, of the level water) at the water's edge.
// Each bore that reaches the shore runs up the beach as a sheet of water
// that gravity slows, stops and brings back down, the edge rising and
// falling as a thrown body does, once a wave period. The vertical reach of
// the swash is that of the long waves on a gently sloping beach,
// 0.06 sqrt(H L) for the wave height H and wavelength L (Stockdon et al.,
// 2006), at most 0.4 of the breaking depth, varying from wave to wave in
// groups. uprush is 1 while the water runs up, falling to 0 as it runs back.
float WaterSwash(vec2 xy, out float uprush) {
  float period = WaterPeriod();
  float e = fract((WATER_WAVE.z + WaterTravelTime(xy, 0.0)) / period);
  float reach = min(0.06 * sqrt(max(WATER_PARAMS.y, 1e-3) * WATER_PARAMS.z),
                    0.4 * WaterBreakingDepth()) *
                (0.5 + 0.8 * ValueNoise(vec2(dot(xy, vec2(0.04, 0.03)),
                                             WATER_WAVE.z / (6.0 * period)),
                                        75u));
  uprush = 1.0 - smoothstep(0.35, 0.6, e);
  return reach * (1.0 - 4.0 * e * (1.0 - e));
}

// the share of the level water at xy, depth (m) above the bottom, that is
// there: none above the edge of the swash, the water thickening from it
// over a centimeter and a half of depth, so that the water thins out over
// the sand
float WaterSheet(vec2 xy, float depth) {
  float uprush;
  float edge = WaterSwash(xy, uprush);
  return smoothstep(edge, edge + 0.015, depth);
}

// a lace of thin lines, from 0 to 1, as the foam of bubbles left on the
// water: the ridges of noise about scale (m) across, stretched along the
// crests (across d, the direction of travel), drifting with time
float WaterLace(vec2 xy, vec2 d, float scale, float width, uint seed) {
  vec2 q = vec2(dot(xy, d), 0.5 * dot(xy, vec2(-d.y, d.x))) / scale;
  float ridge = abs(Fbm(q, seed) - 0.5);
  return 1.0 - smoothstep(0.0, width, ridge);
}

// the share of the surface covered by foam at xy, depth (m) above the
// bottom: the whitewater of the waves broken in the surf zone, shallower
// than the breaking depth, in bores that run toward the shore once a wave
// period at the speed of shallow water waves (WaterTravelTime), each
// foaming at its front and leaving streaks and a lace of foam behind it,
// weakening as it runs into the shallows; the line of foam at the front of
// the swash as it runs up the beach, left as a lace as the water runs back.
// Its time is that of the waves.
float WaterFoam(vec2 xy, float depth) {
  float time = WATER_WAVE.z;
  float period = WaterPeriod();
  float breaking = WaterBreakingDepth();
  vec2 d = WATER_WAVE.xy;
  float uprush;
  float edge = WaterSwash(xy, uprush);
  // the waves break about the breaking depth; the bores fade into the
  // shallows, short of the swash
  float surf = smoothstep(1.15 * breaking, 0.85 * breaking, depth) *
               smoothstep(edge + 0.02, edge + 0.3 * breaking, depth);
  // where in its period the bore at this depth is: 0 at its front
  float b = fract((time + WaterTravelTime(xy, depth)) / period);
  float front = smoothstep(0.0, 0.02, b) * (1.0 - smoothstep(0.04, 0.14, b));
  float trail = smoothstep(0.0, 0.02, b) * (1.0 - smoothstep(0.04, 0.35, b));
  // the front broken into patches along the crest, the trail into streaks
  vec2 drift = -d * 0.5 * time;
  float patches = Fbm((xy + drift) * vec2(0.9, 0.9), 67u);
  float streaks = WaterLace(xy + drift, d, 1.6, 0.09, 69u);
  float bore = surf * max(front * smoothstep(0.25, 0.5, patches),
                          0.8 * trail * streaks);
  // a faint lace over the surf zone between the bores
  float lace = 0.35 * surf * WaterLace(xy + 0.5 * drift, d, 0.8, 0.05, 81u);
  // the swash's front: a line of foam a few centimeters of depth wide as
  // it runs up, left behind as a lace as it runs back
  float swash = (1.0 - smoothstep(edge + 0.002, edge + 0.012, depth)) *
                (0.25 + 0.75 * uprush) *
                mix(WaterLace(xy, d, 0.5, 0.12, 71u), 1.0, 0.6 * uprush);
  // the edge of the swash itself, fading over a few millimeters of depth
  swash *= smoothstep(edge - 0.001, edge + 0.002, depth);
  // the foam is a lace of bubbles, not a sheet: holes of open water in it
  float holes = smoothstep(0.25, 0.6, Fbm(xy * 4.0 + vec2(0.2 * time), 83u));
  return clamp(max(max(bore, lace), swash) * (0.5 + 0.5 * holes), 0.0, 1.0);
}

// the light foam of normal n at xy scatters toward the camera: a diffuse
// surface of bubbles lit by the sun and the sky, its brightness mottled,
// thin foam taking a little of the water's color from below
vec3 WaterFoamLight(vec3 n, vec2 xy, float foam) {
  float sun = max(dot(n, SUN_DIRECTION.xyz), 0.0);
  float sky = 0.6 + 0.4 * max(-n.z, 0.0);
  float mottle = 0.78 + 0.17 * Fbm(xy * 3.0, 79u);
  vec3 albedo = mottle * mix(vec3(0.7, 0.82, 0.85), vec3(1.0),
                             smoothstep(0.2, 0.9, foam));
  return albedo * (SUN_COLOR.rgb * SUN_DIRECTION.w * sun / kWaterPi +
                   sky * SKY_ZENITH.rgb);
}

// light the water scatters back toward the surface: its color lit as a
// diffuse surface by the sun and the sky
vec3 WaterInscatter() {
  float sun = max(-SUN_DIRECTION.z, 0.0);
  return WATER_COLOR.rgb * (SUN_COLOR.rgb * SUN_DIRECTION.w * sun / kWaterPi +
                            SKY_ZENITH.rgb);
}

// the sun reflected by the waves: GGX of the roughness of the waves too
// small to draw, with the facets hidden from the sun or the camera by the
// waves in front of them (Smith's masking and shadowing, the form
// correlated in height of Heitz, Understanding the Masking-Shadowing
// Function in Microfacet-Based BRDFs, 2014). At a grazing view nearly all
// the facets that would reflect the sun are hidden: without it the glint
// grows without bound toward the horizon.
vec3 WaterGlint(vec3 n, vec3 v, float roughness) {
  n = WaterVisibleNormal(n, v);
  vec3 l = SUN_DIRECTION.xyz;
  float n_dot_l = dot(n, l);
  float n_dot_v = max(dot(n, v), 1e-4);
  if(n_dot_l <= 0.0 || SUN_DIRECTION.w <= 0.0) {
    return vec3(0.0);
  }
  vec3 h = normalize(v + l);
  float a2 = roughness * roughness;
  float n_dot_h = max(dot(n, h), 0.0);
  float d = n_dot_h * n_dot_h * (a2 - 1.0) + 1.0;
  float distribution = a2 / (kWaterPi * d * d);
  float f = WaterFresnel(max(dot(h, v), 0.0));
  // G / (4 n.l n.v)
  float visibility =
    0.5 / (n_dot_l * sqrt(n_dot_v * n_dot_v * (1.0 - a2) + a2) +
           n_dot_v * sqrt(n_dot_l * n_dot_l * (1.0 - a2) + a2));
  return SUN_COLOR.rgb * SUN_DIRECTION.w * distribution * f * visibility *
         n_dot_l;
}

// the sky reflected by the surface at a point of normal n toward v, for
// the slope of the facet moved by the standard normal pair g times the
// spread of the slopes too short to draw (from roughness): seen in its
// place, the face of the wave in front when it faces away from the camera
vec3 WaterSkyReflection(vec3 n, vec3 v, float roughness, vec2 g) {
  float sigma = sqrt(max(roughness * roughness - 0.002, 0.0) * 0.5);
  vec2 slope = -n.xy / -n.z + sigma * g;
  vec3 m = WaterVisibleNormal(normalize(vec3(-slope, -1.0)), v);
  vec3 r = reflect(-v, m);
  r.z = min(r.z, -1e-3);
  return WaterSky(normalize(r)) * WaterFresnel(max(dot(m, v), 0.0));
}

// light reflected by the surface at a point of normal n toward v: the sky
// over the slopes too short to draw (Bruneton, Neyret and Holzschuch, 2010,
// sampled at four slopes of their spread, turned and scaled at random by
// xi, uniform in [0, 1)^2, for the ray tracer when random, so that its
// frames average them), never below the horizon, and the sun
vec3 WaterReflection(vec3 n, vec3 v, float roughness, bool random, vec2 xi) {
  vec3 sky = vec3(0.0);
  // the four points of a square at a spread of one standard deviation
  // along each axis; for the ray tracer, the square turned and scaled at
  // random each frame (a sample of the normal distribution, with the
  // three others at right angles to it about the middle)
  vec2 g[4] = vec2[4](vec2(1.0, 1.0), vec2(-1.0, 1.0), vec2(1.0, -1.0),
                      vec2(-1.0, -1.0));
  if(random) {
    vec2 first = WaterGaussian(xi);
    g[0] = first;
    g[1] = vec2(-first.y, first.x);
    g[2] = -first;
    g[3] = vec2(first.y, -first.x);
  }
  for(int i = 0; i < 4; i++) {
    sky += 0.25 * WaterSkyReflection(n, v, roughness, g[i]);
  }
  return sky + WaterGlint(n, v, roughness);
}

// what the camera at c sees of the surface at crossing, with color, the
// light leaving the point p below it toward the camera: reflected light, and
// the light of p passed through the water and dimmed by it on its way down
// to p and back up, with the light the water scatters back on the way, the
// water thinning out over the sand at the edge of the swash, and the foam
// over it; a point p at infinity (deep is true) is the water's own color.
// footprint is the size of a pixel across the view at crossing.
vec3 WaterOver(vec3 c, vec3 crossing, vec3 p, vec3 color, bool deep,
               float footprint) {
  float roughness;
  vec3 n = WaterNormal(crossing.xy, c, footprint,
                       deep ? 1e6 : p.z - WaterZ(), roughness);
  vec3 v = normalize(c - crossing);
  float f = WaterFresnel(dot(WaterVisibleNormal(n, v), v));
  vec3 below = WaterInscatter();
  float foam = 0.0;
  float sheet = 1.0;
  if(!deep) {
    vec3 view = exp(-WATER_EXTINCTION.rgb * distance(crossing, p));
    vec3 down = exp(-WATER_EXTINCTION.rgb * (p.z - WaterZ()));
    below = color * view * down + below * (1.0 - view);
    foam = WaterFoam(crossing.xy, p.z - WaterZ());
    sheet = WaterSheet(crossing.xy, p.z - WaterZ());
  }
  vec3 water = WaterReflection(n, v, roughness, false, vec2(0.0)) +
               (1.0 - f) * below;
  return mix(mix(color, water, sheet), WaterFoamLight(n, crossing.xy, foam),
             foam);
}

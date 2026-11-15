// variation of the textures of terrain and the surfaces drawn over it in
// patches (shift::core::terrain::SurfaceTexture), shared by mesh.frag and
// raytrace.comp. The noise is repeated in scene.cc (SurfaceNoise), which
// finds the noise level of a surface's coverage: keep the two in sync.

// a random number from 0 to 1 for integer cell c, from an integer hash
float CellRandom(ivec2 c, uint seed) {
  uint h = uint(c.x) * 0x8da6b343u ^ uint(c.y) * 0xd8163841u ^
           seed * 0xcb1ab31fu;
  h ^= h >> 13;
  h *= 0x5bd1e995u;
  h ^= h >> 15;
  return float(h & 0xffffffu) / 16777215.0;
}

// smooth value noise, 0 to 1, with features about one unit across
float ValueNoise(vec2 x, uint seed) {
  vec2 i = floor(x);
  vec2 f = x - i;
  vec2 u = f * f * (3.0 - 2.0 * f);
  ivec2 c = ivec2(i);
  float a = CellRandom(c, seed);
  float b = CellRandom(c + ivec2(1, 0), seed);
  float d = CellRandom(c + ivec2(0, 1), seed);
  float e = CellRandom(c + ivec2(1, 1), seed);
  return mix(mix(a, b, u.x), mix(d, e, u.x), u.y);
}

// four octaves of value noise, 0 to 1, with the largest features about one
// unit across
float Fbm(vec2 x, uint seed) {
  float sum = 0.0;
  float amplitude = 0.5;
  for(int k = 0; k < 4; k++) {
    sum += amplitude * ValueNoise(x, seed + uint(k));
    x = x * 2.0 + vec2(17.31, 9.13);
    amplitude *= 0.5;
  }
  return sum / 0.9375;
}

// the shifts of the tiles of a texture around q (in tiles): patches of
// tiles a few across share a random shift; between patches, two shifts
// blend over fraction f (Quilez, "Texture Repetition", technique 3)
struct TileShift {
  vec2 a;
  vec2 b;
  float f;
  bool on;
};

TileShift TileVariation(vec2 q, bool on) {
  TileShift t;
  t.on = on;
  t.a = vec2(0.0);
  t.b = vec2(0.0);
  t.f = 0.0;
  if(on) {
    float k = ValueNoise(q * 1.3, 7u) * 8.0;
    float i = floor(k);
    t.f = k - i;
    t.a = vec2(CellRandom(ivec2(int(i), 0), 11u),
               CellRandom(ivec2(int(i), 1), 11u));
    t.b = vec2(CellRandom(ivec2(int(i) + 1, 0), 11u),
               CellRandom(ivec2(int(i) + 1, 1), 11u));
  }
  return t;
}

// the blend of the samples x (shift a) and y (shift b), sharpened by their
// difference so that the stones of one stand out of the other
vec4 BlendTiles(vec4 x, vec4 y, float f) {
  vec3 d = x.rgb - y.rgb;
  return mix(x, y, smoothstep(0.2, 0.8, f - 0.1 * (d.r + d.g + d.b)));
}

// brightness of terrain varied over tens of meters, about 0.82 to 1.18; p
// in meters along the ground
float MacroBrightness(vec2 p) {
  return mix(0.82, 1.18, Fbm(p / 35.0, 3u));
}

// how much a surface covers point p (meters along the ground): its noise
// over patches 1 / inverse_patch across, above its level, with soft edges
// shifted by the brightness of the texture below, so that bright stones
// stand out of the surface laid over them
float SurfaceMask(vec2 p, int surface, float inverse_patch, float level,
                  float below_brightness) {
  float n = Fbm(p * inverse_patch, 101u + 17u * uint(surface));
  return smoothstep(level - 0.012, level + 0.012,
                    n - 0.2 * (below_brightness - 0.5));
}

// Hex-tiling of a texture, so that a texture repeated over a large surface
// (a terrain) shows no grid: M. Mikkelsen, "Practical Real-Time
// Hex-Tiling", Journal of Computer Graphics Techniques 11(3), 2022, after
// E. Heitz and F. Neyret, "High-Performance By-Example Noise using a
// Histogram-Preserving Blending Operator", 2018.
//
// The plane is covered by hexagons. Each hexagon shows the texture moved by
// a random offset and turned by a random angle about its center. A point is
// in the triangle of the centers of three hexagons, and blends their three
// lookups with its barycentric weights raised to a high power, so that each
// hexagon is the texture itself over most of its area. The weights favor the
// brighter lookup (the contrast of the texture is kept where they meet,
// instead of washing out to its mean). A normal map is turned back by each
// hexagon's angle.
//
// The includer defines HexLookup(layer, st, dx, dy): the texture layer at st
// with the gradients dx, dy of st across a pixel.

const float kHexRotation = 1.0;   // of the random angles, 0 for none
const float kHexSharpness = 7.0;  // power of the barycentric weights
const float kHexContrast = 0.6;   // share of the weights from brightness

// three hexagons over a point: their texture coordinates and the rotations
// (cosine, sine) from the surface's coordinates to them, and the weights
struct HexTile {
  vec2 st[3];
  vec2 rotation[3];
  vec3 weights;
};

vec2 HexHash(ivec2 v) {
  vec2 p = vec2(v);
  vec2 r = vec2(127.1 * p.x + 311.7 * p.y, 269.5 * p.x + 183.3 * p.y);
  return fract(sin(r) * 43758.5453);
}

vec2 HexRotate(vec2 rotation, vec2 v) {
  return vec2(rotation.x * v.x - rotation.y * v.y,
              rotation.y * v.x + rotation.x * v.y);
}

// v turned back, by the inverse of rotation
vec2 HexRotateBack(vec2 rotation, vec2 v) {
  return vec2(rotation.x * v.x + rotation.y * v.y,
              -rotation.y * v.x + rotation.x * v.y);
}

HexTile HexSetup(vec2 st) {
  // the triangle grid of the hexagons' centers, skewed to a square grid
  const float scale = 2.0 * 1.7320508;
  vec2 g = st * scale;
  vec2 skewed = vec2(g.x, -0.57735027 * g.x + 1.15470054 * g.y);
  ivec2 base = ivec2(floor(skewed));
  vec2 f = fract(skewed);
  float z = 1.0 - f.x - f.y;
  float s = z < 0.0 ? 1.0 : 0.0;
  float s2 = 2.0 * s - 1.0;
  vec3 w = vec3(-z * s2, s - f.y * s2, s - f.x * s2);
  ivec2 is = ivec2(int(s));
  ivec2 vertex[3];
  vertex[0] = base + is;
  vertex[1] = base + ivec2(is.x, 1 - is.y);
  vertex[2] = base + ivec2(1 - is.x, is.y);
  HexTile h;
  for(int k = 0; k < 3; k++) {
    vec2 v = vec2(vertex[k]);
    // the hexagon's center, back from the skewed grid
    vec2 center = vec2(v.x, 0.5 * v.x + 0.8660254 * v.y) / scale;
    float angle = abs(v.x * v.y) + abs(v.x + v.y) + 3.14159265;
    angle = mod(angle, 6.2831853);
    angle = (angle > 3.14159265 ? angle - 6.2831853 : angle) * kHexRotation;
    h.rotation[k] = vec2(cos(angle), sin(angle));
    h.st[k] = HexRotate(h.rotation[k], st - center) + center +
              HexHash(vertex[k]);
  }
  w = pow(max(w, vec3(0.0)), vec3(kHexSharpness));
  h.weights = w / max(w.x + w.y + w.z, 1e-20);
  return h;
}

vec4 HexLookupTurned(HexTile h, int k, int layer, vec2 dx, vec2 dy) {
  return HexLookup(layer, h.st[k], HexRotate(h.rotation[k], dx),
                   HexRotate(h.rotation[k], dy));
}

// the color of a layer, which sets the weights by its brightness
vec4 HexColor(inout HexTile h, int layer, vec2 dx, vec2 dy) {
  vec4 c0 = HexLookupTurned(h, 0, layer, dx, dy);
  vec4 c1 = HexLookupTurned(h, 1, layer, dx, dy);
  vec4 c2 = HexLookupTurned(h, 2, layer, dx, dy);
  const vec3 luminance = vec3(0.299, 0.587, 0.114);
  vec3 bright = vec3(dot(c0.rgb, luminance), dot(c1.rgb, luminance),
                     dot(c2.rgb, luminance));
  vec3 w = mix(vec3(1.0), bright, kHexContrast) * h.weights;
  h.weights = w / max(w.x + w.y + w.z, 1e-20);
  return c0 * h.weights.x + c1 * h.weights.y + c2 * h.weights.z;
}

vec4 HexSample(HexTile h, int layer, vec2 dx, vec2 dy) {
  return HexLookupTurned(h, 0, layer, dx, dy) * h.weights.x +
         HexLookupTurned(h, 1, layer, dx, dy) * h.weights.y +
         HexLookupTurned(h, 2, layer, dx, dy) * h.weights.z;
}

// a tangent space normal from a normal map, in [-1, 1], in the surface's
// coordinates
vec3 HexNormal(HexTile h, int layer, vec2 dx, vec2 dy) {
  vec3 n = vec3(0.0);
  for(int k = 0; k < 3; k++) {
    vec3 t = HexLookupTurned(h, k, layer, dx, dy).xyz * 2.0 - 1.0;
    n += vec3(HexRotateBack(h.rotation[k], t.xy), t.z) * h.weights[k];
  }
  return n;
}

// ---------------------------------------------------------------------------
// triplanar projection along the body axes (as SampleTriplanar), each plane
// hex-tiled; w the weights of the planes, dpdx and dpdy the gradients of p
// across a pixel (zero for an explicit level of detail)

struct HexPlanes {
  HexTile plane[3];
  vec2 dx[3];
  vec2 dy[3];
  vec3 w;
};

// planes of less weight than this are not looked up
const float kHexPlaneCutoff = 0.003;

HexPlanes HexPlanesSetup(vec3 p, vec3 w, vec3 dpdx, vec3 dpdy) {
  HexPlanes h;
  h.w = w;
  h.plane[0] = HexSetup(p.zy);
  h.plane[1] = HexSetup(p.xz);
  h.plane[2] = HexSetup(p.xy);
  h.dx[0] = dpdx.zy;
  h.dx[1] = dpdx.xz;
  h.dx[2] = dpdx.xy;
  h.dy[0] = dpdy.zy;
  h.dy[1] = dpdy.xz;
  h.dy[2] = dpdy.xy;
  return h;
}

vec4 HexPlanesColor(inout HexPlanes h, int layer) {
  vec4 c = vec4(0.0);
  for(int k = 0; k < 3; k++) {
    if(h.w[k] > kHexPlaneCutoff) {
      c += HexColor(h.plane[k], layer, h.dx[k], h.dy[k]) * h.w[k];
    }
  }
  return c / max(dot(step(vec3(kHexPlaneCutoff), h.w), h.w), 1e-20);
}

vec4 HexPlanesSample(HexPlanes h, int layer) {
  vec4 c = vec4(0.0);
  for(int k = 0; k < 3; k++) {
    if(h.w[k] > kHexPlaneCutoff) {
      c += HexSample(h.plane[k], layer, h.dx[k], h.dy[k]) * h.w[k];
    }
  }
  return c / max(dot(step(vec3(kHexPlaneCutoff), h.w), h.w), 1e-20);
}

// as TriplanarNormal: the whiteout blend of the planes' normals, n the body
// space surface normal
vec3 HexPlanesNormal(HexPlanes h, int layer, vec3 n, float scale) {
  vec3 t[3];
  for(int k = 0; k < 3; k++) {
    t[k] = h.w[k] > kHexPlaneCutoff
             ? HexNormal(h.plane[k], layer, h.dx[k], h.dy[k])
             : vec3(0.0, 0.0, 1.0);
    t[k].xy *= scale;
  }
  vec3 tx = vec3(t[0].xy + n.zy, abs(t[0].z) * n.x);
  vec3 ty = vec3(t[1].xy + n.xz, abs(t[1].z) * n.y);
  vec3 tz = vec3(t[2].xy + n.xy, abs(t[2].z) * n.z);
  return normalize(tx.zyx * h.w.x + ty.xzy * h.w.y + tz.xyz * h.w.z);
}

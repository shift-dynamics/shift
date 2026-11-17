// the surface materials of terrains by area (shift::core::terrain::
// MaterialMap), shared by mesh.frag and raytrace.comp; the layout is that of
// GroundBuffer in viewport.cc. Include after surfaces.glsl (Fbm) and define
// GROUND_BINDING.

const int kMaxGroundMaps = 8;
const int kMaxGroundMaterials = 64;

struct GroundMaterial {
  vec4 color;   // rgb: linear factor of the texture, w = roughness factor
  vec4 scale;   // x = 1 / texture scale, y = normal scale, z = height
                // scale (m, the depth of the height map's relief)
  ivec4 layers; // base color, normal, roughness (G), height (R); -1: none
};

struct GroundMap {
  mat4 world_to_map;
  vec4 scale;  // x, y = 1 / spacing of the samples, z = blend width (m)
  ivec4 size;  // x, y = samples along x, y; z = first index (byte) in
               // ground_indices, w = first material in ground_materials
};

layout(std430, binding = GROUND_BINDING) readonly buffer Ground {
  ivec4 ground_counts; // x = maps, y = materials
  GroundMap ground_maps[kMaxGroundMaps];
  GroundMaterial ground_materials[kMaxGroundMaterials];
  uint ground_indices[]; // index in the map's materials, four per word
};

// the material (in ground_materials) of sample (i, j) of map m, clamped to
// the map
int GroundSample(GroundMap m, int i, int j) {
  i = clamp(i, 0, m.size.x - 1);
  j = clamp(j, 0, m.size.y - 1);
  int byte = m.size.z + j * m.size.x + i;
  uint word = ground_indices[byte >> 2];
  return m.size.w + int((word >> (8u * uint(byte & 3))) & 0xffu);
}

// the materials at world position p of map index and their weights, at most
// four, summing to 1; returns how many. Each material's share of the nine
// nearest samples, weighted by the quadratic B-spline, has its contour at
// one half where the nearest sample changes, as the soils see it, though
// smoothly instead of in steps along the grid; noise moves it about. The
// shares are sharpened to blend over the blend width on either side of the
// contour (a softmax whose temperature makes the share of a pair of
// materials go from 0.12 to 0.88 over twice the blend width).
int GroundWeights(int index, vec3 p, out ivec4 material, out vec4 weight) {
  GroundMap m = ground_maps[index];
  vec2 xy = (m.world_to_map * vec4(p, 1.0)).xy;
  vec2 s = xy * m.scale.xy; // in samples
  vec2 jitter = vec2(Fbm(s * 0.3, 211u), Fbm(s * 0.3, 223u)) - 0.5;
  s += jitter * 2.0;
  vec2 center = floor(s + 0.5);
  vec2 t = s - center + 0.5; // 0 to 1
  vec2 w0 = 0.5 * (1.0 - t) * (1.0 - t);
  vec2 w1 = 0.5 + t * (1.0 - t);
  vec2 w2 = 0.5 * t * t;
  ivec2 c = ivec2(center);
  int count = 0;
  material = ivec4(0);
  vec4 share = vec4(0.0);
  for(int dy = -1; dy <= 1; dy++) {
    float wy = dy < 0 ? w0.y : (dy == 0 ? w1.y : w2.y);
    for(int dx = -1; dx <= 1; dx++) {
      float wx = dx < 0 ? w0.x : (dx == 0 ? w1.x : w2.x);
      int k = GroundSample(m, c.x + dx, c.y + dy);
      int found = -1;
      for(int e = 0; e < count; e++) {
        if(material[e] == k) {
          found = e;
        }
      }
      if(found < 0 && count < 4) {
        found = count++;
        material[found] = k;
      }
      if(found >= 0) {
        share[found] += wx * wy;
      }
    }
  }
  // the share changes by about 1 per sample across a boundary
  float samples = m.scale.z * 0.5 * (m.scale.x + m.scale.y);
  float temperature = max(samples, 1e-3);
  float top = max(max(share.x, share.y), max(share.z, share.w));
  weight = vec4(0.0);
  for(int e = 0; e < count; e++) {
    weight[e] = exp((share[e] - top) / temperature);
  }
  weight /= weight.x + weight.y + weight.z + weight.w;
  // drop the materials that hardly show
  int kept = 0;
  ivec4 kept_material = ivec4(0);
  vec4 kept_weight = vec4(0.0);
  for(int e = 0; e < count; e++) {
    if(weight[e] > 2e-3) {
      kept_material[kept] = material[e];
      kept_weight[kept] = weight[e];
      kept++;
    }
  }
  material = kept_material;
  weight = kept_weight / (kept_weight.x + kept_weight.y + kept_weight.z +
                          kept_weight.w);
  return kept;
}

// the weights of materials where they meet, given their heights there (0
// to 1, of their height maps, 0.5 without): the higher shows, the lower
// only where its weight lifts it within a few hundredths of the higher, so
// that stones stand out of the sand instead of fading into it (height
// blending). Unchanged unless a material has a height map.
vec4 GroundHeightBlend(int count, ivec4 material, vec4 weight,
                       vec4 height) {
  bool any = false;
  for(int e = 0; e < count; e++) {
    any = any || ground_materials[material[e]].layers.w >= 0;
  }
  if(!any || count < 2) {
    return weight;
  }
  const float depth = 0.06;
  float top = -1e9;
  vec4 lifted = vec4(-1e9);
  for(int e = 0; e < count; e++) {
    // a weight near 0 sinks a material below any texture's highest point
    lifted[e] = height[e] + 1.2 * weight[e];
    top = max(top, lifted[e]);
  }
  vec4 blended = vec4(0.0);
  for(int e = 0; e < count; e++) {
    blended[e] = max(lifted[e] - top + depth, 0.0);
  }
  return blended / (blended.x + blended.y + blended.z + blended.w);
}

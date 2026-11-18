#version 450
#extension GL_GOOGLE_include_directive : require

#ifdef FLUID
// compiled as fluid.frag, over the whole view (fullscreen.vert): the soil
// surface of SPH soils at the smoothed depth of their particles
// (fluid_blur.frag), with the material of the particles; the surface's
// point and normal take the place of the inputs of mesh.vert
layout(location = 0) in vec2 screen_uv;
layout(binding = 10) uniform sampler2D fluid_depth;
#define FLUID_BINDING 11
#include "fluid_common.glsl"
vec3 world_position;
vec3 world_normal;
vec3 body_position;
vec3 body_normal;
vec2 uv;
mat3 body_rotation;
#else
layout(location = 0) in vec3 world_position;
layout(location = 1) in vec3 world_normal;
layout(location = 2) in vec3 body_position;
layout(location = 3) in vec3 body_normal;
layout(location = 4) in vec2 uv;
layout(location = 5) flat in mat3 body_rotation; // body to world
#endif

layout(location = 0) out vec4 out_color;

#ifdef CAD
// compiled as cad.frag for the faces of CAD geometry (cad.tese): uv are the
// parameters of the face's surface; fragments outside its trims are dropped
#define CAD_TRIM
#define CAD_TRIM_ONLY
#include "cad_common.glsl"
layout(location = 8) flat in ivec2 cad_face; // face, flags (1: test trims)
// a flexible body's stress drawn, mapped to [0, 1] (cad.tese)
layout(location = 9) in float cad_stress;
layout(std140, binding = 6) uniform Cad {
  vec4 tessellation;
  vec4 edges; // x = half width of edge lines (pixels), y = their darkening,
              // z = widening of the trims (pixels)
} cad;
#endif

layout(std140, binding = 0) uniform Frame {
  mat4 view_projection;
  mat4 inverse_view_projection;
  vec4 camera_position;
  vec4 sun_direction;
  vec4 sun_color;
  vec4 sky_zenith;
  vec4 sky_horizon;
  vec4 floor_params;
  mat4 floor_frame;
  vec4 fog_params;
  vec4 fog_color;
  vec4 water_params;     // height (up), wave height, wavelength, 1 if water
  vec4 water_wave;       // direction of travel (x, y), time
  vec4 water_color;
  vec4 water_extinction;
  // the colors of the stress drawn on flexible bodies, sRGB, at 16 even
  // steps from 0 to 1 (Viewport::set_color_map)
  vec4 color_map[16];
} frame;

layout(std140, binding = 1) uniform Material {
  vec4 albedo;        // w = normal map scale
  vec4 material;      // x = metallic, y = roughness, z = triplanar (2:
                      // hex-tiled too),
                      // w = 1 / texture scale
  ivec4 layers;       // base color, metallic roughness, normal; -1: none;
                      // w = holes used
  vec4 variation;     // x = 1: vary the tiles of triplanar textures, y =
                      // material map (ground.glsl) + 1, 0 for none, z = alpha
                      // cutoff, w = opacity: below 1, blended over what is
                      // behind (viewport.cc: transparent_pipeline_)
  ivec4 surface_layers; // base color and normal of surfaces 0 and 1
  vec4 surface[2];    // x = 1 / texture scale, y = normal scale, z = noise
                      // level, w = 1 / patch size
  vec4 emission;      // rgb: factor of the base color emitted (a lamp's
                      // lens, lit)
  vec4 holes[4];      // of a terrain, in its uv: x0, y0, x1, y1
  vec4 flex;          // x = 1: a flexible body's stress drawn, its color
                      // map at uv.x (a CAD's: at cad_stress)
} mesh;

// image based lighting of the sky, equirectangular; see environment.h
layout(binding = 2) uniform sampler2D environment_specular;   // per roughness
layout(binding = 3) uniform sampler2D environment_irradiance;
layout(binding = 4) uniform sampler2D environment_brdf;       // n.v, roughness
#define ENVIRONMENT_TABLES
#include "environment.glsl"
// the radiance of the sky, equirectangular, for the water's reflections
layout(binding = 18) uniform sampler2D sky_radiance;
#define WATER_SKY sky_radiance

// texture layers of all materials, and those larger than the others, from
// layer kLargeLayer on (scene.h: Scene::large_texture_layers)
layout(binding = 5) uniform sampler2DArray textures;
layout(binding = 21) uniform sampler2DArray large_textures;
const int kLargeLayer = 65536;

#define LIGHTS_BINDING 8
#define IES_BINDING 9
#include "lights.glsl"
#include "surfaces.glsl"
#define GROUND_BINDING 17
#include "ground.glsl"
#define WATER_PARAMS frame.water_params
#define WATER_WAVE frame.water_wave
#define WATER_COLOR frame.water_color
#define WATER_EXTINCTION frame.water_extinction
#define SUN_DIRECTION frame.sun_direction
#define SUN_COLOR frame.sun_color
#define SKY_ZENITH frame.sky_zenith
vec3 WaterSky(vec3 d) {
  return textureLod(WATER_SKY, EquirectUv(d), 0.0).rgb;
}
#include "water.glsl"

const float PI = 3.14159265358979;

float DistributionGgx(float n_dot_h, float roughness) {
  float a2 = roughness * roughness * roughness * roughness;
  float d = n_dot_h * n_dot_h * (a2 - 1.0) + 1.0;
  return a2 / (PI * d * d);
}

float GeometrySmith(float n_dot_v, float n_dot_l, float roughness) {
  float k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
  return n_dot_v / (n_dot_v * (1.0 - k) + k) *
         n_dot_l / (n_dot_l * (1.0 - k) + k);
}

// ---------------------------------------------------------------------------
// materials, keep in sync with raytrace.comp

vec3 SrgbToLinear(vec3 c) {
  return mix(c / 12.92, pow((c + 0.055) / 1.055, vec3(2.4)),
             step(vec3(0.04045), c));
}

vec4 SampleLayer(int layer, vec2 p) {
  if(layer >= kLargeLayer) {
    return texture(large_textures, vec3(p, float(layer - kLargeLayer)));
  }
  return texture(textures, vec3(p, float(layer)));
}

vec4 SampleLayerGrad(int layer, vec2 p, vec2 dx, vec2 dy) {
  if(layer >= kLargeLayer) {
    return textureGrad(large_textures, vec3(p, float(layer - kLargeLayer)),
                       dx, dy);
  }
  return textureGrad(textures, vec3(p, float(layer)), dx, dy);
}

// the level of detail of a layer at p
float LayerLod(int layer, vec2 p) {
  return layer >= kLargeLayer ? textureQueryLod(large_textures, p).y
                              : textureQueryLod(textures, p).y;
}

// one plane of a triplanar projection, q in texture tiles with its screen
// space derivatives (taken where control flow is uniform), its tiles shifted
// at random if vary
vec4 SamplePlane(int layer, vec2 q, vec2 dx, vec2 dy, bool vary) {
  if(!vary) {
    return SampleLayerGrad(layer, q, dx, dy);
  }
  TileShift t = TileVariation(q, true);
  return BlendTiles(SampleLayerGrad(layer, q + t.a, dx, dy),
                    SampleLayerGrad(layer, q + t.b, dx, dy), t.f);
}

// the weights of the three planes, without those too small to see
vec3 PlaneWeights(vec3 w) {
  w *= step(vec3(0.02), w);
  return w / max(w.x + w.y + w.z, 1e-6);
}

// triplanar projection along the body axes, p in texture tiles, dpx and dpy
// its screen space derivatives
vec4 SampleTriplanar(int layer, vec3 p, vec3 w, vec3 dpx, vec3 dpy,
                     bool vary) {
  w = PlaneWeights(w);
  vec4 s = vec4(0.0);
  if(w.x > 0.0) {
    s += SamplePlane(layer, p.zy, dpx.zy, dpy.zy, vary) * w.x;
  }
  if(w.y > 0.0) {
    s += SamplePlane(layer, p.xz, dpx.xz, dpy.xz, vary) * w.y;
  }
  if(w.z > 0.0) {
    s += SamplePlane(layer, p.xy, dpx.xy, dpy.xy, vary) * w.z;
  }
  return s;
}

vec4 HexLookup(int layer, vec2 st, vec2 dx, vec2 dy) {
  return textureGrad(textures, vec3(st, float(layer)), dx, dy);
}
#include "hex_tiling.glsl"

// triplanar normal mapping with the whiteout blend (B. Golus, "Normal
// Mapping for a Triplanar Shader"); n is the body space surface normal
vec3 TriplanarNormal(int layer, vec3 p, vec3 n, vec3 w, float scale,
                     vec3 dpx, vec3 dpy, bool vary) {
  w = PlaneWeights(w);
  vec3 sum = vec3(0.0);
  if(w.x > 0.0) {
    vec3 tx = SamplePlane(layer, p.zy, dpx.zy, dpy.zy, vary).xyz * 2.0 - 1.0;
    tx.xy *= scale;
    sum += vec3(tx.xy + n.zy, abs(tx.z) * n.x).zyx * w.x;
  }
  if(w.y > 0.0) {
    vec3 ty = SamplePlane(layer, p.xz, dpx.xz, dpy.xz, vary).xyz * 2.0 - 1.0;
    ty.xy *= scale;
    sum += vec3(ty.xy + n.xz, abs(ty.z) * n.y).xzy * w.y;
  }
  if(w.z > 0.0) {
    vec3 tz = SamplePlane(layer, p.xy, dpx.xy, dpy.xy, vary).xyz * 2.0 - 1.0;
    tz.xy *= scale;
    sum += vec3(tz.xy + n.xy, abs(tz.z) * n.z) * w.z;
  }
  return normalize(sum);
}

// tangent frame from screen space derivatives (C. Schueler, "Followup:
// Normal Mapping Without Precomputed Tangents")
mat3 CotangentFrame(vec3 n, vec3 p, vec2 t) {
  vec3 dp1 = dFdx(p);
  vec3 dp2 = dFdy(p);
  vec2 dt1 = dFdx(t);
  vec2 dt2 = dFdy(t);
  vec3 dp2_perp = cross(dp2, n);
  vec3 dp1_perp = cross(n, dp1);
  vec3 tangent = dp2_perp * dt1.x + dp1_perp * dt2.x;
  vec3 bitangent = dp2_perp * dt1.y + dp1_perp * dt2.y;
  float scale = inversesqrt(max(max(dot(tangent, tangent),
                                    dot(bitangent, bitangent)), 1e-20));
  return mat3(tangent * scale, bitangent * scale, n);
}

// light from direction l, with the colors of its diffuse and specular
// reflection (GGX, Smith-Schlick, Fresnel-Schlick)
vec3 DirectLight(vec3 n, vec3 v, vec3 l, vec3 albedo, float metallic,
                 float roughness, vec3 diffuse_light, vec3 specular_light) {
  float n_dot_l = dot(n, l);
  if(n_dot_l <= 0.0) {
    return vec3(0.0);
  }
  vec3 h = normalize(v + l);
  float n_dot_v = max(dot(n, v), 1e-4);
  vec3 f0 = mix(vec3(0.04), albedo, metallic);
  vec3 f = f0 + (1.0 - f0) * pow(1.0 - max(dot(h, v), 0.0), 5.0);
  vec3 specular = DistributionGgx(max(dot(n, h), 0.0), roughness) *
                  GeometrySmith(n_dot_v, n_dot_l, roughness) * f /
                  (4.0 * n_dot_v * n_dot_l + 1e-4);
  vec3 diffuse = (1.0 - f) * (1.0 - metallic) * albedo / PI;
  return (diffuse * diffuse_light + specular * specular_light) * n_dot_l;
}

// the depth below the surface (m) of the relief of the ground materials k,
// of weights weight, at the body position b: each sunk by its height scale
// where its height map is dark
float GroundDepth(int count, ivec4 k, vec4 weight, vec3 b, vec3 w, vec3 dbx,
                  vec3 dby, bool vary) {
  float depth = 0.0;
  for(int e = 0; e < count; e++) {
    GroundMaterial g = ground_materials[k[e]];
    if(g.layers.w >= 0 && g.scale.z > 0.0) {
      float scale = g.scale.x;
      float h = SampleTriplanar(g.layers.w, b * scale, w, dbx * scale,
                                dby * scale, vary).r;
      depth += weight[e] * g.scale.z * (1.0 - h);
    }
  }
  return depth;
}

// parallax occlusion mapping (Tatarchuk, Practical Parallax Occlusion
// Mapping, 2006): the point of the relief of the ground materials that the
// view ray, toward view (a unit vector in the body frame) through the
// surface at b of normal nb (toward the camera), meets first, found by
// stepping down the ray and refining between the last steps above and below
// the relief. It fades out where a pixel spans more than the relief, which
// would only shimmer.
vec3 GroundParallax(int count, ivec4 k, vec4 weight, vec3 b, vec3 view,
                    vec3 nb, vec3 w, vec3 dbx, vec3 dby, bool vary) {
  float deepest = 0.0;
  for(int e = 0; e < count; e++) {
    GroundMaterial g = ground_materials[k[e]];
    if(g.layers.w >= 0) {
      deepest += weight[e] * g.scale.z;
    }
  }
  float pixel = max(length(dbx), length(dby));
  float fade = 1.0 - smoothstep(0.5 * deepest, 2.0 * deepest, pixel);
  if(deepest <= 0.0 || fade <= 0.0) {
    return b;
  }
  float c = max(-dot(view, nb), 0.15); // the cosine of the view to the normal
  int steps = int(mix(32.0, 8.0, c));
  float ds = deepest / c / float(steps);
  float s = 0.0;
  float gap = GroundDepth(count, k, weight, b, w, dbx, dby, vary);
  for(int i = 0; i < steps; i++) {
    float next = GroundDepth(count, k, weight, b + view * (s + ds), w, dbx,
                             dby, vary) -
                 (s + ds) * c;
    if(next <= 0.0) {
      // between the last step above the relief and this one below it
      s += ds * gap / max(gap - next, 1e-6);
      return b + fade * view * s;
    }
    gap = next;
    s += ds;
  }
  return b + fade * view * s;
}

#ifdef FLUID
// the surface's point at pixel p, with depth d; false where there is none
bool FluidPoint(ivec2 p, vec2 size, out vec3 point) {
  float d = texelFetch(fluid_depth, p, 0).r;
  point = FluidPosition(vec2(p) + 0.5, size, d);
  return d > 0.0;
}

// the difference to the nearer of the neighbours p - step and p + step
// from the point x at p; false if neither is on the surface
bool FluidDifference(ivec2 p, ivec2 step, vec2 size, vec3 x, out vec3 dx) {
  vec3 a, b;
  ivec2 last = ivec2(size) - 1;
  bool has_a = FluidPoint(clamp(p - step, ivec2(0), last), size, a);
  bool has_b = FluidPoint(clamp(p + step, ivec2(0), last), size, b);
  if(has_a && (!has_b || distance(x, a) < distance(x, b))) {
    dx = x - a;
  } else {
    dx = b - x;
  }
  return has_a || has_b;
}
#endif

void main() {
#ifdef FLUID
  {
    vec2 size = vec2(textureSize(fluid_depth, 0));
    ivec2 p = ivec2(gl_FragCoord.xy);
    vec3 x;
    if(!FluidPoint(p, size, x)) {
      discard;
    }
    vec3 dx, dy;
    vec3 n = -fluid.camera_forward.xyz;
    if(FluidDifference(p, ivec2(1, 0), size, x, dx) &&
       FluidDifference(p, ivec2(0, 1), size, x, dy)) {
      vec3 c = cross(dx, dy);
      if(dot(c, c) > 0.0) {
        n = normalize(c);
      }
    }
    world_position = x;
    world_normal = n;
    body_position = x;
    body_normal = n;
    uv = vec2(0.0);
    body_rotation = mat3(1.0);
    gl_FragDepth = FluidFragDepth(x);
  }
#endif
#ifndef FLUID
  // a terrain's holes, where regions of it draw their own surfaces
  for(int k = 0; k < mesh.layers.w; k++) {
    vec4 h = mesh.holes[k];
    if(all(greaterThanEqual(uv, h.xy)) && all(lessThanEqual(uv, h.zw))) {
      discard;
    }
  }
#endif
#ifdef CAD
  // distance from the edges of the face, in pixels, where the patch
  // crosses them; a little outside still counts, so that adjacent faces
  // overlap instead of leaving cracks between their tessellations
  vec2 duv_dx = dFdx(uv);
  vec2 duv_dy = dFdy(uv);
  float trim_pixels = 1e30;
  float edge_pixels = 1e30; // from the edges drawn with lines
  if((cad_face.y & 1) != 0 &&
     !InsideTrims(cad_face.x, uv, duv_dx, duv_dy, trim_pixels,
                  edge_pixels) &&
     trim_pixels > cad.edges.z) {
    discard;
  }
#endif
  // a leaf or a blade of grass cut out of its card where the base color's
  // alpha is below the cutoff; the cutoff is lowered at coarser levels of
  // detail, whose texels average the leaves with the clear texels between
  // them, so that foliage does not thin out in the distance
  if(mesh.variation.z > 0.0 && mesh.layers.x >= 0) {
    float lod = LayerLod(mesh.layers.x, uv);
    float cutoff = mesh.variation.z * max(1.0 - 0.15 * lod, 0.4);
    if(SampleLayer(mesh.layers.x, uv).a < cutoff) {
      discard;
    }
  }
  vec3 v = normalize(frame.camera_position.xyz - world_position);
  vec3 n = normalize(world_normal);
  vec3 nb = normalize(body_normal);
  // two sided: seen from behind, the surface is lit as its back face
  float side = dot(n, v) < 0.0 ? -1.0 : 1.0;
#ifdef CAD
  // which side is seen, from the triangle drawn: near a silhouette the
  // surface's normal may turn a little away from the camera on its front
  vec3 g = cross(dFdx(world_position), dFdy(world_position));
  if(dot(g, g) > 0.0) {
    side = dot(n, g) * dot(v, g) < 0.0 ? -1.0 : 1.0;
  }
#endif
  n *= side;
  nb *= side;

  vec3 albedo = mesh.albedo.rgb;
#if !defined(CAD) && !defined(FLUID)
  if(mesh.flex.x > 0.5) {
    float t = clamp(uv.x, 0.0, 1.0) * 15.0;
    int k = min(int(t), 14);
    albedo = SrgbToLinear(mix(frame.color_map[k].rgb,
                              frame.color_map[k + 1].rgb, t - float(k)));
  }
#endif
#ifdef CAD
  vec4 face_color = faces[cad_face.x].color;
  if(mesh.flex.x > 0.5) {
    // a flexible body's stress drawn, over the faces' colors
    float t = clamp(cad_stress, 0.0, 1.0) * 15.0;
    int k = min(int(t), 14);
    albedo = SrgbToLinear(mix(frame.color_map[k].rgb,
                              frame.color_map[k + 1].rgb, t - float(k)));
  } else if(face_color.w > 0.5) {
    albedo = SrgbToLinear(face_color.rgb);
  }
#endif
  float metallic = mesh.material.x;
  float roughness = mesh.material.y;
  bool triplanar = mesh.material.z > 0.5;
  bool vary = mesh.variation.x > 0.5;
  // derivatives of the body position, here where control flow is uniform
  vec3 dbx = dFdx(body_position);
  vec3 dby = dFdy(body_position);
  // terrains without texture variation: triplanar and hex-tiled, so that
  // the texture shows no grid
  bool hex = mesh.material.z > 1.5;
  vec3 p = body_position * mesh.material.w;
  vec3 dpx = dbx * mesh.material.w;
  vec3 dpy = dby * mesh.material.w;
  vec3 w = pow(abs(nb), vec3(4.0));
  w /= w.x + w.y + w.z;
  HexPlanes hex_planes = HexPlanesSetup(p, w, dFdx(p), dFdy(p));

  float below_brightness = 0.5; // of the texture, for the surfaces' edges
  int ground_map = triplanar ? int(mesh.variation.y + 0.5) - 1 : -1;
  if(ground_map >= 0) {
    // the materials of the map in place of the mesh's own textures
    ivec4 k;
    vec4 weight;
    int count = GroundWeights(ground_map, world_position, k, weight);
    // the point of the relief seen, sunk into the surface
    vec3 relief = GroundParallax(
      count, k, weight, body_position,
      normalize(transpose(body_rotation) * -v), nb, w, dbx, dby, vary);
    vec4 height = vec4(0.5);
    for(int e = 0; e < count; e++) {
      GroundMaterial g = ground_materials[k[e]];
      if(g.layers.w >= 0) {
        float scale = g.scale.x;
        height[e] = SampleTriplanar(g.layers.w, relief * scale, w,
                                    dbx * scale, dby * scale, vary).r;
      }
    }
    weight = GroundHeightBlend(count, k, weight, height);
    vec3 albedo_sum = vec3(0.0);
    vec3 normal_sum = vec3(0.0);
    float roughness_sum = 0.0;
    float brightness_sum = 0.0;
    vec3 seen = normalize(body_rotation * nb);
    for(int e = 0; e < count; e++) {
      GroundMaterial g = ground_materials[k[e]];
      float scale = g.scale.x;
      vec3 q = relief * scale;
      vec3 c = g.color.rgb;
      float brightness = 0.5;
      if(g.layers.x >= 0) {
        vec4 t = SampleTriplanar(g.layers.x, q, w, dbx * scale, dby * scale,
                                 vary);
        brightness = dot(t.rgb, vec3(0.333));
        c *= SrgbToLinear(t.rgb);
      }
      float r = g.color.w;
      if(g.layers.z >= 0) {
        r *= SampleTriplanar(g.layers.z, q, w, dbx * scale, dby * scale,
                             vary).g;
      }
      vec3 ng = seen;
      if(g.layers.y >= 0) {
        ng = side * normalize(body_rotation *
                              TriplanarNormal(g.layers.y, q, side * nb, w,
                                              g.scale.y, dbx * scale,
                                              dby * scale, vary));
      }
      albedo_sum += weight[e] * c;
      roughness_sum += weight[e] * r;
      normal_sum += weight[e] * ng;
      brightness_sum += weight[e] * brightness;
    }
    albedo = albedo_sum;
    roughness = roughness_sum;
    metallic = 0.0;
    n = normalize(normal_sum);
    below_brightness = brightness_sum;
    if(vary) {
      albedo *= MacroBrightness(body_position.xy);
    }
  }
  if(ground_map < 0 && mesh.layers.x >= 0) {
    vec4 s = hex ? HexPlanesColor(hex_planes, mesh.layers.x)
             : triplanar
               ? SampleTriplanar(mesh.layers.x, p, w, dpx, dpy, vary)
               : SampleLayer(mesh.layers.x, uv);
    below_brightness = dot(s.rgb, vec3(0.333));
    albedo *= SrgbToLinear(s.rgb);
  }
  if(ground_map < 0 && triplanar && vary) {
    albedo *= MacroBrightness(body_position.xy);
  }
  if(ground_map < 0 && mesh.layers.y >= 0) {
    vec4 s = hex ? HexPlanesSample(hex_planes, mesh.layers.y)
             : triplanar
               ? SampleTriplanar(mesh.layers.y, p, w, dpx, dpy, vary)
               : SampleLayer(mesh.layers.y, uv);
    roughness *= s.g;
    metallic *= s.b;
  }
  if(ground_map < 0 && mesh.layers.z >= 0) {
    if(triplanar) {
      // the bumps of the front face; the back face has the opposite normal
      n = side * normalize(
                   body_rotation *
                   (hex ? HexPlanesNormal(hex_planes, mesh.layers.z,
                                          side * nb, mesh.albedo.w)
                        : TriplanarNormal(mesh.layers.z, p, side * nb, w,
                                          mesh.albedo.w, dpx, dpy, vary)));
    } else {
      vec3 t = SampleLayer(mesh.layers.z, uv).xyz * 2.0 - 1.0;
      t.xy *= mesh.albedo.w;
      n = normalize(CotangentFrame(n, world_position, uv) * t);
    }
  }
  // the surfaces drawn over the texture in patches, each with its own
  // texture and normal map
  for(int i = 0; i < 2 && triplanar; i++) {
    int color_layer = i == 0 ? mesh.surface_layers.x : mesh.surface_layers.z;
    int normal_layer = i == 0 ? mesh.surface_layers.y : mesh.surface_layers.w;
    vec4 surface = mesh.surface[i];
    if(color_layer < 0) {
      continue;
    }
    float m = SurfaceMask(body_position.xy, i, surface.w, surface.z,
                          below_brightness);
    if(m <= 0.0) {
      continue;
    }
    vec3 q = body_position * surface.x;
    vec4 s = SampleTriplanar(color_layer, q, w, dbx * surface.x,
                             dby * surface.x, vary);
    vec3 color = SrgbToLinear(s.rgb);
    if(vary) {
      color *= MacroBrightness(body_position.xy);
    }
    albedo = mix(albedo, color, m);
    below_brightness = mix(below_brightness, dot(s.rgb, vec3(0.333)), m);
    vec3 surface_normal = normalize(body_rotation * nb); // seen side
    if(normal_layer >= 0) {
      surface_normal = side * normalize(body_rotation *
                                        TriplanarNormal(normal_layer, q,
                                                        side * nb, w, surface.y,
                                                        dbx * surface.x,
                                                        dby * surface.x, vary));
    }
    n = normalize(mix(n, surface_normal, m));
  }
  roughness = clamp(roughness, 0.05, 1.0);
  metallic = clamp(metallic, 0.0, 1.0);
  float n_dot_v = max(dot(n, v), 1e-4);
  vec3 f0 = mix(vec3(0.04), albedo, metallic);

  vec3 sun = frame.sun_color.rgb * frame.sun_direction.w;
  vec3 color = DirectLight(n, v, frame.sun_direction.xyz, albedo, metallic,
                           roughness, sun, sun);
  for(int i = 0; i < min(lights.count.x, kMaxLights); i++) {
    vec3 l;
    float distance;
    float factor = LightAt(i, world_position, l, distance);
    if(factor > 0.0) {
      color += factor * DirectLight(n, v, l, albedo, metallic, roughness,
                                    lights.light[i].diffuse.rgb,
                                    lights.light[i].specular.rgb);
    }
  }
  color += lights.ambient.rgb * (1.0 - metallic) * albedo;

  // sky light, split sum approximation
  vec3 r = reflect(-v, n);
  vec3 prefiltered = SpecularLight(environment_specular, r, roughness);
  vec2 brdf = texture(environment_brdf, vec2(n_dot_v, roughness)).rg;
  vec3 irradiance = DiffuseLight(environment_irradiance, n);
  vec3 f_ambient = f0 + (max(vec3(1.0 - roughness), f0) - f0) *
                   pow(1.0 - n_dot_v, 5.0);
  color += (1.0 - f_ambient) * (1.0 - metallic) * albedo * irradiance +
           prefiltered * (f0 * brdf.x + brdf.y);

#ifdef CAD
  // edge lines, half on each face of an edge
  color *= 1.0 - cad.edges.y *
                   (1.0 - smoothstep(cad.edges.x, cad.edges.x + 1.0,
                                     edge_pixels));
#endif
  color += mesh.emission.rgb * albedo;
  // seen through water: the size of a pixel here, scaled to the surface
  float pixel = length(fwidth(world_position));
  vec3 crossing;
  if(WaterCrossing(frame.camera_position.xyz, world_position, crossing)) {
    // the size of the pixel across the view at the crossing (the size of
    // the pixel on a surface it grazes is the size along the view)
    vec3 to_camera = normalize(frame.camera_position.xyz - crossing);
    float footprint =
      pixel * max(-to_camera.z, 0.02) *
      distance(frame.camera_position.xyz, crossing) /
      max(distance(frame.camera_position.xyz, world_position), 1e-6);
    color = WaterOver(frame.camera_position.xyz, crossing, world_position,
                      color, false, footprint);
  }
  color = Aerial(frame.camera_position.xyz, world_position, color,
                 frame.fog_params, frame.fog_color,
                 WaterSky(HorizonDirection(world_position -
                                           frame.camera_position.xyz)));
  color = color / (color + vec3(1.0));
  out_color = vec4(pow(color, vec3(1.0 / 2.2)), mesh.variation.w);
}

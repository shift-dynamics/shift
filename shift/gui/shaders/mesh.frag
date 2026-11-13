#version 450
#extension GL_GOOGLE_include_directive : require

layout(location = 0) in vec3 world_position;
layout(location = 1) in vec3 world_normal;
layout(location = 2) in vec3 body_position;
layout(location = 3) in vec3 body_normal;
layout(location = 4) in vec2 uv;
layout(location = 5) flat in mat3 body_rotation; // body to world

layout(location = 0) out vec4 out_color;

#ifdef CAD
// compiled as cad.frag for the faces of CAD geometry (cad.tese): uv are the
// parameters of the face's surface; fragments outside its trims are dropped
#define CAD_TRIM
#define CAD_TRIM_ONLY
#include "cad_common.glsl"
layout(location = 8) flat in ivec2 cad_face; // face, flags (1: test trims)
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
} frame;

layout(std140, binding = 1) uniform Material {
  vec4 albedo;        // w = normal map scale
  vec4 material;      // x = metallic, y = roughness, z = triplanar,
                      // w = 1 / texture scale
  ivec4 layers;       // base color, metallic roughness, normal; -1: none
  vec4 variation;     // x = 1: vary the tiles of triplanar textures
  ivec4 surface_layers; // base color and normal of surfaces 0 and 1
  vec4 surface[2];    // x = 1 / texture scale, y = normal scale, z = noise
                      // level, w = 1 / patch size
  vec4 emission;      // rgb: factor of the base color emitted (a lamp's
                      // lens, lit)
} mesh;

// image based lighting of the sky, indexed by the z component of a direction
// (the sky is symmetric about z); see environment.h
layout(binding = 2) uniform sampler2D environment_specular;   // z, roughness
layout(binding = 3) uniform sampler2D environment_irradiance; // z
layout(binding = 4) uniform sampler2D environment_brdf;       // n.v, roughness

// texture layers of all materials
layout(binding = 5) uniform sampler2DArray textures;

#define LIGHTS_BINDING 8
#define IES_BINDING 9
#include "lights.glsl"
#include "surfaces.glsl"

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
  return texture(textures, vec3(p, float(layer)));
}

// one plane of a triplanar projection, q in texture tiles with its screen
// space derivatives (taken where control flow is uniform), its tiles shifted
// at random if vary
vec4 SamplePlane(int layer, vec2 q, vec2 dx, vec2 dy, bool vary) {
  if(!vary) {
    return textureGrad(textures, vec3(q, float(layer)), dx, dy);
  }
  TileShift t = TileVariation(q, true);
  return BlendTiles(textureGrad(textures, vec3(q + t.a, float(layer)), dx, dy),
                    textureGrad(textures, vec3(q + t.b, float(layer)), dx, dy),
                    t.f);
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

void main() {
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
#ifdef CAD
  vec4 face_color = faces[cad_face.x].color;
  if(face_color.w > 0.5) {
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
  vec3 p = body_position * mesh.material.w;
  vec3 dpx = dbx * mesh.material.w;
  vec3 dpy = dby * mesh.material.w;
  vec3 w = pow(abs(nb), vec3(4.0));
  w /= w.x + w.y + w.z;

  float below_brightness = 0.5; // of the texture, for the surfaces' edges
  if(mesh.layers.x >= 0) {
    vec4 s = triplanar ? SampleTriplanar(mesh.layers.x, p, w, dpx, dpy, vary)
                       : SampleLayer(mesh.layers.x, uv);
    below_brightness = dot(s.rgb, vec3(0.333));
    albedo *= SrgbToLinear(s.rgb);
  }
  if(triplanar && vary) {
    albedo *= MacroBrightness(body_position.xy);
  }
  if(mesh.layers.y >= 0) {
    vec4 s = triplanar ? SampleTriplanar(mesh.layers.y, p, w, dpx, dpy, vary)
                       : SampleLayer(mesh.layers.y, uv);
    roughness *= s.g;
    metallic *= s.b;
  }
  if(mesh.layers.z >= 0) {
    if(triplanar) {
      // the bumps of the front face; the back face has the opposite normal
      n = side * normalize(body_rotation *
                           TriplanarNormal(mesh.layers.z, p, side * nb, w,
                                           mesh.albedo.w, dpx, dpy, vary));
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
  vec3 prefiltered =
    texture(environment_specular, vec2(0.5 + 0.5 * r.z, roughness)).rgb;
  vec2 brdf = texture(environment_brdf, vec2(n_dot_v, roughness)).rg;
  vec3 irradiance =
    texture(environment_irradiance, vec2(0.5 + 0.5 * n.z, 0.5)).rgb;
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
  color = color / (color + vec3(1.0));
  out_color = vec4(pow(color, vec3(1.0 / 2.2)), 1.0);
}

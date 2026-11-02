#version 450

layout(location = 0) in vec3 world_position;
layout(location = 1) in vec3 world_normal;
layout(location = 2) in vec3 body_position;
layout(location = 3) in vec3 body_normal;
layout(location = 4) in vec2 uv;
layout(location = 5) flat in mat3 body_rotation; // body to world

layout(location = 0) out vec4 out_color;

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
} mesh;

// image based lighting of the sky, indexed by the z component of a direction
// (the sky is symmetric about z); see environment.h
layout(binding = 2) uniform sampler2D environment_specular;   // z, roughness
layout(binding = 3) uniform sampler2D environment_irradiance; // z
layout(binding = 4) uniform sampler2D environment_brdf;       // n.v, roughness

// texture layers of all materials
layout(binding = 5) uniform sampler2DArray textures;

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

// triplanar projection along the body axes, p in texture tiles
vec4 SampleTriplanar(int layer, vec3 p, vec3 w) {
  return SampleLayer(layer, p.zy) * w.x + SampleLayer(layer, p.xz) * w.y +
         SampleLayer(layer, p.xy) * w.z;
}

// triplanar normal mapping with the whiteout blend (B. Golus, "Normal
// Mapping for a Triplanar Shader"); n is the body space surface normal
vec3 TriplanarNormal(int layer, vec3 p, vec3 n, vec3 w, float scale) {
  vec3 tx = SampleLayer(layer, p.zy).xyz * 2.0 - 1.0;
  vec3 ty = SampleLayer(layer, p.xz).xyz * 2.0 - 1.0;
  vec3 tz = SampleLayer(layer, p.xy).xyz * 2.0 - 1.0;
  tx.xy *= scale;
  ty.xy *= scale;
  tz.xy *= scale;
  tx = vec3(tx.xy + n.zy, abs(tx.z) * n.x);
  ty = vec3(ty.xy + n.xz, abs(ty.z) * n.y);
  tz = vec3(tz.xy + n.xy, abs(tz.z) * n.z);
  return normalize(tx.zyx * w.x + ty.xzy * w.y + tz.xyz * w.z);
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

void main() {
  vec3 v = normalize(frame.camera_position.xyz - world_position);
  vec3 n = normalize(world_normal);
  vec3 nb = normalize(body_normal);
  if(dot(n, v) < 0.0) {
    n = -n; // two sided
    nb = -nb;
  }

  vec3 albedo = mesh.albedo.rgb;
  float metallic = mesh.material.x;
  float roughness = mesh.material.y;
  bool triplanar = mesh.material.z > 0.5;
  vec3 p = body_position * mesh.material.w;
  vec3 w = pow(abs(nb), vec3(4.0));
  w /= w.x + w.y + w.z;

  if(mesh.layers.x >= 0) {
    vec4 s = triplanar ? SampleTriplanar(mesh.layers.x, p, w)
                       : SampleLayer(mesh.layers.x, uv);
    albedo *= SrgbToLinear(s.rgb);
  }
  if(mesh.layers.y >= 0) {
    vec4 s = triplanar ? SampleTriplanar(mesh.layers.y, p, w)
                       : SampleLayer(mesh.layers.y, uv);
    roughness *= s.g;
    metallic *= s.b;
  }
  if(mesh.layers.z >= 0) {
    if(triplanar) {
      n = normalize(body_rotation *
                    TriplanarNormal(mesh.layers.z, p, nb, w, mesh.albedo.w));
    } else {
      vec3 t = SampleLayer(mesh.layers.z, uv).xyz * 2.0 - 1.0;
      t.xy *= mesh.albedo.w;
      n = normalize(CotangentFrame(n, world_position, uv) * t);
    }
  }
  roughness = clamp(roughness, 0.05, 1.0);
  metallic = clamp(metallic, 0.0, 1.0);
  vec3 l = frame.sun_direction.xyz;
  vec3 h = normalize(v + l);
  float n_dot_l = max(dot(n, l), 0.0);
  float n_dot_v = max(dot(n, v), 1e-4);

  vec3 f0 = mix(vec3(0.04), albedo, metallic);
  vec3 f = f0 + (1.0 - f0) * pow(1.0 - max(dot(h, v), 0.0), 5.0);
  vec3 specular = DistributionGgx(max(dot(n, h), 0.0), roughness) *
                  GeometrySmith(n_dot_v, n_dot_l, roughness) * f /
                  (4.0 * n_dot_v * n_dot_l + 1e-4);
  vec3 diffuse = (1.0 - f) * (1.0 - metallic) * albedo / PI;
  vec3 sun = frame.sun_color.rgb * frame.sun_direction.w;
  vec3 color = (diffuse + specular) * sun * n_dot_l;

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

  color = color / (color + vec3(1.0));
  out_color = vec4(pow(color, vec3(1.0 / 2.2)), 1.0);
}

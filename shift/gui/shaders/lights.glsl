// the lights of the model (shift::gui::SceneLight), shared by mesh.frag,
// floor.frag and raytrace.comp; keep in sync with LightUniforms in
// viewport.cc. The including shader defines LIGHTS_BINDING and IES_BINDING.

const int kMaxLights = 16;
const int kLightPoint = 0;
const int kLightSpot = 1;
const int kLightDirectional = 2;
const int kLightIes = 3;

struct Light {
  vec4 position;    // xyz: world position, w: type
  vec4 direction;   // xyz: direction the light shines (-z of its frame),
                    // w: cosine of the spot cone angle
  vec4 axis_x;      // xyz: x axis of the light frame (IES horizontal angle
                    // 0), w: cosine of the spot inner cone angle
  vec4 diffuse;     // rgb: pi Kd intensity, w: IES profile, -1 if none
  vec4 specular;    // rgb: pi Ks intensity
  vec4 attenuation; // xyz: c, l, q of 1 / (c + l d + q d^2)
};

layout(std140, binding = LIGHTS_BINDING) uniform Lights {
  ivec4 count;   // x: number of lights, y: number of IES profiles
  vec4 ambient;  // rgb: sum of the Ka of the lights
  Light light[kMaxLights];
} lights;

// IES distributions, one block of kIesHorizontal rows per profile: column:
// vertical angle 0 (nadir) to pi, row: horizontal angle 0 to 2 pi
layout(binding = IES_BINDING) uniform sampler2D ies_profiles;
const float kIesVertical = 128.0;   // Scene::kIesVertical
const float kIesHorizontal = 65.0;  // Scene::kIesHorizontal

float IesDistribution(Light light, vec3 shine, float cos_axis) {
  const float PI_ = 3.14159265358979;
  vec3 z = -light.direction.xyz;
  vec3 x = light.axis_x.xyz;
  vec3 y = cross(z, x);
  float vertical = acos(clamp(cos_axis, -1.0, 1.0));
  float horizontal = atan(dot(shine, y), dot(shine, x));
  if(horizontal < 0.0) {
    horizontal += 2.0 * PI_;
  }
  // texel centers: the angles are sampled at both ends of their ranges
  vec2 size = vec2(textureSize(ies_profiles, 0));
  float u = (vertical / PI_ * (kIesVertical - 1.0) + 0.5) / size.x;
  float v = (light.diffuse.w * kIesHorizontal +
             horizontal / (2.0 * PI_) * (kIesHorizontal - 1.0) + 0.5) /
            size.y;
  return textureLod(ies_profiles, vec2(u, v), 0.0).r;
}

// light i at point p: l, the unit vector towards the light; distance to it
// (1e30 for a directional light); returns the factor of its colors there:
// attenuation, spot cone and IES distribution
float LightAt(int i, vec3 p, out vec3 l, out float distance) {
  Light light = lights.light[i];
  int type = int(light.position.w + 0.5);
  if(type == kLightDirectional) {
    l = -light.direction.xyz;
    distance = 1e30;
    return 1.0;
  }
  vec3 d = light.position.xyz - p;
  distance = length(d);
  l = d / max(distance, 1e-6);
  float factor = 1.0 / max(dot(light.attenuation.xyz,
                               vec3(1.0, distance, distance * distance)),
                           1e-6);
  vec3 shine = -l; // from the light to p
  float cos_axis = dot(shine, light.direction.xyz);
  if(type == kLightSpot) {
    factor *= smoothstep(light.direction.w, light.axis_x.w, cos_axis);
  } else if(type == kLightIes) {
    factor *= IesDistribution(light, shine, cos_axis);
  }
  return factor;
}

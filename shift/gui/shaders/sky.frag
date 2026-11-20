#version 450
#extension GL_GOOGLE_include_directive : require

layout(location = 0) in vec2 uv;
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
  mat4 floor_frame;
  vec4 fog_params;
  vec4 fog_color;
  vec4 water_params;     // height (up), wave height, wavelength, 1 if water
  vec4 water_wave;       // direction of travel (x, y), time
  vec4 water_color;
  vec4 water_extinction;
} frame;

#include "tonemap.glsl"

// the radiance of the sky, equirectangular (see environment.glsl)
layout(binding = 10) uniform sampler2D sky_radiance;
#include "environment.glsl"
#define WATER_SKY sky_radiance
#include "surfaces.glsl"
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

// keep in sync with Sky() in raytrace.comp
vec3 Sky(vec3 d) {
  vec3 color = textureLod(sky_radiance, EquirectUv(d), 0.0).rgb;
  float s = max(dot(d, frame.sun_direction.xyz), 0.0);
  color += frame.sun_color.rgb * (pow(s, 1500.0) * 20.0 + pow(s, 30.0) * 0.15);
  return color;
}

void main() {
  vec4 p = frame.inverse_view_projection * vec4(uv * 2.0 - 1.0, 1.0, 1.0);
  vec3 d = normalize(p.xyz / p.w - frame.camera_position.xyz);
  vec3 color = Sky(d);
  // open water where the ray goes down to it; the surfaces drawn after the
  // sky cover the rest, with the water over those below it
  vec3 c = frame.camera_position.xyz;
  float t = (WaterZ() - c.z) / max(d.z, 1e-6);
  // across the view; the water stretches it along the view where it grazes
  float footprint = t * length(fwidth(d));
  if(HasWater() && c.z < WaterZ() && d.z > 1e-6) {
    vec3 crossing = c + d * t;
    color = WaterOver(c, crossing, crossing, vec3(0.0), true, footprint);
    color = Aerial(c, crossing, color, frame.fog_params, frame.fog_color,
                   WaterSky(HorizonDirection(d)));
  }
  color = ToneMap(color, frame.sky_zenith.w);
  out_color = vec4(pow(color, vec3(1.0 / 2.2)), 1.0);
}

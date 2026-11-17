#version 450
#extension GL_GOOGLE_include_directive : require

layout(location = 0) in vec3 world_position;
layout(location = 1) in vec2 plane_position;
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

#include "environment.glsl"
// the radiance of the sky, equirectangular, for the water's reflections
layout(binding = 10) uniform sampler2D sky_radiance;
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

#define LIGHTS_BINDING 8
#define IES_BINDING 9
#include "lights.glsl"

const vec3 kFloorColor = vec3(0.42, 0.42, 0.40);
const vec3 kLineColor = vec3(0.22, 0.22, 0.22);
const float kMinorWidth = 0.02; // line width, fraction of a grid cell
const float kMajorWidth = 0.004; // every 10th line, twice as wide

// coverage of one family of grid lines, anti-aliased per axis: lines are at
// least a pixel wide, fading in proportion when thinner, and fade to their
// mean coverage once the cells shrink below a pixel (after B. Golus, "The
// Best Darn Grid Shader (Yet)")
float GridCoverage(vec2 uv, float width) {
  vec4 uv_dxy = vec4(dFdx(uv), dFdy(uv));
  vec2 uv_derivative = vec2(length(uv_dxy.xz), length(uv_dxy.yw));
  vec2 draw_width = clamp(vec2(width), uv_derivative, vec2(0.5));
  vec2 line_aa = max(uv_derivative, vec2(1e-6)) * 1.5;
  vec2 grid_uv = 1.0 - abs(fract(uv) * 2.0 - 1.0); // 0 on a line
  vec2 grid = smoothstep(draw_width + line_aa, draw_width - line_aa, grid_uv);
  grid *= clamp(width / draw_width, 0.0, 1.0);
  grid = mix(grid, vec2(width), clamp(uv_derivative * 2.0 - 1.0, 0.0, 1.0));
  return mix(grid.x, 1.0, grid.y);
}

// keep in sync with FloorAlbedo() in raytrace.comp
vec3 FloorAlbedo(vec2 p, float spacing) {
  float minor = GridCoverage(p / spacing, kMinorWidth);
  float major = GridCoverage(p / (10.0 * spacing), kMajorWidth);
  return mix(kFloorColor, kLineColor, max(minor * 0.6, major));
}

void main() {
    vec3 albedo = FloorAlbedo(plane_position, frame.floor_params.y);
  vec3 n = frame.floor_frame[2].xyz; // up, out of the solid
  float n_dot_l = max(dot(n, frame.sun_direction.xyz), 0.0);
  vec3 color = albedo / 3.14159265 * frame.sun_color.rgb * frame.sun_direction.w * n_dot_l +
               albedo * frame.sky_zenith.rgb;
  for(int i = 0; i < min(lights.count.x, kMaxLights); i++) {
    vec3 l;
    float distance;
    float factor = LightAt(i, world_position, l, distance);
    color += albedo / 3.14159265 * lights.light[i].diffuse.rgb * factor *
             max(dot(n, l), 0.0);
  }
  color += albedo * lights.ambient.rgb;

  // fade into the horizon
  vec3 to_point = world_position - frame.camera_position.xyz;
  float fade = clamp(length(to_point) / frame.floor_params.w, 0.0, 1.0);
  color = mix(color, frame.sky_horizon.rgb, fade * fade);
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
  out_color = vec4(pow(color, vec3(1.0 / 2.2)), 1.0);
}

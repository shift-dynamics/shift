#version 450

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
} frame;

// keep in sync with Sky() in raytrace.comp
vec3 Sky(vec3 d) {
  float altitude = -d.z; // +z is down
  vec3 color;
  if(altitude >= 0.0) {
    color = mix(frame.sky_horizon.rgb, frame.sky_zenith.rgb,
                pow(altitude, 0.5));
  } else {
    color = mix(frame.sky_horizon.rgb, vec3(0.12, 0.11, 0.10),
                clamp(-altitude * 4.0, 0.0, 1.0));
  }
  float s = max(dot(d, frame.sun_direction.xyz), 0.0);
  color += frame.sun_color.rgb * (pow(s, 1500.0) * 20.0 + pow(s, 30.0) * 0.15);
  return color;
}

void main() {
  vec4 p = frame.inverse_view_projection * vec4(uv * 2.0 - 1.0, 1.0, 1.0);
  vec3 d = normalize(p.xyz / p.w - frame.camera_position.xyz);
  vec3 color = Sky(d);
  color = color / (color + vec3(1.0));
  out_color = vec4(pow(color, vec3(1.0 / 2.2)), 1.0);
}

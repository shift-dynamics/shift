// the uniforms of the soil surface drawn over the particles of SPH soils
// (screen space surface: fluid_depth.vert/.frag, fluid_blur.frag and
// mesh.frag compiled with FLUID); keep in sync with FluidUniforms in
// viewport.cc. The including shader defines FLUID_BINDING.

layout(std140, binding = FLUID_BINDING) uniform Fluid {
  mat4 view_projection;
  mat4 inverse_view_projection;
  vec4 camera_position; // w: 1 if clip space depth runs 0 to 1, else -1 to 1
  vec4 camera_forward;  // the camera's view direction, a unit vector
  vec4 particles;       // x: radius of the spheres drawn, y: pixels per unit
                        // length at unit depth
  vec4 filter_params;   // x: radius of the blur (world), y: its range in
                        // depth beyond that of a slant (world), z: its
                        // largest radius (pixels)
} fluid;

// the depth buffer's value of a point in the world
float FluidFragDepth(vec3 p) {
  vec4 clip = fluid.view_projection * vec4(p, 1.0);
  float z = clip.z / clip.w;
  return fluid.camera_position.w > 0.0 ? z : 0.5 * z + 0.5;
}

// the point in the world seen at pixel coordinates frag (gl_FragCoord.xy)
// of a target of size size, at depth (along camera_forward)
vec3 FluidPosition(vec2 frag, vec2 size, float depth) {
  vec2 ndc = 2.0 * frag / size - 1.0;
  vec4 a = fluid.inverse_view_projection * vec4(ndc, 0.0, 1.0);
  vec4 b = fluid.inverse_view_projection * vec4(ndc, 0.5, 1.0);
  vec3 ray = normalize(b.xyz / b.w - a.xyz / a.w);
  return fluid.camera_position.xyz +
         ray * (depth / dot(ray, fluid.camera_forward.xyz));
}

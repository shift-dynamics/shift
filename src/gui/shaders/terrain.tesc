#version 420 core
// #extension GL_ARB_tessellation_shader : enable

layout (vertices = 4) out;

in VS_OUT {
  vec2 tc;
} tcs_in[];

out TCS_OUT {
  vec2 tc;
} tcs_out[];

uniform mat4 mv;
uniform mat4 mvp;
uniform mat4 proj_matrix;
uniform vec2 viewport;
uniform sampler2D tex_displacement;
uniform float zMax;
uniform float zMin;

// vec4 project(vec4 vertex){
//   vec4 result = mvp * vertex;
//   result /= result.w;
//   return result;
// }
//
// vec2 screen_space(vec4 vertex){
//   return (clamp(vertex.xy, -1.3, 1.3) + 1) * (screen_size * 0.5);
// }
//
// bool offscreen(vec4 vertex){
//   if(vertex.z < -0.5){
//     return true;
//   }
//   return any(
//     lessThan(vertex.xy, vec2(-1.7)) ||
//     greaterThan(vertex.xy, vec2(1.7))
//   );
// }

// vec3 worldToScreen(vec3 p) {
//   vec4 r = mvp * vec4(p, 1.0);
//   r.xy /= r.w;
//   r.xy = r.xy * 0.5 + 0.5;  // to NDC
//   r.xy *= 16.0; //viewport.zw;      // to pixels in screen space
//   return r.xyz;
// }

// vec3 eyeToScreen(vec4 p) {
//   vec4 r = proj_matrix * p;   // to clip space
//   r.xy /= r.w;            // project
//   r.xy = r.xy * 0.5 + 0.5;  // to NDC
//   r.xy *= viewport.zw;    // to pixels
//   return r.xyz;
// }

// float calcEdgeTessellation(vec2 s0, vec2 s1) {
//   float d = distance(s0, s1);
//   return clamp(d / 16.0, 1, 64);
// }
//
// vec2 toScreen(vec4 p) {
//   p.xy = p.xy * 0.5 + 0.5;  // to NDC
//   p.xy *= 16.0; //viewport.zw;      // to pixels in screen space
//   return p.xy;
// }

float distTess(vec4 p0, vec4 p1, vec2 t0, vec2 t1) {

	// g_tessellatedTriWidth is desired pixels per tri edge
	float g_tessellatedTriWidth = 50.0;

  p0.z += (texture(tex_displacement, t0).r * (zMax - zMin)) + zMin;
  p1.z += (texture(tex_displacement, t1).r * (zMax - zMin)) + zMin;

  p0.z *= 10.0;
  p1.z *= 10.0;

	vec4 center = 0.5 * (p0 + p1);
	float radius = distance(p0, p1) / 2;

	vec4 sc0 = mv * center;
	vec4 sc1 = sc0;
	sc0.x -= radius;
	sc1.x += radius;

	vec4 clip0 = proj_matrix * sc0;
	vec4 clip1 = proj_matrix * sc1;

	clip0 /= clip0.w;
	clip1 /= clip1.w;

	clip0.xy *= viewport;
	clip1.xy *= viewport;

	float d = distance(clip0, clip1);

	return clamp(d / g_tessellatedTriWidth, 0, 64);
}



void main(void) {

  gl_TessLevelOuter[0] =
    distTess(
      gl_in[3].gl_Position,
      gl_in[0].gl_Position,
      tcs_in[3].tc,
      tcs_in[0].tc);

  gl_TessLevelOuter[1] =
    distTess(
      gl_in[0].gl_Position,
      gl_in[1].gl_Position,
      tcs_in[0].tc,
      tcs_in[1].tc);

  gl_TessLevelOuter[2] =
    distTess(
      gl_in[1].gl_Position,
      gl_in[2].gl_Position,
      tcs_in[1].tc,
      tcs_in[2].tc);

  gl_TessLevelOuter[3] =
    distTess(
      gl_in[2].gl_Position,
      gl_in[3].gl_Position,
      tcs_in[2].tc,
      tcs_in[3].tc);

  gl_TessLevelInner[0] = 0.5 * (gl_TessLevelOuter[0] + gl_TessLevelOuter[3]);
	gl_TessLevelInner[1] = 0.5 * (gl_TessLevelOuter[2] + gl_TessLevelOuter[1]);

//   if(gl_InvocationID == 0) {
//     vec4 p0 = mvp * gl_in[0].gl_Position;
//     vec4 p1 = mvp * gl_in[1].gl_Position;
//     vec4 p2 = mvp * gl_in[2].gl_Position;
//     vec4 p3 = mvp * gl_in[3].gl_Position;
//
//     p0 /= p0.w;
//     p1 /= p1.w;
//     p2 /= p2.w;
//     p3 /= p3.w;
//     if (
//       p0.z <= 0.0 ||
//       p1.z <= 0.0 ||
//       p2.z <= 0.0 ||
//       p3.z <= 0.0) {
//
//       gl_TessLevelOuter[0] = -1;
//       gl_TessLevelOuter[1] = -1;
//       gl_TessLevelOuter[2] = -1;
//       gl_TessLevelOuter[3] = -1;
//
//       gl_TessLevelInner[0] = -1;
//       gl_TessLevelInner[1] = -1;
//
//     } else {
//
//       vec2 s0 = toScreen(p0);
//       vec2 s1 = toScreen(p1);
//       vec2 s2 = toScreen(p2);
//       vec2 s3 = toScreen(p3);
//
//
//       // float l0 = length(p2.xy - p0.xy) * 16.0 + 1.0;
//       // float l1 = length(p3.xy - p2.xy) * 16.0 + 1.0;
//       // float l2 = length(p3.xy - p1.xy) * 16.0 + 1.0;
//       // float l3 = length(p1.xy - p0.xy) * 16.0 + 1.0;
//
//       float l0 = calcEdgeTessellation(s2, s0);
//       float l1 = calcEdgeTessellation(s3, s2);
//       float l2 = calcEdgeTessellation(s3, s1);
//       float l3 = calcEdgeTessellation(s1, s0);
//
//       gl_TessLevelOuter[0] = l0;
//       gl_TessLevelOuter[1] = l1;
//       gl_TessLevelOuter[2] = l2;
//       gl_TessLevelOuter[3] = l3;
//
//       // gl_TessLevelOuter[0] = 2;
//       // gl_TessLevelOuter[1] = 2;
//       // gl_TessLevelOuter[2] = 2;
//       // gl_TessLevelOuter[3] = 2;
//
//       // gl_TessLevelOuter[0] = 1;
//       // gl_TessLevelOuter[1] = 1;
//       // gl_TessLevelOuter[2] = 1;
//       // gl_TessLevelOuter[3] = 1;
//
//
//       gl_TessLevelInner[0] = min(l1, l3);
//       gl_TessLevelInner[1] = min(l0, l2);
//
//       // gl_TessLevelInner[0] = 2;
//       // gl_TessLevelInner[1] = 2;
//
//       // gl_TessLevelInner[0] = 1;
//       // gl_TessLevelInner[1] = 1;
//
//       // gl_TessLevelInner[0] = 2;
//       // gl_TessLevelInner[1] = 2;
//
//       // vec4 v0 = project(gl_in[0].gl_Position);
//       // vec4 v1 = project(gl_in[1].gl_Position);
//       // vec4 v2 = project(gl_in[2].gl_Position);
//       // vec4 v3 = project(gl_in[3].gl_Position);
//       //
//       // vec2 ss0 = screen_space(v0);
//       // vec2 ss1 = screen_space(v1);
//       // vec2 ss2 = screen_space(v2);
//       // vec2 ss3 = screen_space(v3);
//       //
//       // float e0 = level(ss1, ss2);
//       // float e1 = level(ss0, ss1);
//       // float e2 = level(ss3, ss0);
//       // float e3 = level(ss2, ss3);
//       //
//       // gl_TessLevelInner[0] = mix(e1, e2, 0.5);
//       // gl_TessLevelInner[1] = mix(e0, e3, 0.5);
//       // gl_TessLevelOuter[0] = e0;
//       // gl_TessLevelOuter[1] = e1;
//       // gl_TessLevelOuter[2] = e2;
//       // gl_TessLevelOuter[3] = e3;
//
//
//     }
//   }

  gl_out[gl_InvocationID].gl_Position =
    gl_in[gl_InvocationID].gl_Position;

  tcs_out[gl_InvocationID].tc =
    tcs_in[gl_InvocationID].tc;
}

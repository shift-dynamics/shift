#version 420 core
// #extension GL_ARB_tessellation_shader : enable

layout (vertices = 4) out;

layout (binding = 0) uniform sampler2D texture_displacement_u;
layout (binding = 1) uniform sampler2D texture_color_u;

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat4 p;
  mat3 nm;
};

uniform matrices_t matrices_u;

struct material_t {
  vec4 Kd;
  float KaOverKd;
  float KsOverKd;
  int Ns;
};

uniform material_t material_u;

struct lights_t {
  vec3 position[10];
  vec3 Ka[10];
  vec3 Kd[10];
  vec3 Ks[10];
  int directional[10];
  int count;
};

uniform lights_t lights_u;

struct terrain_t {
  float zMin;
  float zMax;
  float xScale;
  float yScale;
  float xRange;
  float yRange;
  vec2 viewport;
};

uniform terrain_t terrain_u;

in VS_OUT {
  vec2 tc;
} tcs_in[];

out TCS_OUT {
  vec2 tc;
} tcs_out[];


float distTess(vec4 p0, vec4 p1, vec2 t0, vec2 t1) {

	// g_tessellatedTriWidth is desired pixels per tri edge
	float g_tessellatedTriWidth = 50.0;

  p0.z += (texture(texture_displacement_u, t0).r * (terrain_u.zMax - terrain_u.zMin)) + terrain_u.zMin;
  p1.z += (texture(texture_displacement_u, t1).r * (terrain_u.zMax - terrain_u.zMin)) + terrain_u.zMin;

  p0.z *= 10.0;   /* delete this once verified its working */
  p1.z *= 10.0;   /* delete this once verified its working */

	vec4 center = 0.5 * (p0 + p1);
	float radius = distance(p0, p1) / 2.0;

	vec4 sc0 = matrices_u.mv * center;
	vec4 sc1 = sc0;
	sc0.x -= radius;
	sc1.x += radius;

	vec4 clip0 = matrices_u.p * sc0;
	vec4 clip1 = matrices_u.p * sc1;

	clip0 /= clip0.w;
	clip1 /= clip1.w;

	clip0.xy *= terrain_u.viewport;
	clip1.xy *= terrain_u.viewport;

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

  gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;

  tcs_out[gl_InvocationID].tc = tcs_in[gl_InvocationID].tc;
}

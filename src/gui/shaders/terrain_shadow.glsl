#version 420 core

/* terrain_shadow.vert */
layout (binding = 1) uniform sampler2D texture_elevation_u;

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat4 p;
};
uniform matrices_t matrices_u;

struct terrain_t {
  float zMin;
  float zMax;
  vec2 scale;
  vec2 viewport;
};
uniform terrain_t terrain_u;

#ifdef VERTEX_SHADER
out VS_OUT {
  vec2 tc;
} vs_out;

void main(void) {
  const vec4 vertices[] = vec4[](
    vec4(0.0, 0.0, 0.0, 1.0),
    vec4(1.0, 0.0, 0.0, 1.0),
    vec4(0.0, 1.0, 0.0, 1.0),
    vec4(1.0, 1.0, 0.0, 1.0));

  ivec2 texture_size = textureSize(texture_elevation_u, 0);
  int x = gl_InstanceID % texture_size.x; // row index into elevation map
  int y = gl_InstanceID / texture_size.x; // column index into elevation map
  vs_out.tc = (vertices[gl_VertexID].xy + vec2(x, y)) / texture_size;
  gl_Position = vertices[gl_VertexID] + vec4(float(x), float(y), 0.0, 0.0);
}
#endif


#ifdef TESSELATION_CONTROL_SHADER
layout (vertices = 4) out;

in VS_OUT {
  vec2 tc;
} tcs_in[];

float distTess(vec4 p0, vec4 p1, vec2 t0, vec2 t1) {
	// g_tessellatedTriWidth is desired pixels per tri edge
	float g_tessellatedTriWidth = 50.0;

  p0.z += (texture(texture_elevation_u, t0).r * (terrain_u.zMax - terrain_u.zMin)) + terrain_u.zMin;
  p1.z += (texture(texture_elevation_u, t1).r * (terrain_u.zMax - terrain_u.zMin)) + terrain_u.zMin;

  // p0.z *= 10.0;   /* delete this once verified its working */
  // p1.z *= 10.0;   /* delete this once verified its working */

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
}
#endif


#ifdef TESSELATION_EVALUATION_SHADER
layout(quads, fractional_odd_spacing) in;
in TCS_OUT {
  vec2 tc;
} tes_in[];

out TES_OUT {
  vec3 world_coord;
  vec3 eye_coord;
} tes_out;

void main(void) {
  vec2 tc1 = mix(tes_in[0].tc, tes_in[1].tc, gl_TessCoord.x);
  vec2 tc2 = mix(tes_in[2].tc, tes_in[3].tc, gl_TessCoord.x);
  vec2 tc = mix(tc2, tc1, gl_TessCoord.y);
  vec4 p1 = mix(gl_in[0].gl_Position, gl_in[1].gl_Position, gl_TessCoord.x);
  vec4 p2 = mix(gl_in[2].gl_Position, gl_in[3].gl_Position, gl_TessCoord.x);

  float f11 = gl_in[0].gl_Position.z;
  float f21 = gl_in[1].gl_Position.z;
  float f12 = gl_in[2].gl_Position.z;
  float f22 = gl_in[3].gl_Position.z;

  float a11 = f11;
  float a21 = f21 - f11;
  float a12 = f12 - f11;
  float a22 = f11 - f21 - f12 + f22;

  float x1 = gl_in[0].gl_Position.x;
  float x2 = gl_in[1].gl_Position.x;
  float y1 = gl_in[0].gl_Position.y;
  float y2 = gl_in[2].gl_Position.y;

  vec4 position = mix(p2, p1, gl_TessCoord.y);

  position.x *= terrain_u.scale.x;
  position.y *= terrain_u.scale.y;
  position.z =
    texture(texture_elevation_u, tc).r *
    (terrain_u.zMax - terrain_u.zMin) + terrain_u.zMin;
  gl_Position = matrices_u.mvp * position;

  vec4 position_eye = matrices_u.mv * position;
  tes_out.world_coord = position.xyz;
  tes_out.eye_coord = position_eye.xyz;
}
#endif


#ifdef FRAGMENT_SHADER
in vec4 positionFrag;
out float fragmentDepth;

void main(void) {
  fragmentDepth = gl_FragCoord.z;
}
#endif

#version 420 core

/* terrain_shadow.tese */

layout (quads, fractional_odd_spacing) in;

layout (binding = 0) uniform sampler2DArrayShadow texture_shadow_u;

layout (binding = 1) uniform sampler2D texture_displacement_u;

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
  float xScale;
  float yScale;
  float xRange;
  float yRange;
  vec2 viewport;
};

uniform terrain_t terrain_u;

in TCS_OUT {
  vec2 tc;
} tes_in[];

out TES_OUT {
  vec2 tc;
  vec3 world_coord;
  vec3 eye_coord;
} tes_out;

void main(void) {

    vec2 tc1 =
      mix(
        tes_in[0].tc,
        tes_in[1].tc,
        gl_TessCoord.x);

    vec2 tc2 =
      mix(
        tes_in[2].tc,
        tes_in[3].tc,
        gl_TessCoord.x);

    vec2 tc =
      mix(
        tc2,
        tc1,
        gl_TessCoord.y);

    vec4 p1 =
      mix(
        gl_in[0].gl_Position,
        gl_in[1].gl_Position,
        gl_TessCoord.x);

    vec4 p2 =
      mix(
        gl_in[2].gl_Position,
        gl_in[3].gl_Position,
        gl_TessCoord.x);

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

    vec4 position =
      mix(
        p2,
        p1,
        gl_TessCoord.y);

    position.z = (texture(texture_displacement_u, tc).r * (terrain_u.zMax - terrain_u.zMin)) + terrain_u.zMin;

    vec4 position_eye = matrices_u.mv * position;

    tes_out.tc = tc;
    tes_out.world_coord = position.xyz;
    tes_out.eye_coord = position_eye.xyz;

    gl_Position = matrices_u.mvp * position;
}

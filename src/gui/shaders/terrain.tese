#version 420 core

/* terrain.tese */

layout (quads, fractional_odd_spacing) in;
layout (binding = 0) uniform sampler2DArrayShadow texture_shadow_u;
layout (binding = 1) uniform sampler2D texture_displacement_u;
layout (binding = 2) uniform sampler2D texture_color_u;

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
  int drawShadows; /*!< 0 = false, 1 = true */
  mat4 shadowMatrix[10];
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

in TCS_OUT {
  vec2 tc;
} tes_in[];

out TES_OUT {
  vec2 tc;
  vec3 normal;
  vec3 world_coord;
  vec3 eye_coord;
} tes_out;

out vec4 shadowCoord[10];

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

    vec3 n =
      normalize(vec3(
        (a21 + a22 * gl_TessCoord.y) / (x2 - x1),
        (a12 + a22 * gl_TessCoord.x) / (y2 - y1),
        -1.0));

    vec4 position =
      mix(
        p2,
        p1,
        gl_TessCoord.y);

    position.z = (texture(texture_displacement_u, tc).r * (terrain_u.zMax - terrain_u.zMin)) + terrain_u.zMin;

    vec4 position_eye = matrices_u.mv * position;

    tes_out.tc = tc;
    tes_out.normal = matrices_u.nm * n;
    tes_out.world_coord = position.xyz;
    tes_out.eye_coord = position_eye.xyz;

    gl_Position = matrices_u.mvp * position;

    if(lights_u.drawShadows == 1) {
      for(int i = 0; i < lights_u.count; i++) {
        shadowCoord[i] =
          lights_u.shadowMatrix[i] * position;
      }
    }
}

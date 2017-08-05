#version 420 core

layout (quads, fractional_odd_spacing) in;

uniform sampler2D tex_displacement;

uniform mat4 mvp;
uniform mat4 mv;
uniform mat4 proj_matrix;
uniform mat3 nm;
uniform float zMax;
uniform float zMin;

in TCS_OUT {
  vec2 tc;
} tes_in[];

out TES_OUT {
  vec2 tc;
  vec3 normal;
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

    vec3 n =
      normalize(vec3(
        (a21 + a22 * gl_TessCoord.y) / (x2 - x1),
        (a12 + a22 * gl_TessCoord.x) / (y2 - y1),
        -1.0));

    vec4 p =
      mix(
        p2,
        p1,
        gl_TessCoord.y);

    p.z = (texture(tex_displacement, tc).r * (zMax - zMin)) + zMin;

    vec4 p_eye = mv * p;

    tes_out.tc = tc;
    tes_out.normal = nm * n;
    tes_out.world_coord = p.xyz;
    tes_out.eye_coord = p_eye.xyz;

    gl_Position = mvp * p;
}

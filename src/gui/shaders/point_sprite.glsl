#version 330
#extension GL_ARB_separate_shader_objects : enable
/* point_sprite.vert */

struct matrices_t {
  mat4 mvp;
  mat4 mv;
};
uniform matrices_t matrices_u;
uniform sampler2D texture_sprite_u;


#ifdef VERTEX_SHADER
layout (location = 0) in vec4 position_a;
layout (location = 1) in vec4 color_a;
out vec4 color;

void main() {
  color = color_a;
  gl_PointSize = max(10.0 / abs((matrices_u.mv * position_a).z), 1.0);
  gl_Position = matrices_u.mvp * position_a;
}
#endif


#ifdef FRAGMENT_SHADER
in vec4 color;
out vec4 fragColor;

void main() {
	vec4 texel = texture2D(texture_sprite_u, gl_PointCoord);
	if(texel.a < 0.1) {
		discard;
	}
	fragColor = texel * color;
}
#endif

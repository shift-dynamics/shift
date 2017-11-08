#version 330

/* point_sprite.frag */

struct matrices_t {
  mat4 mvp;
  mat4 mv;
};
uniform matrices_t matrices_u;
uniform sampler2D texture_sprite_u;

in vec4 color;

out vec4 fragColor;

void main() {
	vec4 texel = texture2D(texture_sprite_u, gl_PointCoord);
	if(texel.a < 0.1) {
		discard;
	}
	fragColor = texel * color;
}

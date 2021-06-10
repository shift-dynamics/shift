#version 420
#extension GL_ARB_separate_shader_objects : enable
/* point_sprite.vert */

struct matrices_t {
  mat4 mvp;
  mat4 mv;
};

layout (std140, binding = 0)  uniform BufferBlock {
  matrices_t matrices_u;
};

layout (binding = 1) uniform sampler2D texture_sprite_u;


#ifdef VERTEX_SHADER
layout (location = 0) in vec4 position_a;
layout (location = 1) in vec4 color_a;

layout (location = 0) out vec4 color;

void main() {
  color = color_a;
  gl_PointSize = max(10.0 / abs((matrices_u.mv * position_a).z), 1.0);
  gl_Position = matrices_u.mvp * position_a;
}
#endif


#ifdef FRAGMENT_SHADER
layout (location = 0) in vec4 color;

layout(location = 0) out vec4 fragColor;

void main() {
	vec4 texel = texture(texture_sprite_u, gl_PointCoord);
	if(texel.a < 0.1) {
		discard;
	}
	fragColor = texel * color;
}
#endif

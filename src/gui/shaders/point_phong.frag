#version 330
#extension GL_ARB_separate_shader_objects : enable

/* point_phong.frag */

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
};

uniform matrices_t matrices_u;

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

uniform sampler2D texture_normal_u;

layout (location = 0) in vec4 position_u;
layout (location = 1) in vec4 color_u;

out vec4 colorFrag;

void main() {
  vec4 normal = texture2D(texture_normal_u, gl_PointCoord);
  if(normal.a < 0.9) {
    discard;
  }
  vec4 fragColor = vec4(0.0, 0.0, 0.0, 1.0);
  vec3 V = vec3(0.0, 0.0, 1.0);

  float KaMaterial = 0.3;
  float KsMaterial = 0.8;
  int Ns = 600;

  vec3 posLight = vec3(100.0, -100.0, 100.0);
  float KaLight = 0.4;
  float KdLight = 0.8;
  float KsLight = 0.8;

	vec3 L = normalize(mat3(matrices_u.v) * posLight);

	// float a = pow(2.0*(gl_PointCoord.x - 0.5), 2.0);
	// float b = pow(2.0*(gl_PointCoord.y - 0.5), 2.0);
	// float c = 1.0 - (a + b);

	// if(c < 0.0){
	// 	discard;
	// }

	/// ambient term
  fragColor.rgb += color_u.rgb * KaMaterial * KaLight;
  float LdotN = max(0.0, dot(L,normal.xyz));

  /// diffuse term
  fragColor.rgb += color_u.rgb * KdLight * LdotN;

  /// specular term
  vec3 R = -normalize(reflect(L, normal.xyz));
  fragColor.rgb += color_u.rgb * KsMaterial * KsLight * pow(max(0, dot(R, V)), Ns);

	colorFrag = fragColor;
}

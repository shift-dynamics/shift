#version 330

uniform mat4 mvp, mv;//, v;

//uniform vec4 posLight;
//uniform float KaLight;
//uniform float KdLight;
//uniform float KsLight;

//uniform float KaMaterial;
//uniform float KdMaterial;
//uniform float KsMaterial;
//uniform int NsMaterial;

uniform sampler2D tex;

in vec4 color;

out vec4 fragColor;

void main() 
{
	vec4 texel = texture2D(tex, gl_PointCoord);
	if(texel.a<0.1)
	{
		discard;
	}
	fragColor = texel * color;
}

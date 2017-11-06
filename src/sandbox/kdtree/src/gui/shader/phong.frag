#version 330

uniform mat4 mvp, mv, v;
uniform sampler2D tex;
///> light properties
//uniform vec3 posLight;
//uniform float KaLight;
//uniform float KdLight;
//uniform float KsLight;


///> material properties
//uniform vec3 materialColor;
//uniform float KaMaterial;
//uniform float KdMaterial;
//uniform float KsMaterial;
//uniform int Ns;

in vec4 color;
in vec4 position; ///< world position;

void main()
{
    vec4 normal = texture2D(tex, gl_PointCoord);
    if(normal.a < 0.9)
    {
        discard;
    }
    vec4 fragColor = vec4(0.0, 0.0, 0.0, 1.0);
    vec3 V = vec3(0.0, 0.0, 1.0);

    float KaMaterial = 0.5;
    float KdMaterial = 0.8;
    float KsMaterial = 0.8;
    int Ns = 600;


    vec3 posLight = vec3( 100.0, -100.0, 100.0 );
    float KaLight = 0.4;
    float KdLight = 0.8;
    float KsLight = 0.8;

    vec4 materialColor = color;

	vec3 L =  normalize( mat3(v) * posLight );

	// float a = pow(2.0*(gl_PointCoord.x - 0.5), 2.0);
	// float b = pow(2.0*(gl_PointCoord.y - 0.5), 2.0);
	// float c = 1.0 - (a + b);

	// if(c < 0.0){
	// 	discard;
	// }
  
  	/// ambient term
    fragColor.rgb += materialColor.rgb * KaMaterial * KaLight;
    float LdotN =  max(0.0, dot(L,normal.xyz));

    /// diffuse term
    fragColor.rgb += materialColor.rgb * KdMaterial * KdLight * LdotN;

    /// specular term
    vec3 R = -normalize(reflect(L,normal.xyz));
    fragColor.rgb += materialColor.rgb * KsMaterial * KsLight * pow( max(0, dot( R, V)), Ns);

	gl_FragColor = fragColor;
}

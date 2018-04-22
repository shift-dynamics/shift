#version 330

uniform mat4 mvp, mv, v;
uniform mat3 nm;

uniform vec3 posLight[10];
uniform int directional[10];
uniform vec3 KaLight[10]; 
uniform vec3 KdLight[10]; 
uniform vec3 KsLight[10];
uniform int numLights;

// material parameters
uniform float KaMaterial;
uniform float KdMaterial;
uniform float KsMaterial;
uniform int Ns;

// texture
uniform sampler2D tex;

in vec3 normal;
in vec4 position;
in vec2 texCoord;

out vec4 fragColor;

void main(){

    vec4 baseColor = texture( tex, texCoord );
	vec4 color = vec4(0.0, 0.0, 0.0, 1.0);
	vec3 V = vec3(0.0, 0.0, 1.0);

    for( int i=0; i<numLights; i++ )
    {
        vec3 L;
        switch( directional[i] )
        {
            case 0: 
            // 0= point light, posLight indicates position of light source
            //need to compute direction of incoming light
            {   
                vec4 temp = v * vec4(posLight[i].x, posLight[i].y, posLight[i].z, 1.0) 
                - mv * position;
                L = normalize( temp.xyz );
            }
            break;
            case 1: 
            // 1=directional light, posLight indicates direction of incoming light
            {
                vec4 temp = v * vec4( posLight[i].x, posLight[i].y, posLight[i].z, 1.0);
                L = normalize( temp.xyz );
            }
            break;
        }

        color.rgb += KaMaterial * baseColor.rgb * KaLight[i];          //ambient term
        float LdotN =  max(0.0, dot(L,normal));     
                          
        color.rgb += (KdMaterial * baseColor.rgb * KdLight[i]) * LdotN; //diffuse term

        vec3 R = -normalize(reflect(L,normal));       //specular term
        color.rgb += KsMaterial * baseColor.rgb * (KsLight[i] * pow( max(0, dot( R, V)), Ns));

    }
    fragColor = min(color, vec4(1.0, 1.0, 1.0, 1.0));

}
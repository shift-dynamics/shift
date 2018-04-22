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

in vec4 posAttr;
in vec3 normAttr;
in vec3 tanAttr;
in vec2 uvAttr;

out vec3 normal;
out vec4 position;
out vec2 texCoord;

void main(){
    normal = normalize( nm * normAttr );
	position = posAttr;
    texCoord = uvAttr;
    gl_Position = mvp* posAttr;
}

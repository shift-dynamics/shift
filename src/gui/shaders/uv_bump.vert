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
uniform sampler2D bumpTex;

in vec4 posAttr;
in vec3 normAttr;
in vec3 tanAttr;
in vec2 uvAttr;

out vec4 position;
out vec2 texCoord;
out mat3 TBN;

void main(){
  vec3 n = normalize( nm * normAttr );
  vec3 t = normalize( nm * tanAttr );
  vec3 b = normalize( cross( n, t ));
	mat3 TBN = transpose(mat3(t,b,n));

  gl_Position = mvp* posAttr;
	position = posAttr;
  texCoord = uvAttr;
}

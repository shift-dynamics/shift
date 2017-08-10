#version 330

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat3 nm;
};

uniform matrices_t matrices;

struct material_t {
  vec4 Ka;
  vec4 Kd;
  vec4 Ks;
  vec4 Ke;
  int Ns;
};

uniform material_t material;

struct lights_t {
  vec3 position[10];
  vec3 Ka[10];
  vec3 Kd[10];
  vec3 Ks[10];
  int directional[10];
  int count;
};

uniform lights_t lights;

in vec4 vertexFrag;
in vec3 normalFrag;

out vec4 fragColor;

void main() {

  vec4 color = vec4(0.0, 0.0, 0.0, 1.0);
  vec3 V = vec3(0.0, 0.0, 1.0);

  //int i = 0;
  for(int i = 0; i < lights.count; i++) {
    vec3 L;
    switch(directional[i]) {
      case 0: {
        // point light, posLight indicates position of light source
        //need to compute direction of incoming light
        vec4 temp = v * vec4(
          lights.position[i].x,
          lights.position[i].y,
          lights.position[i].z,
          1.0) - mv * vertexFrag;
        L = normalize(vec3(temp.x, temp.y, temp.z));
      }
      break;
      case 1: {
        //directional light, posLight indicates direction of incoming light
        L = normalize(mat3(v) * lights.position[i]);
      }
      break;
    }

    color.rgb += material.Ka.rgb * lights.Ka[i];          //ambient term
    float LdotN =  max(0.0, dot(L, normalFrag));
    color.rgb += material.Kd.rgb * lights.Kd[i] * LdotN; //diffuse term
    vec3 R = -normalize(reflect(L, normalFrag));       //specular term
    color.rgb += material.Ks.rgb * lights.Ks[i] * pow(max(0, dot(R, V)), Ns);

  }
  //fragColor = vec4(1.0);
  fragColor = min(vec4(color.rgb, materials.Ka.a), vec4(1.0, 1.0, 1.0, 1.0));
  //fragColor = vertexFrag;
  //fragColor = vec4(normalFrag.x, normalFrag.y, normalFrag.z, 1);

}

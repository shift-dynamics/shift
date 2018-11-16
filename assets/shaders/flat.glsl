#version 440
/* flat.glsl */

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat3 nm;
};

struct material_t {
  vec4 Ka;
  vec4 Kd;
  vec4 Ks;
  vec4 Ke;
  int Ns;
};

struct lights_t {
  vec3 position[10];
  vec3 Ka[10];
  vec3 Kd[10];
  vec3 Ks[10];
  int directional[10];
  int count;
};

layout(set=0, binding=0) uniform UniformBlock {
  matrices_t matrices_u;
  material_t material_u;
  lights_t lights_u;
};

#ifdef VERTEX_SHADER
layout (location = 0) in vec4 position_a;
layout (location = 1) in vec3 normal_a;

layout(location = 0) out vec4 vertexGeom;

void main() {
  vertexGeom = position_a;
  gl_Position = matrices_u.mvp * position_a;
}
#endif


#ifdef GEOMETRY_SHADER
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

layout(location = 0) in vec4 vertexGeom[];

layout(location = 0) out vec4 vertexFrag;
layout(location = 1) out vec3 normalFrag;

void main() {
  vec4 a = vertexGeom[0];
  vec4 b = vertexGeom[1];
  vec4 c = vertexGeom[2];
  vec3 normal = matrices_u.nm * normalize(cross(b.xyz - a.xyz, c.xyz - b.xyz));

  for (int i = 0; i < gl_in.length(); i++) {
    vertexFrag = vertexGeom[i];
    normalFrag = normal;
    gl_Position = gl_in[i].gl_Position;
    EmitVertex();
  }
  EndPrimitive();
}
#endif


#ifdef FRAGMENT_SHADER
layout(location = 0) in vec4 vertexFrag;
layout(location = 1) in vec3 normalFrag;

layout(location = 0) out vec4 fragColor;

void main() {

  vec4 color = vec4(0.0, 0.0, 0.0, 1.0);
  vec3 V = vec3(0.0, 0.0, 1.0);

  //int i = 0;
  for(int i = 0; i < lights_u.count; i++) {
    vec3 L;
    switch(lights_u.directional[i]) {
      case 0: {
        // point light, posLight indicates position of light source
        //need to compute direction of incoming light
        vec4 temp = matrices_u.v * vec4(
          lights_u.position[i].x,
          lights_u.position[i].y,
          lights_u.position[i].z,
          1.0) - matrices_u.mv * vertexFrag;
        L = normalize(vec3(temp.x, temp.y, temp.z));
      }
      break;
      case 1: {
        //directional light, posLight indicates direction of incoming light
        L = normalize(mat3(matrices_u.v) * lights_u.position[i]);
      }
      break;
    }

    /* ambient term */
    color.rgb += material_u.Ka.rgb * lights_u.Ka[i];
    float LdotN =  max(0.0, dot(L, normalFrag));

    /* diffuse term */
    color.rgb += material_u.Kd.rgb * lights_u.Kd[i] * LdotN;

    /* specular term */
    vec3 R = -normalize(reflect(L, normalFrag));
    color.rgb +=
      material_u.Ks.rgb * lights_u.Ks[i] * pow(max(0, dot(R, V)), material_u.Ns);

  }
  //fragColor = vec4(1.0);
  fragColor = min(vec4(color.rgb, material_u.Ka.a), vec4(1.0, 1.0, 1.0, 1.0));
  //fragColor = vertexFrag;
  //fragColor = vec4(normalFrag.x, normalFrag.y, normalFrag.z, 1);
}
#endif

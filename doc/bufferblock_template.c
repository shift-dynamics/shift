/* common buffer block layout to be shared between
all shaders */

struct bufferblock_template_t {
  struct matrices_t {
    float mvp[16];
    float mv[16];
    float v[16];
    float p[16];
    float nm[16];
    float scale;
  } matrices_u;

  struct material_t {
    float Ka[4];
    float Kd[4];
    float Ks[4];
    float Ke[4];
    int Ns;
  } material_u;

  struct lights_t {
    float shadowMatrix[16];
    float position[4];
    float Ka[4];
    float Kd[4];
    float Ks[4];
    int directional;
  } lights_u[10];

  struct flags_t {
    int lightCount;
    int enableShadows;
    int enableDiffuseMap;
    int enableNormalMap;
    int enableSpecularMap;
    int enableBumpMap;
  } flags_u;
} bufferblock;

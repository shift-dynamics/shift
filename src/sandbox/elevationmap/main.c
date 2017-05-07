#include "../../ksl/ksl.h"
#include "../../core/elevationmap.h"

int main(int argc, char** argv) {

  generateSampleHeightMap("sinxx+cosyy.tiff");
  printf("wrote sample image 'sinxx+cosyy.tiff'\n");

  s2_elevationmap_t* elevationmap =
    s2_elevationmap("sinxx+cosyy.tiff",
    NULL,
    NULL,
    BICUBIC,
    SAE,
    15.0,
    30.0,
    0.2,
    0.3);

  double x = 23.09;
  double y = 41.5;
  s2_vector_t normal = {0.0, 0.0, 0.0};
  printf(
    "bicubic interpolation: z at (%f, %f) = %f ",
    x, y, elevationmap->getZ(elevationmap, x, y, &normal));
  printf("normal: [%f, %f, %f]\n", normal.x, normal.y, normal.z);


  s2_elevationmap_t* elevationmap2 =
    s2_elevationmap("sinxx+cosyy.tiff", NULL, NULL, BILINEAR, SAE, 15.0, 30.0, 0.2, 0.3);

  s2_vector_t normal2 = {0.0, 0.0, 0.0};
  printf(
    "bilinear interpolation: z at (%f, %f) = %f ",
    x, y, elevationmap2->getZ(elevationmap2, x, y, &normal2));
  printf("normal: [%f, %f, %f]\n", normal2.x, normal2.y, normal2.z);


  return 0;
}

#include "../../ksl/ksl.h"
#include "../../core/terrain.h"

#include <stdlib.h>
#include <string.h>

/*!
@brief generate a simple surface used to test elevation maps
*/
void surface(double x, double y, double* z) {
  *z = ((20.0 * sin(y / 5.0 - 2.0) * (pow(x, 0.5) - 5.0) / 50.0)) + 2.0;
}


double* terrainCreateArray(
  int rows,
  int columns,
  void (*surface)(double x, double y, double* z)) {

  double* controlPoints = calloc(rows * columns, sizeof(double));
  for(int y = 0; y < rows; y++) {
    for(int x = 0; x < columns; x++) {
      surface(x, y, controlPoints + y * rows + x);
    }
  }
  return controlPoints;
}


bool terrainCompare(s2_terrain_t* terrain, double* controlPoints) {
  for(int y = 0; y < terrain->rows; y++) {
    for(int x = 0; x < terrain->columns; x++) {
      if(fabs(terrain->controlPoints[y * terrain->rows + x] -
              controlPoints[y * terrain->rows + x]) > 1e-6) {
        fprintf(
          stderr,
          "   [x=%d, y=%d] %f != %f\n",
          x, y,
          terrain->controlPoints[y * terrain->rows + x],
          controlPoints[y * terrain->rows + x]);
        return false;
      }
    }
  }
  return true;;
}


int main(int argc, char** argv) {

  /* generate sample terrain */
  s2_generateGDALSampleTerrain("sinxx+cosyy.tiff", 64, 64, surface);
  printf("wrote sample image 'sinxx+cosyy.tiff'\n");

  double* controlPoints = terrainCreateArray(64, 64, surface);

  s2_terrain_t* terrain = s2_terrain();
  double scale[2] = {1.0, 1.0};
  s2_terrain_setElevationMap(terrain, "sinxx+cosyy.tiff", scale);

  bool matches = terrainCompare(terrain, controlPoints);

  if(!matches) {
    fprintf(stderr, "control points do not match after reloading data from file\n");
  }
  free(controlPoints);

  //
  // double x = 23.09;
  // double y = 41.5;
  // s2_vector_t normal = {0.0, 0.0, 0.0};
  // printf(
  //   "bicubic interpolation: z at (%f, %f) = %f ",
  //   x, y, elevationmap->getZ(elevationmap, x, y, &normal));
  // printf("normal: [%f, %f, %f]\n", normal.x, normal.y, normal.z);


  // s2_elevationmap_t* elevationmap2 =
  //   s2_elevationmap("sinxx+cosyy.tiff", NULL, NULL, BILINEAR, SAE, 15.0, 30.0, 0.2, 0.3);
  //
  // s2_vector_t normal2 = {0.0, 0.0, 0.0};
  // printf(
  //   "bilinear interpolation: z at (%f, %f) = %f ",
  //   x, y, elevationmap2->getZ(elevationmap2, x, y, &normal2));
  // printf("normal: [%f, %f, %f]\n", normal2.x, normal2.y, normal2.z);


  return 0;
}

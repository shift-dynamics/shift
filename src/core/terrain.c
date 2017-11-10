#include <float.h>
#include <jansson.h>
#include <string.h>
#include <limits.h>
#include <unistd.h>

#include "terrain.h"
#include "body.h"


/*!
@brief generate a simple surface used to test elevation maps
*/
double surface(double x, double y) {
  // return sin((y - 10.0) * (pow(x, 0.5) - 5.0) / 500.0) * 3.0 + 2.0;

  return ((20.0 * sin(y / 5.0 - 2.0) * (pow(x, 0.5) - 5.0) / 50.0)) + 2.0;
}


/*! @brief generates a sample terrain using surface function
saves to 32 bit tiff floating point format.  */
void generateSampleTerrain(const char* filename) {

  MagickWandGenesis();
  ssize_t y;
  register ssize_t x;
  size_t width;

  MagickBooleanType status;
  MagickWand* image = NewMagickWand();
	PixelWand* pixelWand = NewPixelWand();

  status = MagickNewImage(image, 64, 64, pixelWand);
  if(status == MagickFalse) {
    fprintf(stderr, "failed to set size\n");
  }

  status = MagickSetDepth(image, 32);
  if(status == MagickFalse) {
    fprintf(stderr, "failed to set quantum depth\n");
  }

  status = MagickSetType(image, GrayscaleType);
  if(status == MagickFalse) {
    fprintf(stderr, "failed to set GrayScaleType\n");
  }

  MagickSetOption(image, "quantum:format", "floating-point");

  /* image magick convention is that y is the left ordinate of view,
  i.e. the row index. x is the top ordinate of the view. Therefore, z
  is pointing down. This follows the SAE coordinate system for vehicle dynamics.
   */
  PixelIterator* iterator = NewPixelIterator(image);
  if (iterator == (PixelIterator *) NULL) {
    ThrowWandException(image);
  }

  for(y = 0; y < (ssize_t) MagickGetImageHeight(image); y++) {

    PixelWand** pixels = PixelGetNextIteratorRow(iterator, &width);
    if(pixels == (PixelWand **) NULL) {
      break;
    }

    for(x = 0; x < (ssize_t) width; x++) {
      PixelSetRedQuantum(pixels[x], surface(x, y));
      // printf("%f -> %f\n",  surface(x, y), PixelGetRedQuantum(pixels[x]));
    }

    (void) PixelSyncIterator(iterator);
  }

  if (y < (long) MagickGetImageHeight(image)) {
    ThrowWandException(image);
  }

  /* Write the image then destroy it. */
  status = MagickWriteImages(image, filename, MagickTrue);
  if(status == MagickFalse) {
    ThrowWandException(image);
  }

  iterator = DestroyPixelIterator(iterator);
  image = DestroyMagickWand(image);
  MagickWandTerminus();
}


int clamp_int(int val, int min, int max) {
  if(val < min) {
    return min;
  } else if(val >= max) {
    return max;
  } else {
    return val;
  }
}


/*!
@brief interpolate between min and max. f is a value from 0 to 1. sampling points are assumed to be distance 1 apart
*/
double blend(double f, double min, double max) {
  if(f < 1e-9) {
    return min;
  } else if(f >= 1.0) {
    return max;
  } else {
    return (max - min) * f + min;
  }
}


/*!  get value at specified row and column index.  */
double s2_terrain_get(
  s2_terrain_t* terrain,
  int i,
  int j) {

  /* row index i is y dimension, column index j is x dimension */
  assert(i >= 0 && i < terrain->rows);
  assert(j >= 0 && j < terrain->columns);

  return terrain->controlPoints[i * terrain->columns + j];

}


/*!
@brief get z using bilinear interpolation at specified x and y position
following SAE convention with positive z "down". The normal is returned in
the "up" direction.

@TODO normal vector not computed yet
*/
double s2_terrain_getZ_bilinear_sae(
  s2_terrain_t* terrain,
  double x,
  double y,
  s2_vector_t* normal) {

  /* get x and y values in original height map */
  double xPos = (x - terrain->xOffset) / terrain->xScale;
  double yPos = (y - terrain->yOffset) / terrain->yScale;

  /* calculate i,j index into map. row index i is y dimension, column index j is x dimension */
  int j0 = clamp_int(floor(xPos), 0, terrain->columns);
  int j1 = clamp_int(j0 + 1, 0, terrain->columns);

  int i0 = clamp_int(floor(yPos), 0, terrain->rows);
  int i1 = clamp_int(i0 + 1, 0, terrain->rows);

  /* calculate fraction from i1,j1 to i2,j2 (value from 0 to 1) */
  double fx = xPos - j0;
  double fy = yPos - i0;

  double z =
    blend(
      fx,
      blend(
        fy,
        s2_terrain_get(terrain, i0, j0),
        s2_terrain_get(terrain, i1, j0)),
      blend(
        fy,
        s2_terrain_get(terrain, i0, j1),
        s2_terrain_get(terrain, i1, j1)));
  return z;
}


/*!
@brief get z using bilinear interpolation at specified x and y position
following ISO convention with positive z "up". The normal is returned in
the "up" direction.

@TODO normal vector not computed yet
*/
double s2_terrain_getZ_bilinear_iso(
  s2_terrain_t* terrain,
  double x,
  double y,
  s2_vector_t* normal) {

  /* get x and y values in original height map */
  double xPos = (x - terrain->xOffset) / terrain->xScale;
  double yPos = (y - terrain->yOffset) / terrain->yScale;

  /* calculate i,j index into map. row index i is x dimension, column index j is y dimension */
  int i0 = clamp_int(floor(xPos), 0, terrain->rows);
  int i1 = clamp_int(i0 + 1, 0, terrain->rows);

  int j0 = clamp_int(floor(yPos), 0, terrain->columns);
  int j1 = clamp_int(j0 + 1, 0, terrain->columns);

  /* calculate fraction from i1,j1 to i2,j2 (value from 0 to 1) */
  double fx = xPos - i1;
  double fy = yPos - j1;

  double z =
    blend(
      fy,
      blend(
        fx,
        s2_terrain_get(terrain, i0, j0),
        s2_terrain_get(terrain, i1, j0)),
      blend(
        fx,
        s2_terrain_get(terrain, i0, j1),
        s2_terrain_get(terrain, i1, j1)));

  return z;
}


/*!
@brief bicubic interpolation following sae convention with positive z down.

Surface normal is returned in the -z (up) direction.

*/
double s2_terrain_getZ_bicubic_sae(
  s2_terrain_t* terrain,
  double x,
  double y,
  s2_vector_t* normal) {


  /* get x and y values in original height map */
  double xPos = (x - terrain->xOffset) / terrain->xScale;
  double yPos = (y - terrain->yOffset) / terrain->yScale;

  printf("xPos: %f, yPos: %f\n", xPos, yPos);
  printf(
    "rows: %ld, columns: %ld\n",
    terrain->rows,
    terrain->columns);

  /* calculate i,j index into map. row index i is y dimension, column index j is x dimension */

  int i1 = clamp_int(floor(yPos), 0, terrain->rows);
  int i2 = clamp_int(i1 + 1, 0, terrain->rows);
  int i3 = clamp_int(i1 + 2, 0, terrain->rows);
  int i0 = clamp_int(i1 - 1, 0, terrain->rows);

  double u = yPos - i1;
  double u2 = u * u;
  double u3 = u2 * u;

  /* calculate fraction from i1,j1 to i2,j2 (a value from 0 to 1) */
  int j1 = clamp_int(floor(xPos), 0, terrain->columns);
  int j2 = clamp_int(j1 + 1, 0, terrain->columns);
  int j3 = clamp_int(j1 + 2, 0, terrain->columns);
  int j0 = clamp_int(j1 - 1, 0, terrain->columns);

  double v = xPos - j1;
  double v2 = v * v;
  double v3 = v2 * v;

  double wi0 = -u3 / 6.0 + 0.5 * (u2 - u)  + 1.0 / 6.0;
  double wi1 = u3 * 0.5 - u2 + 4.0 / 6.0;
  double wi2 = 0.5 * (-u3 + u2 + u) + 1.0 / 6.0;
  double wi3 = u3 / 6.0;

  double wj0 = -v3 / 6.0 + 0.5 * (v2 - v)  + 1.0 / 6.0;
  double wj1 = v3 * 0.5 - v2 + 4.0 / 6.0;
  double wj2 = 0.5 * (-v3 + v2 + v) + 1.0 / 6.0;
  double wj3 = v3 / 6.0;

  double z =
    wi0 * s2_terrain_get(terrain, i0, j0) * wj0 +
    wi1 * s2_terrain_get(terrain, i1, j0) * wj0 +
    wi2 * s2_terrain_get(terrain, i2, j0) * wj0 +
    wi3 * s2_terrain_get(terrain, i3, j0) * wj0 +

    wi0 * s2_terrain_get(terrain, i0, j1) * wj1 +
    wi1 * s2_terrain_get(terrain, i1, j1) * wj1 +
    wi2 * s2_terrain_get(terrain, i2, j1) * wj1 +
    wi3 * s2_terrain_get(terrain, i3, j1) * wj1 +

    wi0 * s2_terrain_get(terrain, i0, j2) * wj2 +
    wi1 * s2_terrain_get(terrain, i1, j2) * wj2 +
    wi2 * s2_terrain_get(terrain, i2, j2) * wj2 +
    wi3 * s2_terrain_get(terrain, i3, j2) * wj2 +

    wi0 * s2_terrain_get(terrain, i0, j3) * wj3 +
    wi1 * s2_terrain_get(terrain, i1, j3) * wj3 +
    wi2 * s2_terrain_get(terrain, i2, j3) * wj3 +
    wi3 * s2_terrain_get(terrain, i3, j3) * wj3;

  /* compute partial derivative of [u^3, u^2, u, 1] with respect to u */
  double wi0_u = -0.5 * u2 + u - 0.5;
  double wi1_u = 1.5 * u2 - 2.0 * u;
  double wi2_u = -1.5 * u2 + u + 0.5;
  double wi3_u = 0.5 * u2;

  double z_y =
    wi0_u * s2_terrain_get(terrain, i0, j0) * wj0 +
    wi1_u * s2_terrain_get(terrain, i1, j0) * wj0 +
    wi2_u * s2_terrain_get(terrain, i2, j0) * wj0 +
    wi3_u * s2_terrain_get(terrain, i3, j0) * wj0 +

    wi0_u * s2_terrain_get(terrain, i0, j1) * wj1 +
    wi1_u * s2_terrain_get(terrain, i1, j1) * wj1 +
    wi2_u * s2_terrain_get(terrain, i2, j1) * wj1 +
    wi3_u * s2_terrain_get(terrain, i3, j1) * wj1 +

    wi0_u * s2_terrain_get(terrain, i0, j2) * wj2 +
    wi1_u * s2_terrain_get(terrain, i1, j2) * wj2 +
    wi2_u * s2_terrain_get(terrain, i2, j2) * wj2 +
    wi3_u * s2_terrain_get(terrain, i3, j2) * wj2 +

    wi0_u * s2_terrain_get(terrain, i0, j3) * wj3 +
    wi1_u * s2_terrain_get(terrain, i1, j3) * wj3 +
    wi2_u * s2_terrain_get(terrain, i2, j3) * wj3 +
    wi3_u * s2_terrain_get(terrain, i3, j3) * wj3;

  z_y /= terrain->yScale;

  /* compute partial derivative of [v^3,v^2, v, 1] with respect to v */
  double wj0_v = -0.5 * v2 + v - 0.5;
  double wj1_v = 1.5 * v2 - 2.0 * v;
  double wj2_v = -1.5 * v2 + v + 0.5;
  double wj3_v = 0.5 * v2;

  double z_x =
    wi0 * s2_terrain_get(terrain, i0, j0) * wj0_v +
    wi1 * s2_terrain_get(terrain, i1, j0) * wj0_v +
    wi2 * s2_terrain_get(terrain, i2, j0) * wj0_v +
    wi3 * s2_terrain_get(terrain, i3, j0) * wj0_v +

    wi0 * s2_terrain_get(terrain, i0, j1) * wj1_v +
    wi1 * s2_terrain_get(terrain, i1, j1) * wj1_v +
    wi2 * s2_terrain_get(terrain, i2, j1) * wj1_v +
    wi3 * s2_terrain_get(terrain, i3, j1) * wj1_v +

    wi0 * s2_terrain_get(terrain, i0, j2) * wj2_v +
    wi1 * s2_terrain_get(terrain, i1, j2) * wj2_v +
    wi2 * s2_terrain_get(terrain, i2, j2) * wj2_v +
    wi3 * s2_terrain_get(terrain, i3, j2) * wj2_v +

    wi0 * s2_terrain_get(terrain, i0, j3) * wj3_v +
    wi1 * s2_terrain_get(terrain, i1, j3) * wj3_v +
    wi2 * s2_terrain_get(terrain, i2, j3) * wj3_v +
    wi3 * s2_terrain_get(terrain, i3, j3) * wj3_v;

  z_x /= terrain->xScale;

  normal->x = z_x;
  normal->y = z_y;
  normal->z = -1.0;
  s2_vector_normalize(normal);

  return z;
}

/*!
@brief bicubic interpolation following iso convention with positive z up.

Surface normal is returned in the +z (up) direction.

*/
double s2_terrain_getZ_bicubic_iso(
  s2_terrain_t* terrain,
  double x,
  double y,
  s2_vector_t* normal) {


  /* get x and y values in original height map */
  double xPos = (x - terrain->xOffset) / terrain->xScale;
  double yPos = (y - terrain->yOffset) / terrain->yScale;

  /* calculate i,j index into map. row index i is y dimension, column index j is x dimension */

  int i1 = clamp_int(floor(xPos), 0, terrain->rows);
  int i2 = clamp_int(i1 + 1, 0, terrain->rows);
  int i3 = clamp_int(i1 + 2, 0, terrain->rows);
  int i0 = clamp_int(i1 - 1, 0, terrain->rows);

  double u = xPos - i1;
  double u2 = u * u;
  double u3 = u2 * u;

  /* calculate fraction from i1,j1 to i2,j2 (a value from 0 to 1) */
  int j1 = clamp_int(floor(yPos), 0, terrain->columns);
  int j2 = clamp_int(j1 + 1, 0, terrain->columns);
  int j3 = clamp_int(j1 + 2, 0, terrain->columns);
  int j0 = clamp_int(j1 - 1, 0, terrain->columns);

  double v = yPos - j1;
  double v2 = v * v;
  double v3 = v2 * v;

  double wi0 = -u3 / 6.0 + 0.5 * (u2 - u)  + 1.0 / 6.0;
  double wi1 = u3 * 0.5 - u2 + 4.0 / 6.0;
  double wi2 = 0.5 * (-u3 + u2 + u) + 1.0 / 6.0;
  double wi3 = u3 / 6.0;

  double wj0 = -v3 / 6.0 + 0.5 * (v2 - v)  + 1.0 / 6.0;
  double wj1 = v3 * 0.5 - v2 + 4.0 / 6.0;
  double wj2 = 0.5 * (-v3 + v2 + v) + 1.0 / 6.0;
  double wj3 = v3 / 6.0;

  double z =
    wi0 * s2_terrain_get(terrain, i0, j0) * wj0 +
    wi1 * s2_terrain_get(terrain, i1, j0) * wj0 +
    wi2 * s2_terrain_get(terrain, i2, j0) * wj0 +
    wi3 * s2_terrain_get(terrain, i3, j0) * wj0 +

    wi0 * s2_terrain_get(terrain, i0, j1) * wj1 +
    wi1 * s2_terrain_get(terrain, i1, j1) * wj1 +
    wi2 * s2_terrain_get(terrain, i2, j1) * wj1 +
    wi3 * s2_terrain_get(terrain, i3, j1) * wj1 +

    wi0 * s2_terrain_get(terrain, i0, j2) * wj2 +
    wi1 * s2_terrain_get(terrain, i1, j2) * wj2 +
    wi2 * s2_terrain_get(terrain, i2, j2) * wj2 +
    wi3 * s2_terrain_get(terrain, i3, j2) * wj2 +

    wi0 * s2_terrain_get(terrain, i0, j3) * wj3 +
    wi1 * s2_terrain_get(terrain, i1, j3) * wj3 +
    wi2 * s2_terrain_get(terrain, i2, j3) * wj3 +
    wi3 * s2_terrain_get(terrain, i3, j3) * wj3;

  /* compute partial derivative of [u^3, u^2, u, 1] with respect to u */
  double wi0_u = -0.5 * u2 + u - 0.5;
  double wi1_u = 1.5 * u2 - 2.0 * u;
  double wi2_u = -1.5 * u2 + u + 0.5;
  double wi3_u = 0.5 * u2;

  double z_x =
    wi0_u * s2_terrain_get(terrain, i0, j0) * wj0 +
    wi1_u * s2_terrain_get(terrain, i1, j0) * wj0 +
    wi2_u * s2_terrain_get(terrain, i2, j0) * wj0 +
    wi3_u * s2_terrain_get(terrain, i3, j0) * wj0 +

    wi0_u * s2_terrain_get(terrain, i0, j1) * wj1 +
    wi1_u * s2_terrain_get(terrain, i1, j1) * wj1 +
    wi2_u * s2_terrain_get(terrain, i2, j1) * wj1 +
    wi3_u * s2_terrain_get(terrain, i3, j1) * wj1 +

    wi0_u * s2_terrain_get(terrain, i0, j2) * wj2 +
    wi1_u * s2_terrain_get(terrain, i1, j2) * wj2 +
    wi2_u * s2_terrain_get(terrain, i2, j2) * wj2 +
    wi3_u * s2_terrain_get(terrain, i3, j2) * wj2 +

    wi0_u * s2_terrain_get(terrain, i0, j3) * wj3 +
    wi1_u * s2_terrain_get(terrain, i1, j3) * wj3 +
    wi2_u * s2_terrain_get(terrain, i2, j3) * wj3 +
    wi3_u * s2_terrain_get(terrain, i3, j3) * wj3;

  z_x /= terrain->xScale;

  /* compute partial derivative of [v^3,v^2, v, 1] with respect to v */
  double wj0_v = -0.5 * v2 + v - 0.5;
  double wj1_v = 1.5 * v2 - 2.0 * v;
  double wj2_v = -1.5 * v2 + v + 0.5;
  double wj3_v = 0.5 * v2;

  double z_y =
    wi0 * s2_terrain_get(terrain, i0, j0) * wj0_v +
    wi1 * s2_terrain_get(terrain, i1, j0) * wj0_v +
    wi2 * s2_terrain_get(terrain, i2, j0) * wj0_v +
    wi3 * s2_terrain_get(terrain, i3, j0) * wj0_v +

    wi0 * s2_terrain_get(terrain, i0, j1) * wj1_v +
    wi1 * s2_terrain_get(terrain, i1, j1) * wj1_v +
    wi2 * s2_terrain_get(terrain, i2, j1) * wj1_v +
    wi3 * s2_terrain_get(terrain, i3, j1) * wj1_v +

    wi0 * s2_terrain_get(terrain, i0, j2) * wj2_v +
    wi1 * s2_terrain_get(terrain, i1, j2) * wj2_v +
    wi2 * s2_terrain_get(terrain, i2, j2) * wj2_v +
    wi3 * s2_terrain_get(terrain, i3, j2) * wj2_v +

    wi0 * s2_terrain_get(terrain, i0, j3) * wj3_v +
    wi1 * s2_terrain_get(terrain, i1, j3) * wj3_v +
    wi2 * s2_terrain_get(terrain, i2, j3) * wj3_v +
    wi3 * s2_terrain_get(terrain, i3, j3) * wj3_v;

  z_y /= terrain->yScale;

  normal->x = -z_x;
  normal->y = -z_y;
  normal->z = 1.0;
  s2_vector_normalize(normal);

  return z;
}


/*!
@brief not implented yet
@TODO
*/
double s2_terrain_getZ_catmulrom_sae(
  s2_terrain_t* terrain,
  double x,
  double y,
  s2_vector_t* normal) {

  double z = 0;
  return z;
}


/*!
@brief not implented yet
@TODO
*/
double s2_terrain_getZ_catmulrom_iso(
  s2_terrain_t* terrain,
  double x,
  double y,
  s2_vector_t* normal) {

  double z = 0;
  return z;
}


/*!
@brief constructor for s2_terrain_t

*/
s2_terrain_t* s2_terrain(
  const char* terrainFileName,
  const char* textureFileName,
  const char* propertyMapFileName,
  s2_interpolation_enum_t interpolationType,
  s2_coordinate_convention_enum_t coordinateConvention,
  double xOffset,
  double yOffset,
  double xScale,
  double yScale) {

  MagickWandGenesis();
  long y;
  MagickPixelPacket pixel;
  register long x;
  unsigned long width;

  /* initialize elevation map, set all values to zero */
  s2_terrain_t* terrain =
    (s2_terrain_t*)calloc(1, sizeof(s2_terrain_t));
  terrain->controlPoints = NULL;
  terrain->texture = NULL;
  terrain->getZ = NULL;
  terrain->zMin = DBL_MAX;
  terrain->zMax = -DBL_MAX;

  /* load image using imagemagick, Magick::InitializeMagick() must be
  called prior to creating an terrain */
  MagickBooleanType status;
  MagickWand* image = NewMagickWand();
  status = MagickReadImage(image, terrainFileName);
  if (status == MagickFalse) {
    ThrowWandException(image);
  }

  unsigned long columns = MagickGetImageWidth(image);
  unsigned long rows = MagickGetImageHeight(image);

  printf(
    "Loading elevation map %s with size (%lu, %lu)\n",
    terrainFileName, rows, columns);

  terrain->controlPoints = calloc(rows * columns, sizeof(double));


  /* load values into array following sae convention with x as
  top ordinate of view (column index) and y as left ordinate of view
  (row index), and +z is pointing down */

  const char* option = MagickGetImageProperty(image, "quantum:format");
  printf("%s ", option);

  float depth = MagickGetImageDepth(image);
  printf("with depth: %f\n", depth);
  // MagickSetOption(image, "quantum:format", "floating-point");


  PixelIterator* iterator = NewPixelIterator(image);
  for(y = 0; y < (long) MagickGetImageHeight(image); y++) {

    PixelWand** pixels = PixelGetNextIteratorRow(iterator, &width);
    if(pixels == (PixelWand **) NULL) {
      break;
    }

    for(x = 0; x < (long) width; x++) {
      // PixelGetMagickColor(pixels[x], &pixel);
      // printf("%f\n", PixelGetRedQuantum(pixels[x]));

      terrain->controlPoints[y * columns + x] =
        (double)PixelGetRedQuantum(pixels[x]);
      terrain->zMax =
        fmax(
          terrain->controlPoints[y * columns + x],
          terrain->zMax);
      terrain->zMin =
        fmin(
          terrain->controlPoints[y * columns + x],
          terrain->zMin);
    }

    (void) PixelSyncIterator(iterator);
  }

  image = DestroyMagickWand(image);

  /* load texture */
  if(textureFileName != NULL) {
    MagickWand* texture = NewMagickWand();
    status = MagickReadImage(texture, textureFileName);
    if (status == MagickFalse) {
      ThrowWandException(texture);
    }

    size_t range;
    MagickGetQuantumRange(&range);
    printf("range: %lu\n", range);

    unsigned long textureColumns = MagickGetImageWidth(texture);
    unsigned long textureRows = MagickGetImageHeight(texture);

    printf(
      "Loading texture %s with size (%lu, %lu)\n",
      textureFileName,
      textureRows,
      textureColumns);

    terrain->texture = calloc(4 * textureRows * textureColumns, sizeof(unsigned char));

    /* load values into array following sae convention with x as
    top ordinate of view (column index) and y as left ordinate of view
    (row index), and +z is pointing down */

    PixelIterator* textureIterator = NewPixelIterator(texture);
    for(y = 0; y < (long) MagickGetImageHeight(texture); y++) {

      PixelWand** pixels = PixelGetNextIteratorRow(textureIterator, &width);
      if(pixels == (PixelWand **) NULL) {
        break;
      }

      for(x = 0; x < (long) width; x++) {
        // PixelGetMagickColor(pixels[x], &pixel);
        terrain->texture[4 * (y * textureColumns + x) + 0] =
          (unsigned char) (PixelGetRedQuantum(pixels[x]) / range * 255.0);
        terrain->texture[4 * (y * textureColumns + x) + 1] =
          (unsigned char) (PixelGetGreenQuantum(pixels[x]) / range * 255.0);
        terrain->texture[4 * (y * textureColumns + x) + 2] =
          (unsigned char) (PixelGetBlueQuantum(pixels[x]) / range * 255.0);
        // terrain->texture[4 * (y * columns + x) + 3] =
        //   (unsigned char) (PixelGetAlpha(pixels[x]) / range * 255.0);
        terrain->texture[4 * (y * textureColumns + x) + 3] = 255;
        // if(terrain->texture[4 * (y * columns + x) + 3]) {
          // printf(
          //   "[%lu, %lu]: %d, %d, %d, %d\n",
          //   y, x,
          //   terrain->texture[4 * (y * columns + x) + 0],
          //   terrain->texture[4 * (y * columns + x) + 1],
          //   terrain->texture[4 * (y * columns + x) + 2],
          //   terrain->texture[4 * (y * columns + x) + 3]);
        // }
      }

      (void) PixelSyncIterator(iterator);
    }

    texture = DestroyMagickWand(texture);
    terrain->textureColumns = textureColumns;
    terrain->textureRows = textureRows;
  } else {
    terrain->texture = NULL;
    terrain->textureColumns = 0;
    terrain->textureRows = 0;
  }

  /* load property map */
  if(propertyMapFileName != NULL) {
    MagickWand* propertyMap = NewMagickWand();
    status = MagickReadImage(propertyMap, propertyMapFileName);
    if (status == MagickFalse) {
      ThrowWandException(propertyMap);
    }

    unsigned long propertyMapColumns = MagickGetImageWidth(propertyMap);
    unsigned long propertyMapRows = MagickGetImageHeight(propertyMap);

    printf(
      "Loading texture %s with size (%lu, %lu)\n",
      propertyMapFileName,
      propertyMapRows,
      propertyMapColumns);

    terrain->propertyMap =
      calloc(propertyMapRows * propertyMapColumns, sizeof(char));

    /* load values into array following sae convention with x as
    top ordinate of view (column index) and y as left ordinate of view
    (row index), and +z is pointing down */

    PixelIterator* propertyMapIterator = NewPixelIterator(propertyMap);
    for(y = 0; y < (long) MagickGetImageHeight(propertyMap); y++) {

      PixelWand** pixels = PixelGetNextIteratorRow(propertyMapIterator, &width);
      if(pixels == (PixelWand **) NULL) {
        break;
      }

      for(x = 0; x < (long) width; x++) {
        PixelGetMagickColor(pixels[x], &pixel);

        terrain->propertyMap[y * columns + x] =
          PixelGetRedQuantum(pixels[x]);
      }

      (void) PixelSyncIterator(iterator);
    }

    propertyMap = DestroyMagickWand(propertyMap);
    terrain->propertyMapColumns = propertyMapColumns;
    terrain->propertyMapRows = propertyMapRows;
  } else {
    terrain->propertyMap = NULL;
    terrain->propertyMapColumns = 0;
    terrain->propertyMapRows = 0;
  }

  /*!
  @TODO if texture is not used for color, the material properties use these
  hardcoded values, need to modify to allow user defined material properties in
  JSON input file
  */
  terrain->Kd[0] = 63./255.;
  terrain->Kd[1] = 53./255.;
  terrain->Kd[2] = 69./255.;
  terrain->Kd[3] = 1.0;
  terrain->KsOverKd = 0.5;
  terrain->KaOverKd = 0.3;
  terrain->Ns = 1000;

  terrain->columns = columns;
  terrain->rows = rows;
  terrain->xOffset = xOffset;
  terrain->yOffset = yOffset;
  terrain->xScale = xScale;
  terrain->yScale = yScale;
  terrain->interpolationType = interpolationType;
  terrain->coordinateConvention = coordinateConvention;
  terrain->name = NULL;
  terrain->parentBody = NULL;
  terrain->parentBodyPointer = NULL;
  terrain->parentFrame = NULL;
  terrain->parentFramePointer = NULL;

  switch(interpolationType) {
    case S2_INTERPOLATION_BILINEAR: {
      switch(coordinateConvention) {
        case S2_COORDINATECONVENTION_ISO: {
          terrain->getZ = s2_terrain_getZ_bilinear_iso;
        } break;
        case S2_COORDINATECONVENTION_SAE: {
          terrain->getZ = s2_terrain_getZ_bilinear_sae;
        } break;
      }
    } break;
    case S2_INTERPOLATION_BICUBIC: {
      switch(coordinateConvention) {
        case S2_COORDINATECONVENTION_ISO: {
          terrain->getZ = s2_terrain_getZ_bicubic_iso;
        } break;
        case S2_COORDINATECONVENTION_SAE: {
          terrain->getZ = s2_terrain_getZ_bicubic_sae;
        } break;
      }
    } break;
    case S2_INTERPOLATION_CATMULROM: {
      switch(coordinateConvention) {
        case S2_COORDINATECONVENTION_ISO: {
          terrain->getZ = s2_terrain_getZ_catmulrom_iso;
        } break;
        case S2_COORDINATECONVENTION_SAE: {
          terrain->getZ = s2_terrain_getZ_catmulrom_sae;
        } break;
      }
    } break;
  }

  MagickWandTerminus();

  return terrain;
}

/*!
@brief loads terrain datastructure from json file

*/
int s2_terrain_load(
  s2_assembly_t* assembly,
  json_t* assemblyObject,
  s2_context_t* context,
  s2_stackstr_t* prefix) {

  json_t* terrainsObject =
    json_object_get(assemblyObject, "terrain");

  if(json_is_array(terrainsObject)) {
    size_t index;
    json_t* terrainObject;

    context->terrainCount =
      json_array_size(terrainsObject);

    context->terrains =
      malloc(
        context->terrainCount * sizeof(s2_terrain_t*));

    json_array_foreach(
      terrainsObject,
      index,
      terrainObject) {

      /* terrain map file path */
      const char* terrainFilePath =
        json_string_value(
          json_object_get(
            terrainObject,
            "definition"));
      char* fullTerrainFilePath = NULL;
      s2_stackstrnode_t* includeDirectoryHead =
        context->modelParameters.includeDirectory->head;
      while(includeDirectoryHead) {
        asprintf(
          &fullTerrainFilePath,
          "%s%s",
          includeDirectoryHead->value,
          terrainFilePath);
        if(access(fullTerrainFilePath, F_OK) != -1) {
          break;
        } else {
          free(fullTerrainFilePath);
          fullTerrainFilePath = NULL;
        }
        includeDirectoryHead = includeDirectoryHead->next;
      }

      /* get texture map file path */
      json_t* textureMapObject =
        json_object_get(
          terrainObject,
          "texture");
      char* fullTextureMapFilePath = NULL;
      if(textureMapObject != NULL) {
        const char* textureMapFilePath =
          json_string_value(textureMapObject);
        s2_stackstrnode_t* includeDirectoryHead =
          context->modelParameters.includeDirectory->head;
        while(includeDirectoryHead) {
          asprintf(
            &fullTextureMapFilePath,
            "%s%s",
            includeDirectoryHead->value,
            textureMapFilePath);
          if(access(fullTextureMapFilePath, F_OK) != -1) {
            break;
          } else {
            free(fullTextureMapFilePath);
            fullTextureMapFilePath = NULL;
          }
          includeDirectoryHead = includeDirectoryHead->next;
        }
      }

      /* get property map file path */
      json_t* propertyMapObject =
        json_object_get(
          terrainObject,
          "property_map");
      char* fullPropertyMapFilePath = NULL;
      if(propertyMapObject != NULL) {
        const char* propertyMapFilePath =
          json_string_value(propertyMapObject);
        s2_stackstrnode_t* includeDirectoryHead =
          context->modelParameters.includeDirectory->head;
        while(includeDirectoryHead) {
          asprintf(
            &fullPropertyMapFilePath,
            "%s%s",
            includeDirectoryHead->value,
            propertyMapFilePath);
          if(access(fullPropertyMapFilePath, F_OK) != -1) {
            break;
          } else {
            free(fullPropertyMapFilePath);
            fullPropertyMapFilePath = NULL;
          }
          includeDirectoryHead = includeDirectoryHead->next;
        }
      }

      /* get coordinate type (SAE or ISO), defaults to SAE */
      const char* coordinateTypeStr =
        json_string_value(
          json_object_get(
            terrainObject,
            "coordinate_type"));
      s2_coordinate_convention_enum_t coordinateType =
        S2_COORDINATECONVENTION_SAE;
      if(!strcmp(coordinateTypeStr, "sae")) {
        coordinateType = S2_COORDINATECONVENTION_SAE;
      } else if(!strcmp(coordinateTypeStr, "iso")) {
        coordinateType = S2_COORDINATECONVENTION_ISO;
      }

      /* get interpolation type (BICUBIC, BILINEAR or CATMULROM),
      defaults to CATMULROM */
      const char* interpolationTypeStr =
        json_string_value(
          json_object_get(
            terrainObject,
            "coordinate_type"));
      s2_interpolation_enum_t interpolationType = S2_INTERPOLATION_CATMULROM;
      if(!strcmp(interpolationTypeStr, "bicubic")) {
        interpolationType = S2_INTERPOLATION_BICUBIC;
      } else if(!strcmp(interpolationTypeStr, "bilinear")) {
        interpolationType = S2_INTERPOLATION_BILINEAR;
      } else if(!strcmp(interpolationTypeStr, "catmul_rom")) {
        interpolationType = S2_INTERPOLATION_CATMULROM;
      }

      /* get x scale factor, defaults to 1 if no x scale provided */
      double x_scale = 1.0;
      json_t* xScaleObject =
        json_object_get(terrainObject, "x_scale");
      if(xScaleObject != NULL) {
        x_scale = json_number_value(xScaleObject);
      }

      /* get y scale factor, defaults to 1 if no y scale provided */
      double y_scale = 1.0;
      json_t* yScaleObject =
        json_object_get(terrainObject, "y_scale");
      if(yScaleObject != NULL) {
        y_scale = json_number_value(yScaleObject);
      }

      /* get x offset, defaults to 0 if no x offset provided */
      double x_offset = 0.0;
      json_t* xOffsetObject =
        json_object_get(terrainObject, "x_offset");
      if(xScaleObject != NULL) {
        x_offset = json_number_value(xOffsetObject);
      }

      /* get y offset, defaults to 0 if no y offset provided */
      double y_offset = 0.0;
      json_t* yOffsetObject =
        json_object_get(terrainObject, "y_offset");
      if(xScaleObject != NULL) {
        y_offset = json_number_value(yOffsetObject);
      }

      /* generate height map using s2_elevationmap_t constructor,
      this constructor sets the appropriate function pointer
      in the struct given the coordinate convention and interpolation
      type */
      s2_terrain_t* terrain =
        s2_terrain(
          fullTerrainFilePath,
          fullTextureMapFilePath,
          fullPropertyMapFilePath,
          interpolationType,
          coordinateType,
          x_offset,
          y_offset,
          x_scale,
          y_scale);

      /* the terrain constructor does not set the name of the object, therefore check the prefix to prepend the full path
      name to the name provided in the json file */
      terrain->name = s2_stackstr_copy(prefix);

      s2_stackstr_append(
        terrain->name,
        json_string_value(
          json_object_get(terrainObject, "name")));

      /* the constructor does not set the parent body name and
      parent body pointer. Set the name here, the pointer will be
      connected once the json file is finished loading  */
      json_t* parentObject =
        json_object_get(terrainObject, "parent");
      json_t* parentBodyObject =
        json_array_get(parentObject, 0);
      terrain->parentBody = s2_stackstr();
      if(json_is_array(parentBodyObject)) {
        /* if parent body is an array, then the full path is specified.
        */
        size_t index2;
        json_t* value;
        json_array_foreach(parentBodyObject, index2, value) {
          s2_stackstr_append(
            terrain->parentBody,
            json_string_value(value));
        }
      } else if (json_is_string(parentBodyObject)) {
        /*
          otherwise the body is contained within the assembly
        */
        s2_stackstr_append(
          terrain->parentBody,
          json_string_value(parentBodyObject));
      }

      /* the constructor does not set the parent frame name and
      parent frame pointer. Set the name here, the pointer will be
      connected once the json file is finished loading  */
      terrain->parentFrame = s2_stackstr_copy(terrain->parentBody);
      s2_stackstr_append(
        terrain->parentFrame,
        json_string_value(json_array_get(parentObject, 1)));

      /* save the terrain map pointer in the terrain map array */
      context->terrains[index] = terrain;
    }
  }

  return 0;
}


/*!
@brief gets height and surface normal at specified x,y coordinates
@param context [in] star2 context data structure containing model data
@param x [in] x coordinate
@param y [in] y coordinate
@param height [out] height at x, y coordinate
@param normal [out] surface normal at x, y coordinate (in global coordinates)
This is a place holder function for now. In the future need to add functions to
load terrain profiles and report values based on terrain.
The surface normal follows SAE / Aerospace conventions with normal in the "down" direction, i.e. positive z
*/
void s2_terrain_getHeightAndSAENormal(
  s2_context_t* context,
  double x,
  double y,
  double* height,
  s2_vector_t* normal) {

  *height = 0.0;
  normal->x = 0.0;
  normal->y = 0.0;
  normal->z = 1.0;
}


/*!
@brief gets height and surface normal at specified x,y coordinates
@param context [in] star2 context data structure containing model data
@param x [in] x coordinate
@param y [in] y coordinate
@param height [out] height at x, y coordinate
@param normal [out] surface normal at x, y coordinate (in global coordinates)
This is a place holder function for now. In the future need to add functions to
load terrain profiles and report values based on terrain.
The surface normal follows ISO convention with normal in the "up" direction,
i.e. {0, 0, -1}
*/
void s2_terrain_getHeightAndISONormal(
  s2_context_t* context,
  double x,
  double y,
  double* height,
  s2_vector_t* normal) {

  *height = 0.0;
  normal->x = 0.0;
  normal->y = 0.0;
  normal->z = -1.0;
}

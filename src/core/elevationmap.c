#include "elevationmap.h"
#include <float.h>

/*!
@brief generate a simple surface used to test elevation maps
*/
double surface(double x, double y) {
  // return sin((y - 10.0) * (pow(x, 0.5) - 5.0) / 500.0) * 3.0 + 2.0;

  return ((20.0 * sin(y / 5.0 - 2.0) * (pow(x, 0.5) - 5.0) / 50.0)) + 2.0;

}


/*! @brief generates a sample height map using surface function
saves to 32 bit tiff floating point format.  */
void generateSampleHeightMap(const char* filename) {

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
double s2_elevationmap_get(
  s2_elevationmap_t* elevationMap,
  int i,
  int j) {

  /* row index i is y dimension, column index j is x dimension */
  assert(i >= 0 && i < elevationMap->rows);
  assert(j >= 0 && j < elevationMap->columns);

  return elevationMap->controlPoints[i * elevationMap->columns + j];

}


/*!
@brief get z using bilinear interpolation at specified x and y position
following SAE convention with positive z "down". The normal is returned in
the "up" direction.

@TODO normal vector not computed yet
*/
double s2_elevationmap_getZ_bilinear_sae(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  s2_vector_t* normal) {

  /* get x and y values in original height map */
  double xPos = (x - elevationMap->xOffset) / elevationMap->xScale;
  double yPos = (y - elevationMap->yOffset) / elevationMap->yScale;

  /* calculate i,j index into map. row index i is y dimension, column index j is x dimension */
  int j0 = clamp_int(floor(xPos), 0, elevationMap->columns);
  int j1 = clamp_int(j0 + 1, 0, elevationMap->columns);

  int i0 = clamp_int(floor(yPos), 0, elevationMap->rows);
  int i1 = clamp_int(i0 + 1, 0, elevationMap->rows);

  /* calculate fraction from i1,j1 to i2,j2 (value from 0 to 1) */
  double fx = xPos - j0;
  double fy = yPos - i0;

  double z =
    blend(
      fx,
      blend(
        fy,
        s2_elevationmap_get(elevationMap, i0, j0),
        s2_elevationmap_get(elevationMap, i1, j0)),
      blend(
        fy,
        s2_elevationmap_get(elevationMap, i0, j1),
        s2_elevationmap_get(elevationMap, i1, j1)));
  return z;
}


/*!
@brief get z using bilinear interpolation at specified x and y position
following ISO convention with positive z "up". The normal is returned in
the "up" direction.

@TODO normal vector not computed yet
*/
double s2_elevationmap_getZ_bilinear_iso(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  s2_vector_t* normal) {

  /* get x and y values in original height map */
  double xPos = (x - elevationMap->xOffset) / elevationMap->xScale;
  double yPos = (y - elevationMap->yOffset) / elevationMap->yScale;

  /* calculate i,j index into map. row index i is x dimension, column index j is y dimension */
  int i0 = clamp_int(floor(xPos), 0, elevationMap->rows);
  int i1 = clamp_int(i0 + 1, 0, elevationMap->rows);

  int j0 = clamp_int(floor(yPos), 0, elevationMap->columns);
  int j1 = clamp_int(j0 + 1, 0, elevationMap->columns);

  /* calculate fraction from i1,j1 to i2,j2 (value from 0 to 1) */
  double fx = xPos - i1;
  double fy = yPos - j1;

  double z =
    blend(
      fy,
      blend(
        fx,
        s2_elevationmap_get(elevationMap, i0, j0),
        s2_elevationmap_get(elevationMap, i1, j0)),
      blend(
        fx,
        s2_elevationmap_get(elevationMap, i0, j1),
        s2_elevationmap_get(elevationMap, i1, j1)));

  return z;
}


/*!
@brief bicubic interpolation following sae convention with positive z down.

Surface normal is returned in the -z (up) direction.

*/
double s2_elevationmap_getZ_bicubic_sae(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  s2_vector_t* normal) {


  /* get x and y values in original height map */
  double xPos = (x - elevationMap->xOffset) / elevationMap->xScale;
  double yPos = (y - elevationMap->yOffset) / elevationMap->yScale;

  printf("xPos: %f, yPos: %f\n", xPos, yPos);
  printf(
    "rows: %ld, columns: %ld\n",
    elevationMap->rows,
    elevationMap->columns);

  /* calculate i,j index into map. row index i is y dimension, column index j is x dimension */

  int i1 = clamp_int(floor(yPos), 0, elevationMap->rows);
  int i2 = clamp_int(i1 + 1, 0, elevationMap->rows);
  int i3 = clamp_int(i1 + 2, 0, elevationMap->rows);
  int i0 = clamp_int(i1 - 1, 0, elevationMap->rows);

  double u = yPos - i1;
  double u2 = u * u;
  double u3 = u2 * u;

  /* calculate fraction from i1,j1 to i2,j2 (a value from 0 to 1) */
  int j1 = clamp_int(floor(xPos), 0, elevationMap->columns);
  int j2 = clamp_int(j1 + 1, 0, elevationMap->columns);
  int j3 = clamp_int(j1 + 2, 0, elevationMap->columns);
  int j0 = clamp_int(j1 - 1, 0, elevationMap->columns);

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
    wi0 * s2_elevationmap_get(elevationMap, i0, j0) * wj0 +
    wi1 * s2_elevationmap_get(elevationMap, i1, j0) * wj0 +
    wi2 * s2_elevationmap_get(elevationMap, i2, j0) * wj0 +
    wi3 * s2_elevationmap_get(elevationMap, i3, j0) * wj0 +

    wi0 * s2_elevationmap_get(elevationMap, i0, j1) * wj1 +
    wi1 * s2_elevationmap_get(elevationMap, i1, j1) * wj1 +
    wi2 * s2_elevationmap_get(elevationMap, i2, j1) * wj1 +
    wi3 * s2_elevationmap_get(elevationMap, i3, j1) * wj1 +

    wi0 * s2_elevationmap_get(elevationMap, i0, j2) * wj2 +
    wi1 * s2_elevationmap_get(elevationMap, i1, j2) * wj2 +
    wi2 * s2_elevationmap_get(elevationMap, i2, j2) * wj2 +
    wi3 * s2_elevationmap_get(elevationMap, i3, j2) * wj2 +

    wi0 * s2_elevationmap_get(elevationMap, i0, j3) * wj3 +
    wi1 * s2_elevationmap_get(elevationMap, i1, j3) * wj3 +
    wi2 * s2_elevationmap_get(elevationMap, i2, j3) * wj3 +
    wi3 * s2_elevationmap_get(elevationMap, i3, j3) * wj3;

  /* compute partial derivative of [u^3, u^2, u, 1] with respect to u */
  double wi0_u = -0.5 * u2 + u - 0.5;
  double wi1_u = 1.5 * u2 - 2.0 * u;
  double wi2_u = -1.5 * u2 + u + 0.5;
  double wi3_u = 0.5 * u2;

  double z_y =
    wi0_u * s2_elevationmap_get(elevationMap, i0, j0) * wj0 +
    wi1_u * s2_elevationmap_get(elevationMap, i1, j0) * wj0 +
    wi2_u * s2_elevationmap_get(elevationMap, i2, j0) * wj0 +
    wi3_u * s2_elevationmap_get(elevationMap, i3, j0) * wj0 +

    wi0_u * s2_elevationmap_get(elevationMap, i0, j1) * wj1 +
    wi1_u * s2_elevationmap_get(elevationMap, i1, j1) * wj1 +
    wi2_u * s2_elevationmap_get(elevationMap, i2, j1) * wj1 +
    wi3_u * s2_elevationmap_get(elevationMap, i3, j1) * wj1 +

    wi0_u * s2_elevationmap_get(elevationMap, i0, j2) * wj2 +
    wi1_u * s2_elevationmap_get(elevationMap, i1, j2) * wj2 +
    wi2_u * s2_elevationmap_get(elevationMap, i2, j2) * wj2 +
    wi3_u * s2_elevationmap_get(elevationMap, i3, j2) * wj2 +

    wi0_u * s2_elevationmap_get(elevationMap, i0, j3) * wj3 +
    wi1_u * s2_elevationmap_get(elevationMap, i1, j3) * wj3 +
    wi2_u * s2_elevationmap_get(elevationMap, i2, j3) * wj3 +
    wi3_u * s2_elevationmap_get(elevationMap, i3, j3) * wj3;

  z_y /= elevationMap->yScale;

  /* compute partial derivative of [v^3,v^2, v, 1] with respect to v */
  double wj0_v = -0.5 * v2 + v - 0.5;
  double wj1_v = 1.5 * v2 - 2.0 * v;
  double wj2_v = -1.5 * v2 + v + 0.5;
  double wj3_v = 0.5 * v2;

  double z_x =
    wi0 * s2_elevationmap_get(elevationMap, i0, j0) * wj0_v +
    wi1 * s2_elevationmap_get(elevationMap, i1, j0) * wj0_v +
    wi2 * s2_elevationmap_get(elevationMap, i2, j0) * wj0_v +
    wi3 * s2_elevationmap_get(elevationMap, i3, j0) * wj0_v +

    wi0 * s2_elevationmap_get(elevationMap, i0, j1) * wj1_v +
    wi1 * s2_elevationmap_get(elevationMap, i1, j1) * wj1_v +
    wi2 * s2_elevationmap_get(elevationMap, i2, j1) * wj1_v +
    wi3 * s2_elevationmap_get(elevationMap, i3, j1) * wj1_v +

    wi0 * s2_elevationmap_get(elevationMap, i0, j2) * wj2_v +
    wi1 * s2_elevationmap_get(elevationMap, i1, j2) * wj2_v +
    wi2 * s2_elevationmap_get(elevationMap, i2, j2) * wj2_v +
    wi3 * s2_elevationmap_get(elevationMap, i3, j2) * wj2_v +

    wi0 * s2_elevationmap_get(elevationMap, i0, j3) * wj3_v +
    wi1 * s2_elevationmap_get(elevationMap, i1, j3) * wj3_v +
    wi2 * s2_elevationmap_get(elevationMap, i2, j3) * wj3_v +
    wi3 * s2_elevationmap_get(elevationMap, i3, j3) * wj3_v;

  z_x /= elevationMap->xScale;

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
double s2_elevationmap_getZ_bicubic_iso(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  s2_vector_t* normal) {


  /* get x and y values in original height map */
  double xPos = (x - elevationMap->xOffset) / elevationMap->xScale;
  double yPos = (y - elevationMap->yOffset) / elevationMap->yScale;

  /* calculate i,j index into map. row index i is y dimension, column index j is x dimension */

  int i1 = clamp_int(floor(xPos), 0, elevationMap->rows);
  int i2 = clamp_int(i1 + 1, 0, elevationMap->rows);
  int i3 = clamp_int(i1 + 2, 0, elevationMap->rows);
  int i0 = clamp_int(i1 - 1, 0, elevationMap->rows);

  double u = xPos - i1;
  double u2 = u * u;
  double u3 = u2 * u;

  /* calculate fraction from i1,j1 to i2,j2 (a value from 0 to 1) */
  int j1 = clamp_int(floor(yPos), 0, elevationMap->columns);
  int j2 = clamp_int(j1 + 1, 0, elevationMap->columns);
  int j3 = clamp_int(j1 + 2, 0, elevationMap->columns);
  int j0 = clamp_int(j1 - 1, 0, elevationMap->columns);

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
    wi0 * s2_elevationmap_get(elevationMap, i0, j0) * wj0 +
    wi1 * s2_elevationmap_get(elevationMap, i1, j0) * wj0 +
    wi2 * s2_elevationmap_get(elevationMap, i2, j0) * wj0 +
    wi3 * s2_elevationmap_get(elevationMap, i3, j0) * wj0 +

    wi0 * s2_elevationmap_get(elevationMap, i0, j1) * wj1 +
    wi1 * s2_elevationmap_get(elevationMap, i1, j1) * wj1 +
    wi2 * s2_elevationmap_get(elevationMap, i2, j1) * wj1 +
    wi3 * s2_elevationmap_get(elevationMap, i3, j1) * wj1 +

    wi0 * s2_elevationmap_get(elevationMap, i0, j2) * wj2 +
    wi1 * s2_elevationmap_get(elevationMap, i1, j2) * wj2 +
    wi2 * s2_elevationmap_get(elevationMap, i2, j2) * wj2 +
    wi3 * s2_elevationmap_get(elevationMap, i3, j2) * wj2 +

    wi0 * s2_elevationmap_get(elevationMap, i0, j3) * wj3 +
    wi1 * s2_elevationmap_get(elevationMap, i1, j3) * wj3 +
    wi2 * s2_elevationmap_get(elevationMap, i2, j3) * wj3 +
    wi3 * s2_elevationmap_get(elevationMap, i3, j3) * wj3;

  /* compute partial derivative of [u^3, u^2, u, 1] with respect to u */
  double wi0_u = -0.5 * u2 + u - 0.5;
  double wi1_u = 1.5 * u2 - 2.0 * u;
  double wi2_u = -1.5 * u2 + u + 0.5;
  double wi3_u = 0.5 * u2;

  double z_x =
    wi0_u * s2_elevationmap_get(elevationMap, i0, j0) * wj0 +
    wi1_u * s2_elevationmap_get(elevationMap, i1, j0) * wj0 +
    wi2_u * s2_elevationmap_get(elevationMap, i2, j0) * wj0 +
    wi3_u * s2_elevationmap_get(elevationMap, i3, j0) * wj0 +

    wi0_u * s2_elevationmap_get(elevationMap, i0, j1) * wj1 +
    wi1_u * s2_elevationmap_get(elevationMap, i1, j1) * wj1 +
    wi2_u * s2_elevationmap_get(elevationMap, i2, j1) * wj1 +
    wi3_u * s2_elevationmap_get(elevationMap, i3, j1) * wj1 +

    wi0_u * s2_elevationmap_get(elevationMap, i0, j2) * wj2 +
    wi1_u * s2_elevationmap_get(elevationMap, i1, j2) * wj2 +
    wi2_u * s2_elevationmap_get(elevationMap, i2, j2) * wj2 +
    wi3_u * s2_elevationmap_get(elevationMap, i3, j2) * wj2 +

    wi0_u * s2_elevationmap_get(elevationMap, i0, j3) * wj3 +
    wi1_u * s2_elevationmap_get(elevationMap, i1, j3) * wj3 +
    wi2_u * s2_elevationmap_get(elevationMap, i2, j3) * wj3 +
    wi3_u * s2_elevationmap_get(elevationMap, i3, j3) * wj3;

  z_x /= elevationMap->xScale;

  /* compute partial derivative of [v^3,v^2, v, 1] with respect to v */
  double wj0_v = -0.5 * v2 + v - 0.5;
  double wj1_v = 1.5 * v2 - 2.0 * v;
  double wj2_v = -1.5 * v2 + v + 0.5;
  double wj3_v = 0.5 * v2;

  double z_y =
    wi0 * s2_elevationmap_get(elevationMap, i0, j0) * wj0_v +
    wi1 * s2_elevationmap_get(elevationMap, i1, j0) * wj0_v +
    wi2 * s2_elevationmap_get(elevationMap, i2, j0) * wj0_v +
    wi3 * s2_elevationmap_get(elevationMap, i3, j0) * wj0_v +

    wi0 * s2_elevationmap_get(elevationMap, i0, j1) * wj1_v +
    wi1 * s2_elevationmap_get(elevationMap, i1, j1) * wj1_v +
    wi2 * s2_elevationmap_get(elevationMap, i2, j1) * wj1_v +
    wi3 * s2_elevationmap_get(elevationMap, i3, j1) * wj1_v +

    wi0 * s2_elevationmap_get(elevationMap, i0, j2) * wj2_v +
    wi1 * s2_elevationmap_get(elevationMap, i1, j2) * wj2_v +
    wi2 * s2_elevationmap_get(elevationMap, i2, j2) * wj2_v +
    wi3 * s2_elevationmap_get(elevationMap, i3, j2) * wj2_v +

    wi0 * s2_elevationmap_get(elevationMap, i0, j3) * wj3_v +
    wi1 * s2_elevationmap_get(elevationMap, i1, j3) * wj3_v +
    wi2 * s2_elevationmap_get(elevationMap, i2, j3) * wj3_v +
    wi3 * s2_elevationmap_get(elevationMap, i3, j3) * wj3_v;

  z_y /= elevationMap->yScale;

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
double s2_elevationmap_getZ_catmulrom_sae(
  s2_elevationmap_t* elevationMap,
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
double s2_elevationmap_getZ_catmulrom_iso(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  s2_vector_t* normal) {

  double z = 0;
  return z;
}


/*! @brief constructor for s2_elevationmap_t


*/
s2_elevationmap_t* s2_elevationmap(
  const char* elevationmapFileName,
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
  s2_elevationmap_t* elevationmap =
    (s2_elevationmap_t*)calloc(1, sizeof(s2_elevationmap_t));
  elevationmap->controlPoints = NULL;
  elevationmap->texture = NULL;
  elevationmap->getZ = NULL;
  elevationmap->zMin = DBL_MAX;
  elevationmap->zMax = -DBL_MAX;

  /* load image using imagemagick, Magick::InitializeMagick() must be
  called prior to creating an elevationmap */
  MagickBooleanType status;
  MagickWand* image = NewMagickWand();
  status = MagickReadImage(image, elevationmapFileName);
  if (status == MagickFalse) {
    ThrowWandException(image);
  }

  unsigned long columns = MagickGetImageWidth(image);
  unsigned long rows = MagickGetImageHeight(image);

  printf(
    "Loading elevation map %s with size (%lu, %lu)\n",
    elevationmapFileName, rows, columns);

  elevationmap->controlPoints = calloc(rows * columns, sizeof(double));


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

      elevationmap->controlPoints[y * columns + x] =
        (double)PixelGetRedQuantum(pixels[x]);
      elevationmap->zMax =
        fmax(
          elevationmap->controlPoints[y * columns + x],
          elevationmap->zMax);
      elevationmap->zMin =
        fmin(
          elevationmap->controlPoints[y * columns + x],
          elevationmap->zMin);
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

    elevationmap->texture = calloc(4 * textureRows * textureColumns, sizeof(unsigned char));

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
        elevationmap->texture[4 * (y * textureColumns + x) + 0] =
          (unsigned char) (PixelGetRedQuantum(pixels[x]) / range * 255.0);
        elevationmap->texture[4 * (y * textureColumns + x) + 1] =
          (unsigned char) (PixelGetGreenQuantum(pixels[x]) / range * 255.0);
        elevationmap->texture[4 * (y * textureColumns + x) + 2] =
          (unsigned char) (PixelGetBlueQuantum(pixels[x]) / range * 255.0);
        // elevationmap->texture[4 * (y * columns + x) + 3] =
        //   (unsigned char) (PixelGetAlpha(pixels[x]) / range * 255.0);
        elevationmap->texture[4 * (y * textureColumns + x) + 3] = 255;
        // if(elevationmap->texture[4 * (y * columns + x) + 3]) {
          // printf(
          //   "[%lu, %lu]: %d, %d, %d, %d\n",
          //   y, x,
          //   elevationmap->texture[4 * (y * columns + x) + 0],
          //   elevationmap->texture[4 * (y * columns + x) + 1],
          //   elevationmap->texture[4 * (y * columns + x) + 2],
          //   elevationmap->texture[4 * (y * columns + x) + 3]);
        // }
      }

      (void) PixelSyncIterator(iterator);
    }

    texture = DestroyMagickWand(texture);
    elevationmap->textureColumns = textureColumns;
    elevationmap->textureRows = textureRows;
  } else {
    elevationmap->texture = NULL;
    elevationmap->textureColumns = 0;
    elevationmap->textureRows = 0;
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

    elevationmap->propertyMap =
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

        elevationmap->propertyMap[y * columns + x] =
          PixelGetRedQuantum(pixels[x]);
      }

      (void) PixelSyncIterator(iterator);
    }

    propertyMap = DestroyMagickWand(propertyMap);
    elevationmap->propertyMapColumns = propertyMapColumns;
    elevationmap->propertyMapRows = propertyMapRows;
  } else {
    elevationmap->propertyMap = NULL;
    elevationmap->propertyMapColumns = 0;
    elevationmap->propertyMapRows = 0;
  }

  elevationmap->columns = columns;
  elevationmap->rows = rows;
  elevationmap->xOffset = xOffset;
  elevationmap->yOffset = yOffset;
  elevationmap->xScale = xScale;
  elevationmap->yScale = yScale;
  elevationmap->interpolationType = interpolationType;
  elevationmap->coordinateConvention = coordinateConvention;
  elevationmap->name = NULL;
  elevationmap->parentBody = NULL;
  elevationmap->parentBodyPointer = NULL;
  elevationmap->parentFrame = NULL;
  elevationmap->parentFramePointer = NULL;

  switch(interpolationType) {
    case BILINEAR: {
      switch(coordinateConvention) {
        case ISO: {
          elevationmap->getZ = s2_elevationmap_getZ_bilinear_iso;
        } break;
        case SAE: {
          elevationmap->getZ = s2_elevationmap_getZ_bilinear_sae;
        } break;
      }
    } break;
    case BICUBIC: {
      switch(coordinateConvention) {
        case ISO: {
          elevationmap->getZ = s2_elevationmap_getZ_bicubic_iso;
        } break;
        case SAE: {
          elevationmap->getZ = s2_elevationmap_getZ_bicubic_sae;
        } break;
      }
    } break;
    case CATMULROM: {
      switch(coordinateConvention) {
        case ISO: {
          elevationmap->getZ = s2_elevationmap_getZ_catmulrom_iso;
        } break;
        case SAE: {
          elevationmap->getZ = s2_elevationmap_getZ_catmulrom_sae;
        } break;
      }
    } break;
  }

  MagickWandTerminus();

  return elevationmap;
}

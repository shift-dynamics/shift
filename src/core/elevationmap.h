#include <wand/MagickWand.h>
#include "../ksl/ksl.h"
// #include "../star2.h"


#ifndef _STAR2_ELEVATIONMAP_H_
#define _STAR2_ELEVATIONMAP_H_

#define ThrowWandException(wand) \
{ \
  char \
    *description; \
 \
  ExceptionType \
    severity; \
 \
  description = MagickGetException(wand,&severity); \
  (void) fprintf(stderr, "%s %s %lu %s\n", GetMagickModule(), description); \
  description = (char *)MagickRelinquishMemory(description); \
  exit(-1); \
}


typedef enum s2_interpolation_enum_t {
  BILINEAR, /*!< bilinear interpolation is the simplest technique for 2D
  interpolation. The resulting surface is C0 continuous (position continuity is maintained between surface patches, but not tangency and curvature). The surface intersects the control points. */
  BICUBIC,  /*!< bicubic interpolation uses knot points as control points, surface passes near control point, but may not intersect control point, surface is C0, C1 and C2 continuous (position, tangency and curvature continuity are maintained between surface patches) */
  CATMULROM /*!< catmul-rom interpolation, surface passes through control points, but is only C0 and C1 continuous (position and tangency continuity are maintained between surface patches, but not curvature) */
} s2_interpolation_enum_t;


typedef enum s2_coordinate_convention_enum_t {
  ISO, /*!< x is left ordinate of image, y is top ordinate of image, z is pointing up out of height map */
  SAE /*!< x is top ordinate of image, y is left ordinate of image, z is pointing down into height map */
} s2_coordinate_convention_enum_t;


typedef struct s2_elevationmap_t {
  s2_interpolation_enum_t interpolationType; /*!< technique used for 2D interpolation */
  s2_coordinate_convention_enum_t coordinateConvention;
  size_t columns;
  size_t rows;
  double xOffset;    /*!< x offset of origin of height map */
  double yOffset;    /*!< y offset of origin of height map */
  double xScale;     /*!< uniform scale factor for x dimension */
  double yScale;     /*!< uniform scale factor for y dimension */
  double zMin;       /*!< minimum z height in surface */
  double zMax;       /*!< maximum z height in surface */
  double* controlPoints; /*!< control points loaded from image */
  double (*getZ)(
    struct s2_elevationmap_t* elevationMap,
    double x,
    double y,
    ksl_vec3_t* normal); /*!< function used to obtain elevation at specified x, y location. This function pointer is set based on type of interpolation specified when elevationmap is created */
} s2_elevationmap_t;

void generateSampleHeightMap(const char* filename);

double s2_elevationmap_get(
  s2_elevationmap_t* elevationMap,
  int i,
  int j);


double s2_elevationmap_getZ_bilinear_sae(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  ksl_vec3_t* normal);


double s2_elevationmap_getZ_bilinear_iso(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  ksl_vec3_t* normal);


double s2_elevationmap_getZ_bicubic_sae(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  ksl_vec3_t* normal);


double s2_elevationmap_getZ_bicubic_iso(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  ksl_vec3_t* normal);


double s2_elevationmap_getZ_catmulrom_sae(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  ksl_vec3_t* normal);


double s2_elevationmap_getZ_catmulrom_iso(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  ksl_vec3_t* normal);


s2_elevationmap_t* s2_elevationmap(
  const char* filename,
  s2_interpolation_enum_t interpolationType,
  s2_coordinate_convention_enum_t coordinateConvention,
  double xOffset,
  double yOffset,
  double xScale,
  double yScale);

#endif

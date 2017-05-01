#include "../star2.h"
#include <wand/MagickWand.h>

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

void generateSampleHeightMap(const char* filename);


double s2_elevationmap_get(
  s2_elevationmap_t* elevationMap,
  int i,
  int j);


double s2_elevationmap_getZ_bilinear_sae(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  s2_vector_t* normal);


double s2_elevationmap_getZ_bilinear_iso(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  s2_vector_t* normal);


double s2_elevationmap_getZ_bicubic_sae(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  s2_vector_t* normal);


double s2_elevationmap_getZ_bicubic_iso(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  s2_vector_t* normal);


double s2_elevationmap_getZ_catmulrom_sae(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  s2_vector_t* normal);


double s2_elevationmap_getZ_catmulrom_iso(
  s2_elevationmap_t* elevationMap,
  double x,
  double y,
  s2_vector_t* normal);


s2_elevationmap_t* s2_elevationmap(
  const char* elevationmapFileName,
  const char* textureFileName,
  const char* propertyMap,
  s2_interpolation_enum_t interpolationType,
  s2_coordinate_convention_enum_t coordinateConvention,
  double xOffset,
  double yOffset,
  double xScale,
  double yScale);

#endif

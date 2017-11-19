#include "../star2.h"
#include <wand/MagickWand.h>

#ifndef _STAR2_TERRAIN_H_
#define _STAR2_TERRAIN_H_

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
  S2_INTERPOLATION_BILINEAR, /*!< bilinear interpolation is the simplest technique for 2D
  interpolation. The resulting surface is C0 continuous (position continuity is maintained between surface patches, but not tangency and curvature). The surface intersects the control points. */
  S2_INTERPOLATION_BICUBIC,  /*!< bicubic interpolation uses knot points as control points, surface passes near control point, but may not intersect control point, surface is C0, C1 and C2 continuous (position, tangency and curvature continuity are maintained between surface patches) */
  S2_INTERPOLATION_CATMULROM /*!< catmul-rom interpolation, surface passes through control points, but is only C0 and C1 continuous (position and tangency continuity are maintained between surface patches, but not curvature) */
} s2_interpolation_enum_t;


typedef enum s2_coordinate_convention_enum_t {
  S2_COORDINATECONVENTION_ISO, /*!< x is left ordinate of image, y is top ordinate of image, z is pointing up out of height map */
  S2_COORDINATECONVENTION_SAE /*!< x is top ordinate of image, y is left ordinate of image, z is pointing down into height map */
} s2_coordinate_convention_enum_t;


typedef struct s2_terrain_t {
  s2_interpolation_enum_t interpolationType; /*!< technique used for 2D interpolation */
  s2_coordinate_convention_enum_t coordinateConvention; /*!< ISO or SAE coordinate system */
  size_t columns;            /*!< number of columns of elevation map */
  size_t rows;               /*!< number of rows in elevation map */
  double xOffset;            /*!< x offset of origin of elevation map */
  double yOffset;            /*!< y offset of origin of elevation map */
  double xScale;             /*!< uniform scale factor for x dimension */
  double yScale;             /*!< uniform scale factor for y dimension */
  double zMin;               /*!< minimum z height in surface */
  double zMax;               /*!< maximum z height in surface */
  double* controlPoints;     /*!< control points loaded from image */
  unsigned char* texture;    /*!< rgba texture */
  size_t textureRows;        /*!< number of rows in texture */
  size_t textureColumns;     /*!< number of columns in texture */
  char* propertyMap;         /*!< rgba texture */
  size_t propertyMapRows;    /*!< number of rows in texture */
  size_t propertyMapColumns; /*!< number of columns in texture */
  GLfloat Kd[4]; /*! diffuse color value */
  GLfloat KsOverKd;
  GLfloat KaOverKd;
  GLint Ns;
  double (*getZ)(
    struct s2_terrain_t* terrain,
    double x,
    double y,
    s2_vector_t* normal); /*!< function used to obtain elevation at specified x, y location. This function pointer is set based on type of interpolation specified when terrain is created */
  s2_stackstr_t* name;
  s2_stackstr_t* parentBody;
  s2_body_t* parentBodyPointer;
  s2_stackstr_t* parentFrame;
  s2_frame_t* parentFramePointer;
  bool isVisible;
} s2_terrain_t;


void generateSampleHeightMap(const char* filename);


double s2_terrain_get(
  s2_terrain_t* terrain,
  int i,
  int j);


double s2_terrain_getZ_bilinear_sae(
  s2_terrain_t* terrain,
  double x,
  double y,
  s2_vector_t* normal);


double s2_terrain_getZ_bilinear_iso(
  s2_terrain_t* terrain,
  double x,
  double y,
  s2_vector_t* normal);


double s2_terrain_getZ_bicubic_sae(
  s2_terrain_t* terrain,
  double x,
  double y,
  s2_vector_t* normal);


double s2_terrain_getZ_bicubic_iso(
  s2_terrain_t* terrain,
  double x,
  double y,
  s2_vector_t* normal);


double s2_terrain_getZ_catmulrom_sae(
  s2_terrain_t* terrain,
  double x,
  double y,
  s2_vector_t* normal);


double s2_terrain_getZ_catmulrom_iso(
  s2_terrain_t* terrain,
  double x,
  double y,
  s2_vector_t* normal);


s2_terrain_t* s2_terrain(
  const char* terrainFileName,
  const char* textureFileName,
  const char* propertyMap,
  s2_interpolation_enum_t interpolationType,
  s2_coordinate_convention_enum_t coordinateConvention,
  double xOffset,
  double yOffset,
  double xScale,
  double yScale);


void s2_terrain_getHeightAndSAENormal(
  s2_context_t* context,
  double x,
  double y,
  double* height,
  s2_vector_t* normal);


void s2_terrain_getHeightAndISONormal(
  s2_context_t* context,
  double x,
  double y,
  double* height,
  s2_vector_t* normal);


int s2_terrain_load(
  s2_assembly_t* assembly,
  json_t* assemblyObject,
  s2_context_t* context,
  s2_stackstr_t* prefix);


#endif

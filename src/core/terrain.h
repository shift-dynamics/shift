/**
@file
@author Kristopher Wehage
@date 2017
@copyright Kristopher Wehage 2016-2017
@remark
THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.
*/

#include "../shift.h"
#include <stdio.h>
#include <stdlib.h>
#include <wand/MagickWand.h>

#ifndef _SHIFT_TERRAIN_H_
#define _SHIFT_TERRAIN_H_

#define ThrowWandException(wand)                                               \
  {                                                                            \
    char* description;                                                         \
                                                                               \
    ExceptionType severity;                                                    \
                                                                               \
    description = MagickGetException(wand, &severity);                         \
    (void)fprintf(stderr, "%s %s %lu %s\n", GetMagickModule(), description);   \
    description = (char*)MagickRelinquishMemory(description);                  \
    exit(-1);                                                                  \
  }

typedef enum s2_interpolation_enum_t {
  S2_INTERPOLATION_BILINEAR, /*!< bilinear interpolation is the simplest
  technique for 2D interpolation. The resulting surface is C0 continuous
  (position continuity is maintained between surface patches, but not tangency
  and curvature). The surface intersects the control points. */
  S2_INTERPOLATION_BICUBIC,  /*!< bicubic interpolation uses knot points as
                                control points, surface passes near control
                                point, but may not intersect control point,
                                surface is C0, C1 and C2 continuous (position,
                                tangency and curvature continuity are maintained
                                between surface patches) */
  S2_INTERPOLATION_CATMULROM /*!< catmul-rom interpolation, surface passes
                                through control points, but is only C0 and C1
                                continuous (position and tangency continuity are
                                maintained between surface patches, but not
                                curvature) */
} s2_interpolation_enum_t;

typedef enum s2_coordinate_convention_enum_t {
  S2_COORDINATECONVENTION_ISO, /*!< x is left ordinate of image, y is top
                                  ordinate of image, z is pointing up out of
                                  height map */
  S2_COORDINATECONVENTION_SAE  /*!< x is top ordinate of image, y is left
                                  ordinate of image, z is pointing down into
                                  height map */
} s2_coordinate_convention_enum_t;

typedef struct s2_terrain_t {
  s2_interpolation_enum_t
    interpolationType; /*!< technique used for 2D interpolation */

  s2_coordinate_convention_enum_t
    coordinateConvention; /*!< ISO or SAE coordinate system */

  double scale[2]; /*!< scale factor for x,y dimensions of elevation map */
  double textureScale[2]; /*!< scale factor for x,y texture dimensions */

  double zMin; /*!< minimum z height in surface */
  double zMax; /*!< maximum z height in surface */

  double* controlPoints; /*!< control points loaded from image */
  size_t columns;        /*!< number of columns of elevation map */
  size_t rows;           /*!< number of rows in elevation map */

  char* propertyMap;         /*!< property map */
  size_t propertyMapRows;    /*!< number of rows in texture */
  size_t propertyMapColumns; /*!< number of columns in texture */

  GLfloat
    Kd[4]; /*! default diffuse color value if no visual texture is specified */
  GLfloat KsOverKd;
  GLfloat KaOverKd;
  GLint Ns;

  double (
    *getZ)(struct s2_terrain_t* terrain, s2_vector_t* position,
           s2_vector_t* normal); /*!< function used to obtain elevation at
                                    specified x, y location. This function
                                    pointer is set based on type of
                                    interpolation specified when terrain is
                                    created */

  s2_stackstr_t* name;
  s2_stackstr_t* parentBody;
  s2_body_t* parentBodyPointer;
  s2_stackstr_t* parentFrame;
  s2_frame_t* parentFramePointer;

  bool isVisible;

  char* elevationMapFileName;
  char* propertyMapFileName;
  char* diffuseMapFileName;
  char* specularMapFileName;
  char* normalMapFileName;
  char* bumpMapFileName;
} s2_terrain_t;

s2_terrain_t* s2_terrain();

void s2_terrain_free(s2_terrain_t*);

int s2_terrain_load(s2_assembly_t* assembly, json_t* assemblyObject,
                    s2_context_t* context, s2_stackstr_t* prefix);

void s2_generateGDALSampleTerrain(const char* filename, int, int,
                                  void (*surface)(double, double, double*));

void s2_generateSampleTerrain(const char* filename, int, int,
                              void (*surface)(double, double, double*));

double s2_terrain_get(s2_terrain_t* terrain, int i, int j);

double s2_terrain_getZ_default_sae(s2_terrain_t* terrain, s2_vector_t* r_0p,
                                   s2_vector_t* normal);

double s2_terrain_getZ_default_iso(s2_terrain_t* terrain, s2_vector_t* r_0p,
                                   s2_vector_t* normal);

double s2_terrain_getZ_bilinear_sae(s2_terrain_t* terrain, s2_vector_t*,
                                    s2_vector_t* normal);

double s2_terrain_getZ_bilinear_iso(s2_terrain_t* terrain, s2_vector_t*,
                                    s2_vector_t* normal);

double s2_terrain_getZ_bicubic_sae(s2_terrain_t* terrain, s2_vector_t*,
                                   s2_vector_t* normal);

double s2_terrain_getZ_bicubic_iso(s2_terrain_t* terrain, s2_vector_t*,
                                   s2_vector_t* normal);

double s2_terrain_getZ_catmulrom_sae(s2_terrain_t* terrain, s2_vector_t*,
                                     s2_vector_t* normal);

double s2_terrain_getZ_catmulrom_iso(s2_terrain_t* terrain, s2_vector_t*,
                                     s2_vector_t* normal);

void s2_terrain_getHeightAndSAENormal(s2_context_t* context, s2_vector_t*,
                                      double* height, s2_vector_t* normal);

void s2_terrain_getHeightAndISONormal(s2_context_t* context, s2_vector_t*,
                                      double* height, s2_vector_t* normal);

void s2_terrain_setColor(s2_terrain_t* terrain, float Kd[4]);

void s2_terrain_setCoordinateConvention(
  s2_terrain_t* terrain, s2_coordinate_convention_enum_t coordinateConvention);

void s2_terrain_setInterpolationConvention(
  s2_terrain_t* terrain, s2_interpolation_enum_t interpolation);

int s2_terrain_setElevationMap(s2_terrain_t* terrain,
                               const char* elevationMapFileName,
                               double scale[2]);

int s2_terrain_setGDALElevationMap(s2_terrain_t* terrain,
                                   const char* elevationMapFileName);

int s2_terrain_setPropertyMap(s2_terrain_t* terrain,
                              const char* propertyMapFileName);

void s2_terrain_setDiffuseMap(s2_terrain_t* terrain,
                              const char* diffuseMapFileName, double scale[2]);

void s2_terrain_setBumpMap(s2_terrain_t* terrain, const char* bumpMapFileName);

void s2_terrain_setSpecularMap(s2_terrain_t* terrain,
                               const char* specularMapFileName);

#endif

#include <float.h>
#include <jansson.h>
#include <string.h>
#include <limits.h>
#include <unistd.h>

#include "terrain.h"
#include "body.h"

#include "gdal.h"
#include "cpl_conv.h"
#include "ogr_srs_api.h"


/*!
@brief constructor for s2_terrain_t
Allocates memory for s2_terrain_t and sets values to zero or default values.
The default convention for terrain interpolation type is bilinear and coordinate type is SAE with positive Z down.

*/
s2_terrain_t* s2_terrain() {

  s2_terrain_t* terrain = (s2_terrain_t*)calloc(1, sizeof(s2_terrain_t));
  terrain->controlPoints = NULL;
  terrain->getZ = s2_terrain_getZ_default_sae;
  terrain->zMin = 0.0;
  terrain->zMax = 0.0;

  terrain->propertyMapColumns = 0;
  terrain->propertyMapRows = 0;

  terrain->Kd[0] = 63. / 255.;
  terrain->Kd[1] = 53. / 255.;
  terrain->Kd[2] = 69. / 255.;
  terrain->Kd[3] = 1.0;
  terrain->KsOverKd = 0.5;
  terrain->KaOverKd = 0.3;
  terrain->Ns = 1000;

  terrain->columns = 0;
  terrain->rows = 0;
  terrain->scale[0] = 1.0;
  terrain->scale[1] = 1.0;
  terrain->textureScale[0] = 1.0;
  terrain->textureScale[1] = 1.0;
  terrain->interpolationType = S2_INTERPOLATION_BILINEAR;
  terrain->coordinateConvention = S2_COORDINATECONVENTION_SAE;
  terrain->name = NULL;
  terrain->parentBody = NULL;
  terrain->parentBodyPointer = NULL;
  terrain->parentFrame = NULL;
  terrain->parentFramePointer = NULL;
  terrain->isVisible = true;

  terrain->elevationMapFileName = NULL;
  terrain->propertyMapFileName = NULL;
  terrain->diffuseMapFileName = NULL;
  terrain->specularMapFileName = NULL;
  terrain->normalMapFileName = NULL;
  terrain->bumpMapFileName = NULL;

  return terrain;
}


/*!
@brief terrain destructor
*/
void s2_terrain_free(s2_terrain_t* terrain) {

  if(terrain->controlPoints) {
    free(terrain->controlPoints);
    terrain->controlPoints = NULL;
  }

  if(terrain->propertyMap) {
    free(terrain->propertyMap);
    terrain->propertyMap = NULL;
  }

  if(terrain->elevationMapFileName) {
    free(terrain->elevationMapFileName);
    terrain->elevationMapFileName = NULL;
  }

  if(terrain->propertyMapFileName) {
    free(terrain->propertyMapFileName);
    terrain->propertyMapFileName = NULL;
  }

  if(terrain->diffuseMapFileName) {
    free(terrain->diffuseMapFileName);
    terrain->diffuseMapFileName = NULL;
  }

  if(terrain->normalMapFileName) {
    free(terrain->normalMapFileName);
    terrain->normalMapFileName = NULL;
  }

  if(terrain->specularMapFileName) {
    free(terrain->specularMapFileName);
    terrain->specularMapFileName = NULL;
  }

  if(terrain->bumpMapFileName) {
    free(terrain->bumpMapFileName);
    terrain->bumpMapFileName = NULL;
  }

  free(terrain);
  terrain = NULL;

}


/*!
@brief get a valid file path
Search all include directories to make sure file specified in json file exists
*/
char* getValidFilePath(
  s2_stackstr_t* includeDirectory,
  const char* fileName) {

  if(fileName == NULL) {
    return NULL;
  }

  char* validFilePath = NULL;

  /* if the filename begins with :, then an internal qt file object is specified. simply return without checking any include directories */
  if(fileName[0] == ':') {
    validFilePath = strdup(fileName);
    return validFilePath;
  }

  /* check if the user has specified an absolute directory first, in which case there is no need to check any of the include directories */
  asprintf(
    &validFilePath,
    "%s",
    fileName);
  if(access(validFilePath, F_OK) != -1) {
    return validFilePath;
  }

  s2_stackstrnode_t* includeDirectoryHead = includeDirectory->head;
  while(includeDirectoryHead) {
    asprintf(
      &validFilePath,
      "%s%s",
      includeDirectoryHead->value,
      fileName);
    if(access(validFilePath, F_OK) != -1) {
      break;
    } else {
      free(validFilePath);
      validFilePath = NULL;
    }
    includeDirectoryHead = includeDirectoryHead->next;
  }
  return validFilePath;
}


/*!
@brief loads terrain datastructure from json file
*/
int s2_terrain_load(
  s2_assembly_t* assembly,
  json_t* assemblyObject,
  s2_context_t* context,
  s2_stackstr_t* prefix) {

  json_t* terrainsObject = json_object_get(assemblyObject, "terrain");

  if(json_is_array(terrainsObject)) {
    size_t index;
    json_t* terrainObject;

    context->terrainCount =
      json_array_size(terrainsObject);

    context->terrains =
      malloc(context->terrainCount * sizeof(s2_terrain_t*));

    json_array_foreach(
      terrainsObject,
      index,
      terrainObject) {

      s2_terrain_t* terrain = s2_terrain();

      /* terrain map file path */
      const char* elevationMapFilePath =
        json_string_value(
          json_object_get(
            terrainObject,
            "elevation_map"));

      printf("elevation map file path: %s\n", elevationMapFilePath);

      char* fullElevationMapFilePath =
        getValidFilePath(
          context->modelParameters.includeDirectory,
          elevationMapFilePath);

      printf("full elevation map file path: %s\n", fullElevationMapFilePath);

      /* get x,y scale factor, defaults to 1 if no scale provided */
      double scale[2] = {1.0, 1.0};

      json_t* scaleObject = json_object_get(terrainObject, "scale");
      if(scaleObject != NULL) {
        if(json_is_number(scaleObject)) {
          scale[0] = scale[1] = json_number_value(scaleObject);
        } else if(json_is_array(scaleObject)) {
          size_t index;
          json_t* value;
          json_array_foreach(scaleObject, index, value) {
            if(json_is_number(value)) {
              scale[index] = json_number_value(value);
              if(index > 1) {
                break;
              }
            }
          }
        }
      }

      s2_terrain_setElevationMap(
        terrain, fullElevationMapFilePath, scale);

      if(fullElevationMapFilePath) {
        free(fullElevationMapFilePath);
      }


      const char* propertyMapFilePath =
        json_string_value(
          json_object_get(
            terrainObject,
            "property_map"));

      char* fullPropertyMapFilePath =
        getValidFilePath(
          context->modelParameters.includeDirectory,
          propertyMapFilePath);

      s2_terrain_setPropertyMap(terrain, fullPropertyMapFilePath);

      if(fullPropertyMapFilePath) {
        free(fullPropertyMapFilePath);
      }


      const char* diffuseMapFilePath =
        json_string_value(
          json_object_get(
            terrainObject,
            "diffuse_map"));

      char* fullDiffuseMapFilePath =
        getValidFilePath(
          context->modelParameters.includeDirectory,
          diffuseMapFilePath);

      double texture_scale[2] = {1.0, 1.0};

      scaleObject = json_object_get(terrainObject, "texture_scale");
      if(scaleObject != NULL) {
        if(json_is_number(scaleObject)) {
          texture_scale[0] = texture_scale[1] = json_number_value(scaleObject);
        } else if(json_is_array(scaleObject)) {
          size_t index;
          json_t* value;
          json_array_foreach(scaleObject, index, value) {
            if(json_is_number(value)) {
              texture_scale[index] = json_number_value(value);
              if(index > 1) {
                break;
              }
            }
          }
        }
      }

      s2_terrain_setDiffuseMap(terrain, fullDiffuseMapFilePath, texture_scale);
      if(fullDiffuseMapFilePath) {
        free(fullDiffuseMapFilePath);
      }

      const char* normalMapFilePath =
        json_string_value(
          json_object_get(
            terrainObject,
            "normal_map"));

      char* fullNormalMapFilePath =
        getValidFilePath(
          context->modelParameters.includeDirectory,
          normalMapFilePath);

      s2_terrain_setNormalMap(terrain, fullNormalMapFilePath);

      if(fullNormalMapFilePath) {
        free(fullNormalMapFilePath);
      }


      const char* specularMapFilePath =
        json_string_value(
          json_object_get(
            terrainObject,
            "specular_map"));

      char* fullSpecularMapFilePath =
        getValidFilePath(
          context->modelParameters.includeDirectory,
          specularMapFilePath);

      s2_terrain_setSpecularMap(terrain, fullSpecularMapFilePath);

      if(fullSpecularMapFilePath) {
        free(fullSpecularMapFilePath);
      }


      const char* bumpMapFilePath =
        json_string_value(
          json_object_get(
            terrainObject,
            "bump_map"));

      char* fullBumpMapFilePath =
        getValidFilePath(
          context->modelParameters.includeDirectory,
          bumpMapFilePath);

      s2_terrain_setBumpMap(terrain, fullBumpMapFilePath);

      if(fullBumpMapFilePath) {
        free(fullBumpMapFilePath);
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
      s2_terrain_setCoordinateConvention(terrain, coordinateType);

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
      s2_terrain_setInterpolationConvention(terrain, interpolationType);

      json_t* colorObject = json_object_get(terrainObject, "color");
      if(colorObject != NULL) {
        if(json_is_array(colorObject)) {
          size_t index;
          json_t* value;
          float Kd[4] = {0.0, 0.0, 0.0, 1.0};
          json_array_foreach(colorObject, index, value) {
            if(json_is_number(value) && index < 4) {
              Kd[index] = json_number_value(value);
            }
            /* block of code that uses index and value */
          }
          s2_terrain_setColor(terrain, Kd);
        }
      }

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
      } else if(json_is_string(parentBodyObject)) {
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
@brief if a texture is not used for the terrain visual properties, a custom uniform color can be used to for the diffuse terrain color. Color values should be in the range from 0 to 1 and follow the RGBA convention.
*/
void s2_terrain_setColor(s2_terrain_t* terrain, float Kd[4]) {
  memcpy(terrain->Kd, Kd, 4 * sizeof(float));
  for(int i = 0; i < 4; i++) {
    terrain->Kd[i] = fmin(terrain->Kd[i], 1.0);
  }
}


void s2_terrain_setGetZ(s2_terrain_t* terrain) {

  switch(terrain->coordinateConvention) {
    case S2_COORDINATECONVENTION_ISO: {
      if(terrain->controlPoints == NULL) {
        terrain->getZ = s2_terrain_getZ_default_iso;
        return;
      }
      switch(terrain->interpolationType) {
        case S2_INTERPOLATION_BICUBIC: {
          terrain->getZ = s2_terrain_getZ_bicubic_iso;
        }
        break;
        case S2_INTERPOLATION_BILINEAR: {
          terrain->getZ = s2_terrain_getZ_bilinear_iso;
        }
        break;
        case S2_INTERPOLATION_CATMULROM: {
          terrain->getZ = s2_terrain_getZ_catmulrom_iso;
        }
        break;
      }
    }
    break;
    case S2_COORDINATECONVENTION_SAE: {
      if(terrain->controlPoints == NULL) {
        terrain->getZ = s2_terrain_getZ_default_sae;
        return;
      }
      switch(terrain->interpolationType) {
        case S2_INTERPOLATION_BICUBIC: {
          terrain->getZ = s2_terrain_getZ_bicubic_sae;
        }
        break;
        case S2_INTERPOLATION_BILINEAR: {
          terrain->getZ = s2_terrain_getZ_bilinear_sae;
        }
        break;
        case S2_INTERPOLATION_CATMULROM: {
          terrain->getZ = s2_terrain_getZ_catmulrom_sae;
        }
        break;
      }
    }
    break;
  }
}


void s2_terrain_setCoordinateConvention(
  s2_terrain_t* terrain,
  s2_coordinate_convention_enum_t coordinateConvention) {

  terrain->coordinateConvention = coordinateConvention;
  s2_terrain_setGetZ(terrain);
}


void s2_terrain_setInterpolationConvention(
  s2_terrain_t* terrain,
  s2_interpolation_enum_t interpolation) {

  terrain->interpolationType = interpolation;
  s2_terrain_setGetZ(terrain);
}


void s2_image_properties(MagickWand* image) {

  printf(
    "  quantum-format: %s\n",
    MagickGetImageProperty(image, "quantum:format"));

  printf("  quantum-depth: %lu\n", MagickGetImageDepth(image));

  size_t quantumRange;
  printf(
    "  quantum-range: %s\n", MagickGetQuantumRange(&quantumRange));

  printf("  format: %s\n", MagickGetFormat(image));
}


/*!
@brief generate sample terrain using GDAL interface
*/
void s2_generateGDALSampleTerrain(
  const char* filename,
  int rows,
  int columns,
  void (*surface)(double x, double y, double* z)) {

  GDALAllRegister();
  GDALDriverH hDriver = GDALGetDriverByName("GTiff");
  char** gdalOptions = NULL;
  GDALDatasetH hDataset =
    GDALCreate(hDriver, filename, 64, 64, 1, GDT_Float32, gdalOptions);
  if(hDataset == NULL) {
    fprintf(stderr, "failed to open file\n");
    return;
  }

  double geoTransform[6] = {
    444720.0,  /*!< top left x */
    1.5,      /*!< w-e (x) pixel resolution */
    0.0,       /*!< 0 */
    3751320.0, /*!< top left y */
    0.0,       /*!< 0 */
    -1.5
  };    /*!< n-s pixel resolution (negative value) */
  GDALSetGeoTransform(hDataset, geoTransform);

  OGRSpatialReferenceH hSRS = OSRNewSpatialReference(NULL);
  OSRSetUTM(hSRS, 11, TRUE);
  OSRSetWellKnownGeogCS(hSRS, "NAD27");

  char* pszSRS_WKT = NULL;
  OSRExportToWkt(hSRS, &pszSRS_WKT);
  OSRDestroySpatialReference(hSRS);
  GDALSetProjection(hDataset, pszSRS_WKT);
  CPLFree(pszSRS_WKT);

  float data[rows * columns];
  double z;
  for(int y = 0; y < columns; y++) {
    for(int x = 0; x < rows; x++) {
      surface(x, y, &z);
      data[y * rows + x] = (float)z;
    }
  }

  GDALRasterBandH hBand = GDALGetRasterBand(hDataset, 1);
  GDALRasterIO(hBand, GF_Write, 0, 0, rows, columns,
               data, rows, columns, GDT_Float32, 0, 0);

  GDALClose(hDataset);
}


/*! @brief generates a sample terrain using provided "surface" function
saves to 32 bit tiff floating point format.  */
void s2_generateSampleTerrain(
  const char* filename,
  int rows,
  int columns,
  void (*surface)(double x, double y, double* z)) {

  MagickWandGenesis();
  ssize_t y;
  register ssize_t x;
  size_t width;

  MagickBooleanType status;
  MagickWand* image = NewMagickWand();
  PixelWand* pixelWand = NewPixelWand();

  status = MagickNewImage(image, columns, rows, pixelWand);
  if(status == MagickFalse) {
    fprintf(stderr, "failed to set size\n");
  }

  status = MagickSetDepth(image, 32);
  if(status == MagickFalse) {
    fprintf(stderr, "failed to set quantum depth\n");
  }

  status = MagickSetOption(image, "quantum:format", "floating-point");
  if(status == MagickFalse) {
    fprintf(stderr, "failed to set floating point quantum format\n");
  }

  status = MagickSetType(image, GrayscaleType);
  if(status == MagickFalse) {
    fprintf(stderr, "failed to set GrayScaleType\n");
  }

  s2_image_properties(image);

  /* image magick convention is that y is the left ordinate of view,
  i.e. the row index. x is the top ordinate of the view. Therefore, z
  is pointing down. This follows the SAE coordinate system for vehicle dynamics.
   */
  double z;
  PixelIterator* iterator = NewPixelIterator(image);
  if(iterator == (PixelIterator*) NULL) {
    ThrowWandException(image);
  }

  for(y = 0; y < (ssize_t) MagickGetImageHeight(image); y++) {

    PixelWand** pixels = PixelGetNextIteratorRow(iterator, &width);
    if(pixels == (PixelWand**) NULL) {
      break;
    }

    for(x = 0; x < (ssize_t) width; x++) {
      surface(x, y, &z);
      PixelSetRed(pixels[x], z);
      if(fabs(z - PixelGetRed(pixels[x])) > 1e-6) {
        fprintf(stderr, "%f != %f\n", z, PixelGetRed(pixels[x]));
      }
    }

    (void) PixelSyncIterator(iterator);
  }

  if(y < (long) MagickGetImageHeight(image)) {
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


/*!
@brief sets elevation map file location and loads elevation map from memory
@return status, 0 indicates success, -1 indicates no valid file was found and
*/
int s2_terrain_setElevationMap(
  s2_terrain_t* terrain,
  const char* elevationMapFileName,
  double scale[2]) {

  MagickWandGenesis();

  printf("attempting to open %s\n", elevationMapFileName);

  /* load elevation map using imagemagick */
  if(elevationMapFileName == NULL) {
    return -1;
  }
  MagickWand* image = NewMagickWand();
  MagickBooleanType status = MagickReadImage(image, elevationMapFileName);
  if(status == MagickFalse) {
    ThrowWandException(image);
    return -1;
  }
  terrain->elevationMapFileName = strdup(elevationMapFileName);
  memcpy(terrain->scale, scale, 2 * sizeof(double));

  /* initialize elevation map, set all values to zero */
  if(terrain->controlPoints) {
    free(terrain->controlPoints);
    terrain->controlPoints = NULL;
  }
  terrain->zMin = DBL_MAX;
  terrain->zMax = -DBL_MAX;
  unsigned long columns = terrain->columns = MagickGetImageWidth(image);
  unsigned long rows = terrain->rows = MagickGetImageHeight(image);

  printf(
    "Loading elevation map %s with size (%lu, %lu)\n",
    elevationMapFileName, rows, columns);

  terrain->controlPoints = calloc(rows * columns, sizeof(double));


  /* load values into array following sae convention with x as
  top ordinate of view (column index) and y as left ordinate of view
  (row index), and +z is pointing down */

  s2_image_properties(image);

  unsigned long width;
  PixelIterator* iterator = NewPixelIterator(image);
  for(long y = 0; y < (long) MagickGetImageHeight(image); y++) {

    PixelWand** pixels = PixelGetNextIteratorRow(iterator, &width);
    if(pixels == (PixelWand**) NULL) {
      break;
    }

    for(register long x = 0; x < (long) width; x++) {
      // PixelGetMagickColor(pixels[x], &pixel);
      // printf("%f\n", PixelGetRed(pixels[x]));

      double value = PixelGetRed(pixels[x]);
      terrain->controlPoints[y * columns + x] = value;
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

  MagickWandTerminus();
  s2_terrain_setGetZ(terrain);

  return 0;
}


/*!
@brief sets elevation map file location and loads elevation map from memory
@return status, 0 indicates success, -1 indicates no valid file was found and
*/
int s2_terrain_setGDALElevationMap(
  s2_terrain_t* terrain,
  const char* elevationMapFileName) {

  GDALAllRegister();
  GDALDatasetH hDataset = GDALOpen(elevationMapFileName, GA_ReadOnly);
  if(hDataset == NULL) {
    fprintf(stderr, "failed to open GDAL dataset\n");
    return;
  }
  GDALDriverH hDriver = GDALGetDatasetDriver(hDataset);
  printf(
    "Driver: %s/%s\n",
    GDALGetDriverShortName(hDriver),
    GDALGetDriverLongName(hDriver));
  printf(
    "Size is %dx%dx%d\n",
    GDALGetRasterXSize(hDataset),
    GDALGetRasterYSize(hDataset),
    GDALGetRasterCount(hDataset));
  if(GDALGetProjectionRef(hDataset) != NULL) {
    printf("Projection is `%s'\n", GDALGetProjectionRef(hDataset));
  }
  double geoTransform[6];
  if(GDALGetGeoTransform(hDataset, geoTransform) == CE_None) {
    printf("Origin = (%.6f,%.6f)\n",
           geoTransform[0], geoTransform[3]);
    printf("Pixel Size = (%.6f,%.6f)\n",
           geoTransform[1], geoTransform[5]);
  }

  GDALClose(hDataset);

  s2_terrain_setGetZ(terrain);

  return 0;
}


/*!
@brief sets property map file location and loads property map from memory
@return status, 0 indicates success, -1 indicates no valid file was found and
the s2_terrain_t datastructure was not modified
*/
int s2_terrain_setPropertyMap(
  s2_terrain_t* terrain,
  const char* propertyMapFileName) {

  if(propertyMapFileName == NULL) {
    return -1;
  }

  MagickWandGenesis();

  MagickWand* propertyMap = NewMagickWand();
  MagickBooleanType status =
    MagickReadImage(propertyMap, propertyMapFileName);
  if(status == MagickFalse) {
    ThrowWandException(propertyMap);
    return -1;
  }

  if(terrain->propertyMap) {
    free(terrain->propertyMap);
    terrain->propertyMap = NULL;
  }

  unsigned long propertyMapColumns = MagickGetImageWidth(propertyMap);
  unsigned long propertyMapRows = MagickGetImageHeight(propertyMap);
  terrain->propertyMapColumns = propertyMapColumns;
  terrain->propertyMapRows = propertyMapRows;

  terrain->propertyMap =
    calloc(propertyMapRows * propertyMapColumns, sizeof(char));

  unsigned long width;
  MagickPixelPacket pixel;

  PixelIterator* propertyMapIterator = NewPixelIterator(propertyMap);
  for(unsigned long y = 0; y < (long) MagickGetImageHeight(propertyMap); y++) {

    PixelWand** pixels = PixelGetNextIteratorRow(propertyMapIterator, &width);
    if(pixels == (PixelWand**) NULL) {
      break;
    }

    for(register unsigned long x = 0; x < (long) width; x++) {
      PixelGetMagickColor(pixels[x], &pixel);

      terrain->propertyMap[y * propertyMapColumns + x] =
        PixelGetRedQuantum(pixels[x]);
    }

    (void) PixelSyncIterator(propertyMapIterator);
  }

  propertyMap = DestroyMagickWand(propertyMap);

  MagickWandTerminus();
  return 0;

}


/*!
@brief sets visual texture file location, does not load from memory
*/
void s2_terrain_setDiffuseMap(
  s2_terrain_t* terrain,
  const char* diffuseMapFileName,
  double scale[2]) {

  if(diffuseMapFileName) {
    terrain->diffuseMapFileName = strdup(diffuseMapFileName);
  } else {
    terrain->diffuseMapFileName = NULL;
  }
  memcpy(terrain->textureScale, scale, 2 * sizeof(double));
}


/*!
@brief sets normal map file location, does not load into memory in core solver. only used in GUI.
*/
void s2_terrain_setNormalMap(
  s2_terrain_t* terrain,
  const char* normalMapFileName) {

  if(normalMapFileName) {
    terrain->normalMapFileName = strdup(normalMapFileName);
  } else {
    terrain->normalMapFileName = NULL;
  }
}


/*!
@brief sets specular map file location, does not load from memory
*/
void s2_terrain_setSpecularMap(
  s2_terrain_t* terrain,
  const char* specularMapFileName) {

  if(specularMapFileName) {
    terrain->specularMapFileName = strdup(specularMapFileName);
  } else {
    terrain->specularMapFileName = NULL;
  }
}


/*!
@brief sets bump map file location, does not load from memory
*/
void s2_terrain_setBumpMap(
  s2_terrain_t* terrain,
  const char* bumpMapFileName) {

  if(bumpMapFileName) {
    terrain->bumpMapFileName = strdup(bumpMapFileName);
  } else {
    terrain->bumpMapFileName = NULL;
  }
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


/*!
@brief get value at specified row and column index.  */
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
@brief default function to retrieve elevation and normal for flat height map. Follows SAE convention
*/
double s2_terrain_getZ_default_sae(
  s2_terrain_t* terrain,
  s2_vector_t* r_0p,
  s2_vector_t* normal) {

  normal->x = 0.0;
  normal->y = 0.0;
  normal->z = -1.0;
  return 0.0;
}


/*!
@brief default function to retrieve elevation and normal for flat height map. Follows SAE convention
*/
double s2_terrain_getZ_default_iso(
  s2_terrain_t* terrain,
  s2_vector_t* r_0p,
  s2_vector_t* normal) {

  normal->x = 0.0;
  normal->y = 0.0;
  normal->z = 1.0;
  return 0.0;
}


/*!
@brief get z using bilinear interpolation at specified x and y position
following SAE convention with positive z "down". The normal is returned in
the "up" direction.

@TODO normal vector not computed yet
*/
double s2_terrain_getZ_bilinear_sae(
  s2_terrain_t* terrain,
  s2_vector_t* r_0p,
  s2_vector_t* normal) {

  /* transform x and y into local height map frame (1) */
  s2_vector_t r_1p;
  s2_product_dinversev(terrain->parentFramePointer->dAbsolute, r_0p, &r_1p);

  /* get x and y values in original height map */
  double xPos = r_1p.x / terrain->scale[0];
  double yPos = r_1p.y / terrain->scale[1];

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
  s2_vector_t* r_0p,
  s2_vector_t* normal) {

  /* transform x and y into local height map frame (1) */
  s2_vector_t r_1p;
  s2_product_dinversev(terrain->parentFramePointer->dAbsolute, r_0p, &r_1p);

  /* get x and y values in original height map */
  double xPos = r_1p.x / terrain->scale[0];
  double yPos = r_1p.y / terrain->scale[1];

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
  s2_vector_t* r_0p,
  s2_vector_t* normal) {

  /* transform x and y into local height map frame (1) */
  s2_vector_t r_1p;
  s2_product_dinversev(terrain->parentFramePointer->dAbsolute, r_0p, &r_1p);

  /* get x and y values in original height map */
  double xPos = r_1p.x / terrain->scale[0];
  double yPos = r_1p.y / terrain->scale[1];

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

  z_y /= terrain->scale[1];

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

  z_x /= terrain->scale[0];

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
  s2_vector_t* r_0p,
  s2_vector_t* normal) {

  /* transform x and y into local height map frame (1) */
  s2_vector_t r_1p;
  s2_product_dinversev(terrain->parentFramePointer->dAbsolute, r_0p, &r_1p);

  /* get x and y values in original height map */
  double xPos = r_1p.x / terrain->scale[0];
  double yPos = r_1p.y / terrain->scale[1];

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

  z_x /= terrain->scale[0];

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

  z_y /= terrain->scale[1];

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
  s2_vector_t* r_0p,
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
  s2_vector_t* r_0p,
  s2_vector_t* normal) {

  double z = 0;
  return z;
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
  s2_vector_t* r_0p,
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
@param r_0p vector from reference frame to point p to test, expressed in global coordinates
@param height [out] height at x, y coordinate
@param normal [out] surface normal at x, y coordinate (in global coordinates)
This is a place holder function for now. In the future need to add functions to
load terrain profiles and report values based on terrain.
The surface normal follows ISO convention with normal in the "up" direction,
i.e. {0, 0, -1}
*/
void s2_terrain_getHeightAndISONormal(
  s2_context_t* context,
  s2_vector_t* r_0p,
  double* height,
  s2_vector_t* normal) {

  *height = 0.0;
  normal->x = 0.0;
  normal->y = 0.0;
  normal->z = -1.0;
}

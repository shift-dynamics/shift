/**
@file
@author Kristopher Wehage
@brief Utilities to initialize and free a Body datastructure.
@date 2017
@copyright Kristopher Wehage 2016

This function library is used to initialize, free memory for, and operate on
data structures defined in body.h

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

#include <assimp/cimport.h> // Plain-C interface to assimp loader
#include <assimp/material.h>
#include <assimp/postprocess.h> // Post processing flags
#include <assimp/scene.h>       // Output data structure
#include <jansson.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "assembly.h"
#include "body.h"
#include "collisionobject.h"
#include "context.h"
#include "convexhull.h"
#include "frame.h"
#include "inertia.h"
#include "material.h"
#include "stack.h"
#include "utility.h"

int s2_body_loadCollisionGeometry(s2_context_t*, s2_body_t*,
                                  s2_collisionobject_enum_t);

/*!
@brief Initialize an s2_body_t datastructure. Appends the origin frame to the
body frame

@param None
@returns a s2_body_t datastructure
*/
s2_body_t* s2_body(s2_stackstr_t* name) {
  s2_body_t* body = malloc(sizeof(s2_body_t));
  if(!body) {
    fprintf(stderr, "Error: failed to initialize body.\n");
    return NULL;
  }
  body->name = NULL;
  body->index = -1;
  body->level = -1;
  body->assembly = NULL;
  body->definition = NULL;
  body->isVirtual = false;
  body->frameHead = NULL;
  body->frameTail = NULL;
  body->referenceFrame = NULL;
  body->origin = NULL;
  body->frameCount = 0;
  body->inertia = s2_inertia();
  body->parentJointPointer = NULL;
  body->parentBodyPointer = NULL;
  body->next = NULL;
  body->geometryInstanceOf = NULL;
  body->collisionGeometryInstanceOf = NULL;
  body->materials = NULL;
  body->materialCount = 0;
  body->materialIndex = -1;
  body->centerOfMassWRTOrigin = s2_vector();
  body->centerOfMassWRTReference = s2_vector();
  // body->glReferenceFrame = NULL;
  // body->glReferenceFramef = NULL;
  // body->glModelView = NULL;
  // body->glModelViewProjection = NULL;
  body->collisionObjectCount = 0;
  body->collisionObjectOffset = -1;
  body->drawEnabled = true;
  body->collisionObjectDrawEnabled = true;
  body->name = name;

  /* append origin to body */
  s2_frame_t* origin = s2_frame();
  origin->isOrigin = true;
  origin->name = s2_stackstr_copy(body->name);
  s2_stackstr_append(origin->name, "origin");
  body->origin = origin;
  body->referenceFrame = origin;
  s2_body_appendFrame(body, origin);
  body->isVisible = true;
  return body;
}

/*!
@brief Free a s2_body_t datastructure
@param body an s2_body_t datastructure
*/
void s2_body_free(s2_body_t* body) {

  if(body) {
    if(body->isVirtual) {
      s2_virtualbody_free(body);
    } else {
      s2_stackstr_free(body->name);
      if(body->definition) {
        s2_stackstr_free(body->definition);
      }
      s2_inertia_free(body->inertia);
      if(!body->geometryInstanceOf) {
        s2_material_free(body->materials, body->materialCount);
      }
      free(body->centerOfMassWRTOrigin);
      free(body->centerOfMassWRTReference);
      s2_frame_t* frameHead = body->frameHead;
      while(frameHead) {
        s2_frame_t* nextFrame = frameHead->next;
        s2_frame_free(frameHead);
        frameHead = nextFrame;
      }
      free(body);
      body = NULL;
    }
  }
}

/*!
@brief Append a frame datastructure to an s2_body_t datastructure

@param body a Body datastructure to append to
@param frame a Frame datastructure to append
*/
void s2_body_appendFrame(s2_body_t* body, s2_frame_t* frame) {

  if(!body->frameTail) {
    body->frameHead = frame;
    body->frameTail = frame;
  } else {
    body->frameTail->next = frame;
    body->frameTail = frame;
  }
}

/*!
@brief print frames in a s2_body_t datastructure
@param body a Body datastructure
*/
void s2_body_printFrames(FILE* f, s2_body_t* body) {

  s2_frame_t* frameHead = body->frameHead;
  while(frameHead) {
    fprintf(f, "\tFrame: ");
    s2_stackstr_print(f, frameHead->name);
    // printf("\t\tframePointer: %p\n", frameHead);
    fprintf(f, "\t\tdWRTOrigin:\n");
    s2_displacement_print(f, frameHead->dWRTOrigin);
    fprintf(f, "\t\tdWRTOriginInverse:\n");
    s2_displacement_print(f, frameHead->dWRTOriginInverse);
    fprintf(f, "\t\tdWRTReference:\n");
    s2_displacement_print(f, frameHead->dWRTReference);
    fprintf(f, "\t\trotationWRTReferenceInverse:\n");
    s2_displacement_print(f, frameHead->dWRTReferenceInverse);
    fprintf(f, "\t\trotationAbsolute:\n");
    s2_displacement_print(f, frameHead->dAbsolute);
    if(frameHead->velocityAbsolute) {
      fprintf(f, "\t\tvelocityAbsolute:\n");
      s2_screw_print(f, frameHead->velocityAbsolute);
    }
    if(frameHead->accelerationQuadraticAbsolute) {
      fprintf(f, "\t\taccelerationQuadraticAbsolute:\n");
      s2_screw_print(f, frameHead->accelerationQuadraticAbsolute);
    }
    if(frameHead->referenceFramePointer) {
      fprintf(f, "\t\tReference Frame: ");
      s2_stackstr_print(f, frameHead->referenceFramePointer->name);
    }
    fprintf(f, "\t\tisParent: %s\n", frameHead->isReference ? "True" : "False");
    frameHead = frameHead->next;
  }
}

/*!
@brief Free a virtual s2_body_t datastructure
@param body a virtual Body datastructure to free
*/
void s2_virtualbody_free(s2_body_t* body) {

  if(body) {
    s2_stackstr_free(body->name);
    if(body->definition) {
      s2_stackstr_free(body->definition);
    }
    s2_inertia_free(body->inertia);
    free(body->centerOfMassWRTOrigin);
    free(body->centerOfMassWRTReference);
    s2_frame_t* frameHead = body->frameHead;
    while(frameHead) {
      s2_frame_t* nextFrame = frameHead->next;
      s2_stackstr_free(frameHead->name);
      free(frameHead->dWRTOrigin);
      frameHead->dWRTOrigin = NULL;
      free(frameHead->dWRTOriginInverse);
      frameHead->dWRTOriginInverse = NULL;
      // if(frameHead->dAbsolute) free(frameHead->dAbsolute);
      // frameHead->dAbsolute = NULL;
      free(frameHead);
      frameHead = NULL;
      frameHead = nextFrame;
    }
    free(body);
    body = NULL;
  }
}

/*!
@brief load body to an assembly data structure
@return status: 0 indicates that body geometry was successfully loaded, -1
indicates failure in parsing geometry file, -2 indicates no body geometry was
found, -3 indicates body is inactive and nothing was loaded
*/
int s2_body_load(s2_context_t* context, s2_assembly_t* assembly,
                 s2_stackstr_t* prefix, json_t* bodyObject) {

  bool active = true;
  json_t* activeObject = json_object_get(bodyObject, "active");
  if(activeObject != NULL) {
    active = json_boolean_value(activeObject);
  }

  if(!active) {
    return -3;
  }

  /* get body descriptive name */
  s2_stackstr_t* bodyName = s2_stackstr_copy(prefix);
  const char* bodyRootName =
    json_string_value(json_object_get(bodyObject, "name"));
  s2_stackstr_append(bodyName, bodyRootName);
  s2_body_t* body = s2_body(bodyName);

  body->assembly = assembly;

  /* get body definition files */
  json_t* definitionsObject = json_object_get(bodyObject, "definition");

  body->definition = s2_stackstr();

  if(json_is_null(definitionsObject)) {
    // printf("no definition files loaded\n");
  } else if(json_is_string(definitionsObject)) {
    s2_stackstr_append(body->definition, json_string_value(definitionsObject));
  } else if(json_is_array(definitionsObject)) {
    json_t* definitionObject;
    size_t i;
    json_array_foreach(definitionsObject, i, definitionObject) {
      s2_stackstr_append(body->definition, json_string_value(definitionObject));
    }
  }

  // printf("body: ");
  // s2_stackstr_print(stdout, body->name);
  // printf("definition: ");
  // s2_stackstr_print(stdout, body->definition);

  json_t* framesObject = json_object_get(bodyObject, "frames");

  if(!json_is_null(framesObject)) {
    json_t* frameObject;
    size_t frameIndex;

    json_array_foreach(framesObject, frameIndex, frameObject) {

      s2_frame_load(body, frameObject);
    }
  }

  /* get mass */
  body->inertia->m = json_number_value(json_object_get(bodyObject, "mass"));

  /* get body center of mass */
  for(int i = 0; i < 3; i++) {

    ((double*)body->centerOfMassWRTOrigin)[i] = json_number_value(
      json_array_get(json_object_get(bodyObject, "center_of_mass"), i));
  }

  s2_product_vk(body->centerOfMassWRTOrigin, &(body->inertia->m),
                &(body->inertia->mt));

  body->inertia->Ixx = json_number_value(
    json_object_get(json_object_get(bodyObject, "moment_of_inertia"), "Ixx"));

  body->inertia->Ixy = json_number_value(
    json_object_get(json_object_get(bodyObject, "moment_of_inertia"), "Ixy"));

  body->inertia->Izx = json_number_value(
    json_object_get(json_object_get(bodyObject, "moment_of_inertia"), "Izx"));
  body->inertia->Iyy = json_number_value(
    json_object_get(json_object_get(bodyObject, "moment_of_inertia"), "Iyy"));

  body->inertia->Iyz = json_number_value(
    json_object_get(json_object_get(bodyObject, "moment_of_inertia"), "Iyz"));

  body->inertia->Izz = json_number_value(
    json_object_get(json_object_get(bodyObject, "moment_of_inertia"), "Izz"));

  (context->bodyCount)++;

  s2_assembly_appendBody(assembly, body);

  /* load body geometry */
  int exitcode = s2_body_loadGeometry(context, body);

  const char* collisionGeometry = NULL;
  collisionGeometry =
    json_string_value(json_object_get(bodyObject, "collision_geometry"));

  s2_collisionobject_enum_t collisionObjectType = S2_COLLISIONOBJECT_CONVEXHULL;
  if(collisionGeometry) {
    collisionObjectType = s2_collisionobject_enumFromChar(collisionGeometry);
  }

  s2_body_loadCollisionGeometry(context, body, collisionObjectType);
  return exitcode;
}

s2_body_t* s2_geometryInUse(s2_assembly_t* assembly, s2_body_t* body,
                            const char* filename) {

  s2_body_t* bodyHead = assembly->bodyHead;
  while(bodyHead) {

    // printf("name in geometryInUse: ");
    // s2_stackstr_print(stdout, bodyHead->name);
    // printf("definition in geometryInUse: ");
    // s2_stackstr_print(stdout, bodyHead->definition);
    if(body != bodyHead) {
      if(s2_stackstr_in(bodyHead->definition, filename)) {
        // printf("returning bodyHead");
        // s2_stackstr_print(stdout, bodyHead->name);
        return bodyHead;
      }
    }
    bodyHead = bodyHead->next;
  }
  s2_assembly_t* subassemblyHead = assembly->subAssemblyHead;
  while(subassemblyHead) {
    s2_body_t* instance = s2_geometryInUse(subassemblyHead, body, filename);
    if(instance) {
      // printf("returning instance");
      // s2_stackstr_print(stdout, instance->name);
      return instance;
    }
    subassemblyHead = subassemblyHead->next;
  }
  return NULL;
}

s2_body_t*
s2_collisionGeometryInUse(s2_context_t* context, s2_assembly_t* assembly,
                          s2_body_t* body, const char* filename,
                          s2_collisionobject_enum_t collisionObjectType) {

  s2_body_t* bodyHead = assembly->bodyHead;
  while(bodyHead) {
    if(body != bodyHead) {
      if(bodyHead->collisionObjectCount &&
         s2_stackstr_in(bodyHead->definition, filename) &&
         context->collisionObjects[bodyHead->collisionObjectOffset + 0].type ==
           collisionObjectType) {
        return bodyHead;
      }
    }
    bodyHead = bodyHead->next;
  }
  s2_assembly_t* subassemblyHead = assembly->subAssemblyHead;
  while(subassemblyHead) {
    s2_body_t* instance = s2_collisionGeometryInUse(
      context, subassemblyHead, body, filename, collisionObjectType);
    if(instance) {
      return instance;
    }
    subassemblyHead = subassemblyHead->next;
  }
  return NULL;
}

/*! @brief get path to obj file definition
 */
const char* s2_getGeometryFileName(s2_body_t* body) {
  s2_stackstrnode_t* definitionHead = body->definition->head;
  while(definitionHead) {
    if(!strcmp(s2_getFileExtension(definitionHead->value), "obj")) {
      return definitionHead->value;
    }
    definitionHead = definitionHead->next;
  }
  return NULL;
}

/*!
@brief load collision geometry associated with body
*/
int s2_body_loadCollisionGeometry(
  s2_context_t* context, s2_body_t* body,
  s2_collisionobject_enum_t collisionObjectType) {

  /* if the user has specified not to use a collision object, exit and
  print warning to user */
  if(collisionObjectType == S2_COLLISIONOBJECT_NONE) {
    printf("no collision object specified for body:");
    s2_stackstr_print(stdout, body->name);
    return -1;
  }

  /* if the filename is not specified, the body has no geometry specified */
  const char* fileName = s2_getGeometryFileName(body);
  if(fileName == NULL) {
    return -2;
  }

  /* check if collision geometry was used in specification of another body
  Both the filename specifying body geometry and collisionobject type must match
  */
  printf("loading collision geometry for: %s\n", fileName);
  // s2_body_t* body2 = body;
  body->collisionGeometryInstanceOf = s2_collisionGeometryInUse(
    context, context->assembly, body, fileName, collisionObjectType);

  if(body->collisionGeometryInstanceOf) {
    printf("collision geometry already loaded on body: ");
    s2_stackstr_print(stdout, body->name);
  }
  switch(collisionObjectType) {
    case S2_COLLISIONOBJECT_CONVEXHULL: {
      s2_collisionobject_convexhull(context, body);
      break;
    }
    case S2_COLLISIONOBJECT_ELLIPSOID: {
      s2_collisionobject_ellipsoid(context, body);
      break;
    }
    case S2_COLLISIONOBJECT_SPHERE: {
      s2_collisionobject_sphere(context, body);
      break;
    }
    case S2_COLLISIONOBJECT_OBB: {
      printf("object aligned bounding boxes not handled yet\n");
      return -1;
    }
    case S2_COLLISIONOBJECT_AABB: {
      printf("axis aligned bounding boxes not handled yet\n");
      return -1;
    }
    default:
      break;
  }
  return 0;
}

/*! @brief Loads geometry specified in Body properties from an external file
using ASSIMP (open asset importer)

@param context a context datastructure
@param body a body datastructure
*/
int s2_body_loadGeometry(s2_context_t* context, s2_body_t* body) {

  const char* fileName = s2_getGeometryFileName(body);
  printf("filename: %s\n", fileName);

  /* if the filename is not specified, the body has no geometry specified */
  if(fileName == NULL) {
    return -2;
  }

  printf("body before loading: ");
  s2_stackstr_print(stdout, body->name);
  printf("body definition before loading: ");
  s2_stackstr_print(stdout, body->definition);

  /* check if file was used in specification of another body */
  body->geometryInstanceOf =
    s2_geometryInUse(context->assembly, body, fileName);

  printf("body after loading: ");
  s2_stackstr_print(stdout, body->name);
  printf("body definition after loading: ");
  s2_stackstr_print(stdout, body->definition);

  printf("geometry instance of: ");
  if(body->geometryInstanceOf) {
    s2_stackstr_print(stdout, body->geometryInstanceOf->name);
    if(body == body->geometryInstanceOf) {
      printf("body and instance are the same body equal!\n");
    }

    printf("body1 definition: ");
    s2_stackstr_print(stdout, body->definition);
    printf("body2 definition: ");
    s2_stackstr_print(stdout, body->geometryInstanceOf->definition);
  } else {
    printf("null\n");
  }

  /*
    Access the file contents, load in to body data structure
  */
  // printf("number of meshes: %d\n", scene->mNumMeshes);
  if(body->geometryInstanceOf) {
    body->materialIndex = body->geometryInstanceOf->materialIndex;
    body->materialCount = body->geometryInstanceOf->materialCount;
    body->materials = body->geometryInstanceOf->materials;
  } else {
    /* check if file exists in all user specified include directories */
    char* fullPathFileName = NULL;
    s2_stackstrnode_t* includeDirectoryHead =
      context->modelParameters.includeDirectory->head;
    while(includeDirectoryHead) {
      if(fullPathFileName) {
        free(fullPathFileName);
      }
      asprintf(&fullPathFileName, "%s%s", includeDirectoryHead->value,
               fileName);
      if(access(fullPathFileName, F_OK) != -1) {
        break;
      }
      includeDirectoryHead = includeDirectoryHead->next;
    }
    // printf("fullPathFileName: %s\n", fullPathFileName);

    const struct aiScene* scene = aiImportFile(
      fullPathFileName, aiProcess_CalcTangentSpace | aiProcess_Triangulate |
                          aiProcess_SortByPType);

    /* If loading was not successful, report import error and exit */
    if(!scene) {
      printf("%s\n", aiGetErrorString());
      printf("error, exiting.\n");
      return -1;
    }
    body->materialCount = scene->mNumMeshes;
    body->materialIndex = context->materialCount;

    if(!context->materials) {
      context->materials = malloc(body->materialCount * sizeof(s2_material_t));
    } else {
      printf("reallocing materials\n");
      context->materials = realloc(
        context->materials,
        (context->materialCount + body->materialCount) * sizeof(s2_material_t));
    }
    for(int i = context->materialCount;
        i < context->materialCount + body->materialCount; i++) {
      s2_material_init(&context->materials[i]);
      context->materials[i].index = i;
    }
    context->materialCount += body->materialCount;

    uint totalVertexCount = 0;
    uint totalFaceCount = 0;
    for(uint i = 0; i < scene->mNumMeshes; i++) {
      totalVertexCount += scene->mMeshes[i]->mNumVertices;
      totalFaceCount += scene->mMeshes[i]->mNumFaces;
    }

    GLfloat* vertices = malloc(3 * totalVertexCount * sizeof(GLfloat));
    GLfloat* vertexNormals = malloc(3 * totalVertexCount * sizeof(GLfloat));
    GLuint* faces = malloc(3 * totalFaceCount * sizeof(GLuint));

    totalVertexCount = 0;
    totalFaceCount = 0;
    for(uint i = 0; i < scene->mNumMeshes; i++) {

      s2_material_t* material = &context->materials[body->materialIndex + i];

      /* load vertices */
      material->vertices = &vertices[totalVertexCount * 3];
      material->vertexCount = scene->mMeshes[i]->mNumVertices;

      for(uint j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
        material->vertices[3 * j + 0] = scene->mMeshes[i]->mVertices[j].x;
        material->vertices[3 * j + 1] = scene->mMeshes[i]->mVertices[j].y;
        material->vertices[3 * j + 2] = scene->mMeshes[i]->mVertices[j].z;
      }

      /* load vertex normals */
      material->vertexNormals = &vertexNormals[totalVertexCount * 3];
      for(uint j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
        material->vertexNormals[3 * j + 0] = scene->mMeshes[i]->mNormals[j].x;
        material->vertexNormals[3 * j + 1] = scene->mMeshes[i]->mNormals[j].y;
        material->vertexNormals[3 * j + 2] = scene->mMeshes[i]->mNormals[j].z;
      }
      totalVertexCount += material->vertexCount;

      /* load faces */
      material->faceCount = scene->mMeshes[i]->mNumFaces;
      material->faces = &faces[totalFaceCount * 3];
      for(int j = 0; j < material->faceCount; j++) {
        material->faces[j * 3 + 0] = scene->mMeshes[i]->mFaces[j].mIndices[0];
        material->faces[j * 3 + 1] = scene->mMeshes[i]->mFaces[j].mIndices[1];
        material->faces[j * 3 + 2] = scene->mMeshes[i]->mFaces[j].mIndices[2];
      }
      totalFaceCount += material->faceCount;

      /* load color properties */
      int ret1;
      struct aiMaterial* mtl;
      mtl = scene->mMaterials[scene->mMeshes[i]->mMaterialIndex];

      unsigned int max = 1;
      float opacity_f = 1.f;
      ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_OPACITY, &opacity_f, 0);
      if(ret1 == AI_SUCCESS) {
        printf("opacity: %f\n", opacity_f);
      }

      struct aiColor4D diffuse;
      ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse);
      if(ret1 == AI_SUCCESS) {
        printf("diffuse: %f %f %f %f\n", diffuse.r, diffuse.g, diffuse.b,
               diffuse.a);
        material->GLuniforms.Kd[0] = diffuse.r;
        material->GLuniforms.Kd[1] = diffuse.g;
        material->GLuniforms.Kd[2] = diffuse.b;
        material->GLuniforms.Kd[3] = opacity_f;
      } else {
        material->GLuniforms.Kd[0] = 0.6;
        material->GLuniforms.Kd[1] = 0.6;
        material->GLuniforms.Kd[2] = 0.6;
        material->GLuniforms.Kd[3] = opacity_f;
      }

      struct aiColor4D ambient;
      ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient);
      if(ret1 == AI_SUCCESS) {
        material->GLuniforms.Ka[0] = ambient.r;
        material->GLuniforms.Ka[1] = ambient.g;
        material->GLuniforms.Ka[2] = ambient.b;
        material->GLuniforms.Ka[3] = opacity_f;
      } else {
        material->GLuniforms.Ka[0] = 0.6;
        material->GLuniforms.Ka[1] = 0.6;
        material->GLuniforms.Ka[2] = 0.6;
        material->GLuniforms.Ka[3] = opacity_f;
      }

      struct aiColor4D specular;
      ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular);
      if(ret1 == AI_SUCCESS) {
        // printf("specular: %f %f %f %f\n", specular.r, specular.g, specular.b,
        // specular.a);
        material->GLuniforms.Ks[0] = specular.r;
        material->GLuniforms.Ks[1] = specular.g;
        material->GLuniforms.Ks[2] = specular.b;
        material->GLuniforms.Ks[3] = opacity_f;
      } else {
        material->GLuniforms.Ks[0] = 0.6;
        material->GLuniforms.Ks[1] = 0.6;
        material->GLuniforms.Ks[2] = 0.6;
        material->GLuniforms.Ks[3] = opacity_f;
      }

      struct aiColor4D emissive;
      ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emissive);
      if(ret1 == AI_SUCCESS) {
        material->GLuniforms.Ke[0] = emissive.r;
        material->GLuniforms.Ke[1] = emissive.g;
        material->GLuniforms.Ke[2] = emissive.b;
        material->GLuniforms.Ke[3] = emissive.a;
      } else {
        material->GLuniforms.Ke[0] = 0.0;
        material->GLuniforms.Ke[1] = 0.0;
        material->GLuniforms.Ke[2] = 0.0;
        material->GLuniforms.Ke[3] = opacity_f;
      }

      /* this set of parameters is currently not used / loaded */
      float shininess = 0.f;
      ret1 =
        aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
      // if(ret1 == AI_SUCCESS)
      //   printf("shininess: %f\n", shininess);

      float shininess_strength = 0.f;
      ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH,
                                     &shininess_strength, &max);
      // if(ret1 == AI_SUCCESS)
      //   printf("shininess_strength: %f\n", shininess_strength);

      float reflectivity = 0.f;
      ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_REFLECTIVITY, &reflectivity,
                                     &max);
      // if(ret1 == AI_SUCCESS)
      //   printf("reflectivity: %f\n", reflectivity);

      float refracti = 0.f;
      ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_REFRACTI, &refracti, &max);
      // if(ret1 == AI_SUCCESS)
      //   printf("refracti: %f\n", refracti);

      struct aiColor4D reflective;
      reflective.r = 0.f;
      reflective.g = 0.f;
      reflective.b = 0.f;
      reflective.a = 0.f;
      ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_REFLECTIVE, &reflective);
      // if(ret1 == AI_SUCCESS)
      //   printf(
      //     "reflective: %f %f %f %f\n",
      //     reflective.r,
      //     reflective.g,
      //     reflective.b,
      //     reflective.a);

      float opacity = 0.f;
      ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_COLOR_TRANSPARENT, &opacity,
                                     &max);
      // if(ret1 == AI_SUCCESS)
      //   printf("opacity: %f", opacity);

      /*! @todo figure out where specular highlight term is stored in assimp */
      material->GLuniforms.Ns = 32;

      /*! @TODO add loading of tangents and bitangents to support textures */

      /* load tangents */
      // for(int j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
      //   printf(
      //     "[%d] %f, %f, %f\n",
      //     j,
      //     scene->mMeshes[i]->mTangents[j].x,
      //     scene->mMeshes[i]->mTangents[j].y,
      //     scene->mMeshes[i]->mTangents[j].z);
      // }

      /* load bitangents */
      // for(int j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
      //   printf(
      //     "[%d] %f, %f, %f\n",
      //     j,
      //     scene->mMeshes[i]->mBitangents[j].x,
      //     scene->mMeshes[i]->mBitangents[j].y,
      //     scene->mMeshes[i]->mBitangents[j].z);
      // }

      /* load material name */
      asprintf(&(material->name), "%d", i);
    }

    /* Free resources associated with import */
    aiReleaseImport(scene);
    free(fullPathFileName);
  }
  return 0;
}

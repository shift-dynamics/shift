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

#include <jansson.h>
#include <assimp/cimport.h>        // Plain-C interface to assimp loader
#include <assimp/scene.h>          // Output data structure
#include <assimp/postprocess.h>    // Post processing flags
#include <assimp/material.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "stack.h"
#include "context.h"
#include "body.h"
#include "material.h"
#include "convexhull.h"
#include "frame.h"
#include "inertia.h"
#include "assembly.h"
#include "utility.h"



/*!
@brief Initialize an s2_body_t datastructure. Appends the origin frame to the body frame

@param None
@returns a s2_body_t datastructure
*/
s2_body_t* s2_body(s2_stackstr_t* name) {
  s2_body_t* body = malloc(sizeof(s2_body_t));
  if (!body) {
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
  body->materials = NULL;
  body->materialCount = 0;
  body->centerOfMassWRTOrigin = s2_vector();
  body->centerOfMassWRTReference = s2_vector();
  body->glReferenceFrame = NULL;
  body->glReferenceFramef = NULL;
  body->glModelView = NULL;
  body->glModelViewProjection = NULL;
  body->convexHullCount = 0;
  body->convexHulls = NULL;
  body->convexHullColor[0] = 194.f / 255.f;
  body->convexHullColor[1] = 9.f / 255.f;
  body->convexHullColor[2] = 226.f / 255.f;
  body->drawEnabled = true;
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
void s2_body_free(
  s2_body_t* body) {

  if(body) {
    if(body->isVirtual) {
      s2_virtualbody_free(body);
    } else {
      s2_stackstr_free(body->name);
      if (body->definition) {
        s2_stackstr_free(body->definition);
      }
      s2_inertia_free(body->inertia);
      s2_materials_free(body->materials, body->materialCount);

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
void s2_body_appendFrame(
  s2_body_t* body,
  s2_frame_t* frame) {

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
void s2_body_printFrames(
  FILE* f,
  s2_body_t* body) {

  s2_frame_t *frameHead = body->frameHead;
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
void s2_virtualbody_free(
  s2_body_t* body) {

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
@return status: 0 indicates that body geometry was successfully loaded, -1 indicates failure in parsing geometry file, -2 indicates no body geometry was found, -3 indicates body is inactive and nothing was loaded
*/
int s2_body_load(
  s2_context_t* context,
  s2_assembly_t* assembly,
  s2_stackstr_t* prefix,
  json_t* bodyObject) {

  bool active = true;
  json_t* activeObject =
    json_object_get(bodyObject, "active");
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
  json_t* definitionsObject =
    json_object_get(bodyObject, "definition");

  body->definition = s2_stackstr();

  if(json_is_null(definitionsObject)) {
    // printf("no definition files loaded\n");
  } else if(json_is_string(definitionsObject)) {
    s2_stackstr_append(
      body->definition,
      json_string_value(definitionsObject));
  } else if(json_is_array(definitionsObject)) {
    json_t* definitionObject;
    size_t i;
    json_array_foreach(
      definitionsObject, i, definitionObject) {

      s2_stackstr_append(
        body->definition,
        json_string_value(definitionObject));
    }
  }

  json_t* framesObject =
    json_object_get(bodyObject, "frames");

  if(!json_is_null(framesObject)) {
    json_t* frameObject;
    size_t frameIndex;

    json_array_foreach(
      framesObject,
      frameIndex,
      frameObject) {

      s2_frame_load(body, frameObject);

    }
  }

  /* get mass */
  body->inertia->m =
    json_number_value(
      json_object_get(bodyObject, "mass"));

  /* get body center of mass */
  for (int i = 0; i < 3; i++) {

    ((double*)body->centerOfMassWRTOrigin)[i] =
      json_number_value(
        json_array_get(json_object_get(bodyObject, "center_of_mass"), i));
  }

  s2_product_vk(
    body->centerOfMassWRTOrigin,
    &(body->inertia->m),
    &(body->inertia->mt));

  body->inertia->Ixx = json_number_value(
    json_object_get(
      json_object_get(bodyObject, "moment_of_inertia"), "Ixx"));

  body->inertia->Ixy = json_number_value(
    json_object_get(
      json_object_get(bodyObject, "moment_of_inertia"), "Ixy"));

  body->inertia->Izx = json_number_value(
    json_object_get(
      json_object_get(bodyObject, "moment_of_inertia"), "Izx"));
  body->inertia->Iyy = json_number_value(
    json_object_get(
      json_object_get(bodyObject, "moment_of_inertia"), "Iyy"));

  body->inertia->Iyz = json_number_value(
    json_object_get(
      json_object_get(bodyObject, "moment_of_inertia"), "Iyz"));

  body->inertia->Izz = json_number_value(
    json_object_get(
      json_object_get(bodyObject, "moment_of_inertia"), "Izz"));

  (context->bodyCount)++;

  s2_assembly_appendBody(assembly, body);

  /* load body geometry */
  return s2_body_loadGeometry(context, body);
}


/*! @brief Loads geometry specified in Body properties from an external file
using ASSIMP (open asset importer)

@param context a Context datastructure
@param body a Body datastructure
*/
int s2_body_loadGeometry(
  s2_context_t* context,
  s2_body_t* body) {

  const char* fileName = NULL;

  s2_stackstrnode_t* definitionHead = body->definition->head;
  while(definitionHead) {
    if(!strcmp(s2_getFileExtension(definitionHead->value), "obj")) {
      fileName = definitionHead->value;
      break;
    }
    definitionHead = definitionHead->next;
  }

  /* if the filename is not specified, the body has no geometry specified
  */
  if(fileName == NULL) {
    return -2;
  }

  /*
    check if file exists in all user specified include directories
  */
  char* fullPathFileName = NULL;
  s2_stackstrnode_t* includeDirectoryHead =
    context->modelParameters.includeDirectory->head;
  while(includeDirectoryHead) {
    if(fullPathFileName) free(fullPathFileName);
    asprintf(
      &fullPathFileName,
      "%s%s",
      includeDirectoryHead->value,
      fileName);
    if(access(fullPathFileName, F_OK) != -1) {
      break;
    }
    includeDirectoryHead = includeDirectoryHead->next;
  }
  // printf("fullPathFileName: %s\n", fullPathFileName);

  const struct aiScene* scene =
    aiImportFile(
      fullPathFileName,
      aiProcess_CalcTangentSpace |
      aiProcess_Triangulate |
      aiProcess_SortByPType);

  /*
    If loading was not successful, report import error and exit
  */
  if(!scene) {
    printf("%s\n", aiGetErrorString());
    printf("error, exiting.\n");
    return -1;
  }

  /*
    Access the file contents, load in to body data structure
  */
  // printf("number of meshes: %d\n", scene->mNumMeshes);
  body->materialCount = scene->mNumMeshes;
  body->materials = s2_materials(body->materialCount);

  uint totalVertexCount = 0;
  for(uint i = 0; i < scene->mNumMeshes; i++) {
    totalVertexCount += scene->mMeshes[i]->mNumVertices;
  }

  GLfloat* vertices = malloc(3 * totalVertexCount * sizeof(GLfloat));

  totalVertexCount = 0;
  for(uint i = 0; i < scene->mNumMeshes; i++) {
    /*
      load vertices
    */
    body->materials[i].vertices = &vertices[totalVertexCount * 3];
    body->materials[i].vertexCount = scene->mMeshes[i]->mNumVertices;
    totalVertexCount += body->materials[i].vertexCount;
    for(uint j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
      body->materials[i].vertices[3 * j + 0] =
        scene->mMeshes[i]->mVertices[j].x;
      body->materials[i].vertices[3 * j + 1] =
        scene->mMeshes[i]->mVertices[j].y;
      body->materials[i].vertices[3 * j + 2] =
        scene->mMeshes[i]->mVertices[j].z;
    }

    /*
      load vertex normals
    */
    body->materials[i].vertexNormals =
      malloc(3 * body->materials[i].vertexCount * sizeof(GLfloat));
    for(uint j = 0; j < scene->mMeshes[i]->mNumVertices; j++ ) {
      body->materials[i].vertexNormals[3 * j + 0] =
        scene->mMeshes[i]->mNormals[j].x;
      body->materials[i].vertexNormals[3 * j + 1] =
        scene->mMeshes[i]->mNormals[j].y;
      body->materials[i].vertexNormals[3 * j + 2] =
        scene->mMeshes[i]->mNormals[j].z;

      // printf("  vn: ");
      // for(int k = 0; k < 3; k++) {
      //   printf("%f ", body->materials[i].vertexNormals[3 * j + k]);
      // }
      // printf("\n");

    }

    /*
      load faces
    */
    // printf("\nnumber of faces: %d\n", scene->mMeshes[i]->mNumFaces);
    body->materials[i].faceCount = scene->mMeshes[i]->mNumFaces;
    body->materials[i].faces = malloc(
      3 * body->materials[i].faceCount * sizeof(GLuint));
    for(int j = 0; j < body->materials[i].faceCount; j++) {
      body->materials[i].faces[j * 3 + 0] = scene->mMeshes[i]->mFaces[j].mIndices[0];
      body->materials[i].faces[j * 3 + 1] = scene->mMeshes[i]->mFaces[j].mIndices[1];
      body->materials[i].faces[j * 3 + 2] = scene->mMeshes[i]->mFaces[j].mIndices[2];
    }

    /*
      load color properties
    */
    int ret1;
    struct aiMaterial* mtl;
    mtl = scene->mMaterials[scene->mMeshes[i]->mMaterialIndex];

    unsigned int max = 1;
    float opacity_f = 1.f;
    ret1 = aiGetMaterialFloatArray(
      mtl,
      AI_MATKEY_OPACITY,
      &opacity_f,
      0);
    if(ret1 == AI_SUCCESS) {
      printf("opacity: %f\n", opacity_f);
    }

    struct aiColor4D diffuse;
    ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse);
    if(ret1 == AI_SUCCESS) {
      printf("diffuse: %f %f %f %f\n", diffuse.r, diffuse.g, diffuse.b, diffuse.a);
      body->materials[i].GLuniforms.Kd[0] = diffuse.r;
      body->materials[i].GLuniforms.Kd[1] = diffuse.g;
      body->materials[i].GLuniforms.Kd[2] = diffuse.b;
      body->materials[i].GLuniforms.Kd[3] = opacity_f;
    } else {
      body->materials[i].GLuniforms.Kd[0] = 0.6;
      body->materials[i].GLuniforms.Kd[1] = 0.6;
      body->materials[i].GLuniforms.Kd[2] = 0.6;
      body->materials[i].GLuniforms.Kd[3] = opacity_f;
    }

    struct aiColor4D ambient;
    ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient);
    if(ret1 == AI_SUCCESS) {
      // printf("ambient: %f %f %f %f\n", ambient.r, ambient.g, ambient.b, ambient.a);
      body->materials[i].GLuniforms.Ka[0] = ambient.r;
      body->materials[i].GLuniforms.Ka[1] = ambient.g;
      body->materials[i].GLuniforms.Ka[2] = ambient.b;
      body->materials[i].GLuniforms.Ka[3] = opacity_f;
    } else {
      body->materials[i].GLuniforms.Ka[0] = 0.6;
      body->materials[i].GLuniforms.Ka[1] = 0.6;
      body->materials[i].GLuniforms.Ka[2] = 0.6;
      body->materials[i].GLuniforms.Ka[3] = opacity_f;
    }

    struct aiColor4D specular;
    ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular);
    if(ret1 == AI_SUCCESS) {
      // printf("specular: %f %f %f %f\n", specular.r, specular.g, specular.b, specular.a);
      body->materials[i].GLuniforms.Ks[0] = specular.r;
      body->materials[i].GLuniforms.Ks[1] = specular.g;
      body->materials[i].GLuniforms.Ks[2] = specular.b;
      body->materials[i].GLuniforms.Ks[3] = opacity_f;
    } else {
      body->materials[i].GLuniforms.Ks[0] = 0.6;
      body->materials[i].GLuniforms.Ks[1] = 0.6;
      body->materials[i].GLuniforms.Ks[2] = 0.6;
      body->materials[i].GLuniforms.Ks[3] = opacity_f;
    }

    struct aiColor4D emissive;
    ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emissive);
    if(ret1 == AI_SUCCESS) {
      // printf("emissive: %f %f %f %f\n", emissive.r, emissive.g, emissive.b, emissive.a);
      body->materials[i].GLuniforms.Ke[0] = emissive.r;
      body->materials[i].GLuniforms.Ke[1] = emissive.g;
      body->materials[i].GLuniforms.Ke[2] = emissive.b;
      body->materials[i].GLuniforms.Ke[3] = emissive.a;
    } else {
      body->materials[i].GLuniforms.Ke[0] = 0.0;
      body->materials[i].GLuniforms.Ke[1] = 0.0;
      body->materials[i].GLuniforms.Ke[2] = 0.0;
      body->materials[i].GLuniforms.Ke[3] = opacity_f;
    }

    /*
      this set of parameters is currently not used / loaded
    */
    float shininess = 0.f;
    ret1 = aiGetMaterialFloatArray(
      mtl,
      AI_MATKEY_SHININESS,
      &shininess,
      &max);
    // if(ret1 == AI_SUCCESS)
    //   printf("shininess: %f\n", shininess);

    float shininess_strength = 0.f;
    ret1 = aiGetMaterialFloatArray(
      mtl,
      AI_MATKEY_SHININESS_STRENGTH,
      &shininess_strength,
      &max);
    // if(ret1 == AI_SUCCESS)
    //   printf("shininess_strength: %f\n", shininess_strength);

    float reflectivity = 0.f;
    ret1 = aiGetMaterialFloatArray(
      mtl,
      AI_MATKEY_REFLECTIVITY,
      &reflectivity,
      &max);
    // if(ret1 == AI_SUCCESS)
    //   printf("reflectivity: %f\n", reflectivity);

    float refracti = 0.f;
    ret1 = aiGetMaterialFloatArray(
      mtl,
      AI_MATKEY_REFRACTI,
      &refracti,
      &max);
    // if(ret1 == AI_SUCCESS)
    //   printf("refracti: %f\n", refracti);



    struct aiColor4D reflective;
    reflective.r = 0.f;
    reflective.g = 0.f;
    reflective.b = 0.f;
    reflective.a = 0.f;
    ret1 = aiGetMaterialColor(
      mtl,
      AI_MATKEY_COLOR_REFLECTIVE,
      &reflective);
    // if(ret1 == AI_SUCCESS)
    //   printf(
    //     "reflective: %f %f %f %f\n",
    //     reflective.r,
    //     reflective.g,
    //     reflective.b,
    //     reflective.a);

    float opacity = 0.f;
    ret1 = aiGetMaterialFloatArray(
      mtl,
      AI_MATKEY_COLOR_TRANSPARENT,
      &opacity,
      &max);
    // if(ret1 == AI_SUCCESS)
    //   printf("opacity: %f", opacity);

    /*!
      @todo figure out where specular highlight term is stored in assimp
    */
    body->materials[i].GLuniforms.Ns = 10;

    /*
      load tangents
    */
    // for(int j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
    //   printf(
    //     "[%d] %f, %f, %f\n",
    //     j,
    //     scene->mMeshes[i]->mTangents[j].x,
    //     scene->mMeshes[i]->mTangents[j].y,
    //     scene->mMeshes[i]->mTangents[j].z);
    // }

    /*
      load bitangents;
    */
    // for(int j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
    //   printf(
    //     "[%d] %f, %f, %f\n",
    //     j,
    //     scene->mMeshes[i]->mBitangents[j].x,
    //     scene->mMeshes[i]->mBitangents[j].y,
    //     scene->mMeshes[i]->mBitangents[j].z);
    // }

    /* load material name */
    asprintf(&(body->materials[i].name), "%d", i);

  }

  /* compute convex hull */
  s2_convexhull_qhull(body);

  /* Free resources associated with import */
  aiReleaseImport(scene);
  free(fullPathFileName);

  return 0;
}

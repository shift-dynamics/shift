/**
@file
@brief A wrapper for Assimp library to load geometry into mbdl datastructures
@author Kristopher Wehage
@date 2016
@copyright Kristopher Wehage 2014-2015
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

#include <assimp/cimport.h>        // Plain-C interface to assimp loader
#include <assimp/scene.h>          // Output data structure
#include <assimp/postprocess.h>    // Post processing flags
#include <assimp/material.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "context.h"
#include "body.h"
#include "material.h"
#include "loader.h"


/*! @brief Loads geometry specified in Body properties from an external file
using ASSIMP (open asset importer)

@param context a Context datastructure
@param body a Body datastructure
*/
int s2_loadBodyGeometry(
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
    context->modelParameters->includeDirectory->head;
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
    struct aiColor4D diffuse;
    ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse);
    if(ret1 == AI_SUCCESS) {
      // printf("diffuse: %f %f %f %f\n", diffuse.r, diffuse.g, diffuse.b, diffuse.a);
      body->materials[i].GLuniforms.Kd[0] = diffuse.r;
      body->materials[i].GLuniforms.Kd[1] = diffuse.g;
      body->materials[i].GLuniforms.Kd[2] = diffuse.b;
      body->materials[i].GLuniforms.Kd[3] = diffuse.a;
    } else {
      body->materials[i].GLuniforms.Kd[0] = 0.6;
      body->materials[i].GLuniforms.Kd[1] = 0.6;
      body->materials[i].GLuniforms.Kd[2] = 0.6;
      body->materials[i].GLuniforms.Kd[3] = 1.0;
    }

    struct aiColor4D ambient;
    ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient);
    if(ret1 == AI_SUCCESS) {
      // printf("ambient: %f %f %f %f\n", ambient.r, ambient.g, ambient.b, ambient.a);
      body->materials[i].GLuniforms.Ka[0] = ambient.r;
      body->materials[i].GLuniforms.Ka[1] = ambient.g;
      body->materials[i].GLuniforms.Ka[2] = ambient.b;
      body->materials[i].GLuniforms.Ka[3] = ambient.a;
    } else {
      body->materials[i].GLuniforms.Ka[0] = 0.6;
      body->materials[i].GLuniforms.Ka[1] = 0.6;
      body->materials[i].GLuniforms.Ka[2] = 0.6;
      body->materials[i].GLuniforms.Ka[3] = 1.0;
    }

    /*! @todo track down why some colors are loaded in as zero from
    assimp library

    */

    struct aiColor4D specular;
    ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular);
    if(ret1 == AI_SUCCESS) {
      // printf("specular: %f %f %f %f\n", specular.r, specular.g, specular.b, specular.a);
      body->materials[i].GLuniforms.Ks[0] = specular.r;
      body->materials[i].GLuniforms.Ks[1] = specular.g;
      body->materials[i].GLuniforms.Ks[2] = specular.b;
      body->materials[i].GLuniforms.Ks[3] = specular.a;
    } else {
      body->materials[i].GLuniforms.Ks[0] = 0.6;
      body->materials[i].GLuniforms.Ks[1] = 0.6;
      body->materials[i].GLuniforms.Ks[2] = 0.6;
      body->materials[i].GLuniforms.Ks[3] = 1.0;
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
      body->materials[i].GLuniforms.Ke[3] = 1.0;
    }

    /*
      this set of parameters is currently not used / loaded
    */
    float shininess = 0.f;
    unsigned int max = 0;
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

    float opacity_f = 0.f;
    ret1 = aiGetMaterialFloatArray(
      mtl,
      AI_MATKEY_OPACITY,
      &opacity_f,
      &max);
    // if(ret1 == AI_SUCCESS)
    //   printf("opacity: %f\n", opacity_f);

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

  /*
    Free resources associated with import
  */
  aiReleaseImport(scene);
  free(fullPathFileName);

  return 0;
}

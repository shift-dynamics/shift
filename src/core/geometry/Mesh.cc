#include "shift/core/Factory.h"
#include "shift/core/geometry/Mesh.h"
#include "shift/core/utility.h"
#include "shift/core/Exception.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

bool shift::core::geometry::Mesh::registered_ =
  shift::core::Factory<shift::core::geometry::Base>::get()->registerModule(
    "mesh", shift::core::geometry::Mesh::create);

std::unique_ptr<shift::core::geometry::Mesh>
shift::core::geometry::Mesh::create() {
  return std::make_unique<shift::core::geometry::Mesh>();
}

void shift::core::geometry::Mesh::addParameters() {
  addParameter("filename", &filenames_, "Specify one or more filenames to load",
               true);
}

void shift::core::geometry::Mesh::load() {

  bool found = false;
  for(const auto& str : filenames_) {
    std::string extension = shift::core::utility::getFileExtension(str);

    if(extension == "obj" || extension == "gltf") {

      found = true;
      Assimp::Importer importer;
      const aiScene* scene = importer.ReadFile(
        str, aiProcess_CalcTangentSpace | aiProcess_Triangulate |
               aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);

      if(!scene) {
        std::cerr << "Failed importing mesh " << str << std::endl;
        throw shift::core::Exception(importer.GetErrorString());
      }

      for(size_t i = 0; i < scene->mNumMeshes; i++) {
        MeshData mesh_data;

        mesh_data.vertices.resize(3 * scene->mMeshes[i]->mNumVertices);
        mesh_data.normals.resize(3 * scene->mMeshes[i]->mNumVertices);
        mesh_data.tangents.resize(3 * scene->mMeshes[i]->mNumVertices);
        mesh_data.bitangents.resize(3 * scene->mMeshes[i]->mNumVertices);
        mesh_data.faces.resize(3 * scene->mMeshes[i]->mNumFaces);

        for(size_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
          mesh_data.vertices[3 * j + 0] = scene->mMeshes[i]->mVertices[j].x;
          mesh_data.vertices[3 * j + 1] = scene->mMeshes[i]->mVertices[j].y;
          mesh_data.vertices[3 * j + 2] = scene->mMeshes[i]->mVertices[j].z;
        }

        for(size_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
          mesh_data.normals[3 * j + 0] = scene->mMeshes[i]->mNormals[j].x;
          mesh_data.normals[3 * j + 1] = scene->mMeshes[i]->mNormals[j].y;
          mesh_data.normals[3 * j + 2] = scene->mMeshes[i]->mNormals[j].z;
        }

        for(size_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
          mesh_data.tangents[3 * j + 0] = scene->mMeshes[i]->mTangents[j].x;
          mesh_data.tangents[3 * j + 1] = scene->mMeshes[i]->mTangents[j].y;
          mesh_data.tangents[3 * j + 2] = scene->mMeshes[i]->mTangents[j].z;
        }

        for(size_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
          mesh_data.bitangents[3 * j + 0] =
            scene->mMeshes[i]->mBitangents[j].x;
          mesh_data.bitangents[3 * j + 1] =
            scene->mMeshes[i]->mBitangents[j].y;
          mesh_data.bitangents[3 * j + 2] =
            scene->mMeshes[i]->mBitangents[j].z;
        }

        for(size_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
          mesh_data.texture_coordinates[3 * j + 0] =
            scene->mMeshes[i]->mTextureCoords[j][0].x;
          mesh_data.texture_coordinates[3 * j + 1] =
            scene->mMeshes[i]->mTextureCoords[j][0].y;
          mesh_data.texture_coordinates[3 * j + 2] =
            scene->mMeshes[i]->mTextureCoords[j][0].z;
        }

        for(size_t j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
          mesh_data.faces[3 * j + 0] =
            scene->mMeshes[i]->mFaces[j].mIndices[0];
          mesh_data.faces[3 * j + 1] =
            scene->mMeshes[i]->mFaces[j].mIndices[1];
          mesh_data.faces[3 * j + 2] =
            scene->mMeshes[i]->mFaces[j].mIndices[2];
        }

        mesh_data.material_index = scene->mMeshes[i]->mMaterialIndex;
        mesh_data_.push_back(mesh_data);
      }

      for(size_t i = 0; i < scene->mNumMaterials; i++) {
        aiMaterial* mtl = scene->mMaterials[i];
      }

#if 0
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
          mesh_data.GLuniforms.Kd[0] = diffuse.r;
          mesh_data.GLuniforms.Kd[1] = diffuse.g;
          mesh_data.GLuniforms.Kd[2] = diffuse.b;
          mesh_data.GLuniforms.Kd[3] = opacity_f;
        } else {
          mesh_data.GLuniforms.Kd[0] = 0.6;
          mesh_data.GLuniforms.Kd[1] = 0.6;
          mesh_data.GLuniforms.Kd[2] = 0.6;
          mesh_data.GLuniforms.Kd[3] = opacity_f;
        }

        struct aiColor4D ambient;
        ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient);
        if(ret1 == AI_SUCCESS) {
          mesh_data.GLuniforms.Ka[0] = ambient.r;
          mesh_data.GLuniforms.Ka[1] = ambient.g;
          mesh_data.GLuniforms.Ka[2] = ambient.b;
          mesh_data.GLuniforms.Ka[3] = opacity_f;
        } else {
          mesh_data.GLuniforms.Ka[0] = 0.6;
          mesh_data.GLuniforms.Ka[1] = 0.6;
          mesh_data.GLuniforms.Ka[2] = 0.6;
          mesh_data.GLuniforms.Ka[3] = opacity_f;
        }

        struct aiColor4D specular;
        ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular);
        if(ret1 == AI_SUCCESS) {
          // printf("specular: %f %f %f %f\n", specular.r, specular.g,
          // specular.b, specular.a);
          mesh_data.GLuniforms.Ks[0] = specular.r;
          mesh_data.GLuniforms.Ks[1] = specular.g;
          mesh_data.GLuniforms.Ks[2] = specular.b;
          mesh_data.GLuniforms.Ks[3] = opacity_f;
        } else {
          mesh_data.GLuniforms.Ks[0] = 0.6;
          mesh_data.GLuniforms.Ks[1] = 0.6;
          mesh_data.GLuniforms.Ks[2] = 0.6;
          mesh_data.GLuniforms.Ks[3] = opacity_f;
        }

        struct aiColor4D emissive;
        ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emissive);
        if(ret1 == AI_SUCCESS) {
          mesh_data.GLuniforms.Ke[0] = emissive.r;
          mesh_data.GLuniforms.Ke[1] = emissive.g;
          mesh_data.GLuniforms.Ke[2] = emissive.b;
          mesh_data.GLuniforms.Ke[3] = emissive.a;
        } else {
          mesh_data.GLuniforms.Ke[0] = 0.0;
          mesh_data.GLuniforms.Ke[1] = 0.0;
          mesh_data.GLuniforms.Ke[2] = 0.0;
          mesh_data.GLuniforms.Ke[3] = opacity_f;
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
        ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_REFLECTIVITY,
                                       &reflectivity, &max);
        // if(ret1 == AI_SUCCESS)
        //   printf("reflectivity: %f\n", reflectivity);

        float refracti = 0.f;
        ret1 =
          aiGetMaterialFloatArray(mtl, AI_MATKEY_REFRACTI, &refracti, &max);
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
        ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_COLOR_TRANSPARENT,
                                       &opacity, &max);
        // if(ret1 == AI_SUCCESS)
        //   printf("opacity: %f", opacity);

        /*! @todo figure out where specular highlight term is stored in assimp
         */
        mesh_data.GLuniforms.Ns = 32;

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

        /* load mesh_data name */
        asprintf(&(mesh_data.name), "%d", i);
      }

      /* Free resources associated with import */
      aiReleaseImport(scene);
    }
    std::cout << std::endl;
#endif
    }
  }

  assert(false);
}

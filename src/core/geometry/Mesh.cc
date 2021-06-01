#include "shift/core/Context.h"
#include "shift/core/Exception.h"
#include "shift/core/Factory.h"
#include "shift/core/geometry/Mesh.h"
#include "shift/core/utility.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

bool shift::core::geometry::Mesh::registered_ =
  shift::core::Factory<shift::core::geometry::Base>::get()->RegisterModule(
    "mesh", shift::core::geometry::Mesh::Create);

std::unique_ptr<shift::core::geometry::Mesh>
shift::core::geometry::Mesh::Create() {
  return std::make_unique<shift::core::geometry::Mesh>();
}

void shift::core::geometry::Mesh::DeclareParameters() {
  DeclareParameter("filename", &filenames_, "Specify one or more filenames to load",
               true);
}

void shift::core::geometry::Mesh::Load() {

  assert(context_);
  std::string parent_path =
    shift::core::utility::GetParentPath(context_->get_filename());
  const std::vector<std::string> include_directories =
    (*context_)["model_parameters"]["include_directories"]
      .get<std::vector<std::string>>();

  bool found = false;
  for(const auto& str : filenames_) {
    std::string extension = shift::core::utility::GetFileExtension(str);

    if(extension == ".obj" || extension == ".gltf") {

      bool file_found = false;
      std::string file = parent_path + "/" + str;
      if(shift::core::utility::FileExists(file)) {
        file_found = true;
      } else {
        for(auto& directory : include_directories) {
          file = directory + "/" + str;
          std::cout << file << std::endl;
          if(shift::core::utility::FileExists(file)) {
            file_found = true;
            break;
          }
        }
      }
      if(!file_found) {
        std::string msg = "File: " + str + " not found";
        throw shift::core::Exception(msg);
      }

      found = true;
      Assimp::Importer importer;
      const aiScene* scene = importer.ReadFile(
        file, aiProcess_CalcTangentSpace | aiProcess_Triangulate |
               aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);

      if(!scene) {
        std::cerr << "Failed importing mesh " << str << std::endl;
        throw shift::core::Exception(importer.GetErrorString());
      }

      for(size_t i = 0; i < scene->mNumMeshes; i++) {
        shift::gui::MeshData mesh_data;

        mesh_data.vertices.resize(3 * scene->mMeshes[i]->mNumVertices);
        for(size_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
          mesh_data.vertices[3 * j + 0] = scene->mMeshes[i]->mVertices[j].x;
          mesh_data.vertices[3 * j + 1] = scene->mMeshes[i]->mVertices[j].y;
          mesh_data.vertices[3 * j + 2] = scene->mMeshes[i]->mVertices[j].z;
        }

        mesh_data.normals.resize(3 * scene->mMeshes[i]->mNumVertices);
        for(size_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
          mesh_data.normals[3 * j + 0] = scene->mMeshes[i]->mNormals[j].x;
          mesh_data.normals[3 * j + 1] = scene->mMeshes[i]->mNormals[j].y;
          mesh_data.normals[3 * j + 2] = scene->mMeshes[i]->mNormals[j].z;
        }

        mesh_data.tangents.resize(3 * scene->mMeshes[i]->mNumVertices);
        for(size_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
          mesh_data.tangents[3 * j + 0] = scene->mMeshes[i]->mTangents[j].x;
          mesh_data.tangents[3 * j + 1] = scene->mMeshes[i]->mTangents[j].y;
          mesh_data.tangents[3 * j + 2] = scene->mMeshes[i]->mTangents[j].z;
        }

        mesh_data.bitangents.resize(3 * scene->mMeshes[i]->mNumVertices);
        for(size_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
          mesh_data.bitangents[3 * j + 0] = scene->mMeshes[i]->mBitangents[j].x;
          mesh_data.bitangents[3 * j + 1] = scene->mMeshes[i]->mBitangents[j].y;
          mesh_data.bitangents[3 * j + 2] = scene->mMeshes[i]->mBitangents[j].z;
        }

        if(scene->mMeshes[i]->mTextureCoords[0]) {
          mesh_data.texture_coordinates.resize(3 * scene->mMeshes[i]->mNumVertices);
          for(size_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
            mesh_data.texture_coordinates[3 * j + 0] =
              scene->mMeshes[i]->mTextureCoords[0][j].x;
            mesh_data.texture_coordinates[3 * j + 1] =
              scene->mMeshes[i]->mTextureCoords[0][j].y;
            mesh_data.texture_coordinates[3 * j + 2] =
              scene->mMeshes[i]->mTextureCoords[0][j].z;
          }
        }

        mesh_data.faces.resize(3 * scene->mMeshes[i]->mNumFaces);
        for(size_t j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
          mesh_data.faces[3 * j + 0] = scene->mMeshes[i]->mFaces[j].mIndices[0];
          mesh_data.faces[3 * j + 1] = scene->mMeshes[i]->mFaces[j].mIndices[1];
          mesh_data.faces[3 * j + 2] = scene->mMeshes[i]->mFaces[j].mIndices[2];
        }

        mesh_data.material_index = scene->mMeshes[i]->mMaterialIndex;
        mesh_data_.push_back(mesh_data);
      }

      for(size_t i = 0; i < scene->mNumMaterials; i++) {
        aiMaterial* mtl = scene->mMaterials[i];

        shift::gui::MaterialData material_data;
        unsigned int max = 1;
        int ret1;

        float opacity = 1.f;
        ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_OPACITY, &opacity, 0);
        if(ret1 == AI_SUCCESS) {
          material_data.opacity = opacity;
        }
        
        aiColor4D diffuse;
        ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse);
        if(ret1 == AI_SUCCESS) {
          material_data.Kd[0] = diffuse.r;
          material_data.Kd[1] = diffuse.g;
          material_data.Kd[2] = diffuse.b;
        }

        aiColor4D ambient;
        ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient);
        if(ret1 == AI_SUCCESS) {
          material_data.Ka[0] = ambient.r;
          material_data.Ka[1] = ambient.g;
          material_data.Ka[2] = ambient.b;
        }

        aiColor4D specular;
        ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular);
        if(ret1 == AI_SUCCESS) {
          material_data.Ks[0] = specular.r;
          material_data.Ks[1] = specular.g;
          material_data.Ks[2] = specular.b;
        }

        aiColor4D emissive;
        ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emissive);
        if(ret1 == AI_SUCCESS) {
          material_data.Ke[0] = emissive.r;
          material_data.Ke[1] = emissive.g;
          material_data.Ke[2] = emissive.b;
          material_data.Ke[3] = emissive.a;
        }

        float shininess = 0.f;
        ret1 =
          aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
        if(ret1 == AI_SUCCESS) {
          material_data.Ns = shininess;
        }

        float shininess_strength = 0.f;
        ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH,
                                       &shininess_strength, &max);
        if(ret1 == AI_SUCCESS) {
          (void)shininess_strength;
          // std::cout << "  shininess_strength: " << shininess_strength
          //         << std::endl;
        }

        float reflectivity = 0.f;
        ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_REFLECTIVITY,
                                       &reflectivity, &max);
        if(ret1 == AI_SUCCESS) {
          (void)reflectivity;
          // std::cout << "  reflectivity: " << reflectivity << std::endl;
        }

        float refracti = 0.f;
        ret1 =
          aiGetMaterialFloatArray(mtl, AI_MATKEY_REFRACTI, &refracti, &max);
        if(ret1 == AI_SUCCESS) {
          (void)refracti;
          // std::cout << "  refraction: " << refracti << std::endl;
        }

        aiColor4D reflective;
        ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_REFLECTIVE, &reflective);
        if(ret1 == AI_SUCCESS) {
          (void)reflective;
          // std::cout << "  reflective: " << reflective.r << " " << reflective.g
          //          << " " << reflective.b << " " << reflective.a << std::endl;
        }

        opacity = 0.f;
        ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_COLOR_TRANSPARENT,
                                       &opacity, &max);
        if(ret1 == AI_SUCCESS) {
          (void)opacity;
          // std::cout << "  opacity: " << opacity << std::endl;
        }
        found = true;
        break;
      }
    }
  }
  if(!found) {
    throw shift::core::Exception("No file found for mesh");
  }
}

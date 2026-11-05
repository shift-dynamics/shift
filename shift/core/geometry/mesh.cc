#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/geometry/mesh.h"
#include "shift/core/log.h"
#include "shift/core/utility.h"

#ifdef SHIFT_FP_TRAP
#include <fenv.h>
#endif

#include <algorithm>
#include <cmath>
#include <sstream>

#include <assimp/Importer.hpp>
#include <assimp/GltfMaterial.h>
#include <assimp/ObjMaterial.h>
#include <assimp/material.h>
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
  DeclareParameter("filename", &filenames_,
                   "Specify one or more filenames to load", true);
  DeclareParameter(
    "color_space", &color_space_,
    "color space of material colors in OBJ/MTL files, 'linear' (default, as "
    "written by Blender) or 'srgb'; glTF colors are always linear",
    false);
  DeclareParameter(
    "texture_mapping", &texture_mapping_,
    "how textures are mapped: 'uv' (default) uses the texture coordinates of "
    "the mesh, 'triplanar' projects textures along the body axes, for meshes "
    "without usable texture coordinates",
    false);
  DeclareParameter("texture_scale", &texture_scale_,
                   "size of one texture tile for triplanar mapping, in model "
                   "length units, defaults to 1",
                   false);
}


static float SrgbToLinear(float c) {
  return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
}


void shift::core::geometry::Mesh::Load() {

  assert(context_);
  std::string parent_path =
    shift::core::utility::GetParentPath(context_->get_filename());
  const std::vector<std::string> include_directories =
    (*context_)["model_parameters"]["include_directories"]
      .get<std::vector<std::string>>();

  if(color_space_ != "linear" && color_space_ != "srgb") {
    throw shift::core::Exception("Invalid mesh color_space: " + color_space_);
  }
  if(texture_mapping_ != "uv" && texture_mapping_ != "triplanar") {
    throw shift::core::Exception("Invalid mesh texture_mapping: " +
                                 texture_mapping_);
  }
  if(texture_scale_ <= 0.0) {
    throw shift::core::Exception("Mesh texture_scale must be positive");
  }

  // resolve the supported files: next to the model file, else in the
  // include directories
  std::vector<std::string> files;
  for(const auto& str : filenames_) {
    const std::string extension = shift::core::utility::GetFileExtension(str);
    if(extension != ".obj" && extension != ".gltf" && extension != ".glb") {
      continue;
    }
    std::string file = parent_path + "/" + str;
    bool file_found = shift::core::utility::FileExists(file);
    for(size_t i = 0; !file_found && i < include_directories.size(); i++) {
      file = include_directories[i] + "/" + str;
      file_found = shift::core::utility::FileExists(file);
    }
    if(!file_found) {
      throw shift::core::Exception("File: " + str + " not found");
    }
    files.push_back(file);
  }
  if(files.empty()) {
    throw shift::core::Exception("No file found for mesh");
  }

  // meshes with the same files and options share the imported data, e.g.
  // many bodies of one part
  std::ostringstream key;
  key.precision(17);
  key << color_space_ << "|" << texture_mapping_ << "|" << texture_scale_;
  for(const auto& file : files) {
    key << "|" << file;
  }
  if(auto asset = context_->FindMeshAsset(key.str())) {
    asset_ = std::move(asset);
    return;
  }
  auto asset = std::make_shared<MeshAsset>();
  for(const auto& file : files) {
    Import(file, *asset);
  }
  asset_ = asset;
  context_->AddMeshAsset(key.str(), std::move(asset));
}

void shift::core::geometry::Mesh::Import(const std::string& file,
                                         MeshAsset& asset) const {
  const std::string extension = shift::core::utility::GetFileExtension(file);
  const bool gltf = extension == ".gltf" || extension == ".glb";
  Assimp::Importer importer;

// temporarily disable floating point divide by zero exception handling
// in Assimp loading
#ifdef SHIFT_FP_TRAP
#ifdef __APPLE__
  // TODO find fedisableexcept equivalent for Apple
#elif __linux__
  const int enabled_fe = fedisableexcept(FE_DIVBYZERO);
#endif
#endif
  const aiScene* scene = nullptr;
  try {
    scene = importer.ReadFile(
      file, aiProcess_CalcTangentSpace | aiProcess_Triangulate |
              aiProcess_JoinIdenticalVertices | aiProcess_SortByPType |
              aiProcess_PreTransformVertices);
  } catch(shift::core::Exception& e) {
    SHIFT_WARNING(e.what());
    SHIFT_WARNING("Problem detected loading: " + file);
  }
#ifdef SHIFT_FP_TRAP
#ifdef __APPLE__
  // TODO find feenableexcept equivalent for Apple
#elif __linux__
  // restore the trap only if it was enabled on entry
  if(enabled_fe != -1) {
    feenableexcept(enabled_fe & FE_DIVBYZERO);
  }
#endif
#endif

  if(!scene) {
    SHIFT_WARNING("Failed importing mesh: " + file);
    throw shift::core::Exception(importer.GetErrorString());
  }

  // material indices are local to each file; offset them into
  // the asset materials, which accumulate materials across all files
  const int material_offset = static_cast<int>(asset.material_data.size());
  for(size_t i = 0; i < scene->mNumMeshes; i++) {
    shift::gui::MeshData mesh_data;

    mesh_data.vertices.resize(3 * scene->mMeshes[i]->mNumVertices);
    for(size_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
      mesh_data.vertices[3 * j + 0] = scene->mMeshes[i]->mVertices[j].x;
      mesh_data.vertices[3 * j + 1] = scene->mMeshes[i]->mVertices[j].y;
      mesh_data.vertices[3 * j + 2] = scene->mMeshes[i]->mVertices[j].z;
    }

    if(scene->mMeshes[i]->HasNormals()) {
      mesh_data.normals.resize(3 * scene->mMeshes[i]->mNumVertices);
      for(size_t j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
        mesh_data.normals[3 * j + 0] = scene->mMeshes[i]->mNormals[j].x;
        mesh_data.normals[3 * j + 1] = scene->mMeshes[i]->mNormals[j].y;
        mesh_data.normals[3 * j + 2] = scene->mMeshes[i]->mNormals[j].z;
      }
    }

    if(scene->mMeshes[i]->HasTangentsAndBitangents()) {
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
    }

    if(scene->mMeshes[i]->mTextureCoords[0]) {
      mesh_data.texture_coordinates.resize(3 *
                                           scene->mMeshes[i]->mNumVertices);
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

    mesh_data.material_index =
      material_offset + static_cast<int>(scene->mMeshes[i]->mMaterialIndex);
    asset.mesh_data.push_back(mesh_data);
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
      SHIFT_TRACE("shininess_strength: " +
                  std::to_string(shininess_strength));
    }

    float reflectivity = 0.f;
    ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_REFLECTIVITY,
                                   &reflectivity, &max);
    if(ret1 == AI_SUCCESS) {
      (void)reflectivity;
      SHIFT_TRACE("reflectivity: " + std::to_string(reflectivity));
    }

    float refracti = 0.f;
    ret1 =
      aiGetMaterialFloatArray(mtl, AI_MATKEY_REFRACTI, &refracti, &max);
    if(ret1 == AI_SUCCESS) {
      (void)refracti;
      SHIFT_TRACE("refraction: " + std::to_string(refracti));
    }

    aiColor4D reflective;
    ret1 = aiGetMaterialColor(mtl, AI_MATKEY_COLOR_REFLECTIVE, &reflective);
    if(ret1 == AI_SUCCESS) {
      (void)reflective;
      SHIFT_TRACE("reflective: " + std::to_string(reflective.r) + " " +
                  std::to_string(reflective.g) + " " +
                  std::to_string(reflective.b) + " " +
                  std::to_string(reflective.a));
    }

    opacity = 0.f;
    ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_COLOR_TRANSPARENT,
                                   &opacity, &max);
    if(ret1 == AI_SUCCESS) {
      (void)opacity;
      // std::cout << "  opacity: " << opacity << std::endl;
    }
    // metallic-roughness (glTF, or OBJ with Pm/Pr): the base color is
    // linear by definition
    aiColor4D base_color;
    if(aiGetMaterialColor(mtl, AI_MATKEY_BASE_COLOR, &base_color) ==
       AI_SUCCESS) {
      material_data.Kd[0] = base_color.r;
      material_data.Kd[1] = base_color.g;
      material_data.Kd[2] = base_color.b;
    } else if(!gltf && color_space_ == "srgb") {
      for(auto& k : material_data.Kd) {
        k = SrgbToLinear(k);
      }
    }
    float factor = 0.f;
    if(aiGetMaterialFloat(mtl, AI_MATKEY_METALLIC_FACTOR, &factor) ==
       AI_SUCCESS) {
      material_data.metalness = factor;
    }
    if(aiGetMaterialFloat(mtl, AI_MATKEY_ROUGHNESS_FACTOR, &factor) ==
       AI_SUCCESS) {
      material_data.roughness = factor;
    }
    // texture paths are relative to the mesh file
    const std::string directory = shift::core::utility::GetParentPath(file);
    auto texture = [&](aiTextureType type) -> std::string {
      aiString path;
      if(mtl->GetTextureCount(type) == 0 ||
         mtl->GetTexture(type, 0, &path) != AI_SUCCESS) {
        return "";
      }
      std::string name = path.C_Str();
      if(name.empty()) {
        return "";
      }
      if(name[0] == '*') {
        SHIFT_WARNING("Embedded textures are not supported: " + file);
        return "";
      }
      std::replace(name.begin(), name.end(), '\\', '/');
      if(name[0] != '/') {
        name = directory + "/" + name;
      }
      if(!shift::core::utility::FileExists(name)) {
        SHIFT_WARNING("Texture not found: " + name);
        return "";
      }
      return name;
    };
    material_data.base_color_map = texture(aiTextureType_BASE_COLOR);
    if(material_data.base_color_map.empty()) {
      material_data.base_color_map = texture(aiTextureType_DIFFUSE);
    }
    material_data.metallic_roughness_map =
      texture(aiTextureType_GLTF_METALLIC_ROUGHNESS);
    if(material_data.metallic_roughness_map.empty() && gltf) {
      // older importers store it as unknown
      material_data.metallic_roughness_map = texture(aiTextureType_UNKNOWN);
    }
    material_data.metallic_map = texture(aiTextureType_METALNESS);
    material_data.roughness_map = texture(aiTextureType_DIFFUSE_ROUGHNESS);
    material_data.normal_map = texture(aiTextureType_NORMALS);
    if(material_data.normal_map.empty()) {
      material_data.normal_map = texture(aiTextureType_NORMAL_CAMERA);
    }
    // normal map strength: OBJ "norm -bm <scale>", glTF normalTexture.scale
    float normal_scale = 1.f;
    if(aiGetMaterialFloat(mtl, AI_MATKEY_OBJ_BUMPMULT_NORMALS(0),
                          &normal_scale) == AI_SUCCESS ||
       aiGetMaterialFloat(mtl,
                          AI_MATKEY_GLTF_TEXTURE_SCALE(aiTextureType_NORMALS,
                                                       0),
                          &normal_scale) == AI_SUCCESS) {
      material_data.normal_scale = normal_scale;
    }
    material_data.triplanar = texture_mapping_ == "triplanar";
    material_data.texture_scale = static_cast<float>(texture_scale_);

    asset.material_data.push_back(material_data);
  }
  SHIFT_INFO("Loaded mesh: " + file);
}

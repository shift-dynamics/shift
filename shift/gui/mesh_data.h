#ifndef SHIFT_GUI_MESHDATA_H_
#define SHIFT_GUI_MESHDATA_H_

#include <vector>
#include <string>

namespace shift {
namespace gui {
  
  enum class ShadingType { WIREFRAME, FLAT, PHONG, LINE };

  struct MeshData {
    ShadingType shading_type = ShadingType::PHONG;
    std::vector<float> vertices;
    std::vector<float> normals;
    std::vector<float> colors;
    std::vector<float> tangents;
    std::vector<float> bitangents;
    std::vector<float> texture_coordinates;
    std::vector<unsigned int> faces;
    std::string diffuse_map;
    std::string normal_map;
    std::string specular_map;
    std::string displacement_map;
    std::string metalness_map;
    std::string occlusion_map;
    int material_index = -1;
 };

  struct MaterialData {
    float Ka[3] = {0.2, 0.2, 0.2};
    float Kd[3] = {0.6, 0.6, 0.6};
    float Ks[3] = {0.1, 0.1, 0.1};
    float Ke[4] = {0.0, 0.0, 0.0, 0.0};
    float Ns = 32.0;
    float opacity = 1.0;
    float metalness = 0.0;
    float roughness = -1.0; /*!< GGX roughness, < 0 if not specified */

    // texture maps, absolute paths, empty if none. Values from textures are
    // multiplied by the factors above, as in glTF.
    std::string base_color_map;         /*!< sRGB encoded */
    std::string metallic_roughness_map; /*!< glTF packing: G roughness,
                                             B metallic */
    std::string metallic_map;           /*!< single channel (OBJ map_Pm) */
    std::string roughness_map;          /*!< single channel (OBJ map_Pr) */
    std::string normal_map;             /*!< tangent space */
    float normal_scale = 1.0; /*!< scales the normal map's x and y, as glTF */

    /*! map textures by triplanar projection in body coordinates instead of
     * texture coordinates, with one texture tile per texture_scale length */
    bool triplanar = false;
    float texture_scale = 1.0;
  };

}
}

#endif

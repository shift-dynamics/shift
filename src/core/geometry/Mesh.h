#ifndef SHIFT_GEOMETRY_MESH_H_
#define SHIFT_GEOMETRY_MESH_H_

#include "shift/core/geometry/Base.h"

namespace shift {
namespace core {
namespace geometry {

class Mesh : public Base {
public:
  Mesh()
    : Base() {
    addParameters();
  }
  Mesh(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~Mesh() override {}
  static std::unique_ptr<Mesh> create();

  static bool isRegistered() {
    return registered_;
  }
  virtual void load() override;

private:
  void addParameters() override;
  static bool registered_;
  std::vector<std::string> filenames_;

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
    float Ka = 0.1;
    float Kd[4] = {0.2, 0.2, 0.2, 1.0};
    float Ks[4] = {0.1, 0.1, 0.1, 1.0};
    float Ke[4] = {0.0, 0.0, 0.0, 0.0};
    unsigned int Ns = 32;
    size_t material_index = 0;
  };
  std::vector<MeshData> mesh_data_;

  struct MaterialData {
    float Ka[4] = {0.1, 0.1, 0.1, 1.0};
    float Kd[4] = {0.2, 0.2, 0.2, 1.0};
    float Ks[4] = {0.1, 0.1, 0.1, 1.0};
    float Ke[4] = {0.0, 0.0, 0.0, 1.0};
    int Ns = 32;
    std::string texture;
    int texture_index = -1;
  };
  std::vector<MaterialData> material_data_;

};

} // namespace geometry
} // namespace core
} // namespace shift

#endif

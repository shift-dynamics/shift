#ifndef SHIFT_GUI_SCENE_H_
#define SHIFT_GUI_SCENE_H_

#include <QImage>
#include <QMatrix4x4>
#include <QVector3D>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace shift {
namespace gui {

struct Vertex {
  float position[3];
  float normal[3];
  float uv[2];
};

/*! \brief triangle mesh in body (origin frame) coordinates, drawn with the
 * pose of body `body_index` */
struct SceneMesh {
  int body_index = 0;
  std::vector<Vertex> vertices;
  std::vector<uint32_t> indices;
  QVector3D albedo{0.6f, 0.6f, 0.6f};
  float roughness = 0.6f;
  float metallic = 0.f;

  // texture layers in Scene::texture_layers, -1 if none; texture values
  // multiply the factors above
  int base_color_layer = -1;         /*!< sRGB encoded */
  int metallic_roughness_layer = -1; /*!< G roughness, B metallic */
  int normal_layer = -1;             /*!< tangent space */
  float normal_scale = 1.f;
  bool triplanar = false; /*!< project along the body axes instead of uv */
  float texture_scale = 1.f; /*!< triplanar tile size */

  QVector3D bounds_min;
  QVector3D bounds_max;
};

/*! \brief renderable content of a model, in shift world coordinates (+z
 * down) */
struct Scene {
  static constexpr int kTextureSize = 1024;

  std::vector<SceneMesh> meshes;
  int body_count = 0;
  /*! texture images, all kTextureSize square, RGBA8888 */
  std::vector<QImage> texture_layers;
};

/*!
@brief Loads the texture images of a scene into its layers, once per file
*/
class TextureLoader {
public:
  explicit TextureLoader(Scene& scene)
    : scene_(scene) {}

  /*! \brief layer holding an image file, -1 if it can't be read */
  int Load(const std::string& path);

  /*! \brief layer holding metallic (B) and roughness (G), from a packed
   * glTF image or from separate single channel images; -1 if none */
  int LoadMetallicRoughness(const std::string& packed,
                            const std::string& metallic,
                            const std::string& roughness);

private:
  QImage Read(const std::string& path);

  Scene& scene_;
  std::map<std::string, int> layers_;
};

/*! \brief body origin poses with respect to the inertial frame at one
 * output time */
struct Poses {
  double time = 0.0;
  std::vector<QMatrix4x4> bodies;
};

/*! \brief world space bounding box of the scene meshes in the given poses */
void SceneBounds(const Scene& scene, const Poses& poses, QVector3D& box_min,
                 QVector3D& box_max);

} // namespace gui
} // namespace shift

#endif

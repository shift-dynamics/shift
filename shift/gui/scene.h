#ifndef SHIFT_GUI_SCENE_H_
#define SHIFT_GUI_SCENE_H_

#include <QImage>
#include <QMatrix4x4>
#include <QVector3D>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "shift/gui/cad_scene.h"
#include "shift/gui/mesh_data.h"

namespace shift {
namespace gui {

struct Vertex {
  float position[3];
  float normal[3];
  float uv[2];
};

/*! \brief triangle mesh in body (origin frame) coordinates with its
 * material, uploaded once and drawn for each of its SceneInstance */
struct SceneMesh {
  std::vector<Vertex> vertices;
  std::vector<uint32_t> indices;
  QVector3D albedo{0.6f, 0.6f, 0.6f};
  float roughness = 0.6f;
  float metallic = 0.f;
  float opacity = 1.f; /*!< the ray tracer's; 0 lets all light through */

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

  /*! the terrain (Scene::terrains) the mesh triangulates, -1 if none: it is
   * drawn tessellated from the terrain's maps where the device can, and as
   * this mesh otherwise and by the ray tracer */
  int terrain = -1;
};

/*! \brief an elevation map terrain for the tessellation shaders: its
 * control points, from which the shaders evaluate the same surface as
 * terrain::Basic::LocalZ (cubic B-spline or bilinear) */
struct SceneTerrain {
  int mesh_index = -1; /*!< its SceneMesh, for the material */
  int width = 0;       /*!< control points along x */
  int height = 0;      /*!< control points along y */
  /*! control points as z (down) of the terrain frame, x fastest */
  std::vector<float> z;
  float x_scale = 1.f; /*!< spacing of the control points (m) */
  float y_scale = 1.f;
  bool bicubic = true;
  QMatrix4x4 pose; /*!< terrain frame to world */
};

/*! \brief a light of the model (shift::core::light), attached to a frame
 * of a body. Spot, directional and IES lights shine along the -z axis of
 * that frame; an IES light's horizontal angle 0 is its x axis. */
struct SceneLight {
  enum Type { kPoint = 0, kSpot = 1, kDirectional = 2, kIes = 3 };
  Type type = kPoint;
  int body_index = 0;
  QMatrix4x4 frame; /*!< light frame with respect to the body origin */
  QVector3D ambient;  /*!< Ka */
  QVector3D diffuse;  /*!< Kd times intensity */
  QVector3D specular; /*!< Ks times intensity */
  QVector3D attenuation{1.f, 0.f, 0.f}; /*!< c, l, q */
  float cone_angle = 0.f;       /*!< spot: half angle (rad) */
  float inner_cone_angle = 0.f; /*!< spot: half angle of full light */
  int ies_profile = -1;         /*!< IES: index in Scene::ies_profiles */
};

/*! \brief a mesh drawn with the pose of a body */
struct SceneInstance {
  int mesh_index = 0; /*!< in Scene::meshes */
  int body_index = 0;
};

/*! \brief renderable content of a model, in shift world coordinates (+z
 * down). Bodies that load the same mesh files share one SceneMesh and are
 * drawn as instances of it. */
struct Scene {
  static constexpr int kTextureSize = 1024;

  std::vector<SceneMesh> meshes;
  /*! grouped by mesh: the instances of a mesh are contiguous */
  std::vector<SceneInstance> instances;
  int body_count = 0;
  /*! texture images, all kTextureSize square, RGBA8888 */
  std::vector<QImage> texture_layers;
  std::vector<SceneTerrain> terrains;
  /*! CAD geometry, tessellated from its exact surfaces where the device
   * can; not drawn otherwise, nor ray traced */
  std::vector<SceneCad> cads;

  /*! the lights of the model; the viewer draws them in place of its sun */
  std::vector<SceneLight> lights;
  /*! IES distributions over kIesVertical vertical angles, 0 (the nadir,
   * -z of the light frame) to pi, by kIesHorizontal horizontal angles, 0
   * to 2 pi inclusive, vertical fastest; normalized to a peak of 1 */
  static constexpr int kIesVertical = 128;
  static constexpr int kIesHorizontal = 65;
  std::vector<std::vector<float>> ies_profiles;
};

/*!
@brief Loads the texture images of a scene into its layers, once per file
*/
class TextureLoader {
public:
  explicit TextureLoader(Scene& scene)
    : scene_(scene) {}

  /*! \brief images embedded in mesh files, read by Load in place of files
   * when a path is one of their keys; they must outlive the loader */
  void AddEmbedded(const std::map<std::string, EmbeddedTexture>& images);

  /*! \brief layer holding an image file or embedded image, -1 if it can't
   * be read */
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
  std::map<std::string, const EmbeddedTexture*> embedded_;
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

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
/*! \brief a texture drawn over a triplanar mesh's own in patches where a
 * noise of the body x, y is above level (SurfaceNoiseLevel) */
struct SceneSurface {
  int base_color_layer = -1; /*!< in Scene::texture_layers, sRGB */
  int normal_layer = -1;     /*!< tangent space, -1 if none */
  float normal_scale = 1.f;
  float texture_scale = 1.f; /*!< tile size (m) */
  float patch_size = 10.f;   /*!< size of the patches (m) */
  float level = 1.f;         /*!< noise level above which it shows */
};

/*! \brief the noise level above which surface number surface (0 or 1) of
 * a mesh covers the fraction coverage of it: a quantile of the noise of
 * surfaces.glsl (Fbm), which this repeats */
float SurfaceNoiseLevel(int surface, double coverage);

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
  /*! triplanar textures: shift their tiles at random in patches, and vary
   * the brightness, so that the tiles do not show */
  bool texture_variation = false;
  /*! triplanar: textures drawn over the mesh's own in patches
   * (shift::core::terrain::SurfaceTexture), at most two */
  std::vector<SceneSurface> surfaces;

  QVector3D bounds_min;
  QVector3D bounds_max;

  /*! the light (Scene::lights) the mesh glows with, -1 if none
   * (shift::core::light::Base::get_glow): its color times the light's
   * diffuse color and intensity is added to its shading. Bodies that load
   * the same files share the mesh, and so the glow */
  int glow_light = -1;

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
  /*! a deformable terrain (shift::core::terrain::Scm): the control points
   * are the levels of its nodes, which the physics interpolates bilinearly;
   * the viewport may draw them smoothly (Viewport::set_smooth_terrain) */
  bool deformable = false;
  /*! of a deformable terrain: its mesh (mesh_index) has this many
   * triangle pairs along each side of a cell, so that the ray tracer can
   * draw the smooth surface (Viewport::ShapeRtTerrain) */
  int mesh_subdivisions = 1;
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
  QVector3D diffuse;  /*!< Kd */
  QVector3D specular; /*!< Ks */
  float intensity = 1.f; /*!< factor of diffuse and specular at the start;
                          * Poses::lights at the time shown */
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
  /*! the intensity of each of Scene::lights at the time (a controller may
   * change it, as a brake lamp); empty: SceneLight::intensity */
  std::vector<float> lights;
};

/*! \brief a rectangle of control points of a terrain (SceneTerrain) that
 * changed, a deformable terrain (shift::core::terrain::Scm) at the time
 * shown */
struct TerrainUpdate {
  int terrain = 0; /*!< in Scene::terrains */
  int x = 0;       /*!< first control point along x */
  int y = 0;       /*!< first control point along y */
  int width = 0;
  int height = 0;
  std::vector<float> z; /*!< width * height, x fastest */
};

/*! \brief world space bounding box of the scene meshes in the given poses */
void SceneBounds(const Scene& scene, const Poses& poses, QVector3D& box_min,
                 QVector3D& box_max);

} // namespace gui
} // namespace shift

#endif

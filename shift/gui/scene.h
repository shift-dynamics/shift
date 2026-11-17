#ifndef SHIFT_GUI_SCENE_H_
#define SHIFT_GUI_SCENE_H_

#include <QImage>

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/vec3.h"
#include "shift/gui/cad_scene.h"
#include "shift/gui/mesh_data.h"

namespace shift {
namespace gui {

/*! \brief the identity transform */
inline ksl_SE3f_t Identity() {
  ksl_SE3f_t d;
  ksl_SE3f_setIdentity(&d);
  return d;
}

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
  ksl_vec3f_t albedo = ksl_vec3f(0.6f, 0.6f, 0.6f);
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
  bool hex_tiling = false; /*!< triplanar, each plane hex-tiled (terrains) */
  /*! of a terrain: rectangles x0, y0, x1, y1 of its frame (its uv) not
   * drawn, where regions of it are drawn as surfaces of their own; the
   * raster shader takes the first 4 (MaterialUniforms) */
  std::vector<std::array<float, 4>> holes;

  ksl_vec3f_t bounds_min = ksl_vec3f(0.f, 0.f, 0.f);
  ksl_vec3f_t bounds_max = ksl_vec3f(0.f, 0.f, 0.f);

  /*! the light (Scene::lights) the mesh glows with, -1 if none
   * (shift::core::light::Base::get_glow): its color times the light's
   * diffuse color and intensity is added to its shading. Bodies that load
   * the same files share the mesh, and so the glow */
  int glow_light = -1;

  /*! the terrain (Scene::terrains) the mesh triangulates, -1 if none: it is
   * drawn tessellated from the terrain's maps where the device can, and as
   * this mesh otherwise and by the ray tracer */
  int terrain = -1;
  /*! the surface of a flexible body's finite element mesh: its vertices
   * (undeformed here) move with the deformation shown, see FlexUpdate and
   * DeformFlexMesh, drawn, ray traced and picked deformed */
  bool deformable = false;
  /*! of a deformable mesh: the edges of its elements on the surface, pairs
   * of vertices, each 6-node triangle's sides as two lines through its
   * midside nodes, each edge once (Viewport::set_flex_mesh_edges) */
  std::vector<uint32_t> edge_indices;
  /*! of a deformable mesh: a vertex at each node an attachment ties to its
   * frame (body::Flex::get_attachment_nodes) that lies on the surface,
   * drawn as points (Viewport::set_flex_constrained_nodes) */
  std::vector<uint32_t> constrained_vertices;
  /*! of a deformable mesh: the displacement of each vertex in each of
   * flex_columns shapes, x, y, z, the shapes of a vertex together
   * (3 (v flex_columns + j) + c) */
  std::vector<float> flex_basis;
  uint32_t flex_columns = 0;
  /*! of a deformable mesh: the stress at each vertex in each of the
   * flex_columns shapes (body::Flex::get_surface_stresses), sigma_xx,
   * sigma_yy, sigma_zz, tau_xy, tau_yz, tau_zx at 6 (v flex_columns + j) +
   * c; empty if the body has none */
  std::vector<float> flex_stress_basis;
  /*! of a deformable mesh: the point FlexUpdate::rotation turns about */
  ksl_vec3f_t flex_centroid = ksl_vec3f(0.f, 0.f, 0.f);
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
  ksl_SE3f_t pose = Identity(); /*!< terrain frame to world */
};

/*! \brief the particles of an SPH soil (shift::core::terrain::Sph), each
 * drawn as an instance of a sphere mesh (SceneMesh, without instances of
 * its own) at its centre, or as one surface over spheres of radius
 * (Viewport::set_particle_surface), with the sphere's material; their
 * centres change with the time shown (see
 * Simulation::TakeParticleUpdates). Raster only: the ray tracer does not
 * draw them. */
struct SceneParticles {
  int mesh_index = -1; /*!< the sphere, about the origin, in world units */
  float radius = 0.f;
};

/*! \brief a light of the model (shift::core::light), attached to a frame
 * of a body. Spot, directional and IES lights shine along the -z axis of
 * that frame; an IES light's horizontal angle 0 is its x axis. */
struct SceneLight {
  enum Type { kPoint = 0, kSpot = 1, kDirectional = 2, kIes = 3 };
  Type type = kPoint;
  int body_index = 0;
  ksl_SE3f_t frame = Identity(); /*!< light frame with respect to the body
                                    origin */
  ksl_vec3f_t ambient = ksl_vec3f(0.f, 0.f, 0.f);  /*!< Ka */
  ksl_vec3f_t diffuse = ksl_vec3f(0.f, 0.f, 0.f);  /*!< Kd */
  ksl_vec3f_t specular = ksl_vec3f(0.f, 0.f, 0.f); /*!< Ks */
  float intensity = 1.f; /*!< factor of diffuse and specular at the start;
                          * Poses::lights at the time shown */
  ksl_vec3f_t attenuation = ksl_vec3f(1.f, 0.f, 0.f); /*!< c, l, q */
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
  /*! the grains of the DEM (shift::core::force_module::Dem): their poses
   * follow the bodies' in Poses::bodies, from body_count, and each is drawn
   * as an instance of the mesh of its shape */
  int grain_count = 0;
  /*! texture images, all kTextureSize square, RGBA8888 */
  std::vector<QImage> texture_layers;
  std::vector<SceneTerrain> terrains;
  std::vector<SceneParticles> particles;
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
 * output time, then those of the grains (Scene::grain_count): the poses of
 * their principal axes at their centers of mass */
struct Poses {
  double time = 0.0;
  std::vector<ksl_SE3f_t> bodies;
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

/*! \brief the particles of an SPH soil (SceneParticles) at the time shown */
struct ParticleUpdate {
  int particles = 0;         /*!< in Scene::particles */
  std::vector<float> centers; /*!< x, y, z in the world, then 0, each */
};

/*! \brief the deformation of a deformable mesh (SceneMesh::deformable)
 * at the time shown, in its body's origin coordinates: each vertex x0
 * moves to
 * \f[ x_0 + \sum_j a_j \underline{s}_j + \underline{t} +
 *     \underline{\theta} \times (x_0 - \underline{c}) \f]
 * with a the coefficients, s the vertex's shapes (SceneMesh::flex_basis),
 * t the translation, theta the rotation and c SceneMesh::flex_centroid;
 * the normals follow the deformed triangles. Only these are uploaded as
 * the mesh deforms; the viewport deforms it on the GPU */
struct FlexUpdate {
  int mesh = 0; /*!< in Scene::meshes */
  std::vector<float> coefficients; /*!< SceneMesh::flex_columns of them */
  ksl_vec3f_t translation = ksl_vec3f(0.f, 0.f, 0.f);
  ksl_vec3f_t rotation = ksl_vec3f(0.f, 0.f, 0.f);
  /*! the stress drawn (SceneMesh::flex_stress_basis): none, or the von
   * Mises stress of the deformation of stress_coefficients (as the
   * coefficients, but not magnified by the deformation scale), mapped to
   * [0, 1] over the range (FlexStressRange) in each vertex's uv[0] */
  enum class Stress { kNone, kRange, kAutomatic };
  Stress stress = Stress::kNone;
  std::vector<float> stress_coefficients;
  /*! with kRange: the stress mapped to 0 and to 1; with kAutomatic, from 0
   * to the largest stress on the mesh */
  float stress_range[2] = {0.f, 1.f};
};

/*! \brief the von Mises stress of a vertex of a deformable mesh: \f$
 * \sqrt{\frac{1}{2}((\sigma_{xx} - \sigma_{yy})^2 + (\sigma_{yy} -
 * \sigma_{zz})^2 + (\sigma_{zz} - \sigma_{xx})^2) + 3 (\tau_{xy}^2 +
 * \tau_{yz}^2 + \tau_{zx}^2)} \f$ with \f$\underline{\sigma} = \sum_j a_j
 * \underline{\sigma}_j\f$ (as flex_deform.comp) */
float FlexVonMises(const SceneMesh& mesh, const FlexUpdate& update,
                   size_t vertex);

/*! \brief the vertices of a deformable mesh deformed as update says, on
 * the CPU (as the viewport's compute shaders do; where it has none), with
 * the stress drawn in uv[0]; returns the largest von Mises stress on the
 * mesh (0 if update draws none) */
float DeformFlexMesh(const SceneMesh& mesh, const FlexUpdate& update,
                     std::vector<Vertex>& vertices);

/*! \brief the displacement of each sample of a flexible body's CAD
 * (SceneCad::flex_mesh) deformed as update says (of its mesh), body
 * coordinates: x, y, z, 0 per sample, as flex_field.comp */
void FlexCadField(const SceneCad& cad, const SceneMesh& mesh,
                  const FlexUpdate& update, std::vector<float>& field);

/*! \brief values (4 per sample of cad: the field, or the sample points)
 * at (u, v) of a patch, Catmull-Rom over its grid of samples (the end
 * intervals extended linearly), and their derivatives along u and v, as
 * flex_field.glsl */
ksl_vec3f_t FlexCadInterpolate(const SceneCad& cad,
                               const std::vector<float>& values, int patch,
                               float u, float v, ksl_vec3f_t* along_u,
                               ksl_vec3f_t* along_v);

/*! \brief area weighted vertex normals of a triangle mesh; a vertex of no
 * area gets a zero normal */
void ComputeNormals(std::vector<Vertex>& vertices,
                    const std::vector<uint32_t>& indices);

/*! \brief world space bounding box of the scene meshes in the given poses */
void SceneBounds(const Scene& scene, const Poses& poses, ksl_vec3f_t& box_min,
                 ksl_vec3f_t& box_max);

} // namespace gui
} // namespace shift

#endif

#ifndef SHIFT_GUI_CAD_SCENE_H_
#define SHIFT_GUI_CAD_SCENE_H_

#include "ksl/vec3.h"

#include <cstdint>
#include <string>
#include <vector>

#include "shift/core/cad/brep.h"

namespace shift {
namespace gui {

/*! @name GPU records of CAD geometry, std430, see cad_common.glsl */
/*! @{ */

/*! \brief a face: its surface in body coordinates (before the scale of the
 * geometry), its color and where its trim segments are */
struct CadGpuFace {
  float origin[4];    //!< w: surface type (cad::Surface::Type)
  float axis_x[4];    //!< w: radius
  float axis_y[4];    //!< w: minor radius
  float axis_z[4];    //!< w: tan(semi angle)
  float extrusion[4]; //!< xyz: extrusion; w: +1, or -1 if the face's
                      //!< normal is opposite to the surface's
  float color[4];     //!< rgb; w: 1 if the face has a color of its own
  float trim_box[4];  //!< x: v of the first band, y: band height
  int32_t nurbs[4];   //!< u degree, v degree, u count, v count
  int32_t offsets[4]; //!< first u knot (v knots follow), first point,
                      //!< swept curve (revolution, extrusion), -
  int32_t trim[4];    //!< first band header in trim_index, band count
};
static_assert(sizeof(CadGpuFace) == 160);

/*! \brief the curve swept by a surface of revolution or extrusion */
struct CadGpuCurve {
  float origin[4];   //!< w: type (cad::Curve3::Type)
  float a[4];        //!< line: direction; conic: a
  float b[4];        //!< conic: b
  int32_t nurbs[4];  //!< degree, count, first knot, first point
};
static_assert(sizeof(CadGpuCurve) == 64);

/*! \brief a patch of a face: a rectangle of its parameters (u, v), within
 * one knot span of a NURBS surface or swept curve, tessellated on the GPU */
struct CadGpuPatch {
  float uv[4];      //!< u0, v0, u1, v1
  int32_t info[4];  //!< face, flags (1: crosses the trims), u span, v span
                    //!< (knot spans of the NURBS surface or swept curve, or
                    //!< -1)
};
static_assert(sizeof(CadGpuPatch) == 32);
/*! @} */

/*!
@brief CAD geometry (core::geometry::Cad) packed for the tessellation
shaders: the faces of its bodies with their surfaces placed in body
coordinates, the patches that cover them, and their trim loops as segments
in (u, v) sorted into bands of v for the fragment shader's inside test
*/
struct SceneCad {
  std::vector<CadGpuFace> faces;
  std::vector<CadGpuCurve> curves;
  std::vector<CadGpuPatch> patches;
  std::vector<float> points;    //!< x, y, z, weight
  std::vector<float> knots;
  std::vector<float> segments;  //!< u0, v0, u1, v1
  /*! per band of each face: first index of its segment list in trim_index,
   * count; then the lists of segment indices, ~index (negative) for a
   * segment drawn without an edge line (a seam or singular point) */
  std::vector<int32_t> trim_index;

  /*! per patch: the grid of quads (u, v) of its triangles for ray tracing
   * and picking (cad_triangulate.comp), two each, within the ray tracing
   * tolerance of the surface; patches in a row or column of a face share
   * their counts along it, so their triangles meet without cracks */
  std::vector<int32_t> grids;
  /*! first triangle of each patch, and the total in the last entry */
  std::vector<int32_t> first_triangles;

  /*! the bodies drawn with it (Scene body indices) */
  std::vector<int> bodies;
  /*! model length units per meter: the shaders evaluate in meters and the
   * instance poses scale */
  float scale = 1.f;
  ksl_vec3f_t albedo = ksl_vec3f(0.7f, 0.7f, 0.72f);
  float roughness = 0.4f;
  float metallic = 0.f;
  float opacity = 1.f; /*!< the ray tracer's; 0 lets all light through */
  ksl_vec3f_t bounds_min = ksl_vec3f(0.f, 0.f, 0.f); //!< body coordinates
                                                    //!< (scaled)
  ksl_vec3f_t bounds_max = ksl_vec3f(0.f, 0.f, 0.f);

  int faces_skipped = 0; //!< without trims (not repaired)
};

/*!
@brief packs bodies of a repaired model (cad::Repair) for the GPU

tolerance is the largest distance (m) of the trim segments, on the surface,
from the trim curves; 0 for 2e-5 of the size of the model
*/
SceneCad MakeSceneCad(const core::cad::Model& model,
                      const std::vector<core::cad::PlacedBody>& bodies,
                      double scale, double tolerance = 0.0);

/*! \brief whether (u, v) is inside the trims of a packed face (nonzero
 * winding number of its segments), as the fragment shader tests it */
bool CadInside(const SceneCad& cad, int face, float u, float v);

/*!
@brief the faces and trims of several SceneCad, concatenated, for the ray
tracer: face face_base[c] + f is face f of cads[c]
*/
struct CadTrimTables {
  std::vector<CadGpuFace> faces;
  std::vector<float> segments;
  std::vector<int32_t> trim_index;
  std::vector<int> face_base;
};

CadTrimTables MergeCadTrims(const std::vector<SceneCad>& cads);

/*! \brief CadInside for a face of merged tables */
bool CadInside(const CadTrimTables& tables, int face, float u, float v);

} // namespace gui
} // namespace shift

#endif

#ifndef SHIFT_CORE_CAD_REPAIR_H_
#define SHIFT_CORE_CAD_REPAIR_H_

#include <string>
#include <vector>

#include "shift/core/cad/brep.h"

namespace shift {
namespace core {
namespace cad {

struct RepairOptions {
  /*! largest distance (m) at which points are taken to coincide: a vertex
   * and the curve of its edge, an edge's curve and its pcurves */
  double tolerance = 1e-6;
  /*! largest distance (m) of a computed pcurve, on its surface, from the
   * curve of its edge */
  double fit_tolerance = 1e-6;
  /*! most pieces of a computed pcurve that is not exact */
  int max_segments = 512;
};

/*! @brief what Repair found and did */
struct RepairReport {
  int edges = 0;
  int edges_failed = 0;     //!< no parameter range found
  int vertex_gaps = 0;      //!< vertices farther than tolerance from a curve
  double max_vertex_gap = 0.0;
  int pcurves_checked = 0;  //!< read from the file
  int pcurves_not_same_parameter = 0;
  int pcurves_replaced = 0; //!< read, but too far from their edge's curve
  int pcurves_computed = 0; //!< missing from the file, or replaced
  int pcurves_exact = 0;    //!< computed exactly (planes) or found straight
  int pcurves_failed = 0;
  int seam_twins = 0;       //!< second pcurves of seams, computed
  int seams_reassigned = 0; //!< seam coedges given the file's other pcurve
  double max_pcurve_deviation = 0.0; //!< of the pcurves kept or computed
  /*! largest distance of an edge's curve from a surface it should lie on
   * (a computed pcurve follows the nearest points of the surface) */
  double max_curve_to_surface = 0.0;
  int faces = 0;
  int faces_trimmed = 0;
  int singular_segments = 0;
  int seam_segments = 0;
  int join_segments = 0;
  int gap_segments = 0;
  double max_gap = 0.0;     //!< largest gap between edges of a loop (m)
  int domain_outer_loops = 0; //!< faces bounded by their whole domain

  std::string Summary() const;
};

/*!
@brief prepares a model read from a STEP file for rendering and contact

- finds the parameters of every edge's curve at its vertices, and the gaps
  between vertices and curves
- checks the curves of edges in the parameters of their surfaces (pcurves)
  against the edges' curves, and replaces those that are too far from them
- computes the missing pcurves: exactly on planes, as lines where they are
  straight in (u, v) (e.g. circles about the axis of a cylinder), and as
  piecewise cubics within fit_tolerance otherwise; a seam (an edge used
  twice by a face of a closed surface) gets a pcurve for each side
- builds closed trim loops in (u, v) for every face (Face::trims), choosing
  the pcurve of each side of a seam, translating pcurves by whole periods of
  periodic surfaces so that loops are continuous, running along singular
  points of the surface (apex, poles), and cutting open faces bounded by
  loops that wrap around a periodic surface

Problems are added to model.warnings.
*/
RepairReport Repair(Model& model, const RepairOptions& options = {});

/*! @brief point of a trim curve at the fraction f (0 to 1) of the way from
 * its start to its end */
Vec2 TrimPoint(const Model& model, const TrimCurve& curve, double f);

/*! @brief points along a trim loop (the last one not repeating the first),
 * with up to segments pieces per curved trim curve */
std::vector<Vec2> TrimPolygon(const Model& model, const TrimLoop& loop,
                              int segments = 32);

/*! @brief area of a polygon, positive if counterclockwise */
double SignedArea(const std::vector<Vec2>& polygon);

} // namespace cad
} // namespace core
} // namespace shift

#endif

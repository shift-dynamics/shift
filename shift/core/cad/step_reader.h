#ifndef SHIFT_CORE_CAD_STEP_READER_H_
#define SHIFT_CORE_CAD_STEP_READER_H_

#include <string>

#include "shift/core/cad/brep.h"

namespace shift {
namespace core {
namespace cad {

/*!
@brief reads the parts and assemblies of a STEP file (ISO 10303-21) into a
Model

The file is parsed with STEPcode and its AP242 schema, which also reads the
geometry and topology of AP203 and AP214 files. The reader transforms:

- solids (manifold_solid_brep, brep_with_voids) and sets of shells
  (shell_based_surface_model), their shells, faces, loops, edges and
  vertices;
- the surfaces of faces: plane, cylindrical, conical, spherical and toroidal
  surfaces, B-spline surfaces (rational or not, with knots, uniform,
  quasi-uniform or Bezier), surfaces of revolution and of linear extrusion;
- the curves of edges: lines, circles, ellipses, polylines and B-spline
  curves, and their curves in the parameters of the surfaces (pcurves) where
  the file gives them;
- the units of each representation (lengths to meters, angles to radians),
  also in the parameters of pcurves;
- shapes of parts and assemblies, with their names (from the products) and
  the placements of their children (mapped items and representation
  relationships with transformations);
- colors of bodies and faces (styled items).

Entities that are not supported are skipped, each with a warning in
Model::warnings. When a coedge lies on a seam (an edge used twice by a face
of a closed surface, such as a cylinder), the two pcurves of the seam are
assigned by the coedges' orientations; that and missing pcurves are left to
be checked and completed by the repair functions.

Throws shift::core::Exception if the file can't be read, or if shift was
built without STEP support (see StepSupported).
*/
Model ReadStep(const std::string& path);

/*! @brief true if shift was built with STEPcode (meson option step) */
bool StepSupported();

} // namespace cad
} // namespace core
} // namespace shift

#endif

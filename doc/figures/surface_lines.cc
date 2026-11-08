/* Data for the figures of the theory pages on contact (doc/sphinx/contact):
the shapes of collision geometry as polygons, and the faces of STEP files with
their parameter lines, from shift's own geometry and evaluation of surfaces.
make_figures.py renders them with the viewer.

  shift-doc-figures shapes <out.json>
  shift-doc-figures step <file.step> <out.json> [lines per parameter]
  shift-doc-figures fit <model.json> <out.json>
  shift-doc-figures terrain <model.json> <out.json> [points per cell]
  shift-doc-figures broadphase <model.json> <out.json>
  shift-doc-figures cadpair <model.json> <out.json>

Lengths are in meters. Each shape is placed at its own position along x, as
listed in Shapes(). For a STEP file every face of every body gives its surface
type, the frame of its surface (origin and axes, where it has one), lines of
constant u and of constant v inside its trims, and its boundary, all in the
coordinates of the file's root. For a model, every active collision geometry,
fitted to the meshes of its body as the model is loaded, is written as
polygons in the fixed reference frame, and the triangles of the meshes of their
bodies as shift loaded them, in the frame of the body. For a model with a
basic terrain, its samples, its control points and its surface over the map, each
point with its slopes and its curvatures along x and y, in the terrain frame.
For a model with collision geometry, the broad phase of its contact detection
at its initial state: the center, bounding radius and margin of each geometry,
the axis of the sweep, the candidate pairs and the pairs in contact. For a
model with two CAD collision geometries, the steps of their contact: for each,
its pose, box tree and faces (boxes and trim loops on the surface); the boxes
of the second's faces in the frame of the first, the pairs of faces whose
boxes overlap, and the points of contact. */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "shift/core/cad/evaluate.h"
#include "shift/core/cad/repair.h"
#include "shift/core/cad/step_reader.h"
#include "shift/core/collision_geometry/box.h"
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/collision_geometry/capsule.h"
#include "shift/core/collision_geometry/convex_hull.h"
#include "shift/core/collision_geometry/cylinder.h"
#include "shift/core/collision_geometry/ellipsoid.h"
#include "shift/core/collision_geometry/sphere.h"
#include "shift/core/context.h"
#include "shift/core/geometry/mesh.h"
#include "shift/core/terrain/basic.h"

static std::string Number(double x) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.9g", x);
  return text;
}

static std::string Point(double x, double y, double z) {
  return "[" + Number(x) + "," + Number(y) + "," + Number(z) + "]";
}

static std::string Point(const ksl_vec3_t& p) {
  return Point(p.x, p.y, p.z);
}

static std::string Point(const shift::core::cad::Vec3& p) {
  return Point(p[0], p[1], p[2]);
}

/* polygons of a shape: [[points], [normals]] each */
struct Polygons {
  std::vector<std::vector<ksl_vec3_t>> points;
  std::vector<std::vector<ksl_vec3_t>> normals;

  std::string Json() const {
    std::ostringstream out;
    out << "[";
    for(size_t i = 0; i < points.size(); i++) {
      out << (i ? "," : "") << "[[";
      for(size_t k = 0; k < points[i].size(); k++) {
        out << (k ? "," : "") << Point(points[i][k]);
      }
      out << "],[";
      for(size_t k = 0; k < normals[i].size(); k++) {
        out << (k ? "," : "") << Point(normals[i][k]);
      }
      out << "]]";
    }
    out << "]";
    return out.str();
  }
};

/* a round shape from its support points over a grid of directions, each
point with its direction as the normal */
static Polygons SupportPolygons(const shift::core::collision_geometry::Base& g) {
  Polygons polygons;
  const int rows = 48, columns = 96;
  auto direction = [&](int i, int j) {
    const double t = M_PI * i / rows, p = 2.0 * M_PI * j / columns;
    return ksl_vec3_t{{std::sin(t) * std::cos(p), std::sin(t) * std::sin(p),
                       std::cos(t)}};
  };
  auto support = [&](const ksl_vec3_t& d) {
    ksl_vec3_t local, global;
    int hint = 0;
    g.Support(d, local, global, hint);
    return global;
  };
  for(int i = 0; i < rows; i++) {
    for(int j = 0; j < columns; j++) {
      const ksl_vec3_t d[4] = {direction(i, j), direction(i + 1, j),
                               direction(i + 1, j + 1), direction(i, j + 1)};
      polygons.points.push_back(
        {support(d[0]), support(d[1]), support(d[2]), support(d[3])});
      polygons.normals.push_back({d[0], d[1], d[2], d[3]});
    }
  }
  return polygons;
}

/* a convex hull (or box) from its faces, with the normal of each face */
static Polygons HullPolygons(const shift::core::collision_geometry::ConvexHull& hull) {
  Polygons polygons;
  const auto& data = hull.get_hull();
  const ksl_SE3_t& pose = hull.get_d_absolute();
  for(size_t f = 0; f < data.polygons.size(); f++) {
    std::vector<ksl_vec3_t> points;
    for(auto i : data.polygons[f]) {
      ksl_vec3_t w;
      ksl_product_dv(&pose, &data.vertices[i], &w);
      points.push_back(w);
    }
    ksl_vec3_t n;
    ksl_product_drv(&pose.R, &data.polygon_normals[f], &n);
    polygons.points.push_back(points);
    polygons.normals.push_back(std::vector<ksl_vec3_t>(points.size(), n));
  }
  return polygons;
}

static ksl_SE3_t At(double x, double y, double z) {
  return {{{1, 0, 0, 0, 1, 0, 0, 0, 1, x, y, z}}};
}

/* a rotation about the unit axis k by angle a, at t */
static ksl_SE3_t Turned(const ksl_vec3_t& k, double a, const ksl_vec3_t& t) {
  ksl_SE3_t d;
  const double c = std::cos(a), s = std::sin(a);
  const double e[3] = {k.x, k.y, k.z};
  for(int j = 0; j < 3; j++) {
    for(int i = 0; i < 3; i++) {
      double r = (1.0 - c) * e[i] * e[j] + (i == j ? c : 0.0);
      if(i == 0 && j == 1) r -= s * e[2];
      if(i == 0 && j == 2) r += s * e[1];
      if(i == 1 && j == 0) r += s * e[2];
      if(i == 1 && j == 2) r -= s * e[0];
      if(i == 2 && j == 0) r -= s * e[1];
      if(i == 2 && j == 1) r += s * e[0];
      d.R.v[j].at[i] = r;
    }
  }
  d.t = t;
  return d;
}

/* points on an ellipsoid of semi axes a, b, c, or a sphere */
static std::vector<ksl_vec3_t> EllipsoidPoints(double a, double b, double c) {
  std::vector<ksl_vec3_t> points;
  const int n = 400;
  const double golden = M_PI * (3.0 - std::sqrt(5.0));
  for(int i = 0; i < n; i++) {
    const double z = 1.0 - 2.0 * (i + 0.5) / n;
    const double r = std::sqrt(1.0 - z * z);
    points.push_back({{a * r * std::cos(golden * i),
                       b * r * std::sin(golden * i), c * z}});
  }
  return points;
}

/* the rims of a cylinder of radius r and half length h along z */
static std::vector<ksl_vec3_t> RimPoints(double r, double h) {
  std::vector<ksl_vec3_t> points;
  for(int k = 0; k < 64; k++) {
    const double a = 2.0 * M_PI * k / 64;
    for(double z : {-h, h}) {
      points.push_back({{r * std::cos(a), r * std::sin(a), z}});
    }
  }
  return points;
}

static int Shapes(const std::string& path) {
  std::ofstream out(path);
  out << "{\"shapes\":[";
  bool first = true;
  auto write = [&](const char* name, const Polygons& polygons) {
    out << (first ? "" : ",") << "{\"name\":\"" << name
        << "\",\"polygons\":" << polygons.Json() << "}";
    first = false;
  };
  const ksl_vec3_t tilt = {{0.6, 0.8, 0.0}};

  shift::core::collision_geometry::Sphere sphere;
  sphere.FitToPoints(EllipsoidPoints(0.2, 0.2, 0.2));
  sphere.UpdatePose(At(0.0, 0.0, 0.0));
  write("sphere", SupportPolygons(sphere));

  shift::core::collision_geometry::Ellipsoid ellipsoid;
  ellipsoid.FitToPoints(EllipsoidPoints(0.28, 0.18, 0.12));
  ellipsoid.UpdatePose(Turned(tilt, 0.5, {{0.6, 0.0, 0.0}}));
  write("ellipsoid", SupportPolygons(ellipsoid));

  std::vector<ksl_vec3_t> corners;
  for(int i = 0; i < 8; i++) {
    corners.push_back({{(i & 1) ? 0.2 : -0.2, (i & 2) ? 0.14 : -0.14,
                        (i & 4) ? 0.1 : -0.1}});
  }
  shift::core::collision_geometry::Box box;
  box.FitToPoints(corners);
  box.UpdatePose(Turned(tilt, 0.5, {{1.2, 0.0, 0.0}}));
  write("box", HullPolygons(box));

  shift::core::collision_geometry::Cylinder cylinder;
  cylinder.FitToPoints(RimPoints(0.12, 0.2));
  cylinder.UpdatePose(Turned(tilt, 0.9, {{1.8, 0.0, 0.0}}));
  write("cylinder", SupportPolygons(cylinder));

  shift::core::collision_geometry::Capsule capsule;
  capsule.FitToPoints(RimPoints(0.1, 0.16));
  capsule.UpdatePose(Turned(tilt, 0.9, {{2.4, 0.0, 0.0}}));
  write("capsule", SupportPolygons(capsule));

  /* a hull of points scattered about a lumpy ellipsoid, from a fixed seed */
  std::mt19937 random(7);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::vector<ksl_vec3_t> stones;
  for(const auto& p : EllipsoidPoints(0.22, 0.17, 0.13)) {
    const double k = 1.0 + 0.18 * unit(random);
    stones.push_back({{k * p.x, k * p.y, k * p.z}});
  }
  shift::core::collision_geometry::ConvexHull hull;
  hull.FitToPoints(stones);
  hull.UpdatePose(Turned(tilt, 0.3, {{3.0, 0.0, 0.0}}));
  write("convex_hull", HullPolygons(hull));

  out << "]}\n";
  return 0;
}

static const char* SurfaceName(shift::core::cad::Surface::Type type) {
  switch(type) {
    case shift::core::cad::Surface::Type::kPlane:
      return "plane";
    case shift::core::cad::Surface::Type::kCylinder:
      return "cylinder";
    case shift::core::cad::Surface::Type::kCone:
      return "cone";
    case shift::core::cad::Surface::Type::kSphere:
      return "sphere";
    case shift::core::cad::Surface::Type::kTorus:
      return "torus";
    case shift::core::cad::Surface::Type::kNurbs:
      return "nurbs";
    case shift::core::cad::Surface::Type::kRevolution:
      return "revolution";
    case shift::core::cad::Surface::Type::kExtrusion:
      return "extrusion";
  }
  return "";
}

/* inside the trim polygons of a face, by the even-odd rule over all of them
(outer loops counterclockwise and holes clockwise) */
static bool Inside(const std::vector<std::vector<shift::core::cad::Vec2>>& loops,
                   double u, double v) {
  bool inside = false;
  for(const auto& loop : loops) {
    const size_t n = loop.size();
    for(size_t i = 0, j = n - 1; i < n; j = i++) {
      const auto& a = loop[i];
      const auto& b = loop[j];
      if((a[1] > v) != (b[1] > v) &&
         u < (b[0] - a[0]) * (v - a[1]) / (b[1] - a[1]) + a[0]) {
        inside = !inside;
      }
    }
  }
  return inside;
}

static int Step(const std::string& file, const std::string& path, int lines) {
  shift::core::cad::Model model = shift::core::cad::ReadStep(file);
  shift::core::cad::Repair(model);
  std::ofstream out(path);
  out << "{\"faces\":[";
  bool first_face = true;
  for(const auto& placed : model.Flatten()) {
    const auto& body = model.bodies[placed.body];
    for(int shell : body.shells) {
      for(int f : model.shells[shell].faces) {
        const auto& face = model.faces[f];
        const auto& surface = model.surfaces[face.surface];
        std::vector<std::vector<shift::core::cad::Vec2>> loops;
        shift::core::cad::Vec2 lo{HUGE_VAL, HUGE_VAL}, hi{-HUGE_VAL, -HUGE_VAL};
        for(const auto& trim : face.trims) {
          loops.push_back(shift::core::cad::TrimPolygon(model, trim, 64));
          for(const auto& p : loops.back()) {
            for(int k = 0; k < 2; k++) {
              lo[k] = std::min(lo[k], p[k]);
              hi[k] = std::max(hi[k], p[k]);
            }
          }
        }
        if(loops.empty()) {
          continue;
        }
        auto world = [&](double u, double v) {
          return placed.transform.Apply(
            shift::core::cad::SurfacePoint(model, surface, u, v));
        };
        /* runs of points inside the trims along a line of constant u
        (along_v) or of constant v */
        auto runs = [&](bool along_v) {
          std::ostringstream text;
          bool first_run = true;
          const int samples = 240;
          for(int k = 1; k < lines; k++) {
            const int fixed = along_v ? 0 : 1;
            const double c = lo[fixed] + (hi[fixed] - lo[fixed]) * k / lines;
            std::vector<shift::core::cad::Vec3> run;
            auto flush = [&]() {
              if(run.size() > 1) {
                text << (first_run ? "" : ",") << "[";
                for(size_t i = 0; i < run.size(); i++) {
                  text << (i ? "," : "") << Point(run[i]);
                }
                text << "]";
                first_run = false;
              }
              run.clear();
            };
            for(int i = 0; i <= samples; i++) {
              const int moving = 1 - fixed;
              const double s =
                lo[moving] + (hi[moving] - lo[moving]) * i / samples;
              const double u = along_v ? c : s, v = along_v ? s : c;
              if(Inside(loops, u, v)) {
                run.push_back(world(u, v));
              } else {
                flush();
              }
            }
            flush();
          }
          return text.str();
        };
        out << (first_face ? "" : ",") << "{\"surface\":\""
            << SurfaceName(surface.type) << "\",\"step_id\":"
            << surface.step_id;
        /* the frame of the surface, transformed to the root */
        const auto& position = surface.position;
        const auto origin = placed.transform.Apply(position.origin);
        auto axis = [&](const shift::core::cad::Vec3& d) {
          const auto tip = placed.transform.Apply(
            {position.origin[0] + d[0], position.origin[1] + d[1],
             position.origin[2] + d[2]});
          return shift::core::cad::Vec3{tip[0] - origin[0], tip[1] - origin[1],
                                        tip[2] - origin[2]};
        };
        out << ",\"origin\":" << Point(origin) << ",\"x\":"
            << Point(axis(position.x)) << ",\"y\":" << Point(axis(position.y))
            << ",\"z\":" << Point(axis(position.z)) << ",\"radius\":"
            << Number(surface.radius) << ",\"minor_radius\":"
            << Number(surface.minor_radius) << ",\"u_range\":["
            << Number(lo[0]) << "," << Number(hi[0]) << "],\"v_range\":["
            << Number(lo[1]) << "," << Number(hi[1]) << "]";
        /* the control net of a B-spline surface: u_count rows of v_count
        points */
        if(surface.type == shift::core::cad::Surface::Type::kNurbs) {
          const auto& nurbs = surface.nurbs;
          out << ",\"control\":[";
          for(int i = 0; i < nurbs.u_count; i++) {
            out << (i ? "," : "") << "[";
            for(int j = 0; j < nurbs.v_count; j++) {
              out << (j ? "," : "")
                  << Point(placed.transform.Apply(
                       nurbs.points[i * nurbs.v_count + j]));
            }
            out << "]";
          }
          out << "]";
        }
        out << ",\"u_lines\":[" << runs(true) << "],\"v_lines\":["
            << runs(false) << "],\"boundary\":[";
        for(size_t l = 0; l < loops.size(); l++) {
          out << (l ? "," : "") << "[";
          for(size_t i = 0; i <= loops[l].size(); i++) {
            const auto& p = loops[l][i % loops[l].size()];
            out << (i ? "," : "") << Point(world(p[0], p[1]));
          }
          out << "]";
        }
        out << "]}";
        first_face = false;
      }
    }
  }
  /* the vertices at the ends of the edges of the faces' loops */
  out << "],\"vertices\":[";
  bool first_vertex = true;
  for(const auto& placed : model.Flatten()) {
    std::vector<bool> used(model.vertices.size(), false);
    for(int shell : model.bodies[placed.body].shells) {
      for(int f : model.shells[shell].faces) {
        for(const auto& loop : model.faces[f].loops) {
          if(loop.vertex >= 0) {
            used[loop.vertex] = true;
          }
          for(const auto& coedge : loop.coedges) {
            const auto& edge = model.edges[coedge.edge];
            for(int v : {edge.start, edge.end}) {
              if(v >= 0) {
                used[v] = true;
              }
            }
          }
        }
      }
    }
    for(size_t v = 0; v < used.size(); v++) {
      if(used[v]) {
        out << (first_vertex ? "" : ",")
            << Point(placed.transform.Apply(model.vertices[v].point));
        first_vertex = false;
      }
    }
  }
  out << "]}\n";
  return 0;
}

static int Fit(const std::string& model, const std::string& path) {
  shift::core::Context context;
  context.LoadJSON(model);
  context.UpdateD();
  std::ofstream out(path);
  out << "{\"shapes\":[";
  bool first = true;
  for(auto geometry : context.get_active_collision_geometry()) {
    geometry->UpdatePose(
      context.get_body_origin_pose(geometry->get_parent_body()));
    auto hull =
      dynamic_cast<const shift::core::collision_geometry::ConvexHull*>(geometry);
    out << (first ? "" : ",") << "{\"name\":\"" << geometry->get_name()
        << "\",\"type\":\"" << geometry->get_type() << "\",\"polygons\":"
        << (hull ? HullPolygons(*hull) : SupportPolygons(*geometry)).Json()
        << "}";
    first = false;
  }
  out << "],\"meshes\":[";
  first = true;
  std::vector<shift::core::body::Base*> bodies;
  for(auto geometry : context.get_active_collision_geometry()) {
    auto body = geometry->get_parent_body();
    if(std::find(bodies.begin(), bodies.end(), body) != bodies.end()) {
      continue;
    }
    bodies.push_back(body);
    for(auto& geometry : body->get_geometry()) {
      auto mesh = dynamic_cast<shift::core::geometry::Mesh*>(geometry.get());
      if(!mesh) {
        continue;
      }
      for(const auto& data : mesh->get_mesh_data()) {
        out << (first ? "" : ",") << "{\"vertices\":[";
        for(size_t i = 0; i < data.vertices.size(); i++) {
          out << (i ? "," : "") << Number(data.vertices[i]);
        }
        out << "],\"faces\":[";
        for(size_t i = 0; i < data.faces.size(); i++) {
          out << (i ? "," : "") << data.faces[i];
        }
        out << "]}";
        first = false;
      }
    }
  }
  out << "]}\n";
  return 0;
}

/* the surface of the first basic terrain of a model, from Basic::LocalZ: z and
its slopes at each point, and its second derivatives along x and y by central
differences of the slopes */
static int Terrain(const std::string& model, const std::string& path,
                   int per_cell) {
  shift::core::Context context;
  context.LoadJSON(model);
  context.UpdateD();
  const shift::core::terrain::Basic* terrain = nullptr;
  for(auto t : context.get_active_terrain()) {
    if((terrain = dynamic_cast<const shift::core::terrain::Basic*>(t))) {
      break;
    }
  }
  if(!terrain) {
    std::fprintf(stderr, "no basic terrain in %s\n", model.c_str());
    return 1;
  }
  const long nx = terrain->get_x_samples(), ny = terrain->get_y_samples();
  const double dx = terrain->get_x_scale(), dy = terrain->get_y_scale();
  std::ofstream out(path);
  out << "{\"bicubic\":" << (terrain->is_bicubic() ? "true" : "false")
      << ",\"x_scale\":" << Number(dx) << ",\"y_scale\":" << Number(dy)
      << ",\"samples\":[";
  for(long j = 0; j < ny; j++) {
    out << (j ? "," : "") << "[";
    for(long i = 0; i < nx; i++) {
      out << (i ? "," : "")
          << Point(i * dx, j * dy, terrain->get_samples()[j * nx + i]);
    }
    out << "]";
  }
  out << "],\"control\":[";
  for(long j = 0; j < ny; j++) {
    out << (j ? "," : "") << "[";
    for(long i = 0; i < nx; i++) {
      out << (i ? "," : "")
          << Point(i * dx, j * dy, terrain->get_control_points()[j * nx + i]);
    }
    out << "]";
  }
  out << "],\"surface\":[";
  const long mx = (nx - 1) * per_cell, my = (ny - 1) * per_cell;
  const double hx = 1e-5 * dx, hy = 1e-5 * dy;
  for(long b = 0; b <= my; b++) {
    out << (b ? "," : "") << "[";
    for(long a = 0; a <= mx; a++) {
      const double x = a * dx / per_cell, y = b * dy / per_cell;
      double z_x, z_y, p_x, p_y, m_x, m_y;
      const double z = terrain->LocalZ(x, y, z_x, z_y);
      terrain->LocalZ(x + hx, y, p_x, p_y);
      terrain->LocalZ(x - hx, y, m_x, m_y);
      const double z_xx = (p_x - m_x) / (2.0 * hx);
      terrain->LocalZ(x, y + hy, p_x, p_y);
      terrain->LocalZ(x, y - hy, m_x, m_y);
      const double z_yy = (p_y - m_y) / (2.0 * hy);
      out << (a ? "," : "") << "[" << Number(x) << "," << Number(y) << ","
          << Number(z) << "," << Number(z_x) << "," << Number(z_y) << ","
          << Number(z_xx) << "," << Number(z_yy) << "]";
    }
    out << "]";
  }
  out << "]}\n";
  return 0;
}

static int BroadPhase(const std::string& model, const std::string& path) {
  shift::core::Context context;
  context.LoadJSON(model);
  context.UpdateD();
  context.DetectContacts();
  const auto& geometry = context.get_contact_geometry();
  const auto& margins = context.get_contact_margins();
  std::ofstream out(path);
  out << "{\"sweep_axis\":" << context.get_sweep_axis() << ",\"geometry\":[";
  for(size_t i = 0; i < geometry.size(); i++) {
    out << (i ? "," : "") << "{\"name\":\"" << geometry[i]->get_name()
        << "\",\"center\":" << Point(geometry[i]->get_d_absolute().t)
        << ",\"radius\":"
        << (std::isfinite(geometry[i]->get_bounding_radius())
              ? Number(geometry[i]->get_bounding_radius())
              : std::string("null")) // a plane
        << ",\"margin\":" << Number(margins[i]) << "}";
  }
  out << "],\"candidates\":[";
  bool first = true;
  for(const auto& [a, b] : context.get_contact_candidates()) {
    out << (first ? "" : ",") << "[" << a << "," << b << "]";
    first = false;
  }
  out << "],\"contacts\":[";
  first = true;
  for(auto pair : context.get_contacts()) {
    int a = -1, b = -1;
    for(size_t i = 0; i < geometry.size(); i++) {
      if(geometry[i] == pair->a) a = static_cast<int>(i);
      if(geometry[i] == pair->b) b = static_cast<int>(i);
    }
    out << (first ? "" : ",") << "[" << a << "," << b << "]";
    first = false;
  }
  out << "]}\n";
  return 0;
}

static std::string Box(const shift::core::collision_geometry::CadBox& box) {
  return "[" + Point(box.lo) + "," + Point(box.hi) + "]";
}

static std::string Pose(const ksl_SE3_t& d) {
  std::ostringstream out;
  out << "{\"R\":[";
  for(int r = 0; r < 3; r++) {
    out << (r ? "," : "") << Point(d.R.v0.at[r], d.R.v1.at[r], d.R.v2.at[r]);
  }
  out << "],\"t\":" << Point(d.t) << "}";
  return out.str();
}

/* a CAD geometry: its pose, the boxes of its tree with their depth, and its
faces, each with its box and its trim loops on the surface, in its frame */
static std::string CadSide(const shift::core::collision_geometry::Cad& cad) {
  const auto& shape = cad.get_shape();
  std::ostringstream out;
  out << "{\"name\":\"" << cad.get_name() << "\",\"pose\":"
      << Pose(cad.get_d_absolute()) << ",\"nodes\":[";
  std::vector<std::pair<int, int>> stack{{0, 0}}; // node, depth
  bool first = true;
  while(!stack.empty() && !shape.nodes.empty()) {
    const auto [k, depth] = stack.back();
    stack.pop_back();
    const auto& node = shape.nodes[k];
    out << (first ? "" : ",") << "{\"box\":" << Box(node.box)
        << ",\"depth\":" << depth << ",\"leaf\":"
        << (node.left < 0 ? "true" : "false") << "}";
    first = false;
    if(node.left >= 0) {
      stack.push_back({node.right, depth + 1});
      stack.push_back({node.left, depth + 1});
    }
  }
  out << "],\"faces\":[";
  for(size_t f = 0; f < shape.faces.size(); f++) {
    out << (f ? "," : "") << "{\"box\":" << Box(shape.faces[f].box)
        << ",\"loops\":[";
    const auto& trims = shape.faces[f].trims;
    for(size_t l = 0; l < trims.size(); l++) {
      out << (l ? "," : "") << "[";
      for(size_t k = 0; k <= trims[l].size(); k++) {
        const auto& uv = trims[l][k % trims[l].size()];
        out << (k ? "," : "") << Point(shape.Point(static_cast<int>(f), uv));
      }
      out << "]";
    }
    out << "]}";
  }
  out << "]}";
  return out.str();
}

static int CadPair(const std::string& model, const std::string& path) {
  shift::core::Context context;
  context.LoadJSON(model);
  context.UpdateD();
  context.DetectContacts();
  const shift::core::collision_geometry::Cad *a = nullptr, *b = nullptr;
  for(const auto& [i, j] : context.get_contact_candidates()) {
    auto ca = dynamic_cast<const shift::core::collision_geometry::Cad*>(
      context.get_contact_geometry()[i]);
    auto cb = dynamic_cast<const shift::core::collision_geometry::Cad*>(
      context.get_contact_geometry()[j]);
    if(ca && cb) {
      a = ca;
      b = cb;
      break;
    }
  }
  if(!a) {
    std::fprintf(stderr, "no pair of CAD geometries in %s\n", model.c_str());
    return 1;
  }
  /* b's frame in a's, and the boxes of b's faces in a's frame tested against
  a's tree, as CadCadContact does */
  ksl_SE3_t b_in_a;
  ksl_product_dinvd(&a->get_d_absolute(), &b->get_d_absolute(), &b_in_a);
  const auto& shape_a = a->get_shape();
  const auto& shape_b = b->get_shape();
  std::ofstream out(path);
  out << "{\"a\":" << CadSide(*a) << ",\"b\":" << CadSide(*b)
      << ",\"roots_overlap\":"
      << (shape_a.box.Overlaps(shape_b.box.Transformed(b_in_a)) ? "true"
                                                                  : "false")
      << ",\"b_boxes_in_a\":[";
  for(size_t f = 0; f < shape_b.faces.size(); f++) {
    out << (f ? "," : "") << Box(shape_b.faces[f].box.Transformed(b_in_a));
  }
  out << "],\"candidates\":[";
  bool first = true;
  for(int fb = 0; fb < static_cast<int>(shape_b.faces.size()); fb++) {
    std::vector<int> found;
    shape_a.Query(shape_b.faces[fb].box.Transformed(b_in_a), found);
    for(int fa : found) {
      out << (first ? "" : ",") << "[" << fa << "," << fb << "]";
      first = false;
    }
  }
  out << "],\"contacts\":[";
  first = true;
  for(auto pair : context.get_contacts()) {
    if(pair->a != a || pair->b != b) {
      continue;
    }
    for(const auto& c : pair->manifold) {
      out << (first ? "" : ",") << "{\"point_a\":" << Point(c.point_a)
          << ",\"point_b\":" << Point(c.point_b)
          << ",\"normal\":" << Point(c.normal)
          << ",\"depth\":" << Number(c.depth)
          << ",\"length\":" << Number(c.length) << "}";
      first = false;
    }
  }
  out << "]}\n";
  return 0;
}

int main(int argc, char** argv) {
  const std::string mode = argc > 1 ? argv[1] : "";
  if(mode == "shapes" && argc == 3) {
    return Shapes(argv[2]);
  }
  if(mode == "step" && (argc == 4 || argc == 5)) {
    return Step(argv[2], argv[3], argc == 5 ? std::atoi(argv[4]) : 8);
  }
  if(mode == "fit" && argc == 4) {
    return Fit(argv[2], argv[3]);
  }
  if(mode == "terrain" && (argc == 4 || argc == 5)) {
    return Terrain(argv[2], argv[3], argc == 5 ? std::atoi(argv[4]) : 16);
  }
  if(mode == "broadphase" && argc == 4) {
    return BroadPhase(argv[2], argv[3]);
  }
  if(mode == "cadpair" && argc == 4) {
    return CadPair(argv[2], argv[3]);
  }
  std::fprintf(stderr,
               "usage: shift-doc-figures shapes <out.json>\n"
               "       shift-doc-figures step <file.step> <out.json> "
               "[lines per parameter]\n"
               "       shift-doc-figures fit <model.json> <out.json>\n"
               "       shift-doc-figures terrain <model.json> <out.json> "
               "[points per cell]\n"
               "       shift-doc-figures broadphase <model.json> <out.json>\n"
               "       shift-doc-figures cadpair <model.json> <out.json>\n");
  return 1;
}

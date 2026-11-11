#include "test/contact/contact_record.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>

#include "gtest/gtest.h"
#include "ksl/SE3.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/cad.h"
#include "shift/core/collision_geometry/ellipsoid.h"
#include "shift/core/collision_geometry/plane.h"
#include "shift/core/collision_geometry/sphere.h"
#include "shift/core/context.h"

/* a surface in the frame of its geometry */
struct RecordedMesh {
  std::vector<std::array<ksl_vec3_t, 4>> quads;
  std::vector<std::array<ksl_vec3_t, 4>> normals; //!< outward, at the corners
  std::vector<std::vector<ksl_vec3_t>> outlines;
};

/* a geometry as last recorded */
struct RecordedGeometry {
  const void* source = nullptr; //!< the geometry, to compare only
  std::string name;
  std::shared_ptr<const RecordedMesh> mesh; //!< empty for a plane
  ksl_SE3_t pose;
  ksl_vec3_t plane_point, plane_normal;
};

struct RecordedContact {
  std::string a, b;
  bool collision = false;
  std::vector<shift::core::collision_geometry::ContactPoint> points;
};

struct RecordedTest {
  bool touched = false;
  std::vector<RecordedGeometry> geometry;
  std::vector<RecordedContact> contacts;
};

static std::string RecordDirectory() {
  const char* directory = std::getenv("SHIFT_CONTACT_RECORD");
  return directory ? directory : "";
}

static bool Recording() {
  static const bool recording = !RecordDirectory().empty();
  return recording;
}

/* the meshes of CAD parts, which are slow to make: a part in a test may be
destroyed and another made at its address, so the key has more than that */
static std::map<std::string, std::shared_ptr<const RecordedMesh>>& CadMeshes() {
  static std::map<std::string, std::shared_ptr<const RecordedMesh>> meshes;
  return meshes;
}

static RecordedTest& Current() {
  static RecordedTest current;
  return current;
}

/* a cell of a face's grid, u0 to u1 and v0 to v1 in its parameters, as a quad;
a cell a trim passes through is split, so that the edge of the mesh follows the
trim */
static void AddCell(const shift::core::collision_geometry::CadContactShape& shape,
                    int face, double u0, double u1, double v0, double v1, int splits,
                    RecordedMesh& mesh) {
  const shift::core::cad::Vec2 corners[4] = {
    {u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}};
  const double u = 0.5 * (u0 + u1), v = 0.5 * (v0 + v1);
  int inside = shape.Inside(face, {u, v}) ? 1 : 0;
  for(const auto& corner : corners) {
    inside += shape.Inside(face, corner) ? 1 : 0;
  }
  if(inside > 0 && inside < 5 && splits > 0) {
    AddCell(shape, face, u0, u, v0, v, splits - 1, mesh);
    AddCell(shape, face, u, u1, v0, v, splits - 1, mesh);
    AddCell(shape, face, u0, u, v, v1, splits - 1, mesh);
    AddCell(shape, face, u, u1, v, v1, splits - 1, mesh);
    return;
  }
  if(!shape.Inside(face, {u, v})) {
    return;
  }
  mesh.quads.push_back({shape.Point(face, corners[0]), shape.Point(face, corners[1]),
                        shape.Point(face, corners[2]), shape.Point(face, corners[3])});
  mesh.normals.push_back({shape.Normal(face, corners[0]), shape.Normal(face, corners[1]),
                          shape.Normal(face, corners[2]), shape.Normal(face, corners[3])});
}

/* the faces of an exact shape as quads of a grid over their parameters, and
their trims as outlines */
static void AddShape(const shift::core::collision_geometry::CadContactShape& shape,
                     RecordedMesh& mesh) {
  for(size_t i = 0; i < shape.faces.size(); i++) {
    const auto& f = shape.faces[i];
    const int face = static_cast<int>(i);
    const ksl_vec3_t size = {{f.box.hi.x - f.box.lo.x, f.box.hi.y - f.box.lo.y,
                              f.box.hi.z - f.box.lo.z}};
    const double share = ksl_vec3_l2norm(&size) / std::max(shape.size, 1e-12);
    const int count = std::min(36, std::max(4, static_cast<int>(48.0 * share)));
    const double du = (f.hi[0] - f.lo[0]) / count, dv = (f.hi[1] - f.lo[1]) / count;
    for(int u = 0; u < count; u++) {
      for(int v = 0; v < count; v++) {
        AddCell(shape, face, f.lo[0] + u * du, f.lo[0] + (u + 1) * du,
                f.lo[1] + v * dv, f.lo[1] + (v + 1) * dv, 4, mesh);
      }
    }
    for(const auto& trim : f.trims) {
      std::vector<ksl_vec3_t> outline;
      for(const auto& uv : trim) {
        outline.push_back(shape.Point(face, uv));
      }
      if(!outline.empty()) {
        outline.push_back(outline.front());
      }
      mesh.outlines.push_back(outline);
    }
  }
}

/* a convex shape from its extreme points along a grid of directions */
static void AddSupport(const shift::core::collision_geometry::Base& geometry,
                       RecordedMesh& mesh) {
  constexpr int kAround = 96, kAlong = 48;
  auto direction_at = [&](int i, int j) {
    const double theta = M_PI * j / kAlong, phi = 2.0 * M_PI * i / kAround;
    return ksl_vec3_t{{std::sin(theta) * std::cos(phi),
                       std::sin(theta) * std::sin(phi), std::cos(theta)}};
  };
  auto point = [&](int i, int j) {
    const ksl_vec3_t direction = direction_at(i, j);
    ksl_vec3_t local, global, inertial;
    ksl_product_drv(&geometry.get_d_absolute().R, &direction, &inertial);
    int hint = 0;
    geometry.Support(inertial, local, global, hint);
    return local;
  };
  for(int i = 0; i < kAround; i++) {
    for(int j = 0; j < kAlong; j++) {
      mesh.quads.push_back({point(i, j), point(i + 1, j), point(i + 1, j + 1),
                            point(i, j + 1)});
      mesh.normals.push_back({direction_at(i, j), direction_at(i + 1, j),
                              direction_at(i + 1, j + 1),
                              direction_at(i, j + 1)});
    }
  }
}

static std::shared_ptr<const RecordedMesh> MeshOf(
  const shift::core::collision_geometry::Base& geometry) {
  if(dynamic_cast<const shift::core::collision_geometry::Plane*>(&geometry)) {
    return nullptr;
  }
  auto mesh = std::make_shared<RecordedMesh>();
  if(auto cad = dynamic_cast<const shift::core::collision_geometry::Cad*>(&geometry)) {
    const auto& shape = cad->get_shape();
    std::ostringstream key;
    key << &geometry << ' ' << geometry.get_name() << ' ' << shape.faces.size()
        << ' ' << shape.size;
    auto& cached = CadMeshes()[key.str()];
    if(!cached) {
      AddShape(shape, *mesh);
      cached = mesh;
    }
    return cached;
  }
  if(auto exact = geometry.MakeExactShape()) {
    AddShape(*exact, *mesh);
  } else {
    AddSupport(geometry, *mesh);
  }
  return mesh;
}

/* the name of the geometry in the record, added if it is new; shapes made by a
test may have no name */
static std::string AddGeometry(const shift::core::collision_geometry::Base& geometry,
                               RecordedTest& test) {
  for(const auto& g : test.geometry) {
    if(g.source == &geometry) {
      return g.name;
    }
  }
  RecordedGeometry g;
  g.source = &geometry;
  g.name = geometry.get_name();
  if(g.name.empty()) {
    g.name = "shape" + std::to_string(test.geometry.size() + 1);
  }
  g.mesh = MeshOf(geometry);
  g.pose = geometry.get_d_absolute();
  if(auto plane = dynamic_cast<const shift::core::collision_geometry::Plane*>(&geometry)) {
    g.plane_point = plane->get_point();
    g.plane_normal = plane->get_normal();
  }
  test.geometry.push_back(g);
  return g.name;
}

static void AddPair(const shift::core::collision_geometry::ContactPair& pair,
                    RecordedTest& test) {
  RecordedContact contact;
  contact.a = AddGeometry(*pair.a, test);
  contact.b = AddGeometry(*pair.b, test);
  contact.collision = pair.collision;
  contact.points = pair.manifold;
  if(contact.points.empty() && pair.collision) {
    shift::core::collision_geometry::ContactPoint point;
    point.point_a = pair.point_a;
    point.point_b = pair.point_b;
    point.depth = pair.depth;
    contact.points.push_back(point);
  }
  // a point has its own normal only with its own law
  if(!pair.point_laws) {
    for(auto& point : contact.points) {
      point.normal = pair.normal;
    }
  }
  test.contacts.push_back(contact);
}

static void WriteVector(std::ostream& out, const ksl_vec3_t& v) {
  out << '[' << v.x << ',' << v.y << ',' << v.z << ']';
}

static ksl_vec3_t Placed(const ksl_SE3_t& pose, const ksl_vec3_t& p) {
  ksl_vec3_t out;
  ksl_product_dv(&pose, &p, &out);
  return out;
}

static void WriteRecord(const std::string& path, const RecordedTest& test) {
  std::ofstream out(path);
  out.precision(9);
  out << "{\"geometry\":[";
  for(size_t i = 0; i < test.geometry.size(); i++) {
    const RecordedGeometry& g = test.geometry[i];
    out << (i ? "," : "") << "{\"name\":\"" << g.name << "\"";
    if(g.mesh) {
      out << ",\"quads\":[";
      for(size_t q = 0; q < g.mesh->quads.size(); q++) {
        out << (q ? ",[" : "[");
        for(int k = 0; k < 4; k++) {
          if(k) {
            out << ',';
          }
          WriteVector(out, Placed(g.pose, g.mesh->quads[q][k]));
        }
        out << ']';
      }
      out << "],\"normals\":[";
      for(size_t q = 0; q < g.mesh->normals.size(); q++) {
        out << (q ? ",[" : "[");
        for(int k = 0; k < 4; k++) {
          if(k) {
            out << ',';
          }
          ksl_vec3_t turned;
          ksl_product_drv(&g.pose.R, &g.mesh->normals[q][k], &turned);
          WriteVector(out, turned);
        }
        out << ']';
      }
      out << "],\"outlines\":[";
      for(size_t o = 0; o < g.mesh->outlines.size(); o++) {
        out << (o ? ",[" : "[");
        const auto& outline = g.mesh->outlines[o];
        for(size_t k = 0; k < outline.size(); k++) {
          if(k) {
            out << ',';
          }
          WriteVector(out, Placed(g.pose, outline[k]));
        }
        out << ']';
      }
      out << ']';
    } else {
      out << ",\"plane_point\":";
      WriteVector(out, g.plane_point);
      out << ",\"plane_normal\":";
      WriteVector(out, g.plane_normal);
    }
    out << '}';
  }
  out << "],\"contacts\":[";
  for(size_t i = 0; i < test.contacts.size(); i++) {
    const RecordedContact& c = test.contacts[i];
    out << (i ? "," : "") << "{\"a\":\"" << c.a << "\",\"b\":\"" << c.b
        << "\",\"collision\":" << (c.collision ? "true" : "false")
        << ",\"points\":[";
    for(size_t p = 0; p < c.points.size(); p++) {
      const auto& point = c.points[p];
      out << (p ? "," : "") << "{\"a\":";
      WriteVector(out, point.point_a);
      out << ",\"b\":";
      WriteVector(out, point.point_b);
      out << ",\"normal\":";
      WriteVector(out, point.normal);
      out << ",\"depth\":" << point.depth << ",\"length\":" << point.length
          << ",\"area\":" << point.area << ",\"weight\":" << point.weight
          << '}';
    }
    out << "]}";
  }
  out << "]}\n";
}

class ContactRecorder : public testing::EmptyTestEventListener {
public:
  void OnTestStart(const testing::TestInfo&) override {
    Current() = RecordedTest();
    CadMeshes().clear();
  }

  void OnTestEnd(const testing::TestInfo& info) override {
    const RecordedTest& test = Current();
    if(test.geometry.empty() && test.contacts.empty()) {
      return;
    }
    std::string name =
      std::string(info.test_suite_name()) + "." + info.name();
    std::replace(name.begin(), name.end(), '/', '_');
    WriteRecord(RecordDirectory() + "/" + name + ".json", test);
  }
};

void shift::test::InstallContactRecorder() {
  if(Recording()) {
    testing::UnitTest::GetInstance()->listeners().Append(new ContactRecorder);
  }
}

bool shift::test::TestContact(shift::core::collision_geometry::ContactPair& pair) {
  const bool result = shift::core::collision_geometry::TestContact(pair);
  const size_t kept = Current().contacts.empty()
                        ? 0
                        : Current().contacts.front().points.size();
  if(Recording() && (pair.collision ? std::max<size_t>(pair.manifold.size(), 1) >= kept
                                    : !Current().touched)) {
    RecordedTest test;
    test.touched = pair.collision;
    AddPair(pair, test);
    Current() = test;
  }
  return result;
}

void shift::test::RecordContacts(const shift::core::Context& context) {
  if(!Recording()) {
    return;
  }
  RecordedTest test;
  for(auto pair : context.get_contacts()) {
    if(pair->collision) {
      test.touched = true;
      AddPair(*pair, test);
    }
  }
  if(test.touched) {
    Current() = test;
  } else if(!Current().touched) {
    Current() = RecordedTest();
    for(auto geometry : context.get_active_collision_geometry()) {
      AddGeometry(*geometry, Current());
    }
  }
}

void shift::test::RecordShape(const shift::core::collision_geometry::Base& geometry) {
  if(Recording()) {
    AddGeometry(geometry, Current());
  }
}

void shift::test::RecordPoints(const std::vector<ksl_vec3_t>& points,
                               const ksl_vec3_t& normal) {
  if(!Recording()) {
    return;
  }
  RecordedContact contact;
  contact.collision = true;
  for(const ksl_vec3_t& p : points) {
    shift::core::collision_geometry::ContactPoint c;
    c.point_a = c.point_b = p;
    c.normal = normal;
    contact.points.push_back(c);
  }
  Current().touched = true;
  Current().contacts.push_back(contact);
}

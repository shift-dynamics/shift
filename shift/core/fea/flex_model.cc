#include "shift/core/fea/flex_model.h"

#include <algorithm>
#include <cmath>
#include <set>

#include <Eigen/Dense>

#include "shift/core/exception.h"
#include "shift/core/fea/solid_mesh.h"

/* a point of the solid in the body's origin coordinates */
static ksl_vec3_t ToOrigin(const shift::core::fea::PlacedSolid& solid,
                           const shift::core::cad::Vec3& p) {
  const shift::core::cad::Vec3 q = solid.placed.transform.Apply(p);
  return ksl_vec3(solid.scale * q[0], solid.scale * q[1], solid.scale * q[2]);
}

/* a direction of the solid in the body's origin coordinates */
static ksl_vec3_t DirectionToOrigin(const shift::core::fea::PlacedSolid& solid,
                                    const shift::core::cad::Vec3& d) {
  const auto& r = solid.placed.transform.rotation;
  ksl_vec3_t v = ksl_vec3(r[0] * d[0] + r[1] * d[1] + r[2] * d[2],
                          r[3] * d[0] + r[4] * d[1] + r[5] * d[2],
                          r[6] * d[0] + r[7] * d[1] + r[8] * d[2]);
  ksl_vec3_normalize(&v);
  return v;
}

/* a point in the body's origin coordinates in the solid's */
static ksl_vec3_t FromOrigin(const shift::core::fea::PlacedSolid& solid,
                             const ksl_vec3_t& p) {
  const shift::core::cad::Vec3 q = solid.placed.transform.Inverse().Apply(
    {p.x / solid.scale, p.y / solid.scale, p.z / solid.scale});
  return ksl_vec3(q[0], q[1], q[2]);
}

shift::core::fea::TetMesh shift::core::fea::MeshPlacedSolid(
  const PlacedSolid& solid, double element_size, double minimum_element_size,
  const std::vector<ksl_vec3_t>& interior) {
  if(!solid.model || solid.placed.body < 0) {
    throw shift::core::Exception("MeshPlacedSolid: no solid");
  }
  std::vector<ksl_vec3_t> in_solid;
  for(const ksl_vec3_t& p : interior) {
    in_solid.push_back(FromOrigin(solid, p));
  }
  TetMesh mesh = MeshSolid(*solid.model, solid.placed.body,
                           element_size / solid.scale,
                           minimum_element_size / solid.scale, in_solid);
  for(ksl_vec3_t& node : mesh.nodes) {
    node = ToOrigin(solid, {node.x, node.y, node.z});
  }
  return mesh;
}

/* the nodes of the boundary triangles on the faces, and whether the
material lies outside the axis (or centre) of each face: the mean of the
outward normals points towards it */
static std::vector<int> FaceNodes(const shift::core::fea::TetMesh& mesh,
                                  const std::set<int>& faces) {
  std::set<int> nodes;
  for(size_t k = 0; k < mesh.boundary.size(); k++) {
    if(faces.count(mesh.boundary_face[k])) {
      nodes.insert(mesh.boundary[k].begin(), mesh.boundary[k].end());
    }
  }
  return std::vector<int>(nodes.begin(), nodes.end());
}

/* the sum over a face's boundary triangles of (outward normal) . (centroid -
nearest point of the line through c along a, or of c if a is zero) */
static double Concavity(const shift::core::fea::TetMesh& mesh, int face,
                        const ksl_vec3_t& c, const ksl_vec3_t* a) {
  double sum = 0.0;
  for(size_t k = 0; k < mesh.boundary.size(); k++) {
    if(mesh.boundary_face[k] != face) {
      continue;
    }
    const std::array<int, 6>& tri = mesh.boundary[k];
    ksl_vec3_t e1, e2, n, centroid = ksl_vec3(0.0, 0.0, 0.0), r;
    ksl_subtract_vv(&mesh.nodes[tri[1]], &mesh.nodes[tri[0]], &e1);
    ksl_subtract_vv(&mesh.nodes[tri[2]], &mesh.nodes[tri[0]], &e2);
    ksl_cross_vv(&e1, &e2, &n);
    for(int i = 0; i < 3; i++) {
      ksl_add_vv(&centroid, &mesh.nodes[tri[i]], &centroid);
    }
    ksl_product_av(1.0 / 3.0, &centroid, &centroid);
    ksl_subtract_vv(&centroid, &c, &r);
    if(a) {
      const double along = ksl_dot_vv(&r, a);
      ksl_vec3_t axial;
      ksl_product_av(along, a, &axial);
      ksl_subtract_vv(&r, &axial, &r);
    }
    sum += ksl_dot_vv(&n, &r);
  }
  return sum;
}

shift::core::fea::Attachment shift::core::fea::ChooseAttachment(
  const TetMesh& mesh, const PlacedSolid* placed,
  const AttachmentRequest& request, double element_size,
  std::vector<int>* faces_used) {
  Attachment attachment;
  attachment.position = request.position;
  attachment.spider = request.spider;
  if(faces_used) {
    faces_used->clear();
  }
  if(!placed || !placed->model) {
    if(!request.faces.empty()) {
      throw shift::core::Exception(
        "Attachment '" + request.name +
        "': faces are those of a STEP solid; a mesh has none");
    }
    if(request.joint != AttachmentRequest::Joint::kNone) {
      attachment.nodes = BearingNodes(mesh, request);
      if(!attachment.nodes.empty()) {
        return attachment;
      }
    }
    return PatchAttachment(mesh, request, element_size);
  }
  const PlacedSolid& solid = *placed;
  std::set<int> faces(request.faces.begin(), request.faces.end());
  const cad::Model& model = *solid.model;
  for(int f : faces) {
    if(f < 0 || f >= static_cast<int>(model.faces.size())) {
      throw shift::core::Exception(
        "Attachment '" + request.name + "': the file has no face " +
        std::to_string(f));
    }
  }

  if(faces.empty() &&
     request.joint != AttachmentRequest::Joint::kNone) {
    /* the faces of the solid by their surfaces: coaxial cylinders or
    concentric spheres, each with its radius and whether it is a bore */
    struct Candidate {
      int face;
      double radius;
      bool concave;
    };
    std::vector<Candidate> candidates;
    std::set<int> solid_faces(mesh.boundary_face.begin(),
                              mesh.boundary_face.end());
    ksl_vec3_t axis = request.axis;
    ksl_vec3_normalize(&axis);
    for(int f : solid_faces) {
      const cad::Surface& s = model.surfaces[model.faces[f].surface];
      const double radius = solid.scale * s.radius;
      const ksl_vec3_t centre = ToOrigin(solid, s.position.origin);
      ksl_vec3_t r;
      ksl_subtract_vv(&request.position, &centre, &r);
      if(request.joint == AttachmentRequest::Joint::kRevolute &&
         s.type == cad::Surface::Type::kCylinder) {
        const ksl_vec3_t z = DirectionToOrigin(solid, s.position.z);
        ksl_vec3_t cross;
        ksl_cross_vv(&z, &axis, &cross);
        ksl_vec3_t off; /* off the cylinder's axis */
        ksl_cross_vv(&r, &z, &off);
        if(ksl_vec3_l2norm(&cross) < 1e-2 &&
           ksl_vec3_l2norm(&off) < 2e-2 * radius) {
          candidates.push_back(
            {f, radius, Concavity(mesh, f, centre, &z) < 0.0});
        }
      } else if(request.joint == AttachmentRequest::Joint::kSpherical &&
                s.type == cad::Surface::Type::kSphere) {
        if(ksl_vec3_l2norm(&r) < 2e-2 * radius) {
          candidates.push_back(
            {f, radius, Concavity(mesh, f, centre, nullptr) < 0.0});
        }
      }
    }
    /* the bores if any, else the pins; those of the smallest radius */
    const bool any_concave =
      std::any_of(candidates.begin(), candidates.end(),
                  [](const Candidate& c) { return c.concave; });
    double smallest = 1e300;
    for(const Candidate& c : candidates) {
      if(c.concave == any_concave) {
        smallest = std::min(smallest, c.radius);
      }
    }
    for(const Candidate& c : candidates) {
      if(c.concave == any_concave && c.radius < smallest * (1.0 + 1e-3)) {
        faces.insert(c.face);
      }
    }
  }

  if(!faces.empty()) {
    attachment.nodes = FaceNodes(mesh, faces);
    if(attachment.nodes.empty()) {
      throw shift::core::Exception(
        "Attachment '" + request.name +
        "': its faces are not faces of the meshed solid");
    }
    if(faces_used) {
      faces_used->assign(faces.begin(), faces.end());
    }
    return attachment;
  }

  return PatchAttachment(mesh, request, element_size);
}

shift::core::fea::Attachment shift::core::fea::PatchAttachment(
  const TetMesh& mesh, const AttachmentRequest& request,
  double element_size) {
  Attachment attachment;
  attachment.position = request.position;
  attachment.spider = request.spider;
  /* a node at the frame: it and the nodes of the elements around it */
  if(request.node >= 0 && !(request.radius > 0.0)) {
    std::set<int> ring = {request.node};
    for(const std::array<int, 10>& tet : mesh.tets) {
      if(std::find(tet.begin(), tet.begin() + 4, request.node) !=
         tet.begin() + 4) {
        ring.insert(tet.begin(), tet.end());
      }
    }
    attachment.nodes.assign(ring.begin(), ring.end());
    return attachment;
  }
  const double radius = request.radius > 0.0 ? request.radius : element_size;
  for(size_t i = 0; i < mesh.nodes.size(); i++) {
    ksl_vec3_t d;
    ksl_subtract_vv(&mesh.nodes[i], &request.position, &d);
    if(ksl_vec3_l2norm(&d) <= radius) {
      attachment.nodes.push_back(static_cast<int>(i));
    }
  }
  if(attachment.nodes.empty()) {
    throw shift::core::Exception(
      "Attachment '" + request.name + "': no node of the mesh lies within " +
      std::to_string(radius) +
      " of the frame (give the attachment a larger radius, or its faces)");
  }
  return attachment;
}

std::vector<int> shift::core::fea::BearingNodes(
  const TetMesh& mesh, const AttachmentRequest& request) {
  const bool revolute = request.joint == AttachmentRequest::Joint::kRevolute;
  ksl_vec3_t axis = request.axis;
  ksl_vec3_normalize(&axis);
  /* the triangles facing the axis (or centre): their distance from it, the
  direction to them, their area, and whether the material lies outside */
  struct Facing {
    size_t triangle;
    double distance;
    ksl_vec3_t direction;
    double area;
    bool concave;
  };
  std::vector<Facing> facing;
  for(size_t k = 0; k < mesh.boundary.size(); k++) {
    const std::array<int, 6>& tri = mesh.boundary[k];
    ksl_vec3_t e1, e2, n, centroid = ksl_vec3(0.0, 0.0, 0.0), r;
    ksl_subtract_vv(&mesh.nodes[tri[1]], &mesh.nodes[tri[0]], &e1);
    ksl_subtract_vv(&mesh.nodes[tri[2]], &mesh.nodes[tri[0]], &e2);
    ksl_cross_vv(&e1, &e2, &n);
    const double twice_area = ksl_vec3_l2norm(&n);
    if(!(twice_area > 0.0)) {
      continue;
    }
    ksl_product_av(1.0 / twice_area, &n, &n);
    for(int i = 0; i < 3; i++) {
      ksl_add_vv(&centroid, &mesh.nodes[tri[i]], &centroid);
    }
    ksl_product_av(1.0 / 3.0, &centroid, &centroid);
    ksl_subtract_vv(&centroid, &request.position, &r);
    if(revolute) {
      if(std::fabs(ksl_dot_vv(&n, &axis)) > 0.25) {
        continue;
      }
      ksl_vec3_t axial;
      ksl_product_av(ksl_dot_vv(&r, &axis), &axis, &axial);
      ksl_subtract_vv(&r, &axial, &r);
    }
    const double distance = ksl_vec3_l2norm(&r);
    if(!(distance > 0.0)) {
      continue;
    }
    ksl_product_av(1.0 / distance, &r, &r);
    const double radial = ksl_dot_vv(&n, &r);
    if(std::fabs(radial) < std::cos(15.0 * M_PI / 180.0)) {
      continue;
    }
    facing.push_back({k, distance, r, 0.5 * twice_area, radial < 0.0});
  }
  std::sort(facing.begin(), facing.end(),
            [](const Facing& a, const Facing& b) {
              return a.distance < b.distance;
            });
  /* the bores, then the pins: the groups within 5% of the radius of the
  nearest left (a facet of a cylinder of 10 sides or more), until one wraps
  round the axis */
  for(bool concave : {true, false}) {
    std::vector<const Facing*> left;
    for(const Facing& f : facing) {
      if(f.concave == concave) {
        left.push_back(&f);
      }
    }
    size_t first = 0;
    while(first < left.size()) {
      const double limit = 1.05 * left[first]->distance;
      size_t last = first;
      /* the spread of the directions to the triangles: the area-weighted
      mean of d d^T, whose eigenvalues show how far they fan out round the
      axis (for a revolute joint, the second smallest, as the smallest is
      along the axis) or the centre */
      Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
      double area = 0.0;
      while(last < left.size() && left[last]->distance <= limit) {
        const ksl_vec3_t& d = left[last]->direction;
        const Eigen::Vector3d e(d.x, d.y, d.z);
        scatter += left[last]->area * e * e.transpose();
        area += left[last]->area;
        last++;
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> spread(scatter / area);
      const bool wraps = revolute ? spread.eigenvalues()(1) > 0.2
                                  : spread.eigenvalues()(0) > 0.1;
      if(wraps) {
        std::set<int> nodes;
        for(size_t i = first; i < last; i++) {
          const std::array<int, 6>& tri = mesh.boundary[left[i]->triangle];
          nodes.insert(tri.begin(), tri.end());
        }
        return std::vector<int>(nodes.begin(), nodes.end());
      }
      first = last;
    }
  }
  return {};
}

std::vector<int> shift::core::fea::NearestNodes(
  const TetMesh& mesh, const std::vector<ksl_vec3_t>& positions) {
  std::vector<int> nodes;
  for(const ksl_vec3_t& p : positions) {
    double best = 1e300;
    int nearest = -1;
    for(size_t i = 0; i < mesh.nodes.size(); i++) {
      ksl_vec3_t d;
      ksl_subtract_vv(&mesh.nodes[i], &p, &d);
      const double distance = ksl_vec3_l2norm(&d);
      if(distance < best) {
        best = distance;
        nearest = static_cast<int>(i);
      }
    }
    nodes.push_back(nearest);
  }
  return nodes;
}

/* the distance along the ray o + t d to the triangle a, b, c, or -1 if it
misses (Moller and Trumbore, Journal of Graphics Tools 2(1), 1997) */
static double RayTriangle(const ksl_vec3_t& o, const ksl_vec3_t& d,
                          const ksl_vec3_t& a, const ksl_vec3_t& b,
                          const ksl_vec3_t& c) {
  ksl_vec3_t e1, e2, p, t, q;
  ksl_subtract_vv(&b, &a, &e1);
  ksl_subtract_vv(&c, &a, &e2);
  ksl_cross_vv(&d, &e2, &p);
  const double det = ksl_dot_vv(&e1, &p);
  if(std::fabs(det) < 1e-300) {
    return -1.0;
  }
  const double inverse = 1.0 / det;
  ksl_subtract_vv(&o, &a, &t);
  const double u = ksl_dot_vv(&t, &p) * inverse;
  if(u < 0.0 || u > 1.0) {
    return -1.0;
  }
  ksl_cross_vv(&t, &e1, &q);
  const double v = ksl_dot_vv(&d, &q) * inverse;
  if(v < 0.0 || u + v > 1.0) {
    return -1.0;
  }
  return ksl_dot_vv(&e2, &q) * inverse;
}

shift::core::fea::Bore shift::core::fea::FindBore(
  const TetMesh& mesh, const AttachmentRequest& request) {
  ksl_vec3_t axis = request.axis;
  ksl_vec3_normalize(&axis);
  /* two directions at right angles to the axis */
  ksl_vec3_t e1 = std::fabs(axis.x) < 0.9 ? ksl_vec3(1.0, 0.0, 0.0)
                                          : ksl_vec3(0.0, 1.0, 0.0);
  ksl_vec3_t along;
  ksl_product_av(ksl_dot_vv(&e1, &axis), &axis, &along);
  ksl_subtract_vv(&e1, &along, &e1);
  ksl_vec3_normalize(&e1);
  ksl_vec3_t e2;
  ksl_cross_vv(&axis, &e1, &e2);
  /* the unit normals of the boundary triangles */
  std::vector<ksl_vec3_t> normals(mesh.boundary.size());
  for(size_t k = 0; k < mesh.boundary.size(); k++) {
    const std::array<int, 6>& tri = mesh.boundary[k];
    ksl_vec3_t a, b;
    ksl_subtract_vv(&mesh.nodes[tri[1]], &mesh.nodes[tri[0]], &a);
    ksl_subtract_vv(&mesh.nodes[tri[2]], &mesh.nodes[tri[0]], &b);
    ksl_cross_vv(&a, &b, &normals[k]);
    ksl_vec3_normalize(&normals[k]);
  }
  auto parallel = [&](size_t k) {
    /* a flat triangle on a curved wall leans out of it a little */
    return std::fabs(ksl_dot_vv(&normals[k], &axis)) < 0.25;
  };
  const int kRays = 16;
  std::set<int> faces;
  double distance_sum = 0.0;
  for(int r = 0; r < kRays; r++) {
    const double angle = 2.0 * M_PI * r / kRays;
    ksl_vec3_t d;
    ksl_product_av(std::cos(angle), &e1, &d);
    ksl_axpy_vv(std::sin(angle), &e2, &d);
    double nearest = 1e300;
    size_t hit = mesh.boundary.size();
    for(size_t k = 0; k < mesh.boundary.size(); k++) {
      const std::array<int, 6>& tri = mesh.boundary[k];
      const double t = RayTriangle(request.position, d, mesh.nodes[tri[0]],
                                   mesh.nodes[tri[1]], mesh.nodes[tri[2]]);
      if(t > 0.0 && t < nearest) {
        nearest = t;
        hit = k;
      }
    }
    if(hit == mesh.boundary.size()) {
      throw shift::core::Exception(
        "Sliding attachment '" + request.name +
        "': no wall round the slide axis in one direction (an open guide, "
        "which is not supported, or the frame is not inside a bore)");
    }
    if(!parallel(hit)) {
      throw shift::core::Exception(
        "Sliding attachment '" + request.name +
        "': the wall round the slide axis is not parallel to it");
    }
    faces.insert(mesh.boundary_face[hit]);
    distance_sum += nearest;
  }
  Bore bore;
  bore.radius = distance_sum / kRays;
  std::set<int> nodes;
  for(size_t k = 0; k < mesh.boundary.size(); k++) {
    if(faces.count(mesh.boundary_face[k]) && parallel(k)) {
      nodes.insert(mesh.boundary[k].begin(), mesh.boundary[k].end());
    }
  }
  bore.nodes.assign(nodes.begin(), nodes.end());
  bore.low = 1e300;
  bore.high = -1e300;
  for(int node : bore.nodes) {
    ksl_vec3_t r;
    ksl_subtract_vv(&mesh.nodes[node], &request.position, &r);
    const double s = ksl_dot_vv(&r, &axis);
    bore.low = std::min(bore.low, s);
    bore.high = std::max(bore.high, s);
  }
  return bore;
}

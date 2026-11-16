#include "shift/core/fea/flex_model.h"

#include <algorithm>
#include <cmath>
#include <set>

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

shift::core::fea::TetMesh shift::core::fea::MeshPlacedSolid(
  const PlacedSolid& solid, double element_size,
  double minimum_element_size) {
  if(!solid.model || solid.placed.body < 0) {
    throw shift::core::Exception("MeshPlacedSolid: no solid");
  }
  TetMesh mesh = MeshSolid(*solid.model, solid.placed.body,
                           element_size / solid.scale,
                           minimum_element_size / solid.scale);
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
  const TetMesh& mesh, const PlacedSolid& solid,
  const AttachmentRequest& request, double element_size,
  std::vector<int>* faces_used) {
  Attachment attachment;
  attachment.position = request.position;
  attachment.spider = request.spider;
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
        ksl_cross_vv(&r, &z, &r); /* off the cylinder's axis */
        if(ksl_vec3_l2norm(&cross) < 1e-2 && ksl_vec3_l2norm(&r) < 2e-2 * radius) {
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
  if(faces_used) {
    faces_used->clear();
  }
  return attachment;
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

/* body::Flex built from a finite element model of its geometry: the keys
common to Modal and Nodal, and BuildElastic */

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <sstream>

#include "ksl/matrix.h"

#include "shift/config.h"
#include "shift/core/body/flex.h"
#include "shift/core/exception.h"
#include "shift/core/frame.h"
#include "shift/core/log.h"

#if SHIFT_HAVE_FEA
#include <filesystem>
#include <fstream>
#include <iterator>

#include "shift/core/context.h"
#include "shift/core/cad/triangulate.h"
#include "shift/core/fea/cache.h"
#include "shift/core/fea/element.h"
#include "shift/core/fea/flex_model.h"
#include "shift/core/fea/reduction.h"
#include "shift/core/fea/tet_mesh.h"
#include "shift/core/geometry/cad.h"
#include "shift/core/geometry/mesh.h"
#endif

void shift::core::body::FlexAttachment::DeclareParameters() {
  DeclareParameter("frame", &frame, "the frame of the body it ties", true);
  DeclareParameter("spider", &spider,
                   "rbe3 (the frame moves with the mean of the nodes, the "
                   "default; does not stiffen them) or rbe2 (the nodes move "
                   "rigidly with the frame)",
                   false);
  DeclareParameter("faces", &faces,
                   "B-rep faces (indices into the STEP file's faces) whose "
                   "nodes are tied, in place of those found from the joint",
                   false)
    .set_row_dim(-1);
  DeclareParameter("radius", &radius,
                   "the nodes within this distance of the frame are tied, "
                   "where no faces are given or found from the joint "
                   "(default: a frame inside the material is a mesh node, "
                   "tied with the nodes of the elements around it; any "
                   "other frame takes element_size)",
                   false);
  DeclareParameter("bearing_length", &bearing_length,
                   "for the parent frame of a prismatic joint whose child "
                   "slides along a bore of the body: the length of the "
                   "bore wall that carries the child, centred on the "
                   "child's frame (default: where the child's geometry lies "
                   "on the bore, else the bore's diameter)",
                   false);
}

void shift::core::body::Flex::DeclareFiniteElementParameters() {
  DeclareParameter("youngs_modulus", &youngs_modulus_,
                   "Young's modulus of the material (required with "
                   "fe_geometry)",
                   false);
  DeclareParameter("poisson_ratio", &poisson_ratio_,
                   "Poisson's ratio of the material (required with "
                   "fe_geometry)",
                   false);
  DeclareParameter("yield_strength", &yield_strength_,
                   "yield strength of the material, for the safety factor "
                   "against it that the viewer draws (optional; no "
                   "effect on the motion)",
                   false)
    .set_runtime();
  DeclareParameter("fe_geometry", &fe_geometry_,
                   "name of the body's cad geometry whose solid is meshed "
                   "with 10-node tetrahedra, or of its mesh geometry whose "
                   "triangles close a solid; the mass properties then come "
                   "from the mesh and density, and inertia and "
                   "center_of_mass are not used",
                   false);
  DeclareParameter("element_size", &element_size_,
                   "largest element edge of the mesh, in model units "
                   "(required with fe_geometry)",
                   false);
  DeclareParameter("minimum_element_size", &minimum_element_size_,
                   "smallest element edge of the mesh (default: "
                   "element_size / 20)",
                   false);
  DeclareParameter("attachments", &attachments_,
                   "how frames are tied to the mesh, overriding the choice "
                   "made from their joints; a frame no joint or chord uses "
                   "(a force module's or a sensor's) is tied only if listed "
                   "here",
                   false);
  DeclareParameter("fe_cache", &fe_cache_,
                   "the file that keeps the finite element analysis, so that "
                   "it runs again only when what it depends on changes "
                   "(default: <model>.<body>.fe.h5 beside the model file)",
                   false);
  DeclareParameter("damping_ratio", &damping_ratio_,
                   "damping ratio of every mode of the reduced body", false);
  DeclareParameter("damping_ratios", &damping_ratios_,
                   "damping ratios of the reduced body's modes, ascending "
                   "in frequency; the last applies to the modes beyond them",
                   false)
    .set_row_dim(-1);
  DeclareParameter("initial_elastic_coordinates", &initial_elastic_coordinates_,
                   "initial elastic coordinates: the static modes first, "
                   "then the kept modes (default: zero)",
                   false)
    .set_row_dim(-1);
  DeclareParameter("initial_elastic_velocities", &initial_elastic_velocities_,
                   "initial rates of the elastic coordinates (default: zero)",
                   false)
    .set_row_dim(-1);
}

#if SHIFT_HAVE_FEA
/* the stations along a bore are the fewest that carry its sliding child
within kSlidingTolerance of the full model (SlidingError), or beyond which
more do not bring the error below kSlidingGain times it: there it is that
of the mesh (the deformation of the wall under the bearing, which the
stations, each averaging its own nodes, carry only in part) */
static const double kSlidingTolerance = 0.01;
static const double kSlidingGain = 0.9;

/* count segments of a bore, centred spacing apart from first (along the
axis from the sliding frame), each tied to the bore's nodes within half a
spacing of its centre; with clamp the end segments reach the bore's ends,
so that every node of the bore is tied */
static std::vector<shift::core::fea::Attachment>
BoreSegments(const shift::core::fea::TetMesh& mesh,
             const shift::core::fea::AttachmentRequest& slide,
             const shift::core::fea::Bore& bore, double first,
             double spacing, int count, bool clamp) {
  ksl_vec3_t axis = slide.axis;
  ksl_vec3_normalize(&axis);
  std::vector<shift::core::fea::Attachment> segments(
    static_cast<size_t>(count));
  for(int k = 0; k < count; k++) {
    segments[size_t(k)].position = slide.position;
    ksl_axpy_vv(first + k * spacing, &axis, &segments[size_t(k)].position);
  }
  for(int node : bore.nodes) {
    ksl_vec3_t r;
    ksl_subtract_vv(&mesh.nodes[size_t(node)], &slide.position, &r);
    const double x = (ksl_dot_vv(&r, &axis) - first) / spacing + 0.5;
    int k = static_cast<int>(std::floor(x));
    if(clamp) {
      k = std::clamp(k, 0, count - 1);
    } else if(k < 0 || k >= count) {
      continue;
    }
    segments[size_t(k)].nodes.push_back(node);
  }
  return segments;
}

/* the surface of a body's active mesh and STEP geometry, in its origin
coordinates (a STEP solid triangulated with edges up to element_size) */
static void BodySurface(shift::core::body::Base& body, double element_size,
                        std::vector<ksl_vec3_t>* points,
                        std::vector<std::array<int, 3>>* triangles) {
  for(auto& geometry : body.get_geometry()) {
    if(!geometry->is_active()) {
      continue;
    }
    if(auto mesh =
         dynamic_cast<shift::core::geometry::Mesh*>(geometry.get())) {
      mesh->Load();
      for(const auto& data : mesh->get_mesh_data()) {
        const int base = static_cast<int>(points->size());
        for(size_t i = 0; i + 2 < data.vertices.size(); i += 3) {
          points->push_back(ksl_vec3(data.vertices[i], data.vertices[i + 1],
                                     data.vertices[i + 2]));
        }
        for(size_t i = 0; i + 2 < data.faces.size(); i += 3) {
          triangles->push_back({base + int(data.faces[i]),
                                base + int(data.faces[i + 1]),
                                base + int(data.faces[i + 2])});
        }
      }
    } else if(auto cad =
                dynamic_cast<shift::core::geometry::Cad*>(geometry.get())) {
      cad->Load();
      const shift::core::geometry::CadAsset* asset = cad->get_asset();
      shift::core::cad::TriangulateOptions options;
      options.element_size = element_size / cad->get_scale();
      for(const auto& placed : asset->bodies) {
        if(!asset->model.bodies[size_t(placed.body)].solid) {
          continue;
        }
        const shift::core::cad::Triangulation t =
          shift::core::cad::Triangulate(asset->model, placed.body, options);
        const int base = static_cast<int>(points->size());
        for(const auto& p : t.points) {
          const shift::core::cad::Vec3 q = placed.transform.Apply(p);
          points->push_back(ksl_vec3(cad->get_scale() * q[0],
                                     cad->get_scale() * q[1],
                                     cad->get_scale() * q[2]));
        }
        for(const auto& triangle : t.triangles) {
          triangles->push_back(
            {base + triangle[0], base + triangle[1], base + triangle[2]});
        }
      }
    }
  }
}

/* the nodes of a bore within the part of a bearing within it, the child
at s, offsets along the axis from the sliding frame */
static std::vector<int>
BearingNodes(const shift::core::fea::TetMesh& mesh,
             const shift::core::fea::AttachmentRequest& slide,
             const shift::core::fea::Bore& bore,
             const shift::core::body::SlidingBearing& bearing, double s) {
  ksl_vec3_t axis = slide.axis;
  ksl_vec3_normalize(&axis);
  std::vector<int> nodes;
  for(int node : bore.nodes) {
    ksl_vec3_t r;
    ksl_subtract_vv(&mesh.nodes[size_t(node)], &slide.position, &r);
    const double x = ksl_dot_vv(&r, &axis);
    for(const auto& interval : bearing.intervals) {
      if(x >= std::max(s + interval[0], bearing.low) &&
         x <= std::min(s + interval[1], bearing.high)) {
        nodes.push_back(node);
        break;
      }
    }
  }
  return nodes;
}

/* the full model's compliance at the child of a sliding frame, as it bears
on the bore (BearingNodes tied to it as one attachment at its point s +
center along the axis), at positions s spread along its travel (where it
bears on 3 nodes or more): each 36 values, row-major, translation first */
struct BearingCompliance {
  double s = 0.0;
  std::vector<double> compliance;
};
static std::vector<BearingCompliance>
FullBearingCompliances(const shift::core::fea::TetMesh& mesh,
                       const shift::core::fea::HeldCompliance& held,
                       const shift::core::fea::AttachmentRequest& slide,
                       const shift::core::fea::Bore& bore,
                       const shift::core::body::SlidingBearing& bearing) {
  ksl_vec3_t axis = slide.axis;
  ksl_vec3_normalize(&axis);
  double first = 1e300, last = -1e300;
  for(const auto& interval : bearing.intervals) {
    first = std::min(first, interval[0]);
    last = std::max(last, interval[1]);
  }
  const double center = shift::core::body::Flex::BearingCenter(bearing);
  const double from = bore.low - last, to = bore.high - first;
  const int positions = 5;
  std::vector<BearingCompliance> result;
  for(int k = 0; k < positions; k++) {
    BearingCompliance entry;
    entry.s = from + (to - from) * (k + 1) / (positions + 1);
    shift::core::fea::Attachment attachment;
    attachment.nodes = BearingNodes(mesh, slide, bore, bearing, entry.s);
    if(attachment.nodes.size() < 3) {
      continue;
    }
    attachment.position = slide.position;
    ksl_axpy_vv(entry.s + center, &axis, &attachment.position);
    entry.compliance = held.Compliance({attachment});
    result.push_back(entry);
  }
  return result;
}

/* how well count stations along a bore carry the child of a sliding frame
as it bears on the bore: at each of the full model's positions, the
compliance at the child as the stations carry it (the stations'
compliance mapped by the cardinal weights of the bearing's mean, with its
transport), against the full model's. The error of the translation block
and of the rotation block (Frobenius norms), each the greatest over the
positions relative to the greatest compliance there (near the parent
frame, where the compliance is least, an error relative to each position's
own would be dominated by the deformation of the bore under the bearing);
the greater of the two. */
static double SlidingError(const shift::core::fea::TetMesh& mesh,
                           const shift::core::fea::HeldCompliance& held,
                           const shift::core::fea::AttachmentRequest& slide,
                           const shift::core::fea::Bore& bore, int count,
                           const shift::core::body::SlidingBearing& bearing,
                           const std::vector<BearingCompliance>& full) {
  if(full.empty()) {
    return 0.0;
  }
  ksl_vec3_t axis = slide.axis;
  ksl_vec3_normalize(&axis);
  const double spacing = (bore.high - bore.low) / count;
  const double first = bore.low + 0.5 * spacing;
  const std::vector<double> c_s = held.Compliance(
    BoreSegments(mesh, slide, bore, first, spacing, count, true));
  const size_t n = 6 * size_t(count);
  double error_max[2] = {0.0, 0.0}, size_max[2] = {0.0, 0.0};
  for(const BearingCompliance& entry : full) {
    /* the child's motion u = T u_s of the stations' */
    const std::vector<double> T = shift::core::body::Flex::SlidingMap(
      size_t(count), first, spacing, axis, entry.s, bearing);
    std::vector<double> TC(6 * n, 0.0);
    for(size_t r = 0; r < 6; r++) {
      for(size_t k = 0; k < n; k++) {
        if(T[r * n + k] == 0.0) {
          continue;
        }
        for(size_t c = 0; c < n; c++) {
          TC[r * n + c] += T[r * n + k] * c_s[k * n + c];
        }
      }
    }
    for(int block = 0; block < 2; block++) {
      double error = 0.0, size = 0.0;
      for(int r = 3 * block; r < 3 * block + 3; r++) {
        for(int c = 3 * block; c < 3 * block + 3; c++) {
          double spline = 0.0;
          for(size_t k = 0; k < n; k++) {
            spline += TC[size_t(r) * n + k] * T[size_t(c) * n + k];
          }
          const double value = entry.compliance[size_t(r) * 6 + size_t(c)];
          error += (spline - value) * (spline - value);
          size += value * value;
        }
      }
      error_max[block] = std::max(error_max[block], std::sqrt(error));
      size_max[block] = std::max(size_max[block], std::sqrt(size));
    }
  }
  return std::max(error_max[0] / size_max[0], error_max[1] / size_max[1]);
}
#endif

void shift::core::body::Flex::BuildElastic(Frame* reference,
                                           const std::vector<FrameUse>& uses) {
  if(!builds_model()) {
    return;
  }
#if !SHIFT_HAVE_FEA
  (void)reference;
  (void)uses;
  throw shift::core::Exception(
    "Flexible body '" + get_name() +
    "': shift was built without finite element support (meson option "
    "'fea')");
#else
  if(!reference) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "' has no parent joint");
  }
  if(!(density_ > 0.0 && youngs_modulus_ > 0.0 && poisson_ratio_ > -1.0 &&
       poisson_ratio_ < 0.5)) {
    throw shift::core::Exception(
      "Flexible body '" + get_name() +
      "': fe_geometry needs a positive density and youngs_modulus, and a "
      "poisson_ratio between -1 and 0.5");
  }
  if(!(element_size_ > 0.0)) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "': fe_geometry needs a positive "
                                 "element_size");
  }
  const int selections = (mode_count_ > 0 ? 1 : 0) +
                         (maximum_frequency_ > 0.0 ? 1 : 0) +
                         (modes_kept_.empty() ? 0 : 1);
  if(selections > 1) {
    throw shift::core::Exception(
      "Flexible body '" + get_name() +
      "': give at most one of mode_count, maximum_frequency and modes_kept");
  }
  for(const char* key : {"inertia", "center_of_mass"}) {
    if(auto value = Find(key); value && value->is_loaded_from_file()) {
      SHIFT_DEBUG("Flexible body '" + get_name() + "': " + key +
                  " is not used; the mass properties come from the mesh");
    }
  }

  /* the solid: a STEP solid, or a closed triangle mesh */
  shift::core::geometry::Cad* cad = nullptr;
  shift::core::geometry::Mesh* triangles = nullptr;
  bool named = false;
  for(auto& geometry : geometry_.get_data()) {
    if(geometry->get_name() == fe_geometry_) {
      named = true;
      cad = dynamic_cast<shift::core::geometry::Cad*>(geometry.get());
      triangles = dynamic_cast<shift::core::geometry::Mesh*>(geometry.get());
      if(!cad && !triangles) {
        throw shift::core::Exception(
          "Flexible body '" + get_name() + "': fe_geometry '" + fe_geometry_ +
          "' is not a cad or mesh geometry");
      }
    }
  }
  if(!named) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "' has no geometry '" + fe_geometry_ + "'");
  }
  /* what is being done, for the viewer's progress */
  auto report = [this](const char* stage) {
    context_->ReportLoad("Flexible body '" + get_name() + "': " + stage);
  };
  report("reading its solid");
  shift::core::fea::PlacedSolid solid;
  shift::core::fea::ClosedSurface surface;
  if(cad) {
    cad->Load();
    const shift::core::geometry::CadAsset* asset = cad->get_asset();
    solid.model = &asset->model;
    solid.scale = cad->get_scale();
    int solids = 0;
    for(const auto& placed : asset->bodies) {
      if(asset->model.bodies[placed.body].solid) {
        solid.placed = placed;
        solids++;
      }
    }
    if(solids != 1) {
      throw shift::core::Exception(
        "Flexible body '" + get_name() + "': fe_geometry '" + fe_geometry_ +
        "' has " + std::to_string(solids) +
        " solids; choose one with the geometry's part");
    }
  } else {
    /* the mesh's triangles, in the body's origin coordinates, with the
    corners its files split (at seams and sharp edges) joined again */
    triangles->Load();
    std::vector<ksl_vec3_t> corners;
    std::vector<std::array<int, 3>> faces;
    ksl_vec3_t low = ksl_vec3(1e300, 1e300, 1e300);
    ksl_vec3_t high = ksl_vec3(-1e300, -1e300, -1e300);
    for(const auto& data : triangles->get_mesh_data()) {
      const int base = static_cast<int>(corners.size());
      for(size_t i = 0; i + 2 < data.vertices.size(); i += 3) {
        const ksl_vec3_t p = ksl_vec3(data.vertices[i], data.vertices[i + 1],
                                      data.vertices[i + 2]);
        corners.push_back(p);
        low = ksl_vec3(std::min(low.x, p.x), std::min(low.y, p.y),
                       std::min(low.z, p.z));
        high = ksl_vec3(std::max(high.x, p.x), std::max(high.y, p.y),
                        std::max(high.z, p.z));
      }
      for(size_t i = 0; i + 2 < data.faces.size(); i += 3) {
        faces.push_back({base + int(data.faces[i]), base + int(data.faces[i + 1]),
                         base + int(data.faces[i + 2])});
      }
    }
    ksl_vec3_t diagonal;
    ksl_subtract_vv(&high, &low, &diagonal);
    surface = shift::core::fea::WeldSurface(
      corners, faces, 1e-6 * ksl_vec3_l2norm(&diagonal));
    if(surface.triangles.empty()) {
      throw shift::core::Exception("Flexible body '" + get_name() +
                                   "': fe_geometry '" + fe_geometry_ +
                                   "' has no triangles");
    }
  }

  const double h = element_size_;
  const double h_min =
    minimum_element_size_ > 0.0 ? minimum_element_size_ : h / 20.0;

  /* the attachments: the reference frame first, then the other frames
  used, then those only listed in attachments */
  std::map<std::string, const FlexAttachment*> overrides;
  for(auto& entry : attachments_.get_data()) {
    bool found = false;
    for(auto& frame : get_frames()) {
      found = found || frame->get_name() == entry->frame;
    }
    if(!found) {
      throw shift::core::Exception("Flexible body '" + get_name() +
                                   "': attachment of an unknown frame '" +
                                   entry->frame + "'");
    }
    if(entry->spider != "rbe3" && entry->spider != "rbe2") {
      throw shift::core::Exception("Flexible body '" + get_name() +
                                   "': attachment '" + entry->frame +
                                   "' has spider '" + entry->spider +
                                   "' (rbe3 or rbe2)");
    }
    overrides[entry->frame] = entry.get();
  }
  std::vector<FrameUse> frames;
  auto add = [&](const FrameUse& use) {
    for(const FrameUse& f : frames) {
      if(f.frame == use.frame) {
        return;
      }
    }
    frames.push_back(use);
  };
  FrameUse reference_use;
  reference_use.frame = reference;
  for(const FrameUse& use : uses) {
    if(use.frame == reference) {
      reference_use = use;
    }
  }
  add(reference_use);
  for(const FrameUse& use : uses) {
    add(use);
  }
  for(auto& entry : attachments_.get_data()) {
    for(auto& frame : get_frames()) {
      if(frame->get_name() == entry->frame) {
        FrameUse use;
        use.frame = frame.get();
        add(use);
      }
    }
  }
  /* the frames that hold the parent side of a prismatic joint slide along
  a bore: tied to stations along it (below), not as an attachment of their
  own */
  std::vector<shift::core::fea::AttachmentRequest> requests;
  std::vector<shift::core::fea::AttachmentRequest> slides;
  std::vector<double> bearing_lengths;
  std::vector<size_t> sliding_frames;
  std::vector<size_t> request_frames; /*!< the frame of each request */
  for(size_t f = 0; f < frames.size(); f++) {
    const FrameUse& use = frames[f];
    const ksl_SE3_t* D = use.frame->get_d_wrt_origin();
    if(use.joint == FrameUse::Joint::kPrismatic) {
      if(use.frame == reference) {
        throw shift::core::Exception(
          "Flexible body '" + get_name() + "': its reference frame '" +
          use.frame->get_name() + "' cannot slide");
      }
      shift::core::fea::AttachmentRequest slide;
      slide.name = use.frame->get_name();
      slide.position = D->t;
      ksl_product_drv(&D->R, &use.axis, &slide.axis);
      auto entry = overrides.find(slide.name);
      bearing_lengths.push_back(
        entry != overrides.end() ? entry->second->bearing_length : 0.0);
      slides.push_back(slide);
      sliding_frames.push_back(f);
      continue;
    }
    shift::core::fea::AttachmentRequest request;
    request.name = use.frame->get_name();
    request.position = D->t;
    ksl_product_drv(&D->R, &use.axis, &request.axis);
    request.joint =
      use.joint == FrameUse::Joint::kRevolute
        ? shift::core::fea::AttachmentRequest::Joint::kRevolute
      : use.joint == FrameUse::Joint::kSpherical
        ? shift::core::fea::AttachmentRequest::Joint::kSpherical
        : shift::core::fea::AttachmentRequest::Joint::kNone;
    auto entry = overrides.find(request.name);
    if(entry != overrides.end()) {
      request.spider = entry->second->spider == "rbe2"
                         ? shift::core::fea::Attachment::Spider::kRbe2
                         : shift::core::fea::Attachment::Spider::kRbe3;
      request.faces = entry->second->faces;
      request.radius = entry->second->radius;
    }
    requests.push_back(request);
    request_frames.push_back(f);
  }
  if(retained_points_.size() % 3) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "': retained_points takes [x, y, z] of "
                                 "each point");
  }
  std::vector<ksl_vec3_t> retained;
  for(size_t i = 0; i < retained_points_.size(); i += 3) {
    retained.push_back(ksl_vec3(retained_points_[i], retained_points_[i + 1],
                                retained_points_[i + 2]));
  }

  /* the fixed-interface modes to compute: as many as the kept need, and
  for a frequency, 10 at first */
  const bool modal = HasFixedInterfaceModes();
  const bool by_frequency = modal && maximum_frequency_ > 0.0;
  unsigned int needed = 0;
  if(modal) {
    if(!modes_kept_.empty()) {
      for(int k : modes_kept_) {
        if(k < 0) {
          throw shift::core::Exception("Flexible body '" + get_name() +
                                       "': modes_kept has a negative index");
        }
        needed = std::max(needed, static_cast<unsigned int>(k) + 1);
      }
    } else if(by_frequency) {
      needed = 10;
    } else {
      needed = mode_count_ > 0 ? static_cast<unsigned int>(mode_count_) : 10;
    }
  }

  /* the surface of each sliding frame's child in its frame's coordinates
  (which are the sliding frame's as the joint's coordinate is zero), where
  it bears on the bore; none with a bearing_length */
  std::vector<std::vector<ksl_vec3_t>> child_points(slides.size());
  std::vector<std::vector<std::array<int, 3>>> child_triangles(slides.size());
  for(size_t i = 0; i < slides.size(); i++) {
    const FrameUse& use = frames[sliding_frames[i]];
    if(bearing_lengths[i] > 0.0 || !use.child_body || !use.child_frame) {
      continue;
    }
    BodySurface(*use.child_body, h, &child_points[i], &child_triangles[i]);
    const ksl_SE3_t* D = use.child_frame->get_d_wrt_origin();
    for(ksl_vec3_t& p : child_points[i]) {
      ksl_vec3_t r;
      ksl_subtract_vv(&p, &D->t, &r);
      ksl_product_drinvv(&D->R, &r, &p);
    }
  }
  /* where the child of sliding frame i bears on a bore of radius: over
  bearing_length centred on its frame if given, else its surface on the
  bore, else the bore's diameter centred on its frame */
  auto bearing_of = [&](size_t i, double radius, std::string* source) {
    shift::core::body::SlidingBearing bearing;
    if(bearing_lengths[i] > 0.0) {
      bearing.intervals = {{-0.5 * bearing_lengths[i], 0.5 * bearing_lengths[i]}};
      *source = "its bearing_length";
      return bearing;
    }
    ksl_vec3_t axis = frames[sliding_frames[i]].axis;
    ksl_vec3_normalize(&axis);
    bearing.intervals = shift::core::fea::BearingIntervals(
      child_points[i], child_triangles[i], ksl_vec3(0.0, 0.0, 0.0), axis,
      radius);
    if(!bearing.intervals.empty()) {
      *source = "its geometry";
    } else {
      bearing.intervals = {{-radius, radius}};
      *source = "the bore's diameter: no surface of the child lies on the bore";
    }
    return bearing;
  };

  /* what the analysis depends on, but for the modes chosen */
  std::stringstream fingerprint;
  fingerprint.precision(17);
  if(cad) {
    const std::string file = context_->FindFile(cad->get_filename(), cad->get_source());
    std::ifstream in(file, std::ios::binary);
    const std::string bytes((std::istreambuf_iterator<char>(in)),
                            std::istreambuf_iterator<char>());
    fingerprint << "step " << std::hex
                << shift::core::fea::Fnv1a(bytes.data(), bytes.size())
                << std::dec << " " << bytes.size() << "\n";
    fingerprint << "body " << solid.placed.body << " scale " << solid.scale
                << " placement";
    for(double r : solid.placed.transform.rotation) {
      fingerprint << " " << r;
    }
    for(double t : solid.placed.transform.translation) {
      fingerprint << " " << t;
    }
  } else {
    /* the triangles, as placed and scaled */
    fingerprint << "triangles " << std::hex
                << shift::core::fea::Fnv1a(
                     surface.points.data(),
                     surface.points.size() * sizeof(ksl_vec3_t))
                << " "
                << shift::core::fea::Fnv1a(
                     surface.triangles.data(),
                     surface.triangles.size() * sizeof(surface.triangles[0]))
                << std::dec << " " << surface.points.size() << " "
                << surface.triangles.size();
  }
  fingerprint << "\nmaterial " << youngs_modulus_ << " " << poisson_ratio_
              << " " << density_ << "\nmesh " << h << " " << h_min
              << " interior frames as nodes\n";
  for(const auto& r : requests) {
    fingerprint << "attachment '" << r.name << "' " << r.position.x << " "
                << r.position.y << " " << r.position.z << " joint "
                << static_cast<int>(r.joint) << " " << r.axis.x << " "
                << r.axis.y << " " << r.axis.z << " spider "
                << static_cast<int>(r.spider) << " radius " << r.radius
                << " faces";
    for(int f : r.faces) {
      fingerprint << " " << f;
    }
    fingerprint << "\n";
  }
  for(const ksl_vec3_t& p : retained) {
    fingerprint << "retained " << p.x << " " << p.y << " " << p.z << "\n";
  }
  for(size_t i = 0; i < slides.size(); i++) {
    const auto& r = slides[i];
    fingerprint << "sliding '" << r.name << "' " << r.position.x << " "
                << r.position.y << " " << r.position.z << " axis " << r.axis.x
                << " " << r.axis.y << " " << r.axis.z << " bearing "
                << bearing_lengths[i] << " child " << std::hex
                << shift::core::fea::Fnv1a(
                     child_points[i].data(),
                     child_points[i].size() * sizeof(ksl_vec3_t))
                << " "
                << shift::core::fea::Fnv1a(child_triangles[i].data(),
                                           child_triangles[i].size() *
                                             sizeof(child_triangles[i][0]))
                << std::dec << "\n";
  }

  /* the cache file: fe_cache, or <model>.<body>.fe.h5, beside the model */
  const std::filesystem::path model_file(context_->get_filename());
  std::string cache_path;
  if(!fe_cache_.empty()) {
    const std::filesystem::path given(fe_cache_);
    cache_path =
      (given.is_absolute() ? given : model_file.parent_path() / given)
        .string();
  } else if(!model_file.empty()) {
    cache_path = (model_file.parent_path() /
                  (model_file.stem().string() + "." +
                   FullyQualifiedName(get_name()) + ".fe.h5"))
                   .string();
  }
  shift::core::fea::FlexCache cache;
  report("reading its finite element cache");
  bool cached = !cache_path.empty() &&
                shift::core::fea::ReadFlexCache(cache_path, fingerprint.str(),
                                                cache);
  if(cached && modal) {
    const auto& f = cache.full.frequencies;
    cached = by_frequency ? !f.empty() && f.back() > maximum_frequency_
                          : f.size() >= needed;
  }
  std::stringstream ties;
  if(!cached) {
    report("meshing its solid");
    /* the frames tied by no faces, and the retained points, are nodes where
    they lie inside the material */
    std::vector<ksl_vec3_t> interior;
    for(const auto& request : requests) {
      interior.push_back(request.position);
    }
    interior.insert(interior.end(), retained.begin(), retained.end());
    shift::core::fea::TetMesh mesh =
      cad ? shift::core::fea::MeshPlacedSolid(solid, h, h_min, interior)
          : shift::core::fea::MeshVolume(
              shift::core::fea::RemeshSurface(surface, h, h_min), h, h_min,
              interior);
    for(size_t a = 0; a < requests.size(); a++) {
      if(requests[a].faces.empty()) {
        requests[a].node = mesh.interior_nodes[a];
      }
    }
    report("assembling its finite element model");
    shift::core::fea::Material material;
    material.youngs_modulus = youngs_modulus_;
    material.poisson_ratio = poisson_ratio_;
    material.density = density_;
    const shift::core::fea::Assembled assembled =
      shift::core::fea::Assemble(mesh, material);
    std::vector<shift::core::fea::Attachment> attachments;
    for(const auto& request : requests) {
      std::vector<int> faces;
      attachments.push_back(
        shift::core::fea::ChooseAttachment(mesh, cad ? &solid : nullptr, request, h, &faces));
      ties << " '" << request.name << "' "
           << (request.spider == shift::core::fea::Attachment::Spider::kRbe2
                 ? "rbe2"
                 : "rbe3")
           << " to " << attachments.back().nodes.size() << " nodes";
      if(!faces.empty()) {
        ties << " of faces";
        for(int f : faces) {
          ties << " " << f;
        }
      }
      ties << ";";
    }
    /* the stations of each sliding frame: the bore cut into equal lengths,
    each tied to its frame-to-be at its middle on the axis; 2 or more, each
    a mesh size long or more, as kSlidingTolerance and kSlidingGain choose */
    std::vector<std::string> owners;
    for(const auto& request : requests) {
      owners.push_back(request.name);
    }
    cache.sliding_stations.clear();
    cache.sliding_radii.clear();
    std::unique_ptr<shift::core::fea::HeldCompliance> held;
    for(size_t i = 0; i < slides.size(); i++) {
      const auto& slide = slides[i];
      report("placing the stations of its sliding frames");
      const shift::core::fea::Bore bore =
        shift::core::fea::FindBore(mesh, slide);
      std::string source;
      shift::core::body::SlidingBearing bearing =
        bearing_of(i, bore.radius, &source);
      bearing.radius = bore.radius;
      bearing.low = bore.low;
      bearing.high = bore.high;
      if(!held) {
        held = std::make_unique<shift::core::fea::HeldCompliance>(
          mesh, assembled, attachments[0]);
      }
      const std::vector<BearingCompliance> full =
        FullBearingCompliances(mesh, *held, slide, bore, bearing);
      const double extent = bore.high - bore.low;
      const int most = std::max(2, static_cast<int>(extent / h));
      int count = 2;
      double error =
        SlidingError(mesh, *held, slide, bore, count, bearing, full);
      while(error > kSlidingTolerance && count < most) {
        const int next = std::min(
          most, std::max(count + 1, static_cast<int>(std::lround(1.5 * count))));
        const double spacing = extent / next;
        const auto segments = BoreSegments(
          mesh, slide, bore, bore.low + 0.5 * spacing, spacing, next, true);
        if(std::any_of(segments.begin(), segments.end(),
                       [](const shift::core::fea::Attachment& a) {
                         return a.nodes.empty();
                       })) {
          break;
        }
        const double next_error =
          SlidingError(mesh, *held, slide, bore, next, bearing, full);
        if(!(next_error < kSlidingGain * error)) {
          break;
        }
        count = next;
        error = next_error;
      }
      const double spacing = extent / count;
      std::vector<shift::core::fea::Attachment> stations =
        BoreSegments(mesh, slide, bore, bore.low + 0.5 * spacing, spacing,
                     count, true);
      for(auto& station : stations) {
        attachments.push_back(station);
        owners.push_back(slide.name);
      }
      cache.sliding_stations.push_back(count);
      cache.sliding_radii.push_back(bore.radius);
      ties << " '" << slide.name << "' sliding along " << count
           << " stations " << spacing << " apart of a bore of radius "
           << bore.radius << " (" << bore.nodes.size()
           << " nodes), its child bearing on it over";
      for(const auto& interval : bearing.intervals) {
        ties << " [" << interval[0] << ", " << interval[1] << "]";
      }
      ties << " from its frame (from " << source
           << "); the compliance at the child within " << error
           << " of the full model's;";
    }
    /* a node in two attachments: name the frames */
    std::map<int, size_t> owner;
    for(size_t a = 0; a < attachments.size(); a++) {
      for(int node : attachments[a].nodes) {
        auto [it, inserted] = owner.emplace(node, a);
        if(!inserted) {
          throw shift::core::Exception(
            "Flexible body '" + get_name() + "': the attachments of frames '" +
            owners[it->second] + "' and '" + owners[a] +
            "' share nodes (give one of them its faces or a smaller radius)");
        }
      }
    }
    std::vector<int> retained_nodes =
      shift::core::fea::NearestNodes(mesh, retained);
    for(size_t i = 0; i < retained.size(); i++) {
      const int node = mesh.interior_nodes[requests.size() + i];
      if(node >= 0) {
        retained_nodes[i] = node;
      }
    }
    /* every mode computed kept, for KeepModes to choose from; for a
    frequency, more until one lies above it */
    report("computing its modes");
    unsigned int computed = needed;
    for(;;) {
      cache.full = shift::core::fea::ReduceKeepingAll(
        mesh, assembled, attachments, 0, retained_nodes, computed);
      const auto& f = cache.full.frequencies;
      if(!by_frequency || f.size() < computed ||
         f.back() > maximum_frequency_) {
        break;
      }
      computed *= 2;
    }
    cache.node_count = static_cast<unsigned int>(mesh.nodes.size());
    cache.attachment_nodes.clear();
    for(const auto& attachment : attachments) {
      cache.attachment_nodes.push_back(attachment.nodes);
    }
    cache.boundary = mesh.boundary;
    cache.boundary_face = mesh.boundary_face;
    /* the stresses on the surface, for the viewer to draw */
    report("computing its stresses");
    std::set<int> surface;
    for(const auto& triangle : mesh.boundary) {
      surface.insert(triangle.begin(), triangle.end());
    }
    cache.surface_nodes.assign(surface.begin(), surface.end());
    cache.surface_stresses = shift::core::fea::NodalStresses(
      mesh, material, cache.surface_nodes, cache.full.shapes,
      cache.full.elastic_count);
    cache.volume = assembled.volume;
    cache.tet_count = static_cast<int>(mesh.tets.size());
    if(!cache_path.empty() &&
       !shift::core::fea::WriteFlexCache(cache_path, fingerprint.str(),
                                         cache)) {
      SHIFT_WARNING("Flexible body '" + get_name() +
                    "': could not write its finite element cache " +
                    cache_path);
    }
  }

  /* the modes kept */
  std::vector<unsigned int> kept;
  if(modal) {
    const auto& f = cache.full.frequencies;
    if(!modes_kept_.empty()) {
      for(int k : modes_kept_) {
        kept.push_back(static_cast<unsigned int>(k));
      }
    } else if(by_frequency) {
      for(unsigned int k = 0; k < f.size(); k++) {
        if(f[k] <= maximum_frequency_) {
          kept.push_back(k);
        }
      }
    } else {
      for(unsigned int k = 0; k < needed && k < f.size(); k++) {
        kept.push_back(k);
      }
    }
  }
  const shift::core::fea::Reduction reduction =
    shift::core::fea::KeepModes(cache.full, kept);
  if(reduction.elastic_count == 0) {
    throw shift::core::Exception(
      "Flexible body '" + get_name() +
      "' has no elastic coordinates (one attachment and no modes kept)");
  }
  std::vector<double> ratios = damping_ratios_;
  if(ratios.empty() && damping_ratio_ > 0.0) {
    ratios.push_back(damping_ratio_);
  }
  const std::vector<double> damping =
    shift::core::fea::ModalDamping(reduction, ratios);

  const size_t points = reduction.positions.size();
  SetModel(reduction.positions, reduction.masses,
           std::vector<ksl_mat3x3_t>(points, ksl_mat3x3_t{}),
           reduction.elastic_count, reduction.shapes, reduction.stiffness,
           damping);
  node_names_.clear();
  node_points_.clear();
  node_sliding_.clear();
  sliding_.clear();
  size_t station_count = 0;
  for(int count : cache.sliding_stations) {
    station_count += size_t(std::max(count, 0));
  }
  if(reduction.attachment_points.size() != requests.size() + station_count ||
     cache.sliding_stations.size() != slides.size() ||
     cache.sliding_radii.size() != slides.size()) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "': its cache does not match its frames");
  }
  /* the frames tied as attachments, in order, then the sliding ones, each
  carried by its stations (at s along the axis from the frame) */
  size_t next = 0;
  for(size_t f = 0; f < frames.size(); f++) {
    if(frames[f].joint != FrameUse::Joint::kPrismatic) {
      SetNode(frames[f].frame->get_name(), reduction.attachment_points[next++]);
    }
  }
  for(size_t i = 0; i < slides.size(); i++) {
    const size_t count = size_t(cache.sliding_stations[i]);
    std::vector<unsigned int> stations(reduction.attachment_points.begin() +
                                         long(next),
                                       reduction.attachment_points.begin() +
                                         long(next + count));
    next += count;
    ksl_vec3_t axis = slides[i].axis;
    ksl_vec3_normalize(&axis);
    auto s_of = [&](unsigned int point) {
      ksl_vec3_t r;
      ksl_subtract_vv(&reduction.positions[point], &slides[i].position, &r);
      return ksl_dot_vv(&r, &axis);
    };
    const double first = s_of(stations[0]);
    const double spacing =
      count > 1 ? (s_of(stations[count - 1]) - first) / double(count - 1)
                : 0.0;
    std::string source;
    shift::core::body::SlidingBearing bearing =
      bearing_of(i, cache.sliding_radii[i], &source);
    bearing.radius = cache.sliding_radii[i];
    if(count > 1) {
      bearing.low = first - 0.5 * spacing;
      bearing.high = first + (double(count) - 0.5) * spacing;
    }
    SetSlidingNode(frames[sliding_frames[i]].frame->get_name(), stations,
                   first, spacing, frames[sliding_frames[i]].axis, bearing);
  }
  const size_t n = reduction.elastic_count;
  if(!initial_elastic_coordinates_.empty() ||
     !initial_elastic_velocities_.empty()) {
    std::vector<double> eta = initial_elastic_coordinates_;
    std::vector<double> eta_dot = initial_elastic_velocities_;
    if(eta.empty()) {
      eta.assign(n, 0.0);
    }
    if(eta_dot.empty()) {
      eta_dot.assign(n, 0.0);
    }
    SetInitialElasticState(eta, eta_dot);
  }
  frequencies_ = reduction.frequencies;
  kept_ = kept;
  static_count_ = reduction.static_count;
  mesh_boundary_ = cache.boundary;
  mesh_boundary_face_ = cache.boundary_face;
  mesh_node_count_ = cache.node_count;
  attachment_nodes_ = cache.attachment_nodes;
  attachment_info_.clear();
  for(size_t a = 0; a < requests.size(); a++) {
    const FrameUse& use = frames[request_frames[a]];
    AttachmentInfo info;
    info.frame = requests[a].name;
    info.user = use.user.empty() ? std::string("listed in attachments")
                                 : use.user;
    info.rbe2 =
      requests[a].spider == shift::core::fea::Attachment::Spider::kRbe2;
    info.reference = a == 0;
    attachment_info_.push_back(info);
  }
  for(size_t i = 0; i < slides.size(); i++) {
    const int count = cache.sliding_stations[i];
    for(int j = 0; j < count; j++) {
      AttachmentInfo info;
      info.frame = slides[i].name;
      info.user = frames[sliding_frames[i]].user;
      info.station = j;
      info.station_count = count;
      attachment_info_.push_back(info);
    }
  }
  /* the surface's stresses in the columns of the elastic coordinates
  (static modes, then the modes kept), then of the modes computed: full's
  static modes, then its modes */
  surface_nodes_ = cache.surface_nodes;
  surface_stresses_.clear();
  {
    const size_t s = cache.full.static_count;
    const size_t n_full = cache.full.elastic_count;
    std::vector<size_t> columns;
    for(size_t j = 0; j < s; j++) {
      columns.push_back(j);
    }
    for(unsigned int k : kept) {
      columns.push_back(s + k);
    }
    for(size_t k = 0; k < reduction.frequencies.size(); k++) {
      columns.push_back(s + k);
    }
    surface_stresses_.reserve(6 * surface_nodes_.size() * columns.size());
    for(size_t i = 0; i < surface_nodes_.size(); i++) {
      for(size_t column : columns) {
        const double* stress =
          cache.surface_stresses.data() + 6 * (i * n_full + column);
        surface_stresses_.insert(surface_stresses_.end(), stress, stress + 6);
      }
    }
  }
  from_cache_ = cached;
  computed_shapes_ = reduction.computed_shapes;
  ResolveNodes();

  std::stringstream ss;
  ss << "Flexible body '" << get_name() << "': " << cache.tet_count
     << " tetrahedra, " << cache.node_count << " nodes, volume "
     << cache.volume << ", mass " << cache.volume * density_ << "; "
     << reduction.static_count << " static and "
     << reduction.elastic_count - reduction.static_count
     << " fixed-interface modes of " << frequencies_.size() << " computed"
     << (cached ? " (from " + cache_path + ")" : std::string(";"))
     << ties.str();
  if(!frequencies_.empty()) {
    ss << " frequencies (Hz)";
    for(double f : frequencies_) {
      ss << " " << f;
    }
  }
  SHIFT_INFO(ss.str());
#endif
}

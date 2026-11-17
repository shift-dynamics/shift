/* body::Flex built from a finite element model of its geometry: the keys
common to Modal and Nodal, and BuildElastic */

#include <algorithm>
#include <cmath>
#include <map>
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
#include "shift/core/fea/cache.h"
#include "shift/core/fea/element.h"
#include "shift/core/fea/flex_model.h"
#include "shift/core/fea/reduction.h"
#include "shift/core/geometry/cad.h"
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
                   "(default: element_size)",
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
  DeclareParameter("fe_geometry", &fe_geometry_,
                   "name of the body's cad geometry whose solid is meshed "
                   "with 10-node tetrahedra; the mass properties then come "
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

  /* the solid */
  shift::core::geometry::Cad* cad = nullptr;
  for(auto& geometry : geometry_.get_data()) {
    if(geometry->get_name() == fe_geometry_) {
      cad = dynamic_cast<shift::core::geometry::Cad*>(geometry.get());
      if(!cad) {
        throw shift::core::Exception(
          "Flexible body '" + get_name() + "': fe_geometry '" + fe_geometry_ +
          "' is not a cad geometry");
      }
    }
  }
  if(!cad) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "' has no geometry '" + fe_geometry_ + "'");
  }
  /* what is being done, for the viewer's progress */
  auto report = [this](const char* stage) {
    context_->ReportLoad("Flexible body '" + get_name() + "': " + stage);
  };
  report("reading its solid");
  cad->Load();
  const shift::core::geometry::CadAsset* asset = cad->get_asset();
  shift::core::fea::PlacedSolid solid;
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
  std::vector<shift::core::fea::AttachmentRequest> requests;
  for(const FrameUse& use : frames) {
    const ksl_SE3_t* D = use.frame->get_d_wrt_origin();
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

  /* what the analysis depends on, but for the modes chosen */
  std::stringstream fingerprint;
  fingerprint.precision(17);
  {
    const std::string file = context_->FindFile(cad->get_filename());
    std::ifstream in(file, std::ios::binary);
    const std::string bytes((std::istreambuf_iterator<char>(in)),
                            std::istreambuf_iterator<char>());
    fingerprint << "step " << std::hex
                << shift::core::fea::Fnv1a(bytes.data(), bytes.size())
                << std::dec << " " << bytes.size() << "\n";
  }
  fingerprint << "body " << solid.placed.body << " scale " << solid.scale
              << " placement";
  for(double r : solid.placed.transform.rotation) {
    fingerprint << " " << r;
  }
  for(double t : solid.placed.transform.translation) {
    fingerprint << " " << t;
  }
  fingerprint << "\nmaterial " << youngs_modulus_ << " " << poisson_ratio_
              << " " << density_ << "\nmesh " << h << " " << h_min << "\n";
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
    shift::core::fea::TetMesh mesh =
      shift::core::fea::MeshPlacedSolid(solid, h, h_min);
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
        shift::core::fea::ChooseAttachment(mesh, solid, request, h, &faces));
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
    /* a node in two attachments: name the frames */
    std::map<int, size_t> owner;
    for(size_t a = 0; a < attachments.size(); a++) {
      for(int node : attachments[a].nodes) {
        auto [it, inserted] = owner.emplace(node, a);
        if(!inserted) {
          throw shift::core::Exception(
            "Flexible body '" + get_name() + "': the attachments of frames '" +
            requests[it->second].name + "' and '" + requests[a].name +
            "' share nodes (give one of them its faces or a smaller radius)");
        }
      }
    }
    const std::vector<int> retained_nodes =
      shift::core::fea::NearestNodes(mesh, retained);
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
  if(reduction.attachment_points.size() != frames.size()) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "': its cache does not match its frames");
  }
  for(size_t a = 0; a < frames.size(); a++) {
    SetNode(frames[a].frame->get_name(), reduction.attachment_points[a]);
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

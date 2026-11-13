#include "shift/core/force_module/soil_contact.h"

#include <algorithm>
#include <cmath>

#include "ksl/vec3.h"
#include "shift/core/body/base.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/force_module/soil.h"
#include "shift/core/terrain/scm.h"
#include "shift/core/terrain/sph.h"

bool shift::core::force_module::SoilContact::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "soil_contact", shift::core::force_module::SoilContact::Create);

std::unique_ptr<shift::core::force_module::SoilContact>
shift::core::force_module::SoilContact::Create() {
  return std::make_unique<shift::core::force_module::SoilContact>();
}

void shift::core::force_module::SoilContact::DeclareParameters() {
  shift::core::force_module::Base::DeclareParameters();
  DeclareParameter("bodies", &body_names_,
                   "the bodies whose collision geometry presses on the soil",
                   true);
  DeclareParameter("geometry", &geometry_names_,
                   "names of the bodies' collision geometry that presses on "
                   "the soil; defaults to all of it",
                   false);
  DeclareOutput("normal_force", &normal_force_,
                "total pressure force of the soil");
  DeclareOutput("contact_nodes", &contact_nodes_,
                "number of nodes of the terrain in contact");
  DeclareOutput("sinkage", &sinkage_,
                "deepest sinkage, from the undisturbed surface");
}

void shift::core::force_module::SoilContact::Load() {
  const std::string prefix = "Soil contact " + get_name() + ": ";
  bodies_.clear();
  geometry_.clear();
  geometry_body_.clear();
  for(const auto& name : body_names_) {
    auto body = context_->FindActiveBody(name, get_parent_assembly());
    if(!body) {
      throw shift::core::Exception(prefix + "body " + name +
                                   " not found among the active bodies");
    }
    bodies_.push_back(body);
    const int index = static_cast<int>(bodies_.size()) - 1;
    for(auto geometry : context_->get_active_collision_geometry()) {
      if(geometry->get_parent_body() == body &&
         (geometry_names_.empty() ||
          std::find(geometry_names_.begin(), geometry_names_.end(),
                    geometry->get_name()) != geometry_names_.end())) {
        geometry_.push_back(geometry);
        geometry_body_.push_back(index);
      }
    }
    if(geometry_body_.empty() || geometry_body_.back() != index) {
      throw shift::core::Exception(
        prefix + "body " + name + " has no collision geometry" +
        (geometry_names_.empty() ? "" : " of the names in geometry"));
    }
  }
  load_indices_.assign(bodies_.size(), -2);
  /* the SPH soils take the bodies as markers, of the same geometry */
  std::vector<std::vector<shift::core::collision_geometry::Base*>> geometry(
    bodies_.size());
  for(size_t g = 0; g < geometry_.size(); g++) {
    geometry[geometry_body_[g]].push_back(geometry_[g]);
  }
  for(shift::core::terrain::Sph* sph : sph_) {
    sph->SetBodies(bodies_, geometry);
  }
}

void shift::core::force_module::SoilContact::Connect() {
  terrain_ = nullptr;
  sph_.clear();
  /* the SPH soils, among the terrains and their regions */
  std::vector<shift::core::terrain::Base*> stack;
  for(auto terrain : context_->get_active_terrain()) {
    if(auto scm = dynamic_cast<shift::core::terrain::Scm*>(terrain)) {
      terrain_ = scm;
    }
    stack.push_back(terrain);
  }
  while(!stack.empty()) {
    shift::core::terrain::Base* terrain = stack.back();
    stack.pop_back();
    if(auto sph = dynamic_cast<shift::core::terrain::Sph*>(terrain)) {
      sph_.push_back(sph);
    }
    for(auto region : terrain->get_regions()) {
      stack.push_back(region);
    }
  }
  if(!terrain_ && sph_.empty()) {
    throw shift::core::Exception("Soil contact " + get_name() +
                                 ": needs a terrain of type \"scm\" or "
                                 "\"sph\"");
  }
}

void shift::core::force_module::SoilContact::Evaluate(bool apply) {
  normal_force_ = contact_nodes_ = sinkage_ = 0.0;
  if(!terrain_) {
    return;
  }
  shift::core::terrain::Scm& scm = *terrain_;
  if(apply) {
    scm.ClearTrials();
  }
  for(size_t k = 0; k < bodies_.size(); k++) {
    if(load_indices_[k] == -2) {
      load_indices_[k] = context_->GetBodyLoadIndex(bodies_[k]);
    }
  }
  const ksl_SE3_t& frame = scm.get_pose();
  const ksl_vec3_t& e_x = frame.R.v0;
  const ksl_vec3_t& e_y = frame.R.v1;
  const ksl_vec3_t& e_z = frame.R.v2;
  const ksl_vec3_t up = {{-e_z.x, -e_z.y, -e_z.z}};
  const double h = scm.get_spacing();
  const double reach = scm.get_max_sinkage();
  const long nx = scm.get_node_count_x(), ny = scm.get_node_count_y();
  /* the range of nodes under each geometry, from its support points along
  the axes of the terrain frame: its lowest point (z down) and its extent in
  x and y; and the box of nodes under them all */
  std::vector<Range>& ranges = ranges_;
  ranges.assign(geometry_.size(), Range{0, -1, 0, -1, 0.0});
  long box_i0 = nx, box_i1 = -1, box_j0 = ny, box_j1 = -1;
  for(size_t g = 0; g < geometry_.size(); g++) {
    auto geometry = geometry_[g];
    geometry->UpdatePose(context_->get_body_origin_pose(
      bodies_[geometry_body_[g]], context_->get_dhva()));
    double low[3], high[3];
    for(int axis = 0; axis < 3; axis++) {
      const ksl_vec3_t& e = frame.R.v[axis];
      for(double sign : {-1.0, 1.0}) {
        const ksl_vec3_t direction = {{sign * e.x, sign * e.y, sign * e.z}};
        ksl_vec3_t local, global;
        int hint = 0;
        geometry->Support(direction, local, global, hint);
        const ksl_vec3_t r = {{global.x - frame.t.x, global.y - frame.t.y,
                               global.z - frame.t.z}};
        (sign < 0.0 ? low : high)[axis] = ksl_dot_vv(&e, &r);
      }
    }
    if(!std::isfinite(high[2]) || !std::isfinite(low[0]) ||
       !std::isfinite(high[0]) || !std::isfinite(low[1]) ||
       !std::isfinite(high[1])) {
      continue; // unbounded, such as a plane
    }
    Range& range = ranges[g];
    range.bottom = high[2];
    range.i0 =
      std::max(0L, static_cast<long>(std::ceil((low[0] - scm.NodeX(0)) / h)));
    range.i1 = std::min(
      nx - 1, static_cast<long>(std::floor((high[0] - scm.NodeX(0)) / h)));
    range.j0 =
      std::max(0L, static_cast<long>(std::ceil((low[1] - scm.NodeY(0)) / h)));
    range.j1 = std::min(
      ny - 1, static_cast<long>(std::floor((high[1] - scm.NodeY(0)) / h)));
    if(range.i0 <= range.i1 && range.j0 <= range.j1) {
      box_i0 = std::min(box_i0, range.i0);
      box_i1 = std::max(box_i1, range.i1);
      box_j0 = std::min(box_j0, range.j0);
      box_j1 = std::max(box_j1, range.j1);
    }
  }
  if(box_i1 < box_i0 || box_j1 < box_j0) {
    return;
  }

  /* the lowest body over each node of the box, below the surface: cells
  index the nodes of the box, x fastest, to their hit or -1 */
  const long box_nx = box_i1 - box_i0 + 1, box_ny = box_j1 - box_j0 + 1;
  std::vector<int>& cells = cells_;
  cells.assign(static_cast<size_t>(box_nx * box_ny), -1);
  std::vector<Hit>& hits = hits_;
  hits.clear();
  auto cell_of = [&](long i, long j) -> int* {
    if(i < box_i0 || i > box_i1 || j < box_j0 || j > box_j1) {
      return nullptr;
    }
    return &cells[static_cast<size_t>((j - box_j0) * box_nx + (i - box_i0))];
  };
  std::vector<double>& offsets = offsets_;
  std::vector<double>& distances = distances_;
  for(size_t g = 0; g < geometry_.size(); g++) {
    auto geometry = geometry_[g];
    const Range& range = ranges[g];
    if(range.i0 > range.i1 || range.j0 > range.j1) {
      continue;
    }
    /* the rays of the nodes under the geometry, up from max_sinkage below
    the surface to it; none where the geometry is above the soil */
    shift::core::collision_geometry::Base::RayGrid grid;
    grid.count_i = range.i1 - range.i0 + 1;
    grid.count_j = range.j1 - range.j0 + 1;
    const double x0 = scm.NodeX(range.i0), y0 = scm.NodeY(range.j0);
    for(int c = 0; c < 3; c++) {
      grid.corner.at[c] = frame.t.at[c] + e_x.at[c] * x0 + e_y.at[c] * y0;
      grid.step_i.at[c] = e_x.at[c] * h;
      grid.step_j.at[c] = e_y.at[c] * h;
    }
    grid.direction = up;
    grid.reach = reach;
    offsets.assign(static_cast<size_t>(grid.count_i * grid.count_j), NAN);
    distances.resize(offsets.size());
    bool any = false;
    /* the nodes under a region of the terrain are the region's (e.g. an SPH
    soil, Sph), not the SCM terrain's */
    const bool regions = !scm.get_regions().empty();
    for(long j = range.j0; j <= range.j1; j++) {
      for(long i = range.i0; i <= range.i1; i++) {
        if(regions && scm.RegionAt(scm.NodeX(i), scm.NodeY(j))) {
          continue;
        }
        const double surface = scm.NodeZ(i, j);
        if(range.bottom > surface) {
          offsets[(i - range.i0) + (j - range.j0) * grid.count_i] =
            surface + reach;
          any = true;
        }
      }
    }
    if(!any) {
      continue;
    }
    grid.offsets = offsets.data();
    geometry->RayCastGrid(grid, distances.data());
    for(long j = range.j0; j <= range.j1; j++) {
      for(long i = range.i0; i <= range.i1; i++) {
        const long k = (i - range.i0) + (j - range.j0) * grid.count_i;
        const double t = distances[k];
        if(t == HUGE_VAL) {
          continue;
        }
        const double start = offsets[k];
        const double z = start - t;
        int& cell = *cell_of(i, j);
        if(cell < 0) {
          cell = static_cast<int>(hits.size());
          hits.push_back(Hit{i, j, -HUGE_VAL, -1, {{0.0, 0.0, 0.0}}, 0.0});
        }
        Hit& hit = hits[cell];
        if(z > hit.z) {
          hit.z = z;
          hit.body = geometry_body_[g];
          const double x = scm.NodeX(i), y = scm.NodeY(j);
          for(int c = 0; c < 3; c++) {
            hit.point.at[c] =
              frame.t.at[c] + e_x.at[c] * x + e_y.at[c] * y + e_z.at[c] * z;
          }
        }
      }
    }
  }
  if(hits.empty()) {
    return;
  }

  /* the patches of nodes in contact, joined along the grid, and their width
  b = 2 A / P */
  std::vector<char> seen(hits.size(), 0);
  std::vector<int> stack, patch;
  for(size_t start = 0; start < hits.size(); start++) {
    if(seen[start]) {
      continue;
    }
    patch.clear();
    stack.assign(1, static_cast<int>(start));
    seen[start] = 1;
    long edges = 0;
    while(!stack.empty()) {
      const int k = stack.back();
      stack.pop_back();
      patch.push_back(k);
      const long i = hits[k].i, j = hits[k].j;
      const long neighbours[4][2] = {
        {i - 1, j}, {i + 1, j}, {i, j - 1}, {i, j + 1}};
      for(const auto& n : neighbours) {
        const int* cell = cell_of(n[0], n[1]);
        if(!cell || *cell < 0) {
          edges++;
        } else if(!seen[*cell]) {
          seen[*cell] = 1;
          stack.push_back(*cell);
        }
      }
    }
    const double b = 2.0 * double(patch.size()) * h / double(edges);
    for(int k : patch) {
      hits[k].width = b;
    }
  }

  /* the forces */
  const double t = context_->get_force_time();
  const double t_a = scm.get_accepted_time();
  const double dt = std::isnan(t_a) ? 0.0 : t - t_a;
  const double area = h * h;
  std::vector<ksl_coscrew_t> loads(bodies_.size(),
                                   ksl_coscrew(0, 0, 0, 0, 0, 0));
  std::vector<ksl_screw_t> velocities(bodies_.size());
  for(size_t k = 0; k < bodies_.size(); k++) {
    velocities[k] = context_->GetBodyVelocity(load_indices_[k]);
  }
  for(const Hit& hit : hits) {
    const shift::core::force_module::SoilProperties* soil =
      scm.NodeSoil(hit.i, hit.j);
    const shift::core::terrain::Scm::Node* accepted =
      scm.FindNode(hit.i, hit.j);
    const double s = hit.z - scm.UndisturbedZ(hit.i, hit.j);
    const double plastic = accepted ? accepted->plastic_sinkage : 0.0;
    const double b = hit.width;
    const double elastic = scm.Pressure(*soil, b, s, plastic);
    if(!(elastic > 0.0)) {
      /* below the surface without pressure: in a pile, or in soil that
      has slid into a rut. With bulldozing, the body cuts the soil down to
      its own depth, and the soil is pushed on (Scm::Bulldoze) */
      const double level = accepted ? accepted->level : 0.0;
      if(apply && scm.get_bulldozing().is_enabled() && s > level) {
        shift::core::terrain::Scm::Node& node = scm.Trial(hit.i, hit.j);
        node.in_contact = true;
        node.level = std::max(node.level, s);
        node.shear[0] = node.shear[1] = 0.0;
        node.slip[0] = node.slip[1] = 0.0;
      }
      continue;
    }
    /* the velocity of the body's point: v + w x p, the spatial velocity
    being referred to the fixed origin */
    const ksl_screw_t& velocity = velocities[hit.body];
    ksl_vec3_t v;
    ksl_cross_vv(&velocity.ang, &hit.point, &v);
    for(int c = 0; c < 3; c++) {
      v.at[c] += velocity.lin.at[c];
    }
    const double rate = ksl_dot_vv(&v, &e_z);
    const double p = std::max(0.0, elastic * (1.0 + scm.DampingAt(t) * rate));
    /* the shear displacement, from the accepted one by the trapezoidal rule
    on the slip velocity while the body stays on the node; from zero where it
    comes onto the node */
    const double slip[2] = {ksl_dot_vv(&v, &e_x), ksl_dot_vv(&v, &e_y)};
    double shear[2] = {slip[0] * dt, slip[1] * dt};
    if(accepted && accepted->in_contact) {
      for(int c = 0; c < 2; c++) {
        shear[c] = accepted->shear[c] + 0.5 * (accepted->slip[c] + slip[c]) * dt;
      }
    }
    const double displacement = std::hypot(shear[0], shear[1]);
    double f[3] = {0.0, 0.0, -p * area}; // terrain frame, pressure up (-z)
    if(displacement > 0.0) {
      const double tau =
        (soil->cohesion + p * std::tan(soil->friction_angle)) *
        (1.0 - std::exp(-displacement / soil->shear_modulus));
      f[0] = -tau * area * shear[0] / displacement;
      f[1] = -tau * area * shear[1] / displacement;
    }
    ksl_vec3_t force;
    for(int c = 0; c < 3; c++) {
      force.at[c] = e_x.at[c] * f[0] + e_y.at[c] * f[1] + e_z.at[c] * f[2];
    }
    ksl_vec3_t moment;
    ksl_cross_vv(&hit.point, &force, &moment);
    for(int c = 0; c < 3; c++) {
      loads[hit.body].lin.at[c] += force.at[c];
      loads[hit.body].ang.at[c] += moment.at[c];
    }
    normal_force_ += p * area;
    contact_nodes_ += 1.0;
    sinkage_ = std::max(sinkage_, s);
    if(apply) {
      shift::core::terrain::Scm::Node& node = scm.Trial(hit.i, hit.j);
      node.in_contact = true;
      node.plastic_sinkage = std::max(plastic, s);
      node.level = std::max(node.level, scm.ReboundLevel(node.plastic_sinkage));
      node.shear[0] = shear[0];
      node.shear[1] = shear[1];
      node.slip[0] = slip[0];
      node.slip[1] = slip[1];
    }
  }
  if(apply) {
    for(size_t k = 0; k < bodies_.size(); k++) {
      context_->AddBodyLoad(load_indices_[k], loads[k]);
    }
  }
}

void shift::core::force_module::SoilContact::Service(const double*, double*) {
  Evaluate(true);
  /* the SPH soils' loads, of the last accepted step */
  for(size_t k = 0; k < bodies_.size(); k++) {
    if(load_indices_[k] == -2) {
      load_indices_[k] = context_->GetBodyLoadIndex(bodies_[k]);
    }
  }
  for(shift::core::terrain::Sph* sph : sph_) {
    for(size_t k = 0; k < bodies_.size(); k++) {
      context_->AddBodyLoad(load_indices_[k], sph->get_body_load(k));
    }
  }
}

void shift::core::force_module::SoilContact::UpdateOutputs(double,
                                                           const double*) {
  Evaluate(false);
}

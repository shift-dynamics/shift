#include "shift/core/terrain/scm.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/force_module/soil.h"

bool shift::core::terrain::Scm::registered_ =
  shift::core::Factory<shift::core::terrain::Base>::get()->RegisterModule(
    "scm", shift::core::terrain::Scm::Create);

std::unique_ptr<shift::core::terrain::Scm> shift::core::terrain::Scm::Create() {
  return std::make_unique<shift::core::terrain::Scm>();
}

void shift::core::terrain::Scm::DeclareParameters() {
  shift::core::terrain::Basic::DeclareParameters();
  DeclareParameter("spacing", &spacing_,
                   "distance between the nodes of the grid along x and y (m)",
                   true);
  DeclareParameter("unloading_ratio", &unloading_ratio_,
                   "stiffness of the soil springing back below its deepest "
                   "sinkage, over the secant of Bekker's law there; the "
                   "soil springs back by 1 / unloading_ratio of a rut's "
                   "depth; more than 1, default 10",
                   false);
  DeclareParameter("damping", &damping_,
                   "Hunt and Crossley's factor of the pressure, per speed of "
                   "sinkage (s/m), so that the force has no jump as contact "
                   "starts; default 0",
                   false);
  DeclareParameter("settle_time", &settle_time_,
                   "the time from the start over which bodies set on the "
                   "soil are settled onto it (s): settle_damping is added "
                   "to the damping, fading smoothly to nothing over it, so "
                   "that they sink to the depth of Bekker's law under their "
                   "weight instead of falling in and sinking about twice as "
                   "deep; default 0, none",
                   false);
  DeclareParameter("settle_damping", &settle_damping_,
                   "the damping added at the start of the settling, s/m; "
                   "less lets the bodies fall in, much more holds them up "
                   "so that they are still sinking as the settling ends; "
                   "with the default, 20, a settle_time of 2 s or more",
                   false);
  DeclareParameter("max_sinkage", &max_sinkage_,
                   "how far below the surface bodies are looked for (m); "
                   "deeper, a body is taken to be at this depth; default 0.5",
                   false);
  DeclareParameter("bulldozing", &bulldozing_,
                   "push the soil bodies press down to the rims of their "
                   "ruts, and erode it to the angle of repose (see "
                   "terrain::Bulldozing); optional",
                   false);
}

void shift::core::terrain::Bulldozing::DeclareParameters() {
  DeclareParameter("enable", &enable_,
                   "whether the soil is bulldozed; default false", false);
  DeclareParameter("flow_factor", &flow_factor_,
                   "the volume pushed to the rim per volume pressed down: 1 "
                   "conserves the soil, more than 1 loosens it; default 1",
                   false);
  DeclareParameter("erosion_angle", &erosion_angle_,
                   "the angle of repose: steeper slopes are eroded down to "
                   "it; defaults to the friction angle of each node's soil",
                   false);
  DeclareParameter("erosion_iterations", &erosion_iterations_,
                   "passes of erosion at each bulldozing; soil moves one "
                   "node per pass, so enough for the slopes to settle "
                   "within an interval; 0 for none; default 30",
                   false);
  DeclareParameter("closing_distance", &closing_distance_,
                   "gaps between the nodes in contact up to this wide (m), "
                   "such as the cracks between the shoes of a track, are "
                   "part of the patch: they take no soil, which goes to the "
                   "rim around the whole patch; 0 for none; default 0.05 m, "
                   "wide enough for the gap that opens between the shoes of "
                   "a track as they turn up around a sprocket or idler",
                   false);
  DeclareParameter("interval", &interval_,
                   "the time between bulldozings (s), at the first accepted "
                   "step at or after each multiple of it; 0 for after every "
                   "accepted step; default 0.001 s",
                   false);
  DeclareParameter("compaction_depth", &compaction_depth_,
                   "the depth of the layer under a body that compacts (m), "
                   "by up to 1 - unit_weight / max_unit_weight of its soil, "
                   "before soil is pushed aside; no compaction for a soil "
                   "without max_unit_weight; default the width of the "
                   "patch, 2 area / perimeter",
                   false);
}

void shift::core::terrain::Bulldozing::Check(const std::string& owner) const {
  if(!(flow_factor_ >= 0.0) || erosion_iterations_ < 0 ||
     closing_distance_ < 0.0 || interval_ < 0.0 || compaction_depth_ < 0.0 ||
     (!std::isnan(erosion_angle_) &&
      !(erosion_angle_ > 0.0 && erosion_angle_ < 0.5 * M_PI))) {
    throw shift::core::Exception(owner + ": bulldozing: flow_factor must not "
                                         "be negative, erosion_iterations "
                                         "closing_distance, interval and "
                                         "compaction_depth not negative, "
                                         "and erosion_angle "
                                         "between 0 and pi / 2");
  }
}

void shift::core::terrain::Scm::Load() {
  shift::core::terrain::Basic::Load();
  const std::string prefix = "Terrain " + get_name() + ": ";
  if(x_range_.size() != 2 || y_range_.size() != 2 ||
     !(x_range_[1] > x_range_[0]) || !(y_range_[1] > y_range_[0])) {
    throw shift::core::Exception(prefix + "x and y must each be [first, "
                                          "last] with last > first");
  }
  if(!(spacing_ > 0.0)) {
    throw shift::core::Exception(prefix + "spacing must be positive");
  }
  /* the nodes inside each range, with a tolerance for a range that is a
  whole number of spacings but for rounding */
  const double nx = (x_range_[1] - x_range_[0]) / spacing_;
  const double ny = (y_range_[1] - y_range_[0]) / spacing_;
  if(nx * ny > 1e12) {
    throw shift::core::Exception(prefix + "the grid has too many nodes; "
                                          "increase the spacing");
  }
  node_count_x_ = static_cast<long>(std::floor(nx + 1e-9)) + 1;
  node_count_y_ = static_cast<long>(std::floor(ny + 1e-9)) + 1;
  /* the nodes are numbered by 32-bit integers in the output */
  if(double(node_count_x_) * double(node_count_y_) > double(INT32_MAX)) {
    throw shift::core::Exception(prefix + "the grid has more than 2^31 - 1 "
                                          "nodes; increase the spacing");
  }
  if(get_soils().empty()) {
    throw shift::core::Exception(prefix + "needs a soil or a soil_map");
  }
  if(!(unloading_ratio_ > 1.0) || damping_ < 0.0 || !(max_sinkage_ > 0.0) ||
     !(settle_time_ >= 0.0) || !(settle_damping_ >= 0.0)) {
    throw shift::core::Exception(prefix + "unloading_ratio must be more than "
                                          "1, damping, settle_time and "
                                          "settle_damping not negative and "
                                          "max_sinkage positive");
  }
  bulldozing_.Check("Terrain " + get_name());
  /* the undisturbed surface and the soil of each node, which never change,
  computed once: the soil models ask for them at every node under a body in
  every evaluation. Not for a grid so large that they would take more than
  about 200 MB; there they are computed when asked for */
  undisturbed_.clear();
  node_soils_.clear();
  const size_t count = size_t(node_count_x_) * size_t(node_count_y_);
  if(count <= 20000000 && get_soils().size() < 32767) {
    undisturbed_.resize(count);
    node_soils_.resize(count);
    for(long j = 0; j < node_count_y_; j++) {
      for(long i = 0; i < node_count_x_; i++) {
        double z_x, z_y;
        const size_t k = size_t(j) * node_count_x_ + i;
        undisturbed_[k] = LocalZ(NodeX(i), NodeY(j), z_x, z_y);
        node_soils_[k] =
          static_cast<int16_t>(LocalSoilIndex(NodeX(i), NodeY(j)));
      }
    }
  }
  tile_count_x_ = (node_count_x_ + kTileWidth - 1) >> kTileBits;
  const long tile_count_y = (node_count_y_ + kTileWidth - 1) >> kTileBits;
  Clear();
  tiles_.resize(size_t(tile_count_x_) * size_t(tile_count_y));
  revision_ = 0;
}

void shift::core::terrain::Scm::Clear() {
  for(std::unique_ptr<Tile>& tile : tiles_) {
    tile.reset();
  }
  made_tiles_.clear();
  trial_keys_.clear();
  pressed_keys_.clear();
  pressed_heights_.clear();
  pressed_sinkages_.clear();
  trial_generation_ = pressed_generation_ = scratch_generation_ = 1;
  next_bulldozing_ = std::nan("");
  contacts_.clear();
  dirty_.clear();
  unsettled_.clear();
  output_changes_.clear();
  accepted_time_ = std::nan("");
}

shift::core::terrain::Scm::Tile& shift::core::terrain::Scm::GetTile(long i,
                                                                    long j) {
  const size_t index = TileIndex(i, j);
  if(!tiles_[index]) {
    tiles_[index] = std::make_unique<Tile>();
    made_tiles_.push_back(index);
  }
  return *tiles_[index];
}

void shift::core::terrain::Scm::NextGeneration(
  uint32_t& generation, uint32_t (Tile::*stamps)[kTileSize]) {
  if(++generation == 0) {
    /* after 2^32 generations: clear the stamps, so none is taken for one
    of a new generation */
    for(size_t index : made_tiles_) {
      std::fill(std::begin((*tiles_[index]).*stamps),
                std::end((*tiles_[index]).*stamps), 0u);
    }
    generation = 1;
  }
}

void shift::core::terrain::Scm::MarkDirty(long long key) {
  const long i = KeyI(key), j = KeyJ(key);
  uint8_t& flags = GetTile(i, j).flags[InTile(i, j)];
  if(!(flags & kDirty)) {
    flags |= kDirty;
    dirty_.push_back(key);
  }
}

void shift::core::terrain::Scm::AddPressed(long long key, double height,
                                           double sinkage) {
  const long i = KeyI(key), j = KeyJ(key);
  Tile& tile = GetTile(i, j);
  const size_t k = InTile(i, j);
  if(tile.pressed_stamp[k] == pressed_generation_) {
    pressed_heights_[size_t(tile.pressed_slot[k])] += height;
    pressed_sinkages_[size_t(tile.pressed_slot[k])] += sinkage;
    return;
  }
  tile.pressed_stamp[k] = pressed_generation_;
  tile.pressed_slot[k] = static_cast<int32_t>(pressed_keys_.size());
  pressed_keys_.push_back(key);
  pressed_heights_.push_back(height);
  pressed_sinkages_.push_back(sinkage);
}

bool shift::core::terrain::Scm::BulldozeDue() {
  const double interval = std::isnan(bulldozing_.get_interval())
                            ? 1e-3
                            : bulldozing_.get_interval();
  if(!(interval > 0.0) || std::isnan(accepted_time_)) {
    return true;
  }
  if(accepted_time_ < next_bulldozing_) {
    return false; // false too for a NaN next_bulldozing_: due now
  }
  next_bulldozing_ = (std::floor(accepted_time_ / interval) + 1.0) * interval;
  return true;
}

bool shift::core::terrain::Scm::NearestNode(double x, double y, long& i,
                                            long& j) const {
  const double u = std::round((x - x_range_[0]) / spacing_);
  const double v = std::round((y - y_range_[0]) / spacing_);
  if(!(u >= 0.0 && v >= 0.0 && u < node_count_x_ && v < node_count_y_)) {
    return false;
  }
  i = static_cast<long>(u);
  j = static_cast<long>(v);
  return true;
}

double shift::core::terrain::Scm::UndisturbedZ(long i, long j) const {
  if(!undisturbed_.empty() && is_node(i, j)) {
    return undisturbed_[size_t(j) * node_count_x_ + i];
  }
  double z_x, z_y;
  return LocalZ(NodeX(i), NodeY(j), z_x, z_y);
}

double shift::core::terrain::Scm::NodeZ(long i, long j) const {
  return UndisturbedZ(i, j) + Level(i, j);
}

const shift::core::force_module::SoilProperties*
shift::core::terrain::Scm::NodeSoil(long i, long j) const {
  if(!node_soils_.empty() && is_node(i, j)) {
    const int index = node_soils_[size_t(j) * node_count_x_ + i];
    return index >= 0 ? &get_soils()[index] : nullptr;
  }
  return LocalSoil(NodeX(i), NodeY(j));
}

const shift::core::terrain::Scm::Node*
shift::core::terrain::Scm::FindNode(long i, long j) const {
  if(!is_node(i, j)) {
    return nullptr;
  }
  const Tile* tile = FindTile(i, j);
  if(!tile) {
    return nullptr;
  }
  const Node& node = tile->nodes[InTile(i, j)];
  return node.is_undisturbed() ? nullptr : &node;
}

shift::core::terrain::Scm::Node& shift::core::terrain::Scm::Trial(long i,
                                                                  long j) {
  if(!is_node(i, j)) {
    throw shift::core::Exception("Terrain " + get_name() + ": (" +
                                 std::to_string(i) + ", " + std::to_string(j) +
                                 ") is not a node of the grid");
  }
  Tile& tile = GetTile(i, j);
  const size_t k = InTile(i, j);
  if(tile.trial_stamp[k] == trial_generation_) {
    return trial_nodes_[size_t(tile.trial_slot[k])];
  }
  const size_t slot = trial_keys_.size();
  if(slot == trial_nodes_.size()) {
    trial_nodes_.push_back(tile.nodes[k]);
  } else {
    trial_nodes_[slot] = tile.nodes[k];
  }
  trial_keys_.push_back(Key(i, j));
  tile.trial_stamp[k] = trial_generation_;
  tile.trial_slot[k] = static_cast<int32_t>(slot);
  return trial_nodes_[slot];
}

const shift::core::terrain::Scm::Node*
shift::core::terrain::Scm::FindTrial(long i, long j) const {
  if(!is_node(i, j)) {
    return nullptr;
  }
  const Tile* tile = FindTile(i, j);
  const size_t k = InTile(i, j);
  if(!tile || tile->trial_stamp[k] != trial_generation_) {
    return nullptr;
  }
  return &trial_nodes_[size_t(tile->trial_slot[k])];
}

void shift::core::terrain::Scm::ClearTrials() {
  trial_keys_.clear();
  NextGeneration(trial_generation_, &Tile::trial_stamp);
}

size_t shift::core::terrain::Scm::get_changed_node_count() const {
  size_t count = 0;
  for(size_t index : made_tiles_) {
    for(const Node& node : tiles_[index]->nodes) {
      count += node.is_undisturbed() ? 0 : 1;
    }
  }
  return count;
}

void shift::core::terrain::Scm::AcceptStep() {
  /* the state accepted is that of the last evaluation of the forces */
  accepted_time_ = context_->get_force_time();
  const bool bulldozing = bulldozing_.is_enabled();
  if(trial_keys_.empty() && contacts_.empty()) {
    if(bulldozing && !(pressed_keys_.empty() && unsettled_.empty()) &&
       BulldozeDue() && Bulldoze()) {
      revision_++;
    }
    return;
  }
  /* the nodes in contact, and how far each was pressed down in the step */
  next_contacts_.clear();
  for(size_t slot = 0; slot < trial_keys_.size(); slot++) {
    const long long key = trial_keys_[slot];
    const Node& trial = trial_nodes_[slot];
    Node& accepted = Accepted(key);
    if(trial.level != accepted.level) {
      MarkDirty(key);
    }
    if(trial.in_contact) {
      if(bulldozing) {
        /* the part below the undisturbed surface (level 0) */
        AddPressed(key, std::max(0.0, trial.level - accepted.level),
                   std::max(0.0, trial.level - std::max(accepted.level, 0.0)));
      }
      next_contacts_.push_back(key);
    }
    accepted = trial;
  }
  /* the nodes bodies have left: their shear displacement goes */
  for(long long key : contacts_) {
    const Node* trial = FindTrial(KeyI(key), KeyJ(key));
    if(trial && trial->in_contact) {
      continue;
    }
    Node& node = Accepted(key);
    node.in_contact = false;
    node.shear[0] = node.shear[1] = 0.0;
    node.slip[0] = node.slip[1] = 0.0;
  }
  std::sort(next_contacts_.begin(), next_contacts_.end());
  bool changed = !trial_keys_.empty() || next_contacts_ != contacts_;
  contacts_.swap(next_contacts_);
  ClearTrials();
  if(bulldozing && BulldozeDue() && Bulldoze()) {
    changed = true;
  }
  if(changed) {
    revision_++;
  }
}

bool shift::core::terrain::Scm::Deposit(double x, double y, double volume) {
  long i, j;
  if(!NearestNode(x, y, i, j)) {
    return false;
  }
  const long long key = Key(i, j);
  /* z down: a raised node has a negative level */
  Accepted(key).level -= volume / (spacing_ * spacing_);
  MarkDirty(key);
  if(bulldozing_.is_enabled()) {
    uint8_t& flags = GetTile(i, j).flags[InTile(i, j)];
    if(!(flags & kUnsettled)) {
      flags |= kUnsettled;
      unsettled_.push_back(key);
    }
  }
  revision_++;
  return true;
}

void shift::core::terrain::Scm::UpdateOutput() {
  output_changes_.clear();
  output_changes_.reserve(dirty_.size());
  std::sort(dirty_.begin(), dirty_.end());
  for(long long key : dirty_) {
    const long i = KeyI(key), j = KeyJ(key);
    Tile& tile = GetTile(i, j);
    tile.flags[InTile(i, j)] &= static_cast<uint8_t>(~kDirty);
    output_changes_.push_back(
      {static_cast<int32_t>(key),
       static_cast<float>(tile.nodes[InTile(i, j)].level)});
  }
  dirty_.clear();
}

void shift::core::terrain::Scm::GetLevels(std::vector<float>& levels) const {
  levels.assign(static_cast<size_t>(node_count_x_ * node_count_y_), 0.0f);
  for(size_t index : made_tiles_) {
    const long i0 = long(index % size_t(tile_count_x_)) * kTileWidth;
    const long j0 = long(index / size_t(tile_count_x_)) * kTileWidth;
    for(long j = j0; j < std::min(j0 + kTileWidth, node_count_y_); j++) {
      for(long i = i0; i < std::min(i0 + kTileWidth, node_count_x_); i++) {
        levels[static_cast<size_t>(Key(i, j))] =
          static_cast<float>(tiles_[index]->nodes[InTile(i, j)].level);
      }
    }
  }
}

void shift::core::terrain::Scm::SetLevels(const std::vector<float>& levels) {
  if(levels.size() != static_cast<size_t>(node_count_x_ * node_count_y_)) {
    throw shift::core::Exception("Terrain " + get_name() +
                                 ": SetLevels needs a level for each of the " +
                                 std::to_string(node_count_x_ * node_count_y_) +
                                 " nodes");
  }
  Clear();
  for(size_t k = 0; k < levels.size(); k++) {
    if(levels[k] == 0.0f) {
      continue;
    }
    Node& node = Accepted(static_cast<long long>(k));
    node.level = levels[k];
    if(node.level > 0.0) {
      node.plastic_sinkage = node.level / (1.0 - 1.0 / unloading_ratio_);
      if(bulldozing_.is_enabled()) {
        node.compaction = node.level;
      }
    }
  }
  revision_++;
}

double shift::core::terrain::Scm::Pressure(
  const shift::core::force_module::SoilProperties& soil, double b, double s,
  double plastic_sinkage) const {
  if(!(s > 0.0)) {
    return 0.0;
  }
  const double load = soil.Pressure(b, s);
  if(!(plastic_sinkage > 0.0) || s >= plastic_sinkage) {
    return load;
  }
  /* below the deepest sinkage: the elastic line back from it */
  const double reached = soil.Pressure(b, plastic_sinkage);
  const double k_u = unloading_ratio_ * reached / plastic_sinkage;
  return std::max(0.0, std::min(load, reached - k_u * (plastic_sinkage - s)));
}

double shift::core::terrain::Scm::Level(long i, long j) const {
  const Node* node = FindNode(i, j);
  return node ? node->level : 0.0;
}

double shift::core::terrain::Scm::SurfaceZ(double x, double y, double& z_x,
                                           double& z_y) const {
  double z = LocalZ(x, y, z_x, z_y);
  if(made_tiles_.empty()) {
    return z;
  }
  /* the cell of the grid, including those one spacing beyond each edge,
  whose outer corners have a level of 0 */
  const double u = (x - x_range_[0]) / spacing_;
  const double v = (y - y_range_[0]) / spacing_;
  if(!(u >= -1.0 && v >= -1.0 && u <= double(node_count_x_) &&
       v <= double(node_count_y_))) {
    return z;
  }
  const long i = std::min(static_cast<long>(std::floor(u)), node_count_x_ - 1);
  const long j = std::min(static_cast<long>(std::floor(v)), node_count_y_ - 1);
  const double s = u - i, t = v - j;
  const double l00 = Level(i, j), l10 = Level(i + 1, j);
  const double l01 = Level(i, j + 1), l11 = Level(i + 1, j + 1);
  z += (l00 * (1.0 - s) + l10 * s) * (1.0 - t) + (l01 * (1.0 - s) + l11 * s) * t;
  z_x += ((l10 - l00) * (1.0 - t) + (l11 - l01) * t) / spacing_;
  z_y += ((l01 - l00) * (1.0 - s) + (l11 - l10) * s) / spacing_;
  return z;
}

bool shift::core::terrain::Scm::Bulldoze() {
  if(pressed_keys_.empty() && unsettled_.empty()) {
    return false;
  }
  bool changed = false;
  const long nx = node_count_x_, ny = node_count_y_;
  /* the nodes changed, the patches and their rims, where the erosion
  starts */
  std::vector<long long>& active = bulldoze_active_;
  active.clear();
  auto raise = [&](long long key, double height) {
    changed = true;
    Accepted(key).level -= height;
    MarkDirty(key);
  };

  /* each patch: the nodes in contact joined along x and y, with the gaps
  between them up to closing_distance wide closed (the cracks between the
  shoes of a track, between grousers); the soil it pressed down that did not
  compact goes evenly to its rim, the nodes around it (also diagonally)
  outside it. The nodes in the gaps take none: they are under the body,
  between its parts */
  const double flow_factor = bulldozing_.get_flow_factor();
  const double distance = std::isnan(bulldozing_.get_closing_distance())
                            ? 0.05
                            : bulldozing_.get_closing_distance();
  const long r =
    std::max(0L, static_cast<long>(std::ceil(distance / (2.0 * spacing_) -
                                             1e-9)));
  /* the box of the nodes pressed, with a margin for the closing and the
  rim; it may reach beyond the grid. Empty without nodes pressed */
  long ci0 = 0, cj0 = 0, cbx = 0, cby = 0;
  std::vector<char>& closed = bulldoze_closed_;
  auto cell = [&](long i, long j) {
    return static_cast<size_t>((j - cj0) * cbx + (i - ci0));
  };
  /* whether node (i, j) is in a patch: a wall to the erosion */
  auto in_patch = [&](long i, long j) {
    return i >= ci0 && j >= cj0 && i < ci0 + cbx && j < cj0 + cby &&
           closed[cell(i, j)];
  };
  if(!pressed_keys_.empty()) {
    long pi0 = nx, pi1 = -1, pj0 = ny, pj1 = -1;
    for(long long key : pressed_keys_) {
      const long i = KeyI(key), j = KeyJ(key);
      pi0 = std::min(pi0, i);
      pi1 = std::max(pi1, i);
      pj0 = std::min(pj0, j);
      pj1 = std::max(pj1, j);
    }
    const long margin = 2 * r + 1;
    ci0 = pi0 - margin;
    cj0 = pj0 - margin;
    cbx = pi1 - pi0 + 1 + 2 * margin;
    cby = pj1 - pj0 + 1 + 2 * margin;
    const size_t cells = static_cast<size_t>(cbx * cby);
    std::vector<char>& contact = bulldoze_contact_;
    std::vector<char>& work = bulldoze_work_;
    contact.assign(cells, 0);
    work.resize(cells);
    closed.resize(cells);
    for(long long key : pressed_keys_) {
      contact[cell(KeyI(key), KeyJ(key))] = 1;
    }
    /* the closing: a dilation by r nodes, then an erosion by r, each along
    x and then along y (a square of 2 r + 1 nodes). Each sweep counts the
    cells set in the window of 2 r + 1 along a line as it slides, so it
    costs the same for any r */
    std::vector<long>& window = bulldoze_window_;
    auto sweep = [&](const std::vector<char>& in, std::vector<char>& out,
                     bool along_x, bool dilate) {
      const long n = along_x ? cbx : cby;
      const long lines = along_x ? cby : cbx;
      window.assign(static_cast<size_t>(n) + 1, 0);
      for(long b = 0; b < lines; b++) {
        auto at = [&](long a) {
          return static_cast<size_t>(along_x ? b * cbx + a : a * cbx + b);
        };
        /* window[a + 1]: the cells set in 0 .. a */
        for(long a = 0; a < n; a++) {
          window[a + 1] = window[a] + (in[at(a)] ? 1 : 0);
        }
        for(long a = 0; a < n; a++) {
          const long first = a - r, last = a + r;
          const long set =
            window[std::min(last, n - 1) + 1] - window[std::max(first, 0L)];
          /* beyond the box, nothing is set */
          out[at(a)] = dilate ? set > 0 : (first >= 0 && last < n &&
                                           set == 2 * r + 1);
        }
      }
    };
    if(r > 0) {
      sweep(contact, work, true, true);
      sweep(work, closed, false, true);
      sweep(closed, work, true, false);
      sweep(work, closed, false, false);
      for(size_t k = 0; k < cells; k++) {
        closed[k] |= contact[k];
      }
    } else {
      closed = contact;
    }
    std::vector<char>& visited = bulldoze_visited_;
    std::vector<char>& on_rim = bulldoze_on_rim_;
    visited.assign(cells, 0);
    on_rim.assign(cells, 0);
    std::vector<long long>& stack = bulldoze_stack_;
    std::vector<long long>& rim = bulldoze_rim_;
    for(long long start : pressed_keys_) {
      const long si = KeyI(start), sj = KeyJ(start);
      if(visited[cell(si, sj)]) {
        continue;
      }
      visited[cell(si, sj)] = 1;
      rim.clear();
      std::vector<long long>& patch = bulldoze_patch_;
      patch.clear();
      long area = 0, edges = 0; // nodes of the patch, and its edges
      stack.assign(1, start);
      while(!stack.empty()) {
        const long long key = stack.back();
        stack.pop_back();
        const long i = KeyI(key), j = KeyJ(key);
        area++;
        if(contact[cell(i, j)]) {
          patch.push_back(key);
        }
        active.push_back(key);
        for(long dj = -1; dj <= 1; dj++) {
          for(long di = -1; di <= 1; di++) {
            if(di == 0 && dj == 0) {
              continue;
            }
            if(!is_node(i + di, j + dj)) {
              edges += di == 0 || dj == 0 ? 1 : 0;
              continue;
            }
            const size_t c = cell(i + di, j + dj);
            if(!closed[c] && (di == 0 || dj == 0)) {
              edges++;
            }
            if(!closed[c]) {
              if(!on_rim[c]) {
                on_rim[c] = 1;
                rim.push_back(Key(i + di, j + dj));
              }
            } else if((di == 0 || dj == 0) && !visited[c]) {
              visited[c] = 1;
              stack.push_back(Key(i + di, j + dj));
            }
          }
        }
      }
      for(long long key : rim) {
        on_rim[cell(KeyI(key), KeyJ(key))] = 0;
      }
      /* the soil pressed down at each node in contact compacts while the
      layer under it can take it; the rest is pushed aside */
      const double depth =
        std::isnan(bulldozing_.get_compaction_depth())
          ? 2.0 * double(area) * spacing_ / double(std::max(edges, 1L))
          : bulldozing_.get_compaction_depth();
      double volume = 0.0; // as a height over one node
      for(long long key : patch) {
        const long i = KeyI(key), j = KeyJ(key);
        Tile& tile = *FindTile(i, j);
        const size_t k = InTile(i, j);
        const size_t slot = size_t(tile.pressed_slot[k]);
        const double height = pressed_heights_[slot];
        const auto* soil = NodeSoil(i, j);
        const double capacity = soil ? soil->CompactionRatio() * depth : 0.0;
        Node& node = tile.nodes[k];
        const double compacted =
          std::min(pressed_sinkages_[slot],
                   std::max(0.0, capacity - node.compaction));
        node.compaction += compacted;
        volume += height - compacted;
      }
      volume *= flow_factor;
      if(!(volume > 0.0) || rim.empty()) {
        continue; // nothing pressed, or a patch over the whole grid
      }
      const double height = volume / double(rim.size());
      for(long long key : rim) {
        raise(key, height);
        active.push_back(key);
      }
    }
  }

  for(long long key : unsettled_) {
    const long i = KeyI(key), j = KeyJ(key);
    GetTile(i, j).flags[InTile(i, j)] &= static_cast<uint8_t>(~kUnsettled);
    active.push_back(key);
  }
  unsettled_.clear();

  /* erosion, over the nodes within erosion_iterations of those changed or
  still flowing (soil moves one node per pass): soil flows between neighbors
  along x and y outside the patches, from the higher to the lower, where the
  slope of their levels is steeper than the angle of repose */
  const int iterations = bulldozing_.get_erosion_iterations();
  if(iterations == 0 || active.empty()) {
    pressed_keys_.clear();
    pressed_heights_.clear();
    pressed_sinkages_.clear();
    NextGeneration(pressed_generation_, &Tile::pressed_stamp);
    return changed;
  }
  /* the set: a node is in it when its scratch stamp is this generation */
  NextGeneration(scratch_generation_, &Tile::scratch_stamp);
  std::vector<long long>& nodes = bulldoze_nodes_;
  nodes.clear();
  auto add = [&](long i, long j) {
    Tile& tile = GetTile(i, j);
    const size_t k = InTile(i, j);
    if(tile.scratch_stamp[k] != scratch_generation_) {
      tile.scratch_stamp[k] = scratch_generation_;
      nodes.push_back(Key(i, j));
    }
  };
  for(long long key : active) {
    add(KeyI(key), KeyJ(key));
  }
  for(size_t first = 0, layer = 0; layer < size_t(iterations); layer++) {
    const size_t last = nodes.size();
    for(size_t k = first; k < last; k++) {
      const long i = KeyI(nodes[k]), j = KeyJ(nodes[k]);
      const long neighbor[4][2] = {{i - 1, j}, {i + 1, j}, {i, j - 1},
                                   {i, j + 1}};
      for(const auto& n : neighbor) {
        if(is_node(n[0], n[1])) {
          add(n[0], n[1]);
        }
      }
    }
    first = last;
  }
  /* in the order of the keys, so that every run sums the same */
  std::sort(nodes.begin(), nodes.end());
  const size_t count = nodes.size();
  std::vector<Node*>& node_of = bulldoze_node_of_;
  node_of.resize(count);
  for(size_t k = 0; k < count; k++) {
    const long i = KeyI(nodes[k]), j = KeyJ(nodes[k]);
    Tile& tile = *FindTile(i, j);
    tile.scratch_slot[InTile(i, j)] = static_cast<int32_t>(k);
    node_of[k] = &tile.nodes[InTile(i, j)];
  }
  /* the index of node (i, j) in the set, -1 if it is not in it */
  auto index_of = [&](long i, long j) -> long {
    if(!is_node(i, j)) {
      return -1;
    }
    const Tile* tile = FindTile(i, j);
    const size_t k = InTile(i, j);
    return tile && tile->scratch_stamp[k] == scratch_generation_
             ? long(tile->scratch_slot[k])
             : -1;
  };
  std::vector<double>& level = bulldoze_level_;
  std::vector<double>& delta = bulldoze_delta_;
  std::vector<double>& rise = bulldoze_rise_; // the most a neighbor may be higher
  std::vector<char>& fixed = bulldoze_fixed_;
  /* the neighbors along +x and +y in the set, -1 for none */
  std::vector<long>& next_x = bulldoze_next_x_;
  std::vector<long>& next_y = bulldoze_next_y_;
  level.resize(count);
  delta.resize(count);
  rise.resize(count);
  fixed.resize(count);
  next_x.resize(count);
  next_y.resize(count);
  const double given = bulldozing_.get_erosion_angle();
  /* the height above the slope of repose taken as settled, so that the
  erosion, which only approaches the slope, ends */
  const double settled = 1e-6 * spacing_;
  for(size_t k = 0; k < count; k++) {
    const long i = KeyI(nodes[k]), j = KeyJ(nodes[k]);
    level[k] = node_of[k]->level;
    fixed[k] = in_patch(i, j) ? 1 : 0;
    double angle = given;
    if(std::isnan(angle)) {
      const auto* soil = NodeSoil(i, j);
      angle = soil ? soil->friction_angle : 0.5 * M_PI;
    }
    rise[k] = angle < 0.5 * M_PI ? spacing_ * std::tan(angle)
                                 : std::numeric_limits<double>::infinity();
    next_x[k] = index_of(i + 1, j);
    next_y[k] = index_of(i, j + 1);
  }
  pressed_keys_.clear();
  pressed_heights_.clear();
  pressed_sinkages_.clear();
  NextGeneration(pressed_generation_, &Tile::pressed_stamp);
  for(int pass = 0; pass < iterations; pass++) {
    std::fill(delta.begin(), delta.end(), 0.0);
    bool any = false;
    for(size_t a = 0; a < count; a++) {
      if(fixed[a]) {
        continue;
      }
      for(long neighbor : {next_x[a], next_y[a]}) {
        if(neighbor < 0) {
          continue;
        }
        const size_t b = static_cast<size_t>(neighbor);
        if(fixed[b]) {
          continue;
        }
        /* levels are down: the higher node has the smaller level */
        const double difference = level[a] - level[b];
        const double excess = std::abs(difference) - std::min(rise[a], rise[b]);
        if(!(excess > settled)) {
          continue;
        }
        const double flow = 0.25 * excess;
        const size_t high = difference > 0.0 ? b : a;
        const size_t low = difference > 0.0 ? a : b;
        delta[high] += flow;
        delta[low] -= flow;
        any = true;
      }
    }
    if(!any) {
      break;
    }
    for(size_t k = 0; k < count; k++) {
      level[k] += delta[k];
    }
    if(pass + 1 == iterations) {
      /* still flowing: go on at the next bulldozing */
      for(size_t k = 0; k < count; k++) {
        if(delta[k] != 0.0) {
          const long i = KeyI(nodes[k]), j = KeyJ(nodes[k]);
          uint8_t& flags = FindTile(i, j)->flags[InTile(i, j)];
          if(!(flags & kUnsettled)) {
            flags |= kUnsettled;
            unsettled_.push_back(nodes[k]);
          }
        }
      }
    }
  }
  for(size_t k = 0; k < count; k++) {
    if(level[k] != node_of[k]->level) {
      raise(nodes[k], node_of[k]->level - level[k]);
    }
  }
  return changed;
}

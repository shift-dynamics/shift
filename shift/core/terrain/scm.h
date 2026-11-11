#ifndef SHIFT_TERRAIN_SCM_H_
#define SHIFT_TERRAIN_SCM_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "shift/core/parameter/list.h"
#include "shift/core/terrain/basic.h"

namespace shift {
namespace core {
namespace terrain {

/*!
@brief the bulldozing of an SCM terrain (see Scm): the soil a body presses
down is pushed to the rim of its rut, then slides down slopes steeper than
the angle of repose

The soil is bulldozed every interval of time (by default 1 ms; 0 for after
every accepted step), at the first accepted step at or after each multiple of
it: bulldozing changes the surface the bodies are on, and changing it at
every step of the integrator, which takes steps of microseconds through the
contacts, makes it take many more of them. The nodes bodies were on in the
steps since the last time form patches (nodes next to each other along x or
y, with the gaps between them up to closing_distance wide closed, see below).
Under each node of a patch the soil first compacts: a layer compaction_depth
deep (by default the width of the patch, \f$2 A / P\f$ of its area and
perimeter, about the depth the pressure reaches) takes up to
\f$1 - w / w_{max}\f$ of its depth, from the unit weight of the soil and
its max_unit_weight (force_module::SoilProperties::CompactionRatio; 0, no
compaction, without max_unit_weight). Each node keeps the sinkage it has
taken by compacting (Node::compaction), and only the soil pressed down past
that is pushed aside: so a shallow print leaves no rim, and a deep one pushes
up the soil it could not compact. The soil a patch pushed aside since the
last time, times flow_factor, is spread evenly over its rim: the nodes next
to the patch (also diagonally) outside it. Then, erosion_iterations
times, soil flows between neighboring nodes along x and y outside the
patches, from the higher to the lower, where the slope between them is
steeper than erosion_angle: a quarter of the height above that slope at a
time, so a node with four lower neighbors does not overshoot; a height of
less than 1e-6 spacing above it is taken as settled. The patches are a wall
to the erosion: the soil on the rim slides outward, away from the bodies,
and none slides back under them or into the gaps between their parts until
they have left. Only the nodes near those changed take part: the patches and
their rims, and the nodes the last pass of erosion moved soil between,
widened by erosion_iterations nodes; so erosion goes on every interval until
the slopes settle, also once bodies have left.

Slopes are measured on the levels, from the undisturbed surface, so the
undisturbed surface itself (an elevation map) is not eroded. With a
flow_factor of 1 the volume of soil pushed aside is conserved, but for soil
pushed off the grid; more than 1, the soil pushed aside loosens and takes more
room.

A body that goes into a pile (a node raised above the undisturbed surface)
cuts the pile down to its own depth, and that soil is pushed on with the rest;
the pile does not carry the body (force_module::SoilContact).

JSON (angles in radians):
\code
"bulldozing": {"enable": true, "flow_factor": 1.0, "erosion_angle": 0.6,
               "erosion_iterations": 30, "closing_distance": 0.1,
               "interval": 0.001, "compaction_depth": 0.5}
\endcode
*/
class Bulldozing : public parameter::List {
public:
  Bulldozing() {
    DeclareParameters();
  }
  virtual ~Bulldozing() {}

  bool is_enabled() const {
    return enable_;
  }
  double get_flow_factor() const {
    return flow_factor_;
  }
  /*! \brief the angle of repose given; NaN for that of each node's soil */
  double get_erosion_angle() const {
    return erosion_angle_;
  }
  int get_erosion_iterations() const {
    return erosion_iterations_;
  }
  /*! \brief the widest gap closed between nodes in contact (m); NaN for
   * the default, 0.05 m */
  double get_closing_distance() const {
    return closing_distance_;
  }
  /*! \brief the time between bulldozings (s), 0 for after every accepted
   * step; NaN for the default, 1 ms */
  double get_interval() const {
    return interval_;
  }
  /*! \brief the depth of the layer under a body that compacts (m); NaN for
   * the default, the width of the patch */
  double get_compaction_depth() const {
    return compaction_depth_;
  }
  /*! \brief throw unless the values are valid; owner names the terrain */
  void Check(const std::string& owner) const;

protected:
  void DeclareParameters() override;

private:
  bool enable_ = false;
  double flow_factor_ = 1.0;
  double erosion_angle_ = std::nan("");
  int erosion_iterations_ = 30;
  double closing_distance_ = std::nan("");
  double interval_ = std::nan("");
  double compaction_depth_ = std::nan("");
};

/*!
@brief deformable terrain of the Soil Contact Model (SCM): a grid of nodes
over the surface of a terrain (see Basic), which the soil models lower and
raise

The grid covers the ranges x and y of the terrain frame, with nodes spacing
apart from (x[0], y[0]); a range that is not a whole number of spacings ends
at the last node inside it. Each node starts on the undisturbed surface: the
elevation map (filename, as for "basic"), or without one the xy plane of the
terrain frame. The nodes are stored in tiles of 32 by 32, each made when a
node in it first changes (Node), so a large grid costs little until it is
driven on, and the nodes along a rut are reached by index, not by search.

The deformed surface is the undisturbed one lowered by the levels of the
nodes, interpolated bilinearly in each cell. The cells along the edges of the
grid run to a level of 0 one spacing beyond it, so the surface is continuous
there; further out it is the undisturbed surface. Query and SurfaceZ return the
deformed surface, so that tires and the other models that read the terrain
follow it.

Nodes change in two stages, so that an evaluation of a step the integrator
rejects leaves no trace: while the forces are evaluated, the models that act
on the soil write the nodes they change with Trial, which starts from the
accepted node, after ClearTrials at the start of each evaluation; after each
accepted step, AcceptStep keeps the trials of the last evaluation, that of
the accepted state. Until then the surface is that of the accepted nodes.

The soil of each node is the terrain's soil at the node (NodeSoil): its soil,
or that of the soil map. The terrain needs one of them.

The soil under a body follows Bekker's pressure-sinkage law while it is
loaded past the deepest sinkage \f$z_p\f$ it has taken, and springs back
along a stiffer line below it (Pressure): with the sinkage \f$s\f$ of the
body from the undisturbed surface,

\f[
p = \max\left(0, \; \min\left(p_{load}(s), \;
    p_{load}(z_p) - k_u (z_p - s)\right)\right), \quad
p_{load}(s) = \left(\frac{k_c}{b} + k_\phi\right) s^n, \quad
k_u = \mathrm{unloading\_ratio} \, \frac{p_{load}(z_p)}{z_p}
\f]

so that a rut of depth \f$z_p\f$ springs back to
\f$z_p (1 - 1 / \mathrm{unloading\_ratio})\f$, the level of its node. The
force on bodies is computed by force_module::SoilContact, from the nodes under
them to a depth of max_sinkage below the surface.

A body set on the surface at the start carries none of its weight there,
falls, and with little damping sinks to about twice its depth at rest, which
the soil then keeps. settle_time adds settle_damping to the damping for the
first settle_time seconds, fading to nothing (DampingAt), so the bodies
settle onto the soil to the sinkage of Bekker's law under their weight.

JSON (SI units):
\code
"terrain": [{"type": "scm", "name": "site", "parent": [["fixed"], "terrain"],
             "x": [-10, 10], "y": [-10, 10], "spacing": 0.05,
             "filename": "site.tiff", "x_scale": 0.5, "y_scale": 0.5,
             "soil": {"name": "sandy_loam", "shear_modulus": 0.015},
             "bulldozing": {"enable": true, "erosion_iterations": 30}}]
\endcode

With bulldozing (see Bulldozing), the soil bodies press down is pushed to the
rims of their ruts and slides down to the angle of repose, after each
accepted step (every Bulldozing interval).
*/
class Scm : public Basic {
public:
  /*! \brief the state of a node of the grid that has changed */
  struct Node {
    /*! displacement of the surface along z of the terrain frame (down)
     * from the undisturbed surface: positive where it is lowered (a rut),
     * negative where it is raised (a pile) */
    double level = 0.0;
    /*! the deepest sinkage the soil has taken at the node, from the
     * undisturbed surface: \f$z_p\f$ of the pressure-sinkage law */
    double plastic_sinkage = 0.0;
    /*! shear displacement of the soil at the node, along x and y of the
     * terrain frame, while a body is on it */
    double shear[2] = {0.0, 0.0};
    /*! slip velocity of the body over the node, along x and y of the
     * terrain frame, while a body is on it: the shear displacement grows by
     * the trapezoidal rule from it to the next slip velocity */
    double slip[2] = {0.0, 0.0};
    /*! with bulldozing, the sinkage the soil under the node has taken by
     * compacting, not pushed aside (see Bulldozing) */
    double compaction = 0.0;
    /*! whether a body is on the node: when it leaves, its shear
     * displacement and slip velocity are dropped (see AcceptStep) */
    bool in_contact = false;

    /*! \brief whether the node is as it started, so it need not be kept */
    bool is_undisturbed() const {
      return level == 0.0 && plastic_sinkage == 0.0 && compaction == 0.0 &&
             shear[0] == 0.0 &&
             shear[1] == 0.0 && slip[0] == 0.0 && slip[1] == 0.0 &&
             !in_contact;
    }
  };

  Scm()
    : Basic() {
    DeclareParameters();
  }
  Scm(std::string name)
    : Basic(name) {
    DeclareParameters();
  }
  virtual ~Scm() override {}

  static std::unique_ptr<Scm> Create();

  static bool is_registered() {
    return registered_;
  }

  void Load() override;
  void AcceptStep() override;
  void UpdateOutput() override;

  double SurfaceZ(double x, double y, double& z_x,
                  double& z_y) const override;

  /*! @name The grid */
  /*! @{ */
  long get_node_count_x() const {
    return node_count_x_;
  }
  long get_node_count_y() const {
    return node_count_y_;
  }
  double get_spacing() const {
    return spacing_;
  }
  /*! \brief x and y of node (i, j) in the terrain frame */
  double NodeX(long i) const {
    return x_range_[0] + i * spacing_;
  }
  double NodeY(long j) const {
    return y_range_[0] + j * spacing_;
  }
  /*! \brief whether (i, j) is a node of the grid */
  bool is_node(long i, long j) const {
    return i >= 0 && j >= 0 && i < node_count_x_ && j < node_count_y_;
  }
  /*! \brief the node nearest x, y of the terrain frame; false if that is
   * beyond the grid by more than half a spacing */
  bool NearestNode(double x, double y, long& i, long& j) const;
  /*! \brief z (down) of the undisturbed surface at node (i, j) */
  double UndisturbedZ(long i, long j) const;
  /*! \brief z (down) of the surface at node (i, j), of the accepted state */
  double NodeZ(long i, long j) const;
  /*! \brief the soil at node (i, j); null if the terrain has none there */
  const force_module::SoilProperties* NodeSoil(long i, long j) const;
  /*! @} */

  /*! @name The pressure-sinkage law */
  /*! @{ */
  /*! \brief the pressure of soil under a patch of width b at the sinkage s
   * from the undisturbed surface, where the deepest sinkage has been
   * plastic_sinkage \f$z_p\f$ (see the class description) */
  double Pressure(const force_module::SoilProperties& soil, double b, double s,
                  double plastic_sinkage) const;
  /*! \brief the level of a node whose deepest sinkage is plastic_sinkage,
   * once the soil has sprung back: \f$z_p (1 - 1 / \mathrm{unloading\_ratio})\f$ */
  double ReboundLevel(double plastic_sinkage) const {
    return plastic_sinkage * (1.0 - 1.0 / unloading_ratio_);
  }
  double get_unloading_ratio() const {
    return unloading_ratio_;
  }
  /*! \brief Hunt and Crossley's factor of the pressure, per speed of
   * sinkage (s/m) */
  double get_damping() const {
    return damping_;
  }
  /*! \brief the time from the start over which the soil is settled (s);
   * 0 for none */
  double get_settle_time() const {
    return settle_time_;
  }
  /*! \brief the damping added at the start to settle the bodies onto the
   * soil (s/m) */
  double get_settle_damping() const {
    return settle_damping_;
  }
  /*! \brief the factor of the pressure per speed of sinkage at time t:
   * damping, and over the first settle_time seconds settle_damping, which
   * fades smoothly to nothing, by \f$\frac{1}{2}(1 + \cos(\pi t /
   * t_s))\f$, so the force has no jump when the settling ends */
  double DampingAt(double t) const {
    if(!(settle_time_ > 0.0) || !(t < settle_time_)) {
      return damping_;
    }
    const double fade =
      0.5 * (1.0 + std::cos(M_PI * std::max(0.0, t) / settle_time_));
    return damping_ + settle_damping_ * fade;
  }
  /*! \brief how far below the surface bodies are looked for */
  double get_max_sinkage() const {
    return max_sinkage_;
  }
  const Bulldozing& get_bulldozing() const {
    return bulldozing_;
  }
  /*! @} */

  /*! @name The nodes that have changed */
  /*! @{ */
  /*! \brief the accepted state of node (i, j); null if it is undisturbed */
  const Node* FindNode(long i, long j) const;
  /*! \brief the trial state of node (i, j) for the current evaluation,
   * started from its accepted state; (i, j) must be a node of the grid */
  Node& Trial(long i, long j);
  /*! \brief the trial state of node (i, j), null if it has none */
  const Node* FindTrial(long i, long j) const;
  /*! \brief drop the trials of the previous evaluation, at the start of an
   * evaluation of the forces */
  void ClearTrials();
  /*! \brief number of nodes that have changed (not undisturbed) */
  size_t get_changed_node_count() const;
  /*! \brief incremented by each AcceptStep that changes a node, so that a
   * viewer or a logger can tell when the surface changed */
  long get_revision() const {
    return revision_;
  }
  /*! \brief time of the last accepted step (that of the last evaluation of
   * the forces before AcceptStep), NaN before the first */
  double get_accepted_time() const {
    return accepted_time_;
  }
  /*! @} */

  /*! @name The surface at the output times */
  /*! @{ */
  /*! \brief a node whose level changed */
  struct LevelChange {
    /*! the node, j * get_node_count_x() + i */
    int32_t index;
    float level; //!< its new level
  };
  /*! \brief the nodes whose levels changed between the previous output time
   * and this one, in increasing index, gathered by UpdateOutput: a viewer or
   * a logger keeps the surface from these alone */
  const std::vector<LevelChange>& get_output_changes() const {
    return output_changes_;
  }
  /*! \brief the accepted levels of all nodes, j * get_node_count_x() + i */
  void GetLevels(std::vector<float>& levels) const;
  /*! \brief restart from the levels of all nodes (j * get_node_count_x() +
   * i), such as those of an output time shown by a viewer: each node with a
   * positive level is taken to have sprung back to it, so its deepest
   * sinkage is level / (1 - 1 / unloading_ratio), and with bulldozing to
   * have compacted the soil by its level; the shear displacements and the
   * contacts are dropped */
  void SetLevels(const std::vector<float>& levels);
  /*! @} */

protected:
  virtual void DeclareParameters() override;
  bool requires_elevation_map() const override {
    return false;
  }

private:
  static constexpr long kTileBits = 5;
  static constexpr long kTileWidth = 1L << kTileBits;
  static constexpr size_t kTileSize = size_t(kTileWidth * kTileWidth);
  /* the flags of a node in a tile */
  static constexpr uint8_t kDirty = 1;     // in dirty_
  static constexpr uint8_t kUnsettled = 2; // in unsettled_

  /* the nodes of a square of kTileWidth by kTileWidth of the grid, x
  fastest: the accepted nodes, and the sets a node can be in. A node is in
  the trials, the nodes pressed since the last bulldozing, or the scratch set
  of Bulldoze when its stamp there is the set's generation, and its slot
  indexes the set's arrays; so a set is emptied by a new generation */
  struct Tile {
    Node nodes[kTileSize];
    uint32_t trial_stamp[kTileSize] = {};
    int32_t trial_slot[kTileSize];
    uint32_t pressed_stamp[kTileSize] = {};
    int32_t pressed_slot[kTileSize];
    uint32_t scratch_stamp[kTileSize] = {};
    int32_t scratch_slot[kTileSize];
    uint8_t flags[kTileSize] = {};
  };

  long long Key(long i, long j) const {
    return static_cast<long long>(j) * node_count_x_ + i;
  }
  long KeyI(long long key) const {
    return static_cast<long>(key % node_count_x_);
  }
  long KeyJ(long long key) const {
    return static_cast<long>(key / node_count_x_);
  }
  size_t TileIndex(long i, long j) const {
    return size_t(j >> kTileBits) * size_t(tile_count_x_) +
           size_t(i >> kTileBits);
  }
  static size_t InTile(long i, long j) {
    return size_t((j & (kTileWidth - 1)) * kTileWidth +
                  (i & (kTileWidth - 1)));
  }
  /*! \brief the tile of node (i, j) of the grid; null until it is made */
  Tile* FindTile(long i, long j) const {
    return tiles_[TileIndex(i, j)].get();
  }
  /*! \brief the tile of node (i, j) of the grid, made if need be */
  Tile& GetTile(long i, long j);
  /*! \brief the accepted node of key, its tile made if need be */
  Node& Accepted(long long key) {
    const long i = KeyI(key), j = KeyJ(key);
    return GetTile(i, j).nodes[InTile(i, j)];
  }
  /*! \brief empty the set of stamps by a new generation */
  void NextGeneration(uint32_t& generation,
                      uint32_t (Tile::*stamps)[kTileSize]);
  void MarkDirty(long long key);
  /*! \brief add height to the soil pressed down at key since the last
   * bulldozing, of which sinkage below the undisturbed surface (the rest is
   * a pile cut down) */
  void AddPressed(long long key, double height, double sinkage);
  /*! \brief whether the bulldozing is due after this accepted step (see
   * Bulldozing::get_interval), and if so the next time it is due */
  bool BulldozeDue();
  void Clear();
  /*! \brief the accepted level of node (i, j); 0 if it is undisturbed or
   * not a node of the grid */
  double Level(long i, long j) const;
  /*! \brief push the soil pressed down since the last bulldozing to the
   * rims of the patches of nodes in contact, then erode the slopes there and
   * where the last erosion left them unsettled (see Bulldozing); the nodes
   * pressed are pressed_keys_, with the increases of their levels in
   * pressed_heights_, and of those the parts below the undisturbed surface,
   * which may compact, in pressed_sinkages_; returns whether it changed a
   * level */
  bool Bulldoze();

  double spacing_ = 0.0;
  double unloading_ratio_ = 10.0;
  double damping_ = 0.0;
  double settle_time_ = 0.0;
  double settle_damping_ = 20.0;
  double max_sinkage_ = 0.5;
  Bulldozing bulldozing_;

  long node_count_x_ = 0;
  long node_count_y_ = 0;
  long tile_count_x_ = 0;
  /* the tiles, tile_count_x_ along x; null until a node in one changes */
  std::vector<std::unique_ptr<Tile>> tiles_;
  /* the indices of the tiles made, in the order made */
  std::vector<size_t> made_tiles_;
  /* the trials of the current evaluation: their nodes, in a deque so that
  the references Trial returns stay valid, reused from one evaluation to the
  next, and their keys */
  uint32_t trial_generation_ = 1;
  std::deque<Node> trial_nodes_;
  std::vector<long long> trial_keys_;
  /* the nodes in contact since the last bulldozing, and how far each was
  pressed down */
  uint32_t pressed_generation_ = 1;
  std::vector<long long> pressed_keys_;
  std::vector<double> pressed_heights_;
  std::vector<double> pressed_sinkages_;
  uint32_t scratch_generation_ = 1;
  /* the next time the bulldozing is due; NaN for at the next step */
  double next_bulldozing_ = std::nan("");
  /*! the accepted nodes in contact, in increasing key */
  std::vector<long long> contacts_;
  std::vector<long long> next_contacts_;
  long revision_ = 0;
  double accepted_time_ = 0.0;
  /*! the nodes whose levels changed since the last UpdateOutput */
  std::vector<long long> dirty_;
  /*! the nodes the last pass of erosion moved soil between, whose slopes
   * may still be too steep: erosion goes on there at the next bulldozing */
  std::vector<long long> unsettled_;
  /* the scratch arrays of Bulldoze, kept from one bulldozing to the next */
  std::vector<long long> bulldoze_active_, bulldoze_stack_, bulldoze_rim_,
    bulldoze_nodes_, bulldoze_patch_;
  std::vector<char> bulldoze_contact_, bulldoze_work_, bulldoze_closed_,
    bulldoze_visited_, bulldoze_on_rim_, bulldoze_fixed_;
  std::vector<long> bulldoze_window_, bulldoze_next_x_, bulldoze_next_y_;
  std::vector<double> bulldoze_level_, bulldoze_delta_, bulldoze_rise_;
  std::vector<Node*> bulldoze_node_of_;
  /* the undisturbed z and the index of the soil (-1 for none) of each node,
  x fastest; empty for a very large grid */
  std::vector<double> undisturbed_;
  std::vector<int16_t> node_soils_;
  std::vector<LevelChange> output_changes_;

  static bool registered_;
};

} // namespace terrain
} // namespace core
} // namespace shift

#endif

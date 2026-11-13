#ifndef SHIFT_GUI_SIMULATION_H_
#define SHIFT_GUI_SIMULATION_H_

#include "shift/core/topology.h"
#include "shift/gui/scene.h"

#include <QFloat16>

#include <json/json.h>

#include <condition_variable>
#include <map>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace shift {
namespace core {
class Context;
namespace body {
class Base;
}
namespace light {
class Base;
}
namespace terrain {
class Scm;
class Sph;
} // namespace terrain
} // namespace core

namespace gui {

/*!
@brief One shift::core::Context shared by simulation, playback and jogging

Follows the conventions of the original OpenGL workspace:
- Run integrates the equations of motion from the configuration currently
  shown, recording the state at every output time step (knot points).
  Integration runs on a worker thread so the computed portion can be
  played back while the rest is still being computed.
- Playback evaluates poses at an arbitrary time by interpolating linearly
  between knot points. It uses a separate DHVA buffer, so it never touches
  the context state while the worker is integrating.
- A deformable terrain (shift::core::terrain::Scm) is drawn from the levels
  of its nodes. At each knot point only the nodes whose levels changed are
  recorded (Scm::get_output_changes), as their levels before and after;
  playback moves the levels shown forward or back through these and
  interpolates those of the knot point after the time shown, and the
  viewport uploads only the rectangle of the terrain that changed
  (TakeTerrainUpdates). Run and AdoptShownState restart the terrain from the
  levels shown (Scm::SetLevels), Reset from the undisturbed surface. Where
  an SPH region overrides it, its surface is drawn at the region's floor.
- The particles of an SPH soil (shift::core::terrain::Sph) are recorded at
  each knot point, as half floats, and the knot point at or before the time
  shown is drawn (TakeParticleUpdates). They cannot be restarted from the
  time shown: Run continues the soil from where it was last computed, and
  Reset refills it (Sph::Restart).
- The workspace jogs joints by editing q in the context directly. That is
  only possible while no simulation is being computed; AdoptShownState
  first moves the context to the configuration shown and discards the
  recorded results.
*/
class Simulation {
public:
  Simulation() = default;
  ~Simulation();

  Simulation(const Simulation&) = delete;
  Simulation& operator=(const Simulation&) = delete;

  /*! \brief load a model JSON or YAML file, or an HDF5 log
   * (logger::HDF5) to play back; throws on failure. A log holds its model
   * and the files it read, which are taken out to a temporary directory
   * (removed with the next Load); the model is loaded without its loggers,
   * so that a Run from the log writes no files, and the rows of the log
   * become the knot points, as if they had just been computed. */
  void Load(const std::string& filename);

  /*! \brief load the model again from model, its JSON as edited (see
   * Context::get_model_json), discarding the results; its files are found
   * as they were for the model loaded last. Throws on failure, leaving no
   * model loaded. */
  void Reload(const Json::Value& model);

  bool is_loaded() const {
    return context_ != nullptr;
  }

  const std::string& get_filename() const {
    return filename_;
  }

  shift::core::Context* get_context() {
    return context_.get();
  }

  const Scene& get_scene() const {
    return scene_;
  }

  const std::vector<shift::core::body::Base*>& get_bodies() const {
    return bodies_;
  }

  // ---------------------------------------------------------------- run

  /*! \brief integrate from the configuration currently shown */
  void Run();

  /*! \brief stop computing, keeping the results computed so far */
  void Stop();

  /*! \brief stop, discard results and return to the initial conditions
   * loaded from the model file */
  void Reset();

  bool is_computing();
  bool has_results();
  double get_computed_time();
  double get_end_time() const;
  double get_time_step() const;
  void set_end_time(double time_span);
  void set_time_step(double time_step);
  std::string get_integrator_type() const;

  /*! \brief error message of the last run, empty if it succeeded */
  std::string get_error();

  /*! \brief simulated time over wall-clock time of the current run */
  double get_real_time_factor();

  // ----------------------------------------------------------- playback

  /*! \brief show the recorded state at time t (clamped to the computed
   * range) */
  void ShowTime(double t);

  double get_shown_time() const {
    return shown_time_;
  }

  // ------------------------------------------------------- editing q

  /*! \brief prepare the context for editing q directly (jogging): copy the
   * configuration shown at the playback time into the context and discard
   * the recorded results. Returns false while a simulation is computing. */
  bool AdoptShownState();

  /*! \brief generalized coordinate q[q_index] of the configuration shown */
  double get_shown_q(int q_index);

  /*! \brief update the shown poses from the context's own DHVA, after q was
   * edited */
  void ShowContextState();

  // -------------------------------------------------------------- poses

  /*! \brief copy the shown poses if they changed since the previous call */
  bool TakePoses(Poses& poses);

  /*! \brief the rectangles of the terrains that changed since the previous
   * call; Scene::terrains already holds them, for a viewport that uploads
   * the whole scene again */
  void TakeTerrainUpdates(std::vector<TerrainUpdate>& updates);

  /*! \brief the particles of the SPH soils whose shown particles changed
   * since the previous call, or all */
  void TakeParticleUpdates(std::vector<ParticleUpdate>& updates,
                           bool all = false);

private:
  /*! an SPH soil drawn as Scene::particles[particles] */
  struct SphView {
    shift::core::terrain::Sph* sph = nullptr;
    int particles = 0;
    /*! the particles' positions in their frame (Sph::get_particle_frame),
     * x, y, z each: at the start, and at each knot point (worker thread,
     * mutex_ held) */
    std::vector<qfloat16> initial;
    std::vector<std::vector<qfloat16>> knots;
    long shown = -1;      /*!< the knot shown, -1 for initial (GUI thread) */
    bool changed = true;  /*!< since TakeParticleUpdates */
  };
  /*! \brief the particles' positions of an SPH soil now */
  static void Snapshot(const shift::core::terrain::Sph& sph,
                       std::vector<qfloat16>& positions);
  /*! \brief show knot (-1 for the start) of each SPH soil */
  void ShowParticles(long knot);

  /*! a deformable terrain drawn as Scene::terrains[terrain]: its control
   * points are the nodes of the grid with a ring of the undisturbed surface
   * around them, (nx + 2) by (ny + 2) */
  struct ScmView {
    shift::core::terrain::Scm* scm = nullptr;
    int terrain = 0;
    long nx = 0;
    long ny = 0;
    std::vector<float> undisturbed; /*!< z of the nodes, nx * ny */
    std::vector<float> worker;      /*!< levels recorded, worker thread */
    std::vector<float> base;        /*!< levels at base_delta_, GUI thread */
    std::vector<float> shown;       /*!< levels shown, GUI thread */
    /*! the control points changed since TakeTerrainUpdates, empty if
     * x_min > x_max */
    long x_min = 0, x_max = -1, y_min = 0, y_max = -1;
  };
  /*! a level recorded at a knot point */
  struct LevelDelta {
    int32_t view; /*!< in scm_views_ */
    int32_t index; /*!< node */
    float before;
    float after;
  };

  /*! \brief the view of a deformable terrain drawn as
   * Scene::terrains[index] */
  void AddScmView(shift::core::terrain::Scm& scm, int index);
  /*! \brief show level at node index of a view */
  void ShowLevel(ScmView& view, int32_t index, float level);
  /*! \brief restart the terrains and their records from the levels shown,
   * or from the undisturbed surface (reset) */
  void RestartTerrains(bool reset);
  /*! \brief show the levels at deltas_ position base plus the fraction p of
   * the deltas [partial_begin, partial_end) (GUI thread, mutex_ held) */
  void ShowLevels(size_t base, size_t partial_begin, size_t partial_end,
                  double p);

  struct StopRequested {};

  /*! \brief load the model file path, with Context::set_file_map; from
   * model if it is given, as if read from path */
  void LoadModel(const std::string& path,
                 std::map<std::string, std::string> file_map,
                 const Json::Value* model = nullptr);
  /*! \brief load the model of an HDF5 log and its rows */
  void LoadLog(const std::string& filename);
  void RemoveLogDirectory();
  void JoinWorker();
  void Work();
  void Observe(double time, const std::vector<double>& y);
  void ClearResults();
  /*! \brief poses of the bodies from the displacements in dhva */
  void UpdatePoses(const shift::core::topology::DHVA& dhva);
  void StoreInitialState();

  std::string filename_;
  std::string log_directory_; /*!< files taken out of a log */
  /*! the path and file map of the model loaded last, even if it failed to
   * load, see Context::set_file_map */
  std::string model_path_;
  std::map<std::string, std::string> file_map_;
  std::unique_ptr<shift::core::Context> context_;
  std::vector<shift::core::body::Base*> bodies_;
  Scene scene_;
  std::vector<double> initial_state_; /*!< q and dq loaded from the file */
  size_t state_size_ = 0;             /*!< q count + dq count */

  // shared with the worker, guarded by mutex_
  std::thread worker_;
  std::mutex mutex_;
  bool computing_ = false;
  bool stop_requested_ = false;
  std::string error_;
  std::vector<double> times_;  /*!< knot point times */
  std::vector<double> states_; /*!< knot point q and dq, state_size_ each */
  std::vector<LevelDelta> deltas_; /*!< levels changed, in order */
  /*! the first of deltas_ of each knot point, which lead to it from the
   * previous one */
  std::vector<size_t> knot_deltas_;
  /*! the intensity of each light of the scene at each knot point,
   * light_sources_.size() each */
  std::vector<float> light_values_;
  double wall_start_ = 0.0;
  double real_time_factor_ = 0.0;

  // GUI thread only
  shift::core::topology::DHVA playback_dhva_;
  std::vector<double> interpolated_;
  double shown_time_ = 0.0;
  Poses poses_;
  bool poses_changed_ = false;
  /*! the light of the model of each of the scene's lights */
  std::vector<const shift::core::light::Base*> light_sources_;
  std::vector<float> shown_lights_; /*!< their intensities at the time shown */
  std::vector<ScmView> scm_views_;
  std::vector<SphView> sph_views_;
  size_t base_delta_ = 0;    /*!< deltas_ applied to ScmView::base */
  size_t partial_begin_ = 0; /*!< deltas_ shown in part */
  size_t partial_end_ = 0;
};

} // namespace gui
} // namespace shift

#endif

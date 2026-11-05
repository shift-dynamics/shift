#ifndef SHIFT_GUI_SIMULATION_H_
#define SHIFT_GUI_SIMULATION_H_

#include "shift/core/topology.h"
#include "shift/gui/scene.h"

#include <condition_variable>
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

  /*! \brief load a model JSON file; throws on failure */
  void Load(const std::string& filename);

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

private:
  struct StopRequested {};

  void JoinWorker();
  void Work();
  void Observe(double time, const std::vector<double>& y);
  void ClearResults();
  /*! \brief poses of the bodies from the displacements in dhva */
  void UpdatePoses(const shift::core::topology::DHVA& dhva);
  void StoreInitialState();

  std::string filename_;
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
  double wall_start_ = 0.0;
  double real_time_factor_ = 0.0;

  // GUI thread only
  shift::core::topology::DHVA playback_dhva_;
  std::vector<double> interpolated_;
  double shown_time_ = 0.0;
  Poses poses_;
  bool poses_changed_ = false;
};

} // namespace gui
} // namespace shift

#endif

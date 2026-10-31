#ifndef SHIFT_GUI_WORKSPACE_H_
#define SHIFT_GUI_WORKSPACE_H_

#include "shift/gui/simulation.h"

#include <QElapsedTimer>
#include <QMainWindow>

class QDial;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QPushButton;
class QSlider;
class QTabWidget;
class QTimer;
class QTreeWidget;
class QTreeWidgetItem;

namespace shift {
namespace core {
class Substructure;
namespace joint {
class Base;
}
} // namespace core

namespace gui {

class PlaybackSlider;
class Viewport;

/*!
@brief Main window, following the layout of the original OpenGL workspace

- left panel, Scene tab: model tree (bodies and joints, with joint values and
  mobility numbers) above a context panel for the selected item; selecting a
  joint shows its influence coefficients and jog sliders (Tx..Rz, or roll,
  pitch and yaw for the quaternion of a spherical or float joint)
- left panel, Simulation tab: model parameters and Run
- below the tabs: play/pause button and playback slider
- right: 3D viewport
*/
class Workspace : public QMainWindow {
  Q_OBJECT

public:
  explicit Workspace(QWidget* parent = nullptr);
  ~Workspace() override;

  /*! \brief load a model JSON file, reporting errors in a dialog */
  bool Open(const QString& filename);

private:
  /*! one jog slider row for a joint coordinate; the slider is a rate
   * control that recenters when released, the spin box sets a target value.
   * For the quaternion of a spherical or float joint the row instead drives
   * one of the roll, pitch and yaw angles, R = Rz(yaw) Ry(pitch) Rx(roll),
   * and q_index is the index of the quaternion [x, y, z, w] */
  struct SliderHandle {
    QGroupBox* box = nullptr;
    QLabel* label = nullptr;
    QSlider* slider = nullptr;
    QDoubleSpinBox* spin_box = nullptr;
    QDial* gain_dial = nullptr;
    double single_step = 0.0;
    int q_index = -1;
    int dq_index = -1;
    int euler_index = -1; /*!< 0 roll, 1 pitch, 2 yaw; -1 joint coordinate */
    shift::core::joint::Base* joint = nullptr; /*!< joint of a roll, pitch or
                                                  yaw slider */
    bool enabled = false;
    shift::core::Substructure* substructure = nullptr;
  };

  /*! moving a coordinate to a value typed in a spin box, in small steps */
  struct TargetMove {
    SliderHandle* handle = nullptr;
    double target = 0.0;
    int counter = 0;
    int max_count = 0;
  };

  void InitializeMenus();
  void InitializeLeftPanel();
  void InitializeLayout();
  QWidget* AddSliders();
  void AddSlider(SliderHandle& handle, QLayout* layout, double single_step,
                 QDial* gain_dial);
  QWidget* AddModelParametersBox();

  void OpenDialog();
  void About();
  void PopulateTree();
  void UpdateTree();
  void SelectTreeItem();
  void ClearSliders();
  void SetSliders(shift::core::joint::Base* joint);
  void SetSlider(SliderHandle& handle, const char* label, int q_index,
                 int dq_index, shift::core::Substructure* substructure,
                 int euler_index = -1);
  void RecenterSlider(SliderHandle& handle);
  void PressSlider(SliderHandle& handle);
  void SetTarget(SliderHandle& handle, double value);
  void UpdateJointPanel();
  void UpdateBodyPanel();
  void LoadModelParameters();

  void RunSimulation();
  void StopSimulation();
  void ResetSimulation();
  void PlayPauseSimulation();
  void SetPlaybackTime(double t);

  /*! \brief advance playback and jogging, once per display frame */
  void Tick();
  void Jog();

  /*! \brief increment the coordinate of a slider; for a joint in a
   * substructure, Newton-Raphson iteration with the coordinate specified
   * keeps the loop closure constraints satisfied. Returns false if
   * Newton-Raphson failed. */
  bool IncrementCoordinate(SliderHandle& handle, double delta);
  bool SetCoordinate(SliderHandle& handle, double value);
  double get_mobility(const SliderHandle& handle);

  /*! \brief the value shown for a slider: its joint coordinate, or the roll,
   * pitch or yaw angle of its quaternion */
  double get_shown_value(const SliderHandle& handle);
  /*! \brief roll, pitch and yaw of the quaternion at q_index of the shown
   * state, continuous with the angles last shown */
  void get_shown_roll_pitch_yaw(int q_index, double rpy[3]);
  /*! \brief set the roll, pitch or yaw angle of a slider's quaternion,
   * keeping the other two angles */
  bool SetRollPitchYaw(SliderHandle& handle, double value);
  void UpdateStatus();

  Simulation simulation_;
  Viewport* viewport_ = nullptr;

  QTabWidget* tab_widget_ = nullptr;
  QTreeWidget* tree_widget_ = nullptr;
  QTreeWidgetItem* bodies_item_ = nullptr;
  QTreeWidgetItem* joints_item_ = nullptr;

  struct {
    QGroupBox* group_box = nullptr;
    QLabel* title = nullptr;
    QLabel* details = nullptr;
    QLabel* h[6] = {};
    shift::core::joint::Base* selected_joint = nullptr;
  } joint_panel_;

  struct {
    QGroupBox* group_box = nullptr;
    QLabel* title = nullptr;
    QLabel* m[12] = {};
    int selected_body = -1;
  } body_panel_;

  struct {
    QGroupBox* group_box = nullptr;
    SliderHandle tx, ty, tz, rx, ry, rz;
    QWidget* translation_group = nullptr;
    QWidget* rotation_group = nullptr;
    QDial* translation_gain_dial = nullptr;
    QDial* rotation_gain_dial = nullptr;
    float prismatic_range = 0.2f;
    float revolute_range = 0.2f;
    double rpy_reference[3] = {0.0, 0.0, 0.0}; /*!< roll, pitch, yaw last shown */
  } sliders_;

  struct {
    QGroupBox* group_box = nullptr;
    QLabel* integrator_type = nullptr;
    QDoubleSpinBox* time_span = nullptr;
    QDoubleSpinBox* time_step = nullptr;
    QPushButton* run_button = nullptr;
    QPushButton* stop_button = nullptr;
    QPushButton* reset_button = nullptr;
  } model_parameters_panel_;

  struct {
    QGroupBox* group_box = nullptr;
    QPushButton* play_pause_button = nullptr;
    PlaybackSlider* slider = nullptr;
    QDoubleSpinBox* speed = nullptr;
    QIcon play_icon;
    QIcon pause_icon;
  } playback_panel_;

  QLabel* status_label_ = nullptr;
  QLabel* time_label_ = nullptr;
  QLabel* rate_label_ = nullptr;
  QLabel* fps_label_ = nullptr;

  QTimer* tick_timer_ = nullptr;
  QElapsedTimer tick_clock_;
  int tick_count_ = 0;
  QElapsedTimer fps_clock_;

  bool playing_ = false;
  double playback_time_ = 0.0;
  bool was_computing_ = false;
  SliderHandle* jogging_ = nullptr; /*!< slider currently held */
  QString jog_error_; /*!< why the last joint move failed, if known */
  TargetMove target_move_;
  bool tree_dirty_ = false;
};

} // namespace gui
} // namespace shift

#endif

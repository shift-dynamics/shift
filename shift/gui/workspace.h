#ifndef SHIFT_GUI_WORKSPACE_H_
#define SHIFT_GUI_WORKSPACE_H_

#include "shift/gui/simulation.h"

#include <map>
#include <string>
#include <vector>

#include <json/json.h>

#include <QElapsedTimer>
#include <QMainWindow>
#include <QSize>
#include <QVector3D>

class QAction;
class QCheckBox;
class QComboBox;
class QDial;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class QTabWidget;
class QTimer;
class QTreeWidget;
class QTreeWidgetItem;

namespace shift {
namespace core {
class Substructure;
namespace collision_geometry {
class Base;
}
namespace body {
class Base;
}
class Assembly;
namespace joint {
class Base;
}
namespace parameter {
class List;
class Value;
}
} // namespace core

namespace gui {

class PlaybackSlider;
class VideoTrimmer;
class Viewport;

/*!
@brief Main window, following the layout of the original OpenGL workspace

- left panel, Scene tab: model tree above the parameters of the selected
  object and a context panel for it. The tree follows the nesting of the
  model file: every object read from it (assemblies, bodies, joints,
  constraints, force modules, sensors, controllers, frames, geometry, ...)
  is an item in a folder named by its key, with joint values and mobility
  numbers. The parameters panel lists every parameter the selected object
  declares, read from the object as it is, and edits the selected one: a
  parameter that may change while the model runs is set at once, any other
  is written into the model's JSON and the model is loaded again from it
  (File > Save model as writes it out). Orientations are edited as roll,
  pitch and yaw. Selecting a joint also shows its
  influence coefficients and jog sliders (Tx..Rz, or roll, pitch and yaw for
  the quaternion of a spherical or float joint)
- left panel, Simulation tab: model parameters and Run, and the export of
  the results as a video
- below the tabs: play/pause button and playback slider
- right: 3D viewport
*/
class Workspace : public QMainWindow {
  Q_OBJECT

public:
  /*! one step of a path into the model's JSON: a key, or an array index */
  struct JsonStep {
    std::string key;
    int index = -1;
    bool operator==(const JsonStep& other) const {
      return key == other.key && index == other.index;
    }
  };

  explicit Workspace(QWidget* parent = nullptr);
  ~Workspace() override;

  /*! \brief load a model JSON file, reporting errors in a dialog, or in
   * error if it is given (e.g. when rendering an image without a user) */
  bool Open(const QString& filename, QString* error = nullptr);

  /*! What SaveImage renders */
  struct ImageRequest {
    QString filename;          /*!< image file; the format from its suffix */
    double time = 0.0;         /*!< simulated time shown */
    QSize size{1200, 800};     /*!< pixels */
    bool set_view = false;     /*!< use azimuth and elevation */
    float azimuth = 0.6f;      /*!< radians, see Camera */
    float elevation = 0.35f;   /*!< radians, see Camera */
    bool hide_floor = false;   /*!< hide the floor even if the model shows
                                * it (a model with a terrain hides it) */
    bool bilinear_terrain = false; /*!< View > Smooth deformable terrain
                                      off */
    QString follow_body;       /*!< if given, the camera is placed at the
                                * start and follows this body to time */
    bool follow_ground = false; /*!< follow the ground under the body, see
                                 * Viewport::FollowMode */
    int samples = 0;           /*!< ray traced frames; 0 renders raster */
    bool set_frame = false;    /*!< frame this sphere, not the whole scene */
    QVector3D frame_center;    /*!< of the sphere, inertial frame */
    float frame_radius = 1.f;  /*!< of the sphere */
  };

  /*! \brief render the open model at request.time to an image file and
   * quit the application with exit status 0, or 1 on failure. Simulates
   * up to request.time first if it is after the start. Returns at once;
   * the work is done from the event loop once the viewport can render. */
  void SaveImage(const ImageRequest& request);

  /*! \brief write the model, with the edits made to its parameters, to a
   * JSON file (File > Save model as); the objects of each array keep the
   * order of the model file. Returns false, with why in error, if it could
   * not. */
  bool SaveModel(const QString& filename, QString* error = nullptr);

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
  QWidget* AddCameraBox();
  /*! \brief apply the camera box to the viewport */
  void UpdateFollow();
  QWidget* AddVideoBox();

  /*! \brief load filename, or the model JSON model as if read from it;
   * keep_view leaves the camera and the selection as they were */
  bool LoadModel(const QString& filename, const Json::Value* model,
                 QString* error, bool keep_view);
  /*! \brief load the model again from model_json_, as edited; on failure,
   * load it as it was before the edit, previous, and give why in error */
  bool ReloadModel(const Json::Value& previous, QString& error);
  /*! \brief the file name, marked if the model has been edited */
  void UpdateWindowTitle();
  /*! \brief File > Save model as: write model_json_ */
  void SaveModelAs();
  /*! \brief File > Revert model: load the model file, dropping edits */
  void RevertModel();
  void OpenDialog();
  void About();
  void PopulateTree();
  /*! \brief the item of an object read from the model file, the element of
   * the array key of its parent, with the items of its arrays below it */
  QTreeWidgetItem* AddObjectItem(shift::core::parameter::List* object,
                                 const std::string& key,
                                 QTreeWidgetItem* parent, int depth,
                                 const std::vector<JsonStep>* path);
  /*! \brief below item, a folder for each array of the object that has
   * elements, holding their items, and an item for each object parameter
   * that holds arrays (as model_parameters) */
  void AddObjectChildren(shift::core::parameter::List* object,
                         QTreeWidgetItem* item, int depth,
                         const std::vector<JsonStep>* path);
  /*! \brief the path of an object in model_json_, nullptr if it is not in
   * the model file (as the default floor) */
  const std::vector<JsonStep>* FindObjectPath(const shift::core::parameter::List* object);
  /*! \brief list the parameters of an object in the parameters panel */
  void ShowParameters(shift::core::parameter::List* object,
                      const QString& title);
  /*! \brief the rows of the parameters of object below parent; path is
   * where the object is in model_json_, nullptr if it is not there */
  void AddParameterRows(shift::core::parameter::List* object,
                        QTreeWidgetItem* parent, int depth,
                        const std::vector<JsonStep>* path);
  /*! \brief show the editor of the selected parameter row */
  void SelectParameterRow();
  /*! \brief apply the editor to the selected parameter */
  void ApplyParameterEdit();
  /*! \brief read the shown values again; while the simulation runs, only
   * those of parameters that may change as it runs */
  void UpdateParameterPanel();
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
  void UpdateGeometryPanel();
  /*! \brief set the pose of the selected collision geometry, a plane, from
   * the panel */
  void ApplyGeometryPose();
  /*! \brief show the floor of the fixed body, its first plane, in the
   * viewport */
  void UpdateFloor();
  void LoadModelParameters();

  void RunSimulation();
  void StopSimulation();
  void ResetSimulation();
  void PlayPauseSimulation();
  /*! \brief render the computed results to a video file, frame by frame,
   * at the resolution of the video panel */
  void ExportVideo();
  /*! \brief render the previews of the trimmer from the current camera */
  void RenderThumbnails();
  /*! \brief show the range of the trimmer in the video panel */
  void UpdateVideoRange();
  /*! \brief the results are gone: empty the trimmer */
  void ClearVideoRange();
  /*! \brief show the size of the selected resolution preset */
  void UpdateVideoSize();
  void SetPlaybackTime(double t);
  /*! \brief one step of SaveImage: wait for the viewport and the results,
   * then render; returns true when done */
  bool AdvanceImage();

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
  /*! the tree item of each active joint, nullptr if not in the tree */
  std::vector<QTreeWidgetItem*> joint_items_;

  /* every parameter of the selected object; keys are the aliases of the
  value, the first the one shown and written */
  struct ParameterRow {
    QTreeWidgetItem* item = nullptr;
    shift::core::parameter::Value* value = nullptr;
    std::vector<std::string> keys;
    bool in_file = false; /*!< the object is in model_json_, at path */
    std::vector<JsonStep> path;       /*!< of the object holding the parameter */
  };
  struct {
    QGroupBox* group_box = nullptr;
    QLabel* title = nullptr;
    QTreeWidget* tree = nullptr;
    shift::core::parameter::List* selected = nullptr;
    std::vector<ParameterRow> rows;
    /* the editor of the selected row */
    QWidget* editor = nullptr;
    QLabel* editor_key = nullptr;
    QWidget* editor_fields = nullptr; /*!< rebuilt for each row */
    QLineEdit* line_edit = nullptr;   /*!< a number, string or JSON text */
    QComboBox* combo_box = nullptr;   /*!< a boolean or a string of a set */
    QDoubleSpinBox* angles[3] = {};   /*!< roll, pitch, yaw in degrees */
    QPushButton* apply_button = nullptr;
    QLabel* editor_note = nullptr;
    int editing_row = -1;
  } parameter_panel_;

  /*! \brief set the parameter of row to json in model_json_, replacing
   * the keys of its aliases */
  void SetJsonParameter(const ParameterRow& row, const Json::Value& json);

  /*! the model as read from its file, with the edits made in the viewer */
  Json::Value model_json_;
  bool model_edited_ = false;
  /*! what LoadModel selects again when it keeps the view */
  struct Reselect {
    QString path;          /*!< of the tree item, see PathKey */
    std::string parameter; /*!< key of the parameter row */
    QString follow_body;   /*!< the body the camera follows */
  } reselect_;
  QString model_filename_;
  /*! where each object read from the model file is in model_json_ */
  std::map<const shift::core::parameter::List*, std::vector<JsonStep>> object_paths_;

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

  /* collision geometry of the fixed body; the pose of a plane can be set */
  struct {
    QGroupBox* group_box = nullptr;
    QLabel* title = nullptr;
    QLabel* details = nullptr;
    QCheckBox* contact = nullptr;
    QDoubleSpinBox* translation[3] = {};
    QDoubleSpinBox* rotation[3] = {}; /*!< roll, pitch, yaw in degrees */
    shift::core::collision_geometry::Base* selected = nullptr;
  } geometry_panel_;

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

  /* the camera can follow any body of the model */
  struct {
    QGroupBox* group_box = nullptr;
    QCheckBox* follow = nullptr;
    QComboBox* body = nullptr; /*!< item i is body i of the simulation */
    QComboBox* mode = nullptr; /*!< item i is Viewport::FollowMode i */
  } camera_panel_;

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
    QComboBox* resolution = nullptr; /*!< presets, viewport or custom */
    QSpinBox* width = nullptr;
    QSpinBox* height = nullptr;
    QSpinBox* frame_rate = nullptr;
    QDoubleSpinBox* speed = nullptr; /*!< simulated time per video time */
    QSpinBox* samples = nullptr;     /*!< ray traced frames per video frame */
    VideoTrimmer* trimmer = nullptr; /*!< the part of the results to export */
    QDoubleSpinBox* start = nullptr;
    QDoubleSpinBox* end = nullptr;
    QLabel* range_info = nullptr;
    QPushButton* refresh_button = nullptr;
    QPushButton* export_button = nullptr;
    bool exporting = false;
    double thumbnails_time = -1.0; /*!< computed time they show, -1 if none */
  } video_panel_;

  struct {
    QGroupBox* group_box = nullptr;
    QPushButton* play_pause_button = nullptr;
    PlaybackSlider* slider = nullptr;
    QDoubleSpinBox* speed = nullptr;
    QIcon play_icon;
    QIcon pause_icon;
  } playback_panel_;

  QAction* floor_action_ = nullptr; /*!< View > Floor */
  /*! View > Smooth deformable terrain */
  QAction* smooth_terrain_action_ = nullptr;
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
  ImageRequest image_request_;
  QTimer* image_timer_ = nullptr;
};

} // namespace gui
} // namespace shift

#endif

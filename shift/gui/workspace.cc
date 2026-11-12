#include "shift/gui/workspace.h"

#include "shift/core/assembly.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/collision_geometry/plane.h"
#include "shift/core/context.h"
#include "shift/core/substructure.h"
#include "shift/core/body/base.h"
#include "shift/core/body/fixed.h"
#include "shift/core/body/virtual.h"
#include "shift/core/joint/base.h"
#include "shift/core/light/base.h"
#include "shift/core/parameter/array_base.h"
#include "shift/core/parameter/list.h"
#include "shift/core/parameter/value.h"
#include "shift/core/terrain/base.h"
#include "shift/gui/playback_slider.h"
#include "shift/gui/video_encoder.h"
#include "shift/gui/video_trimmer.h"
#include "shift/gui/viewport.h"

#include "ksl/axis.h"
#include "ksl/mat3x3.h"
#include "ksl/quaternion.h"

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDial>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QDir>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QMessageBox>
#include <QPalette>
#include <QProgressDialog>
#include <QPushButton>
#include <QShortcut>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QStringList>
#include <QStatusBar>
#include <QTabWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <tuple>
#include <stdexcept>


static constexpr int kBodyItem = QTreeWidgetItem::UserType + 1;
static constexpr int kJointItem = QTreeWidgetItem::UserType + 2;
static constexpr int kGeometryItem = QTreeWidgetItem::UserType + 3;
/*! any other object read from the model file */
static constexpr int kObjectItem = QTreeWidgetItem::UserType + 4;
static constexpr int kTickMilliseconds = 16;

/*! gain dial 0..100 maps to a slider rate multiplier of 0.01..100 */
static double DialGain(const QDial* dial) {
  return std::pow(10.0, (dial->value() - 50) / 25.0);
}

static QString Number(double value, int decimals = 4) {
  return QString::number(value, 'f', decimals);
}


shift::gui::Workspace::Workspace(QWidget* parent)
  : QMainWindow(parent) {
  setWindowTitle("Shift");
  resize(1400, 860);
  InitializeLayout();
  InitializeMenus();

  tick_timer_ = new QTimer(this);
  connect(tick_timer_, &QTimer::timeout, this, &Workspace::Tick);
  tick_timer_->start(kTickMilliseconds);
  tick_clock_.start();
  fps_clock_.start();
  UpdateStatus();
}

shift::gui::Workspace::~Workspace() {
  simulation_.Stop();
  viewport_->set_simulation(nullptr);
}

// ------------------------------------------------------------------ layout

void shift::gui::Workspace::InitializeMenus() {
  QMenu* file_menu = menuBar()->addMenu("&File");
  QAction* open_action = file_menu->addAction("&Open…");
  open_action->setShortcut(QKeySequence::Open);
  connect(open_action, &QAction::triggered, this, &Workspace::OpenDialog);
  QAction* save_action = file_menu->addAction("&Save model as…");
  save_action->setShortcut(QKeySequence::SaveAs);
  save_action->setToolTip("write the model, with the edits made to its "
                          "parameters, to a JSON file");
  connect(save_action, &QAction::triggered, this, &Workspace::SaveModelAs);
  QAction* revert_action = file_menu->addAction("&Revert model");
  revert_action->setToolTip("drop the edits and load the model file again");
  connect(revert_action, &QAction::triggered, this, &Workspace::RevertModel);
  file_menu->addSeparator();
  QAction* quit_action = file_menu->addAction("&Quit");
  quit_action->setShortcut(QKeySequence::Quit);
  connect(quit_action, &QAction::triggered, this, &QWidget::close);

  QMenu* view_menu = menuBar()->addMenu("&View");
  QAction* ray_traced = view_menu->addAction("&Ray traced");
  ray_traced->setCheckable(true);
  ray_traced->setShortcut(Qt::CTRL | Qt::Key_T);
  connect(ray_traced, &QAction::toggled, this, [this](bool checked) {
    viewport_->set_render_mode(checked ? Viewport::RenderMode::kRayTraced
                                       : Viewport::RenderMode::kRaster);
  });
  QAction* floor = view_menu->addAction(QIcon(":/icons/floor.png"), "&Floor");
  floor->setCheckable(true);
  floor->setChecked(true);
  connect(floor, &QAction::toggled, viewport_, &Viewport::set_floor_enabled);
  floor_action_ = floor;
  QAction* smooth_terrain = view_menu->addAction("&Smooth deformable terrain");
  smooth_terrain->setCheckable(true);
  smooth_terrain->setChecked(true);
  smooth_terrain->setToolTip("draw SCM terrain by a Catmull-Rom surface "
                             "through its nodes; unchecked, bilinear between "
                             "them as the soil models see it");
  connect(smooth_terrain, &QAction::toggled, viewport_,
          &Viewport::set_smooth_terrain);
  smooth_terrain_action_ = smooth_terrain;
  view_menu->addSeparator();
  QAction* frame = view_menu->addAction("&Frame model");
  frame->setShortcut(Qt::Key_F);
  connect(frame, &QAction::triggered, viewport_, &Viewport::FrameScene);

  QMenu* help_menu = menuBar()->addMenu("&Help");
  QAction* about = help_menu->addAction("&About");
  connect(about, &QAction::triggered, this, &Workspace::About);
}

void shift::gui::Workspace::InitializeLeftPanel() {
  tab_widget_ = new QTabWidget;

  // scene tab: model tree above a context dependent panel
  auto* scene_splitter = new QSplitter(Qt::Vertical);
  scene_splitter->setHandleWidth(1);
  tab_widget_->addTab(scene_splitter, QIcon(":/icons/scene.png"), "Scene");

  tree_widget_ = new QTreeWidget;
  tree_widget_->setIconSize(QSize(24, 24));
  tree_widget_->setAlternatingRowColors(true);
  tree_widget_->setColumnCount(3);
  tree_widget_->setHeaderLabels({"", "value", "mobility"});
  // the names take the width the value and mobility columns leave
  tree_widget_->header()->setStretchLastSection(false);
  tree_widget_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  tree_widget_->setColumnWidth(1, 70);
  tree_widget_->setColumnWidth(2, 70);
  scene_splitter->addWidget(tree_widget_);
  connect(tree_widget_, &QTreeWidget::itemSelectionChanged, this,
          &Workspace::SelectTreeItem);

  // parameters panel: every parameter of the selected object
  auto& parameters = parameter_panel_;
  parameters.group_box = new QGroupBox;
  auto* parameters_layout = new QVBoxLayout(parameters.group_box);
  parameters.title = new QLabel;
  parameters.title->setStyleSheet("font-weight: bold");
  parameters_layout->addWidget(parameters.title);
  parameters.tree = new QTreeWidget;
  parameters.tree->setColumnCount(2);
  parameters.tree->setHeaderLabels({"parameter", "value"});
  parameters.tree->setColumnWidth(0, 180);
  parameters.tree->setAlternatingRowColors(true);
  parameters.tree->setToolTip(QString());
  parameters_layout->addWidget(parameters.tree);
  connect(parameters.tree, &QTreeWidget::itemSelectionChanged, this,
          &Workspace::SelectParameterRow);

  // the editor of the selected parameter
  parameters.editor = new QWidget;
  auto* editor_layout = new QVBoxLayout(parameters.editor);
  editor_layout->setContentsMargins(0, 0, 0, 0);
  auto* editor_row = new QHBoxLayout;
  parameters.editor_key = new QLabel;
  parameters.editor_key->setStyleSheet("font-weight: bold");
  editor_row->addWidget(parameters.editor_key);
  parameters.editor_fields = new QWidget;
  editor_row->addWidget(parameters.editor_fields, 1);
  parameters.apply_button = new QPushButton("Apply");
  connect(parameters.apply_button, &QPushButton::clicked, this,
          &Workspace::ApplyParameterEdit);
  editor_row->addWidget(parameters.apply_button);
  editor_layout->addLayout(editor_row);
  // below the fields, so that a long message wraps to its full height
  parameters.editor_note = new QLabel;
  parameters.editor_note->setWordWrap(true);
  editor_layout->addWidget(parameters.editor_note);
  parameters.editor->setVisible(false);
  parameters_layout->addWidget(parameters.editor);
  parameters.group_box->setVisible(false);
  scene_splitter->addWidget(parameters.group_box);

  auto* dynamic_box = new QWidget;
  auto* dynamic_layout = new QVBoxLayout(dynamic_box);
  dynamic_layout->setContentsMargins(0, 0, 0, 0);
  dynamic_layout->setSpacing(0);

  // joint panel: name and influence coefficients
  joint_panel_.group_box = new QGroupBox;
  auto* joint_layout = new QGridLayout(joint_panel_.group_box);
  joint_panel_.title = new QLabel;
  joint_panel_.title->setStyleSheet("font-weight: bold");
  joint_layout->addWidget(joint_panel_.title, 0, 0, 1, 6);
  joint_panel_.details = new QLabel;
  joint_panel_.details->setWordWrap(true);
  joint_layout->addWidget(joint_panel_.details, 1, 0, 1, 6);
  const char* h_names[6] = {"Tx", "Ty", "Tz", "Rx", "Ry", "Rz"};
  joint_layout->addWidget(new QLabel("influence coefficients h"), 2, 0, 1, 6);
  for(int i = 0; i < 6; i++) {
    joint_layout->addWidget(new QLabel(h_names[i]), 3, i);
    joint_panel_.h[i] = new QLabel("0.000");
    joint_layout->addWidget(joint_panel_.h[i], 4, i);
  }
  joint_panel_.group_box->setVisible(false);
  dynamic_layout->addWidget(joint_panel_.group_box);

  dynamic_layout->addWidget(AddSliders());

  // body panel: name and origin pose
  body_panel_.group_box = new QGroupBox;
  auto* body_layout = new QGridLayout(body_panel_.group_box);
  body_panel_.title = new QLabel;
  body_panel_.title->setStyleSheet("font-weight: bold");
  body_layout->addWidget(body_panel_.title, 0, 0, 1, 4);
  body_layout->addWidget(new QLabel("origin pose"), 1, 0, 1, 4);
  for(int row = 0; row < 3; row++) {
    for(int column = 0; column < 4; column++) {
      body_panel_.m[4 * row + column] = new QLabel("0.000");
      body_layout->addWidget(body_panel_.m[4 * row + column], 2 + row, column);
    }
  }
  body_panel_.group_box->setVisible(false);
  dynamic_layout->addWidget(body_panel_.group_box);

  // collision geometry panel
  auto& geometry = geometry_panel_;
  geometry.group_box = new QGroupBox;
  auto* geometry_layout = new QGridLayout(geometry.group_box);
  geometry.title = new QLabel;
  geometry.title->setStyleSheet("font-weight: bold");
  geometry_layout->addWidget(geometry.title, 0, 0, 1, 4);
  geometry.details = new QLabel;
  geometry.details->setWordWrap(true);
  geometry_layout->addWidget(geometry.details, 1, 0, 1, 4);
  geometry.contact = new QCheckBox("contact");
  geometry.contact->setToolTip(
    "test this geometry for contact; a default floor starts without contact");
  geometry_layout->addWidget(geometry.contact, 2, 0, 1, 4);
  connect(geometry.contact, &QCheckBox::toggled, this, [this](bool checked) {
    if(geometry_panel_.selected && !simulation_.is_computing()) {
      geometry_panel_.selected->set_contact_enabled(checked);
    }
  });
  geometry_layout->addWidget(new QLabel("pose: translation"), 3, 0, 1, 4);
  const char* translation_names[3] = {"x", "y", "z"};
  const char* rotation_names[3] = {"roll", "pitch", "yaw"};
  for(int i = 0; i < 3; i++) {
    geometry_layout->addWidget(new QLabel(translation_names[i]), 4, i);
    geometry.translation[i] = new QDoubleSpinBox;
    geometry.translation[i]->setRange(-1e6, 1e6);
    geometry.translation[i]->setDecimals(4);
    geometry.translation[i]->setSingleStep(0.01);
    geometry_layout->addWidget(geometry.translation[i], 5, i);
  }
  geometry_layout->addWidget(new QLabel("pose: rotation (degrees)"), 6, 0, 1,
                             4);
  for(int i = 0; i < 3; i++) {
    geometry_layout->addWidget(new QLabel(rotation_names[i]), 7, i);
    geometry.rotation[i] = new QDoubleSpinBox;
    geometry.rotation[i]->setRange(-180.0, 180.0);
    geometry.rotation[i]->setDecimals(2);
    geometry.rotation[i]->setSingleStep(1.0);
    geometry.rotation[i]->setWrapping(true);
    geometry_layout->addWidget(geometry.rotation[i], 8, i);
  }
  for(int i = 0; i < 3; i++) {
    for(auto spin_box : {geometry.translation[i], geometry.rotation[i]}) {
      connect(spin_box, &QDoubleSpinBox::valueChanged, this,
              &Workspace::ApplyGeometryPose);
    }
  }
  geometry.group_box->setVisible(false);
  dynamic_layout->addWidget(geometry.group_box);

  dynamic_layout->addStretch();
  scene_splitter->addWidget(dynamic_box);

  // simulation tab: model parameters
  auto* simulation_box = new QWidget;
  auto* simulation_layout = new QVBoxLayout(simulation_box);
  simulation_layout->setContentsMargins(0, 0, 0, 0);
  simulation_layout->addWidget(AddModelParametersBox());
  simulation_layout->addWidget(AddCameraBox());
  simulation_layout->addWidget(AddVideoBox());
  simulation_layout->addStretch();
  tab_widget_->addTab(simulation_box, QIcon(":/icons/clapper.png"),
                      "Simulation");
}

QWidget* shift::gui::Workspace::AddSliders() {
  sliders_.group_box = new QGroupBox;
  auto* grid = new QGridLayout(sliders_.group_box);
  grid->setContentsMargins(0, 0, 0, 0);
  grid->setVerticalSpacing(0);

  // translation sliders and gain
  sliders_.translation_group = new QWidget;
  auto* translation_layout = new QVBoxLayout(sliders_.translation_group);
  translation_layout->setContentsMargins(0, 0, 0, 0);
  translation_layout->setSpacing(0);
  sliders_.translation_gain_dial = new QDial;
  sliders_.translation_gain_dial->setRange(0, 100);
  sliders_.translation_gain_dial->setValue(50);
  sliders_.translation_gain_dial->setMaximumSize(QSize(60, 60));
  sliders_.translation_gain_dial->setToolTip("slider gain");
  double single_step = sliders_.prismatic_range / 100.0;
  for(auto handle : {&sliders_.tx, &sliders_.ty, &sliders_.tz}) {
    AddSlider(*handle, translation_layout, single_step,
              sliders_.translation_gain_dial);
  }
  grid->addWidget(sliders_.translation_group, 0, 0);
  grid->addWidget(sliders_.translation_gain_dial, 0, 1);

  // rotation sliders and gain
  sliders_.rotation_group = new QWidget;
  auto* rotation_layout = new QVBoxLayout(sliders_.rotation_group);
  rotation_layout->setContentsMargins(0, 0, 0, 0);
  rotation_layout->setSpacing(0);
  sliders_.rotation_gain_dial = new QDial;
  sliders_.rotation_gain_dial->setRange(0, 100);
  sliders_.rotation_gain_dial->setValue(50);
  sliders_.rotation_gain_dial->setMaximumSize(QSize(60, 60));
  sliders_.rotation_gain_dial->setToolTip("slider gain");
  single_step = sliders_.revolute_range / 100.0;
  for(auto handle : {&sliders_.rx, &sliders_.ry, &sliders_.rz}) {
    AddSlider(*handle, rotation_layout, single_step,
              sliders_.rotation_gain_dial);
  }
  grid->addWidget(sliders_.rotation_group, 1, 0);
  grid->addWidget(sliders_.rotation_gain_dial, 1, 1);

  sliders_.group_box->setVisible(false);
  return sliders_.group_box;
}

void shift::gui::Workspace::AddSlider(SliderHandle& handle, QLayout* layout,
                                      double single_step, QDial* gain_dial) {
  handle.single_step = single_step;
  handle.gain_dial = gain_dial;
  handle.box = new QGroupBox;
  auto* slider_layout = new QGridLayout(handle.box);
  slider_layout->setContentsMargins(4, 0, 4, 0);
  slider_layout->setVerticalSpacing(0);

  handle.label = new QLabel;
  handle.label->setMinimumWidth(24);
  slider_layout->addWidget(handle.label, 0, 0);

  handle.slider = new QSlider(Qt::Horizontal);
  handle.slider->setRange(-100, 100);
  handle.slider->setSingleStep(1);
  handle.slider->setPageStep(5);
  handle.slider->setValue(0);
  handle.slider->setFocusPolicy(Qt::NoFocus);
  slider_layout->addWidget(handle.slider, 0, 1);

  handle.spin_box = new QDoubleSpinBox;
  handle.spin_box->setDecimals(4);
  handle.spin_box->setRange(-1e7, 1e7);
  handle.spin_box->setKeyboardTracking(false);
  handle.spin_box->setSingleStep(0.01);
  slider_layout->addWidget(handle.spin_box, 0, 2);

  // pressing the slider removes constraint violations and sets up the
  // factorization, holding it off center jogs the coordinate, releasing it
  // recenters the slider
  SliderHandle* h = &handle;
  connect(handle.slider, &QSlider::sliderPressed, this,
          [this, h]() { PressSlider(*h); });
  connect(handle.slider, &QSlider::sliderReleased, this,
          [this, h]() { RecenterSlider(*h); });
  connect(handle.spin_box, &QDoubleSpinBox::valueChanged, this,
          [this, h](double value) { SetTarget(*h, value); });

  layout->addWidget(handle.box);
  handle.box->setVisible(false);
}

QWidget* shift::gui::Workspace::AddModelParametersBox() {
  auto& panel = model_parameters_panel_;
  panel.group_box = new QGroupBox;
  auto* layout = new QGridLayout(panel.group_box);

  layout->addWidget(new QLabel("Integrator Type"), 0, 0);
  panel.integrator_type = new QLabel;
  layout->addWidget(panel.integrator_type, 0, 1);

  layout->addWidget(new QLabel("Time Span"), 1, 0);
  panel.time_span = new QDoubleSpinBox;
  panel.time_span->setDecimals(4);
  panel.time_span->setRange(0, 1e7);
  panel.time_span->setKeyboardTracking(false);
  panel.time_span->setSingleStep(0.01);
  layout->addWidget(panel.time_span, 1, 1);
  connect(panel.time_span, &QDoubleSpinBox::valueChanged, this,
          [this](double value) {
            simulation_.set_end_time(value);
            playback_panel_.slider->set_time_span(0.0,
                                                  simulation_.get_end_time());
          });

  layout->addWidget(new QLabel("Time Step"), 2, 0);
  panel.time_step = new QDoubleSpinBox;
  panel.time_step->setDecimals(4);
  panel.time_step->setRange(0.0001, 10);
  panel.time_step->setKeyboardTracking(false);
  panel.time_step->setSingleStep(0.001);
  layout->addWidget(panel.time_step, 2, 1);
  connect(panel.time_step, &QDoubleSpinBox::valueChanged, this,
          [this](double value) { simulation_.set_time_step(value); });

  panel.run_button = new QPushButton("&Run");
  layout->addWidget(panel.run_button, 3, 0, 1, 2);
  connect(panel.run_button, &QPushButton::clicked, this,
          &Workspace::RunSimulation);
  auto* run_shortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_R), this);
  connect(run_shortcut, &QShortcut::activated, this,
          &Workspace::RunSimulation);

  panel.stop_button = new QPushButton("Stop");
  layout->addWidget(panel.stop_button, 4, 0);
  connect(panel.stop_button, &QPushButton::clicked, this,
          &Workspace::StopSimulation);

  panel.reset_button = new QPushButton("Reset");
  panel.reset_button->setToolTip(
    "discard results and return to the initial conditions of the model file");
  layout->addWidget(panel.reset_button, 4, 1);
  connect(panel.reset_button, &QPushButton::clicked, this,
          &Workspace::ResetSimulation);

  panel.group_box->setVisible(false);
  return panel.group_box;
}

QWidget* shift::gui::Workspace::AddCameraBox() {
  auto& camera = camera_panel_;
  camera.group_box = new QGroupBox("Camera");
  auto* camera_layout = new QHBoxLayout(camera.group_box);
  camera.follow = new QCheckBox("follow");
  camera.follow->setToolTip(
    "keep the camera where it is relative to the body: it moves and turns "
    "with the body, and orbiting, panning and zooming still work");
  camera.body = new QComboBox;
  camera.body->setToolTip("the body to follow; selecting a body in the tree "
                          "picks it here");
  camera.mode = new QComboBox;
  camera.mode->addItems({"body", "ground"});
  camera.mode->setToolTip(
    "body: move and turn with the body. ground: stay level, at the height of "
    "the ground under the body and turning with its heading, so the body is "
    "seen to bounce and tilt");
  camera_layout->addWidget(camera.follow);
  camera_layout->addWidget(camera.body, 1);
  camera_layout->addWidget(camera.mode);
  connect(camera.follow, &QCheckBox::toggled, this, &Workspace::UpdateFollow);
  connect(camera.body, &QComboBox::currentIndexChanged, this,
          &Workspace::UpdateFollow);
  connect(camera.mode, &QComboBox::currentIndexChanged, this,
          &Workspace::UpdateFollow);
  camera.group_box->setVisible(false);
  return camera.group_box;
}

void shift::gui::Workspace::UpdateFollow() {
  const auto& camera = camera_panel_;
  if(!camera.follow->isChecked()) {
    if(viewport_->get_follow_body() >= 0) {
      viewport_->set_follow_body(-1);
    }
    return;
  }
  viewport_->set_follow_body(camera.body->currentIndex(),
                             camera.mode->currentIndex() == 1
                               ? Viewport::FollowMode::kGround
                               : Viewport::FollowMode::kBody);
}

QWidget* shift::gui::Workspace::AddVideoBox() {
  auto& panel = video_panel_;
  panel.group_box = new QGroupBox("Video");
  auto* layout = new QGridLayout(panel.group_box);

  layout->addWidget(new QLabel("Resolution"), 0, 0);
  panel.resolution = new QComboBox;
  panel.resolution->addItem("1280 × 720 (720p)", QSize(1280, 720));
  panel.resolution->addItem("1920 × 1080 (1080p)", QSize(1920, 1080));
  panel.resolution->addItem("2560 × 1440 (1440p)", QSize(2560, 1440));
  panel.resolution->addItem("3840 × 2160 (4K)", QSize(3840, 2160));
  panel.resolution->addItem("Viewport", QSize());
  panel.resolution->addItem("Custom", QSize(-1, -1));
  panel.resolution->setCurrentIndex(1);
  layout->addWidget(panel.resolution, 0, 1, 1, 3);

  /* 4:2:0 video needs an even width and height */
  auto make_size = [](int value) {
    auto* spin_box = new QSpinBox;
    spin_box->setRange(16, 16384);
    spin_box->setSingleStep(2);
    spin_box->setValue(value);
    spin_box->setSuffix(" px");
    return spin_box;
  };
  layout->addWidget(new QLabel("Size"), 1, 0);
  panel.width = make_size(1920);
  layout->addWidget(panel.width, 1, 1);
  layout->addWidget(new QLabel("×"), 1, 2);
  panel.height = make_size(1080);
  layout->addWidget(panel.height, 1, 3);
  connect(panel.resolution, &QComboBox::currentIndexChanged, this,
          &Workspace::UpdateVideoSize);

  layout->addWidget(new QLabel("Frame Rate"), 2, 0);
  panel.frame_rate = new QSpinBox;
  panel.frame_rate->setRange(1, 240);
  panel.frame_rate->setValue(30);
  panel.frame_rate->setSuffix(" fps");
  layout->addWidget(panel.frame_rate, 2, 1, 1, 3);

  layout->addWidget(new QLabel("Speed"), 3, 0);
  panel.speed = new QDoubleSpinBox;
  panel.speed->setRange(0.001, 1000.0);
  panel.speed->setDecimals(3);
  panel.speed->setValue(1.0);
  panel.speed->setSuffix("×");
  panel.speed->setToolTip(
    "simulated time per second of video: 1 is real time, 0.1 slow motion");
  layout->addWidget(panel.speed, 3, 1, 1, 3);

  layout->addWidget(new QLabel("Ray Traced Samples"), 4, 0);
  panel.samples = new QSpinBox;
  panel.samples->setRange(1, 1024);
  panel.samples->setValue(16);
  panel.samples->setToolTip(
    "ray traced frames rendered and averaged for each video frame; the "
    "raster renderer takes one");
  layout->addWidget(panel.samples, 4, 1, 1, 3);

  panel.trimmer = new VideoTrimmer;
  panel.trimmer->setToolTip(
    "drag the handles to choose the part to export; the view shows the frame "
    "under a handle or on the strip as you drag");
  layout->addWidget(panel.trimmer, 5, 0, 1, 4);
  auto make_time = [](const char* tip) {
    auto* spin_box = new QDoubleSpinBox;
    spin_box->setDecimals(3);
    spin_box->setSuffix(" s");
    spin_box->setToolTip(tip);
    return spin_box;
  };
  layout->addWidget(new QLabel("Start"), 6, 0);
  panel.start = make_time("simulated time of the first frame");
  layout->addWidget(panel.start, 6, 1);
  layout->addWidget(new QLabel("End"), 6, 2);
  panel.end = make_time("simulated time of the last frame");
  layout->addWidget(panel.end, 6, 3);
  panel.range_info = new QLabel;
  layout->addWidget(panel.range_info, 7, 0, 1, 4);
  panel.refresh_button = new QPushButton("Refresh Preview");
  panel.refresh_button->setToolTip(
    "render the previews again from the current camera");
  layout->addWidget(panel.refresh_button, 8, 0, 1, 4);
  connect(panel.refresh_button, &QPushButton::clicked, this,
          &Workspace::RenderThumbnails);
  connect(panel.trimmer, &VideoTrimmer::range_changed, this,
          &Workspace::UpdateVideoRange);
  connect(panel.trimmer, &VideoTrimmer::scrubbed, this, [this](double t) {
    SetPlaybackTime(t);
    playback_panel_.slider->set_time(t);
  });
  connect(panel.start, &QDoubleSpinBox::valueChanged, this, [this](double t) {
    video_panel_.trimmer->set_range(t, video_panel_.trimmer->get_end());
  });
  connect(panel.end, &QDoubleSpinBox::valueChanged, this, [this](double t) {
    video_panel_.trimmer->set_range(video_panel_.trimmer->get_start(), t);
  });
  connect(panel.frame_rate, &QSpinBox::valueChanged, this,
          &Workspace::UpdateVideoRange);
  connect(panel.speed, &QDoubleSpinBox::valueChanged, this,
          &Workspace::UpdateVideoRange);

  panel.export_button = new QPushButton("Export Video…");
  panel.export_button->setToolTip(
    "render the computed results to a video file (.mp4, .mkv, .mov, .webm)");
  layout->addWidget(panel.export_button, 9, 0, 1, 4);
  connect(panel.export_button, &QPushButton::clicked, this,
          &Workspace::ExportVideo);

  UpdateVideoSize();
  UpdateVideoRange();
  panel.group_box->setVisible(false);
  return panel.group_box;
}

void shift::gui::Workspace::UpdateVideoRange() {
  auto& panel = video_panel_;
  const double start = panel.trimmer->get_start();
  const double end = panel.trimmer->get_end();
  for(auto* spin_box : {panel.start, panel.end}) {
    spin_box->blockSignals(true);
    spin_box->setRange(0.0, panel.trimmer->get_span());
  }
  panel.start->setValue(start);
  panel.end->setValue(end);
  for(auto* spin_box : {panel.start, panel.end}) {
    spin_box->blockSignals(false);
  }
  const double speed = panel.speed->value();
  const int frame_rate = panel.frame_rate->value();
  const int frames = VideoTrimmer::FrameCount(start, end, speed, frame_rate);
  panel.range_info->setText(panel.trimmer->get_span() > 0.0
                              ? QString("%1 frames, %2 s of video")
                                  .arg(frames)
                                  .arg(double(frames) / frame_rate, 0, 'f', 2)
                              : QString("Run the model to choose a range"));
}

void shift::gui::Workspace::ClearVideoRange() {
  video_panel_.thumbnails_time = -1.0;
  video_panel_.trimmer->Clear();
  UpdateVideoRange();
}

void shift::gui::Workspace::RenderThumbnails() {
  auto& panel = video_panel_;
  if(!simulation_.has_results() || simulation_.is_computing() ||
     panel.exporting) {
    return;
  }
  // enough for a preview every few pixels of the strip; raster, because a
  // single ray traced frame is noisy
  constexpr int kThumbnails = 32;
  constexpr int kHeight = 108;
  const int width = std::max(
    16, (kHeight * panel.width->value() / std::max(panel.height->value(), 1)) &
          ~1);
  const QSize size(width, kHeight);
  QString error;
  const Viewport::RenderMode mode = viewport_->get_render_mode();
  if(!viewport_->BeginOffscreen(size, error)) {
    return; // the window is not exposed yet: the next tick tries again
  }
  if(mode != Viewport::RenderMode::kRaster) {
    viewport_->set_render_mode(Viewport::RenderMode::kRaster);
  }
  const double computed = simulation_.get_computed_time();
  std::vector<QImage> thumbnails;
  std::vector<uint8_t> rgba;
  for(int i = 0; i < kThumbnails; i++) {
    simulation_.ShowTime(computed * i / (kThumbnails - 1));
    if(!viewport_->RenderOffscreen(1, rgba)) {
      break;
    }
    thumbnails.push_back(QImage(rgba.data(), size.width(), size.height(),
                                4 * size.width(), QImage::Format_RGBA8888)
                           .convertToFormat(QImage::Format_RGB888));
  }
  viewport_->EndOffscreen();
  if(mode != Viewport::RenderMode::kRaster) {
    viewport_->set_render_mode(mode);
  }
  simulation_.ShowTime(playback_time_);
  viewport_->update();
  panel.trimmer->set_thumbnails(std::move(thumbnails));
  panel.thumbnails_time = computed;
}

void shift::gui::Workspace::UpdateVideoSize() {
  auto& panel = video_panel_;
  QSize size = panel.resolution->currentData().toSize();
  const bool custom = size == QSize(-1, -1);
  if(size.isEmpty() && !custom) { // the viewport's pixels, rounded down to even
    const QSize pixels = viewport_ ? viewport_->size() * viewport_->devicePixelRatio()
                                   : QSize(1920, 1080);
    size = QSize(std::max(16, pixels.width() & ~1),
                 std::max(16, pixels.height() & ~1));
  }
  if(!custom) {
    panel.width->setValue(size.width());
    panel.height->setValue(size.height());
  }
  panel.width->setEnabled(custom);
  panel.height->setEnabled(custom);
}

void shift::gui::Workspace::ExportVideo() {
  auto& panel = video_panel_;
  if(!simulation_.has_results() || simulation_.is_computing() ||
     panel.exporting) {
    return;
  }
  UpdateVideoSize(); // the viewport may have been resized
  const QSize size(panel.width->value() & ~1, panel.height->value() & ~1);
  const int frame_rate = panel.frame_rate->value();
  const double speed = panel.speed->value();
  const double start = panel.trimmer->get_start();
  const double end = panel.trimmer->get_end();
  const int frames = VideoTrimmer::FrameCount(start, end, speed, frame_rate);

  const QString base = QFileInfo(QString::fromStdString(simulation_.get_filename()))
                         .completeBaseName();
  QString filename = QFileDialog::getSaveFileName(
    this, "Export Video", base + ".mp4",
    "Video (*.mp4 *.mkv *.mov *.webm *.avi);;All files (*)");
  if(filename.isEmpty()) {
    return;
  }
  if(QFileInfo(filename).suffix().isEmpty()) {
    filename += ".mp4";
  }

  playing_ = false;
  playback_panel_.play_pause_button->setIcon(playback_panel_.play_icon);
  QString error;
  if(!viewport_->BeginOffscreen(size, error)) {
    QMessageBox::warning(this, "Export failed", "Could not render the video: " + error);
    return;
  }
  panel.exporting = true;
  panel.export_button->setEnabled(false);

  QProgressDialog progress(QString("Rendering %1 × %2 video…")
                             .arg(size.width())
                             .arg(size.height()),
                           "Cancel", 0, frames, this);
  progress.setWindowModality(Qt::WindowModal);
  progress.setMinimumDuration(0);
  bool cancelled = false;
  VideoEncoder encoder;
  std::vector<uint8_t> rgba;
  try {
    encoder.Open(filename.toStdString(), size.width(), size.height(),
                 frame_rate);
    for(int i = 0; i < frames; i++) {
      progress.setValue(i); // processes events, e.g. Cancel
      if(progress.wasCanceled()) {
        cancelled = true;
        break;
      }
      simulation_.ShowTime(
        VideoTrimmer::FrameTime(i, start, end, speed, frame_rate));
      if(!viewport_->RenderOffscreen(panel.samples->value(), rgba)) {
        throw std::runtime_error("could not render frame " + std::to_string(i));
      }
      encoder.Write(rgba.data(), 4 * size.width());
    }
    encoder.Close();
  } catch(const std::exception& e) {
    error = e.what();
  }
  progress.setValue(frames);
  viewport_->EndOffscreen();
  simulation_.ShowTime(playback_time_);
  viewport_->update();
  panel.exporting = false;
  UpdateStatus();

  if(!error.isEmpty()) {
    QMessageBox::warning(this, "Export failed",
                         QString("Could not write %1:\n%2").arg(filename, error));
  } else if(cancelled) {
    status_label_->setText("Video export cancelled; the frames rendered were saved");
  } else {
    status_label_->setText(QString("Saved %1 frames to %2 (%3)")
                             .arg(frames)
                             .arg(filename)
                             .arg(QString::fromStdString(encoder.get_encoder_name())));
  }
}

void shift::gui::Workspace::InitializeLayout() {
  InitializeLeftPanel();

  viewport_ = new Viewport;
  auto* viewport_container = QWidget::createWindowContainer(viewport_);
  viewport_container->setMinimumSize(200, 200);
  viewport_container->setFocusPolicy(Qt::StrongFocus);
  QSizePolicy policy = viewport_container->sizePolicy();
  policy.setHorizontalStretch(3);
  viewport_container->setSizePolicy(policy);

  // playback panel below the tabs
  auto& playback = playback_panel_;
  playback.group_box = new QGroupBox;
  auto* playback_layout = new QGridLayout(playback.group_box);
  playback.play_icon = QIcon(":/icons/play_button.png");
  playback.pause_icon = QIcon(":/icons/pause_button.png");
  playback.play_pause_button = new QPushButton;
  playback.play_pause_button->setIcon(playback.play_icon);
  playback.play_pause_button->setToolTip("play / pause (space)");
  playback.play_pause_button->setShortcut(Qt::Key_Space);
  connect(playback.play_pause_button, &QPushButton::clicked, this,
          &Workspace::PlayPauseSimulation);
  playback_layout->addWidget(playback.play_pause_button, 0, 0);
  playback.slider = new PlaybackSlider;
  connect(playback.slider, &PlaybackSlider::time_changed, this,
          &Workspace::SetPlaybackTime);
  playback_layout->addWidget(playback.slider, 0, 1);
  playback_layout->setColumnStretch(1, 2);
  playback.speed = new QDoubleSpinBox;
  playback.speed->setRange(0.01, 1000.0);
  playback.speed->setValue(1.0);
  playback.speed->setDecimals(2);
  playback.speed->setSuffix("×");
  playback.speed->setToolTip("playback speed relative to real time");
  playback_layout->addWidget(playback.speed, 0, 2);

  auto* left_panel = new QWidget;
  auto* left_layout = new QVBoxLayout(left_panel);
  left_layout->setContentsMargins(0, 0, 0, 0);
  left_layout->setSpacing(0);
  left_layout->addWidget(tab_widget_);
  left_layout->addWidget(playback.group_box);

  auto* main_splitter = new QSplitter;
  main_splitter->setHandleWidth(1);
  main_splitter->setOpaqueResize(true);
  main_splitter->addWidget(left_panel);
  main_splitter->addWidget(viewport_container);
  main_splitter->setStretchFactor(1, 3);
  main_splitter->setSizes({420, 980});
  setCentralWidget(main_splitter);

  status_label_ = new QLabel;
  time_label_ = new QLabel;
  rate_label_ = new QLabel;
  fps_label_ = new QLabel;
  statusBar()->addWidget(status_label_, 1);
  statusBar()->addPermanentWidget(time_label_);
  statusBar()->addPermanentWidget(rate_label_);
  statusBar()->addPermanentWidget(fps_label_);
}

// -------------------------------------------------------------- model file

bool shift::gui::Workspace::Open(const QString& filename,
                                 QString* error_message) {
  return LoadModel(filename, nullptr, error_message, false);
}

static constexpr int kPathRole = Qt::UserRole + 3; /*!< QString, see PathKey */

bool shift::gui::Workspace::LoadModel(const QString& filename,
                                      const Json::Value* model,
                                      QString* error_message, bool keep_view) {
  // what to select again in the model loaded again; after a load that
  // failed (the tree is empty), what was selected before it
  auto& reselect = reselect_;
  if(!keep_view) {
    reselect = Reselect();
  } else if(tree_widget_->topLevelItemCount() > 0) {
    reselect = Reselect();
    const auto items = tree_widget_->selectedItems();
    if(!items.isEmpty()) {
      reselect.path = items.front()->data(0, kPathRole).toString();
    }
    auto& panel = parameter_panel_;
    if(panel.editing_row >= 0 &&
       panel.editing_row < static_cast<int>(panel.rows.size())) {
      reselect.parameter = panel.rows[panel.editing_row].keys.front();
    }
    if(camera_panel_.follow->isChecked()) {
      reselect.follow_body = camera_panel_.body->currentText();
    }
  }

  QApplication::setOverrideCursor(Qt::WaitCursor);
  viewport_->set_simulation(nullptr, !keep_view);
  playing_ = false;
  jogging_ = nullptr;
  target_move_ = TargetMove();
  ClearSliders();
  ShowParameters(nullptr, QString());
  tree_widget_->clear();
  joint_items_.clear();
  object_paths_.clear();
  geometry_panel_.selected = nullptr;

  std::string error;
  try {
    if(model) {
      simulation_.Reload(*model);
    } else {
      simulation_.Load(filename.toStdString());
    }
  } catch(std::exception& e) {
    error = e.what();
  }
  QApplication::restoreOverrideCursor();
  if(!error.empty()) {
    const QString message = QString("Could not load %1:\n%2")
                              .arg(filename, QString::fromStdString(error));
    if(error_message) {
      *error_message = message;
    } else {
      QMessageBox::critical(this, "Open failed", message);
    }
    model_parameters_panel_.group_box->setVisible(false);
    camera_panel_.group_box->setVisible(false);
    video_panel_.group_box->setVisible(false);
    UpdateStatus();
    return false;
  }
  if(!model) {
    model_json_ = simulation_.get_context()->get_model_json();
    model_edited_ = false;
    model_filename_ = filename;
  }

  PopulateTree();
  LoadModelParameters();
  ClearVideoRange();
  playback_time_ = 0.0;
  // an HDF5 log comes with its results, which may run past the time span
  playback_panel_.slider->set_time_span(
    0.0, std::max(simulation_.get_end_time(), simulation_.get_computed_time()));
  playback_panel_.slider->set_progress_time(simulation_.get_computed_time());
  playback_panel_.slider->set_time(0.0);
  playback_panel_.slider->Enable(simulation_.has_results());
  playback_panel_.play_pause_button->setIcon(playback_panel_.play_icon);
  viewport_->set_simulation(&simulation_, !keep_view);
  camera_panel_.follow->setChecked(false);
  camera_panel_.body->clear();
  for(const auto* body : simulation_.get_bodies()) {
    camera_panel_.body->addItem(QString::fromStdString(body->get_name()));
  }
  camera_panel_.group_box->setVisible(true);
  if(!reselect.follow_body.isEmpty()) {
    const int index = camera_panel_.body->findText(reselect.follow_body);
    if(index >= 0) {
      camera_panel_.body->setCurrentIndex(index);
      camera_panel_.follow->setChecked(true);
    }
  }
  UpdateFloor();
  if(!keep_view) {
    // the floor plane would cut through a terrain
    floor_action_->setChecked(simulation_.get_scene().terrains.empty());
  }
  UpdateWindowTitle();

  if(!reselect.path.isEmpty()) {
    for(QTreeWidgetItemIterator it(tree_widget_); *it; ++it) {
      if((*it)->data(0, kPathRole).toString() == reselect.path) {
        tree_widget_->setCurrentItem(*it);
        tree_widget_->scrollToItem(*it);
        break;
      }
    }
    auto& panel = parameter_panel_;
    for(size_t i = 0; i < panel.rows.size(); i++) {
      if(panel.rows[i].keys.front() == reselect.parameter) {
        panel.tree->setCurrentItem(panel.rows[i].item);
        break;
      }
    }
  }
  UpdateStatus();
  return true;
}

bool shift::gui::Workspace::ReloadModel(const Json::Value& previous,
                                        QString& error) {
  if(LoadModel(model_filename_, &model_json_, &error, true)) {
    model_edited_ = true;
    UpdateWindowTitle();
    return true;
  }
  // back to the model before the edit, which loaded
  model_json_ = previous;
  QString again;
  LoadModel(model_filename_, &model_json_, &again, true);
  return false;
}

void shift::gui::Workspace::UpdateWindowTitle() {
  if(!simulation_.is_loaded()) {
    setWindowTitle("Shift");
    return;
  }
  setWindowTitle(QString("Shift — %1%2")
                   .arg(QFileInfo(model_filename_).fileName(),
                        model_edited_ ? " (edited)" : ""));
}

void shift::gui::Workspace::SaveModelAs() {
  if(!simulation_.is_loaded()) {
    return;
  }
  // the files the model names are found relative to it: save it beside the
  // file it was read from
  const QFileInfo info(model_filename_);
  const QString suggestion =
    info.dir().filePath(info.completeBaseName() + ".json");
  const QString filename = QFileDialog::getSaveFileName(
    this, "Save model as", suggestion, "Shift models (*.json)");
  QString error;
  if(!filename.isEmpty() && !SaveModel(filename, &error)) {
    QMessageBox::critical(this, "Save failed", error);
  }
}

bool shift::gui::Workspace::SaveModel(const QString& filename,
                                      QString* error) {
  if(!simulation_.is_loaded()) {
    if(error) {
      *error = "no model is loaded";
    }
    return false;
  }
  // the elements of each array keep their order; the keys of an object are
  // written in alphabetical order
  std::ofstream file(filename.toStdString());
  Json::StreamWriterBuilder builder;
  builder.settings_["indentation"] = "  ";
  file << Json::writeString(builder, model_json_) << std::endl;
  if(!file) {
    if(error) {
      *error = "Could not write " + filename;
    }
    return false;
  }
  model_filename_ = filename;
  model_edited_ = false;
  UpdateWindowTitle();
  return true;
}

void shift::gui::Workspace::RevertModel() {
  if(!simulation_.is_loaded()) {
    return;
  }
  if(model_edited_ &&
     QMessageBox::question(this, "Revert model",
                           "Drop the edits and load " +
                             QFileInfo(model_filename_).fileName() +
                             " again?") != QMessageBox::Yes) {
    return;
  }
  Open(model_filename_);
}

void shift::gui::Workspace::SaveImage(const ImageRequest& request) {
  image_request_ = request;
  if(request.bilinear_terrain) {
    smooth_terrain_action_->setChecked(false);
  }
  if(request.hide_floor) {
    floor_action_->setChecked(false);
  }
  if(request.samples > 0) {
    viewport_->set_render_mode(Viewport::RenderMode::kRayTraced);
  }
  // an HDF5 log that reaches the time is shown as it is
  if(request.time > 0.0 && simulation_.is_loaded() &&
     simulation_.get_computed_time() < request.time - 1e-9) {
    simulation_.set_end_time(request.time);
    simulation_.Run();
  }
  image_timer_ = new QTimer(this);
  connect(image_timer_, &QTimer::timeout, this, [this]() {
    if(AdvanceImage()) {
      image_timer_->stop();
    }
  });
  image_timer_->start(50);
}

bool shift::gui::Workspace::AdvanceImage() {
  const ImageRequest& request = image_request_;
  auto fail = [](const QString& message) {
    std::fprintf(stderr, "shift: %s\n", message.toLocal8Bit().constData());
    QCoreApplication::exit(1);
    return true;
  };
  if(!simulation_.is_loaded()) {
    return fail("no model loaded");
  }
  if(request.time > 0.0) {
    if(simulation_.is_computing() &&
       simulation_.get_computed_time() < request.time) {
      return false;
    }
    simulation_.Stop();
    if(!simulation_.get_error().empty()) {
      return fail(QString::fromStdString(simulation_.get_error()));
    }
    if(simulation_.get_computed_time() < request.time - 1e-9) {
      return fail(QString("the simulation stopped at t = %1 s")
                    .arg(simulation_.get_computed_time()));
    }
  }
  QString error;
  if(!viewport_->BeginOffscreen(request.size, error)) {
    return false; // the window is not exposed yet
  }
  const int follow = camera_panel_.body->findText(request.follow_body);
  if(!request.follow_body.isEmpty() && follow < 0) {
    viewport_->EndOffscreen();
    return fail("no body named " + request.follow_body);
  }
  // with a body to follow, the view is set at the start and carried along
  simulation_.ShowTime(request.follow_body.isEmpty() ? request.time : 0.0);
  viewport_->UpdatePoses();
  if(request.set_view) {
    viewport_->set_view(request.azimuth, request.elevation);
  } else {
    viewport_->FrameScene();
  }
  if(request.set_frame) {
    viewport_->FrameSphere(request.frame_center, request.frame_radius);
  }
  if(!request.follow_body.isEmpty()) {
    viewport_->set_follow_body(follow, request.follow_ground
                                         ? Viewport::FollowMode::kGround
                                         : Viewport::FollowMode::kBody);
    simulation_.ShowTime(request.time);
    viewport_->UpdatePoses();
  }
  std::vector<uint8_t> rgba;
  const bool rendered = viewport_->RenderOffscreen(request.samples, rgba);
  viewport_->EndOffscreen();
  if(!rendered) {
    return fail("could not render the image");
  }
  QImage image(rgba.data(), request.size.width(), request.size.height(),
               4 * request.size.width(), QImage::Format_RGBA8888);
  if(!image.convertToFormat(QImage::Format_RGB888).save(request.filename)) {
    return fail("could not write " + request.filename);
  }
  std::printf("shift: saved %s at t = %g s\n",
              request.filename.toLocal8Bit().constData(), request.time);
  QCoreApplication::exit(0);
  return true;
}

void shift::gui::Workspace::OpenDialog() {
  const QString filename = QFileDialog::getOpenFileName(
    this, "Open model", QString(), "Shift models and logs (*.json *.yaml *.yml *.h5 *.hdf5);;All files (*)");
  if(!filename.isEmpty()) {
    Open(filename);
  }
}

void shift::gui::Workspace::About() {
  QMessageBox::about(
    this, "About Shift",
    "<b>Shift</b> multibody dynamics<br>"
    "Kristopher Wehage<br>"
    "<a href=\"https://shift-dynamics.github.io/shift/\">shift-dynamics.github.io/shift</a>");
}

void shift::gui::Workspace::LoadModelParameters() {
  auto& panel = model_parameters_panel_;
  panel.group_box->setVisible(true);
  video_panel_.group_box->setVisible(true);
  panel.integrator_type->setText(
    QString::fromStdString(simulation_.get_integrator_type()));
  panel.time_span->blockSignals(true);
  panel.time_span->setValue(simulation_.get_end_time());
  panel.time_span->blockSignals(false);
  panel.time_step->blockSignals(true);
  panel.time_step->setValue(simulation_.get_time_step());
  panel.time_step->blockSignals(false);
}

// -------------------------------------------------------------- model tree

/* the keys of the arrays and objects in the order the tree shows them, as a
model file usually has them; other keys follow in alphabetical order */
static int KeyRank(const std::string& key) {
  static const char* const kOrder[] = {
    "model_parameters", "units", "solvers", "integrators", "loggers",
    "frames", "lights", "geometry", "collision_geometry", "terrain",
    "bodies", "joints", "constraints", "force_modules", "sensors",
    "controllers", "assemblies"};
  const int count = static_cast<int>(std::size(kOrder));
  for(int i = 0; i < count; i++) {
    if(key == kOrder[i]) {
      return i;
    }
  }
  return count;
}

/* the keys of the parameters of an object, in the order of KeyRank */
static std::vector<std::string> SortedKeys(
  const shift::core::parameter::List* object) {
  std::vector<std::string> keys;
  for(const auto& parameter : object->get_parameters()) {
    keys.push_back(parameter.first);
  }
  std::stable_sort(keys.begin(), keys.end(),
                   [](const std::string& a, const std::string& b) {
                     return KeyRank(a) < KeyRank(b);
                   });
  return keys;
}

/* the value of a string parameter of an object, empty if it has none */
static std::string StringParameter(shift::core::parameter::List* object,
                                   const std::string& key) {
  shift::core::parameter::Value* value = object->Find(key);
  if(value && value->has_data() &&
     value->get_value_type() == shift::core::parameter::Value::STRING) {
    return value->get<std::string>();
  }
  return std::string();
}

/* whether an object has an array with elements, itself or in an object
parameter; such an object is shown in the model tree */
static bool HasObjects(shift::core::parameter::List* object, int depth) {
  if(!object || depth > 32) {
    return false;
  }
  for(const auto& parameter : object->get_parameters()) {
    const shift::core::parameter::Value& value = parameter.second;
    if(!value.has_data()) {
      continue;
    }
    if(value.is_array()) {
      auto* array = value.get_data<shift::core::parameter::ArrayBase>();
      if(array && array->get_size() > 0) {
        return true;
      }
    } else if(value.is_list() &&
              HasObjects(value.get_data<shift::core::parameter::List>(),
                         depth + 1)) {
      return true;
    }
  }
  return false;
}

static constexpr int kObjectRole = Qt::UserRole + 1;   /*!< parameter::List* */
static constexpr int kInactiveRole = Qt::UserRole + 2; /*!< bool */

static shift::core::parameter::List* ItemObject(const QTreeWidgetItem* item) {
  return reinterpret_cast<shift::core::parameter::List*>(
    item->data(0, kObjectRole).value<quintptr>());
}

/* the value at path in a model's JSON, nullptr if there is none */
static const Json::Value* JsonAt(
  const Json::Value& root,
  const std::vector<shift::gui::Workspace::JsonStep>& path) {
  const Json::Value* json = &root;
  for(const auto& step : path) {
    if(step.index >= 0) {
      if(!json->isArray() ||
         step.index >= static_cast<int>(json->size())) {
        return nullptr;
      }
      json = &(*json)[step.index];
    } else {
      if(!json->isObject() || !json->isMember(step.key)) {
        return nullptr;
      }
      json = &(*json)[step.key];
    }
  }
  return json;
}

/* the element of the JSON array that object, element index of the array
of objects read from it, was read from: the one of its name, as names are
unique in an array, else the element at index if the array has as many
elements as objects were read; -1 if none (as the default floor) */
static int JsonElementIndex(const Json::Value& array,
                            shift::core::parameter::List* object, int index) {
  if(!array.isArray()) {
    return -1;
  }
  const std::string name = StringParameter(object, "name");
  if(!name.empty()) {
    for(Json::ArrayIndex i = 0; i < array.size(); i++) {
      if(array[i].isObject() && array[i].isMember("name") &&
         array[i]["name"].isString() &&
         array[i]["name"].asString() == name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }
  return index < static_cast<int>(array.size()) ? index : -1;
}

/* what identifies a tree item when the model is loaded again: its path in
the model's JSON, or else the names of the items to it */
static QString PathKey(
  const std::vector<shift::gui::Workspace::JsonStep>* path,
  const QTreeWidgetItem* item) {
  QStringList parts;
  if(path) {
    for(const auto& step : *path) {
      parts << (step.index >= 0 ? QString("#%1").arg(step.index)
                                : QString::fromStdString(step.key));
    }
    return "/" + parts.join("/");
  }
  for(; item; item = item->parent()) {
    parts.prepend(item->text(0));
  }
  return "~" + parts.join("/");
}

static QTreeWidgetItem* NewItem(QTreeWidgetItem* parent, QTreeWidget* tree,
                                const std::string& text, const char* icon,
                                int type = QTreeWidgetItem::Type) {
  QTreeWidgetItem* item =
    parent ? new QTreeWidgetItem(parent, {QString::fromStdString(text)}, type)
           : new QTreeWidgetItem(tree, {QString::fromStdString(text)}, type);
  if(icon) {
    item->setIcon(0, QIcon(icon));
  }
  return item;
}

static const char* GeometryIcon(const std::string& type) {
  if(type == "plane") {
    return ":/icons/floor.png";
  }
  if(type == "sphere" || type == "ellipsoid") {
    return ":/icons/sphere.png";
  }
  return ":/icons/convexhull.png";
}

/* the icon of the folder of an array */
static const char* FolderIcon(const std::string& key) {
  static const std::pair<const char*, const char*> kIcons[] = {
    {"assemblies", ":/icons/assembly_folder.png"},
    {"bodies", ":/icons/body_folder.png"},
    {"joints", ":/icons/joint_folder.png"},
    {"constraints", ":/icons/io_folder.png"},
    {"force_modules", ":/icons/restraint_folder.png"},
    {"sensors", ":/icons/sensor_folder.png"},
    {"controllers", ":/icons/controller_folder.png"},
    {"frames", ":/icons/frame_folder.png"},
    {"lights", ":/icons/light_folder.png"},
    {"terrain", ":/icons/terrain_folder.png"},
    {"solvers", ":/icons/clipboard_folder.png"},
    {"integrators", ":/icons/clipboard_folder.png"},
    {"loggers", ":/icons/clipboard_folder.png"},
  };
  for(const auto& icon : kIcons) {
    if(key == icon.first) {
      return icon.second;
    }
  }
  return ":/icons/blank_folder.png";
}

/* the icon of an element of the array key, or of the object parameter key */
static const char* ObjectIcon(const std::string& key, const std::string& type) {
  if(key == "collision_geometry") {
    return GeometryIcon(type);
  }
  static const std::pair<const char*, const char*> kIcons[] = {
    {"assemblies", ":/icons/assembly.png"},
    {"bodies", ":/icons/body.png"},
    {"joints", ":/icons/joint.png"},
    {"constraints", ":/icons/io.png"},
    {"force_modules", ":/icons/restraint.png"},
    {"sensors", ":/icons/sensor.png"},
    {"controllers", ":/icons/controller.png"},
    {"frames", ":/icons/frame.png"},
    {"lights", ":/icons/light.png"},
    {"terrain", ":/icons/terrain.png"},
    {"geometry", ":/icons/cube.png"},
    {"model_parameters", ":/icons/clipboard.png"},
    {"solvers", ":/icons/clipboard.png"},
    {"integrators", ":/icons/clipboard.png"},
    {"loggers", ":/icons/clipboard.png"},
  };
  for(const auto& icon : kIcons) {
    if(key == icon.first) {
      return icon.second;
    }
  }
  return nullptr;
}

/* the tree follows the layout of the model file: its top level, which is
the fixed body, with a folder for each array of objects (assemblies, bodies,
joints, force modules, sensors, controllers, frames, ...) holding an item for
each object, and so on down the nested assemblies */
void shift::gui::Workspace::PopulateTree() {
  tree_widget_->clear();
  object_paths_.clear();
  shift::core::Context* context = simulation_.get_context();
  joint_items_.assign(context->get_active_joints().size(), nullptr);

  // the fixed body is the first body of the simulation
  std::string name = QFileInfo(model_filename_).fileName().toStdString();
  if(name.empty()) {
    name = context->get_name();
  }
  QTreeWidgetItem* root =
    NewItem(nullptr, tree_widget_, name, ":/icons/anchor.png", kBodyItem);
  root->setToolTip(0, "the model file; its top level is the fixed body, "
                      "named \"" +
                        QString::fromStdString(context->get_name()) + "\"");
  root->setData(0, Qt::UserRole, 0);
  root->setData(0, kObjectRole,
                QVariant::fromValue(reinterpret_cast<quintptr>(
                  static_cast<shift::core::parameter::List*>(context))));
  const std::vector<JsonStep> path;
  object_paths_[context] = path;
  root->setData(0, kPathRole, PathKey(&path, root));
  AddObjectChildren(context, root, 1, &path);
  root->setExpanded(true);
  UpdateTree();
}

const std::vector<shift::gui::Workspace::JsonStep>*
shift::gui::Workspace::FindObjectPath(
  const shift::core::parameter::List* object) {
  auto found = object_paths_.find(object);
  return found == object_paths_.end() ? nullptr : &found->second;
}

void shift::gui::Workspace::AddObjectChildren(
  shift::core::parameter::List* object, QTreeWidgetItem* item, int depth,
  const std::vector<JsonStep>* path) {
  if(depth > 32) {
    return;
  }
  for(const std::string& key : SortedKeys(object)) {
    shift::core::parameter::Value* value = object->Find(key);
    if(!value || !value->has_data()) {
      continue;
    }
    std::vector<JsonStep> key_path;
    if(path) {
      key_path = *path;
      key_path.push_back({key, -1});
    }
    if(value->is_array()) {
      auto* array = value->get_data<shift::core::parameter::ArrayBase>();
      if(!array || array->get_size() == 0) {
        continue;
      }
      QTreeWidgetItem* folder = NewItem(item, nullptr, key, FolderIcon(key));
      folder->setData(0, kInactiveRole, item->data(0, kInactiveRole));
      folder->setData(0, kPathRole, PathKey(path ? &key_path : nullptr, folder));
      // the elements of the array in the file, to find each object's
      const Json::Value* json = path ? JsonAt(model_json_, key_path) : nullptr;
      for(unsigned int i = 0; i < array->get_size(); i++) {
        auto* element =
          (*array)[i].get_data<shift::core::parameter::List>();
        if(!element) {
          continue;
        }
        const int index =
          json ? JsonElementIndex(*json, element, static_cast<int>(i)) : -1;
        std::vector<JsonStep> element_path = key_path;
        element_path.push_back({std::string(), index});
        AddObjectItem(element, key, folder, depth + 1,
                      index >= 0 ? &element_path : nullptr);
      }
      if(folder->childCount() == 0) {
        delete folder;
      } else {
        folder->setExpanded(true);
      }
    } else if(value->is_list()) {
      // an object parameter holding arrays, as model_parameters
      auto* nested = value->get_data<shift::core::parameter::List>();
      if(HasObjects(nested, depth + 1)) {
        QTreeWidgetItem* nested_item = NewItem(
          item, nullptr, key, ObjectIcon(key, std::string()), kObjectItem);
        nested_item->setData(0, Qt::UserRole, -1);
        nested_item->setData(0, kObjectRole,
                             QVariant::fromValue(
                               reinterpret_cast<quintptr>(nested)));
        nested_item->setData(0, kInactiveRole, item->data(0, kInactiveRole));
        nested_item->setData(0, kPathRole,
                             PathKey(path ? &key_path : nullptr, nested_item));
        nested_item->setToolTip(
          0, QString::fromStdString(value->get_description()));
        if(path) {
          object_paths_[nested] = key_path;
        }
        AddObjectChildren(nested, nested_item, depth + 1,
                          path ? &key_path : nullptr);
      }
    }
  }
}

QTreeWidgetItem* shift::gui::Workspace::AddObjectItem(
  shift::core::parameter::List* object, const std::string& key,
  QTreeWidgetItem* parent, int depth, const std::vector<JsonStep>* path) {
  // virtual bodies are massless helpers generated for joints
  if(dynamic_cast<shift::core::body::Virtual*>(object)) {
    return nullptr;
  }
  const std::string name = StringParameter(object, "name");
  const std::string type = StringParameter(object, "type");
  std::string text = name;
  if(text.empty()) {
    text = key == "assemblies" ? "assembly" : (type.empty() ? key : type);
  } else if(!type.empty() && key != "assemblies" && key != "bodies") {
    text += " (" + type + ")";
  }
  const bool inactive =
    !object->is_active() || parent->data(0, kInactiveRole).toBool();

  // bodies, joints and the collision geometry of the fixed body have panels
  // of their own
  int item_type = kObjectItem;
  int index = -1;
  shift::core::Context* context = simulation_.get_context();
  if(auto* body = dynamic_cast<shift::core::body::Base*>(object)) {
    const auto& bodies = simulation_.get_bodies();
    auto found = std::find(bodies.begin(), bodies.end(), body);
    if(found != bodies.end()) {
      item_type = kBodyItem;
      index = static_cast<int>(found - bodies.begin());
    }
  } else if(auto* joint = dynamic_cast<shift::core::joint::Base*>(object)) {
    const auto& active = context->get_active_joints();
    auto found = std::find(active.begin(), active.end(), joint);
    if(found != active.end()) {
      item_type = kJointItem;
      index = static_cast<int>(found - active.begin());
    }
  } else if(auto* geometry =
              dynamic_cast<shift::core::collision_geometry::Base*>(object)) {
    const auto& fixed = context->get_fixed_collision_geometry();
    for(size_t i = 0; i < fixed.size(); i++) {
      if(fixed[i].get() == geometry) {
        item_type = kGeometryItem;
        index = static_cast<int>(i);
      }
    }
  }

  QTreeWidgetItem* item =
    NewItem(parent, nullptr, text, ObjectIcon(key, type), item_type);
  item->setData(0, Qt::UserRole, index);
  item->setData(0, kObjectRole,
                QVariant::fromValue(reinterpret_cast<quintptr>(object)));
  item->setData(0, kInactiveRole, inactive);
  item->setData(0, kPathRole, PathKey(path, item));
  if(path) {
    object_paths_[object] = *path;
  }
  if(item_type == kJointItem) {
    joint_items_[index] = item;
  }
  if(inactive) {
    item->setForeground(
      0, tree_widget_->palette().brush(QPalette::Disabled, QPalette::Text));
    item->setToolTip(0, "inactive: not part of the simulation");
  }
  AddObjectChildren(object, item, depth, path);
  // an assembly shows what it holds; other objects show their own arrays
  // (as the frames and geometry of a body) when opened
  item->setExpanded(key == "assemblies");
  return item;
}

// ---------------------------------------------------------- parameters panel

static QString FormatNumber(double value) {
  return QString::number(value, 'g', 6);
}

static QString FormatNumbers(const double* values, int count) {
  QStringList list;
  for(int i = 0; i < count; i++) {
    list << FormatNumber(values[i]);
  }
  return "[" + list.join(", ") + "]";
}

/* a row of a matrix, given in column major order */
static QString FormatMatrixRow(const double* values, int rows, int columns,
                               int row) {
  QStringList list;
  for(int column = 0; column < columns; column++) {
    list << FormatNumber(values[column * rows + row]);
  }
  return "[" + list.join(", ") + "]";
}

/* roll, pitch and yaw of a rotation, R = Rz(yaw) Ry(pitch) Rx(roll), in
radians */
static void RollPitchYaw(const ksl_mat3x3_t& rotation, double rpy[3]) {
  ksl_vec3_t angles; // yaw, pitch, roll
  ksl_mat3x3_getEulerAnglesWithReference(&rotation, KSL_AXIS_ZYX, &angles,
                                         NULL);
  for(int i = 0; i < 3; i++) {
    rpy[i] = angles.at[2 - i];
  }
}

static ksl_mat3x3_t RotationOfRollPitchYaw(const double rpy[3]) {
  ksl_vec3_t angles = ksl_vec3(rpy[2], rpy[1], rpy[0]);
  ksl_mat3x3_t rotation;
  ksl_mat3x3_setFromEulerAngles(&rotation, KSL_AXIS_ZYX, &angles);
  return rotation;
}

/* the rotation of an orientation parameter: a rotation matrix or a
quaternion */
static bool IsOrientation(const shift::core::parameter::Value& value) {
  return value.get_value_type() == shift::core::parameter::Value::KSL_MAT3X3 ||
         value.get_value_type() ==
           shift::core::parameter::Value::KSL_QUATERNION;
}

static ksl_mat3x3_t OrientationOf(const shift::core::parameter::Value& value) {
  if(value.get_value_type() == shift::core::parameter::Value::KSL_QUATERNION) {
    ksl_quaternion_t quaternion = value.get<ksl_quaternion_t>();
    ksl_mat3x3_t rotation;
    ksl_quaternion_toMat3x3(&quaternion, &rotation);
    return rotation;
  }
  return value.get<ksl_mat3x3_t>();
}

static QString FormatRollPitchYaw(const ksl_mat3x3_t& rotation) {
  double rpy[3];
  RollPitchYaw(rotation, rpy);
  QStringList list;
  for(double angle : rpy) {
    const double degrees = angle * 180.0 / M_PI;
    // no -0.00
    list << QString::number(std::fabs(degrees) < 0.005 ? 0.0 : degrees, 'f',
                            2);
  }
  return "roll, pitch, yaw = [" + list.join(", ") + "]°";
}

/* the text of a value, and the rows below it of a value too long for one
line, as label and text */
static QString FormatValue(const shift::core::parameter::Value& value,
                           std::vector<std::pair<QString, QString>>& rows) {
  rows.clear();
  switch(value.get_value_type()) {
    case shift::core::parameter::Value::STRING:
      return QString::fromStdString(value.get<std::string>());
    case shift::core::parameter::Value::STRING_ARRAY: {
      QStringList list;
      for(const auto& entry : value.get<std::vector<std::string>>()) {
        list << "\"" + QString::fromStdString(entry) + "\"";
      }
      return "[" + list.join(", ") + "]";
    }
    case shift::core::parameter::Value::STRING_2DARRAY: {
      const auto& table = value.get<std::vector<std::vector<std::string>>>();
      for(size_t i = 0; i < table.size(); i++) {
        QStringList list;
        for(const auto& entry : table[i]) {
          list << "\"" + QString::fromStdString(entry) + "\"";
        }
        rows.emplace_back(QString("[%1]").arg(i), "[" + list.join(", ") + "]");
      }
      return QString("%1 rows").arg(table.size());
    }
    case shift::core::parameter::Value::INT:
      return QString::number(value.get<int>());
    case shift::core::parameter::Value::INT_ARRAY: {
      QStringList list;
      for(int entry : value.get<std::vector<int>>()) {
        list << QString::number(entry);
      }
      return "[" + list.join(", ") + "]";
    }
    case shift::core::parameter::Value::DOUBLE:
      return FormatNumber(value.get<double>());
    case shift::core::parameter::Value::DOUBLE_ARRAY: {
      const auto& array = value.get<std::vector<double>>();
      const int count = static_cast<int>(array.size());
      if(count <= 6) {
        return FormatNumbers(array.data(), count);
      }
      for(int i = 0; i < count; i += 6) {
        rows.emplace_back(QString("[%1]").arg(i),
                          FormatNumbers(array.data() + i, std::min(6, count - i)));
      }
      return QString("%1 values").arg(count);
    }
    case shift::core::parameter::Value::KSL_VEC3:
      return FormatNumbers(value.get<ksl_vec3_t>().at, 3);
    case shift::core::parameter::Value::KSL_VEC4:
      return FormatNumbers(value.get<ksl_vec4_t>().at, 4);
    case shift::core::parameter::Value::KSL_QUATERNION:
      rows.emplace_back("x, y, z, w",
                        FormatNumbers(value.get<ksl_quaternion_t>().at, 4));
      return FormatRollPitchYaw(OrientationOf(value));
    case shift::core::parameter::Value::KSL_SCREW: {
      const ksl_screw_t& screw = value.get<ksl_screw_t>();
      rows.emplace_back("lin", FormatNumbers(screw.lin.at, 3));
      rows.emplace_back("ang", FormatNumbers(screw.ang.at, 3));
      return FormatNumbers(screw.at, 6);
    }
    case shift::core::parameter::Value::KSL_COSCREW: {
      const ksl_coscrew_t& coscrew = value.get<ksl_coscrew_t>();
      rows.emplace_back("lin", FormatNumbers(coscrew.lin.at, 3));
      rows.emplace_back("ang", FormatNumbers(coscrew.ang.at, 3));
      return FormatNumbers(coscrew.at, 6);
    }
    case shift::core::parameter::Value::KSL_INERTIA: {
      const ksl_inertia_t& inertia = value.get<ksl_inertia_t>();
      rows.emplace_back("m", FormatNumber(inertia.m));
      rows.emplace_back("m t", FormatNumbers(inertia.mt.at, 3));
      rows.emplace_back("Ixx, Iyy, Izz", FormatNumbers(&inertia.Ixx, 3));
      rows.emplace_back("Ixy, Iyz, Izx", FormatNumbers(&inertia.Ixy, 3));
      return "m = " + FormatNumber(inertia.m);
    }
    case shift::core::parameter::Value::KSL_MAT3X3: {
      const ksl_mat3x3_t& matrix = value.get<ksl_mat3x3_t>();
      for(int row = 0; row < 3; row++) {
        rows.emplace_back(QString("row %1").arg(row),
                          FormatMatrixRow(matrix.as_array, 3, 3, row));
      }
      return FormatRollPitchYaw(matrix);
    }
    case shift::core::parameter::Value::KSL_MAT4X4: {
      const ksl_mat4x4_t& matrix = value.get<ksl_mat4x4_t>();
      for(int row = 0; row < 4; row++) {
        rows.emplace_back(QString("row %1").arg(row),
                          FormatMatrixRow(matrix.as_array, 4, 4, row));
      }
      return "4 × 4";
    }
    case shift::core::parameter::Value::KSL_EULERANGLES: {
      const ksl_eulerangles_t& angles = value.get<ksl_eulerangles_t>();
      QString axes;
      if(angles.axis >= KSL_AXIS_XYZ && angles.axis < KSL_AXIS_INVALID) {
        const ksl_vec3i_t axis = ksl_axis_getVector(angles.axis);
        for(int i = 0; i < 3; i++) {
          axes += QChar('X' + axis.at[i]);
        }
      }
      return FormatNumbers(angles.angles.at, 3) + " rad, " + axes;
    }
    case shift::core::parameter::Value::KSL_SE3: {
      const ksl_SE3_t& pose = value.get<ksl_SE3_t>();
      rows.emplace_back("t", FormatNumbers(pose.t.at, 3));
      for(int row = 0; row < 3; row++) {
        rows.emplace_back(QString("R row %1").arg(row),
                          FormatMatrixRow(pose.R.as_array, 3, 3, row));
      }
      return "t = " + FormatNumbers(pose.t.at, 3);
    }
    case shift::core::parameter::Value::BOOLEAN:
      return value.get<bool>() ? "true" : "false";
    case shift::core::parameter::Value::PARAMETER_LIST:
    case shift::core::parameter::Value::PARAMETER_ARRAY:
      break;
  }
  return QString();
}

static Json::Value JsonNumbers(const double* values, int count) {
  Json::Value array(Json::arrayValue);
  for(int i = 0; i < count; i++) {
    array.append(values[i]);
  }
  return array;
}

/* the rows of a matrix in column major order, as the model file has them */
static Json::Value JsonMatrix(const double* values, int size) {
  Json::Value matrix(Json::arrayValue);
  for(int row = 0; row < size; row++) {
    Json::Value json_row(Json::arrayValue);
    for(int column = 0; column < size; column++) {
      json_row.append(values[column * size + row]);
    }
    matrix.append(json_row);
  }
  return matrix;
}

/* a value in the form the model file gives it, see Value::ReadJSON */
static Json::Value ValueToJson(const shift::core::parameter::Value& value) {
  switch(value.get_value_type()) {
    case shift::core::parameter::Value::STRING:
      return Json::Value(value.get<std::string>());
    case shift::core::parameter::Value::STRING_ARRAY: {
      Json::Value array(Json::arrayValue);
      for(const auto& entry : value.get<std::vector<std::string>>()) {
        array.append(entry);
      }
      return array;
    }
    case shift::core::parameter::Value::STRING_2DARRAY: {
      Json::Value table(Json::arrayValue);
      for(const auto& row : value.get<std::vector<std::vector<std::string>>>()) {
        Json::Value json_row(Json::arrayValue);
        for(const auto& entry : row) {
          json_row.append(entry);
        }
        table.append(json_row);
      }
      return table;
    }
    case shift::core::parameter::Value::INT:
      return Json::Value(value.get<int>());
    case shift::core::parameter::Value::INT_ARRAY: {
      Json::Value array(Json::arrayValue);
      for(int entry : value.get<std::vector<int>>()) {
        array.append(entry);
      }
      return array;
    }
    case shift::core::parameter::Value::DOUBLE:
      return Json::Value(value.get<double>());
    case shift::core::parameter::Value::DOUBLE_ARRAY: {
      const auto& array = value.get<std::vector<double>>();
      return JsonNumbers(array.data(), static_cast<int>(array.size()));
    }
    case shift::core::parameter::Value::KSL_VEC3:
      return JsonNumbers(value.get<ksl_vec3_t>().at, 3);
    case shift::core::parameter::Value::KSL_VEC4:
      return JsonNumbers(value.get<ksl_vec4_t>().at, 4);
    case shift::core::parameter::Value::KSL_QUATERNION:
      return JsonNumbers(value.get<ksl_quaternion_t>().at, 4);
    case shift::core::parameter::Value::KSL_SCREW:
      return JsonNumbers(value.get<ksl_screw_t>().at, 6);
    case shift::core::parameter::Value::KSL_COSCREW:
      return JsonNumbers(value.get<ksl_coscrew_t>().at, 6);
    case shift::core::parameter::Value::KSL_INERTIA: {
      const ksl_inertia_t& inertia = value.get<ksl_inertia_t>();
      Json::Value object(Json::objectValue);
      object["mass"] = inertia.m;
      object["Ixx"] = inertia.Ixx;
      object["Iyy"] = inertia.Iyy;
      object["Izz"] = inertia.Izz;
      object["Ixy"] = inertia.Ixy;
      object["Iyz"] = inertia.Iyz;
      object["Izx"] = inertia.Izx;
      return object;
    }
    case shift::core::parameter::Value::KSL_MAT3X3:
      return JsonMatrix(value.get<ksl_mat3x3_t>().as_array, 3);
    case shift::core::parameter::Value::KSL_MAT4X4:
      return JsonMatrix(value.get<ksl_mat4x4_t>().as_array, 4);
    case shift::core::parameter::Value::KSL_EULERANGLES: {
      Json::Value object(Json::objectValue);
      object["angles"] = JsonNumbers(value.get<ksl_eulerangles_t>().angles.at, 3);
      return object;
    }
    case shift::core::parameter::Value::KSL_SE3: {
      Json::Value object(Json::objectValue);
      object["R"] = JsonMatrix(value.get<ksl_SE3_t>().R.as_array, 3);
      return object;
    }
    case shift::core::parameter::Value::BOOLEAN:
      return Json::Value(value.get<bool>());
    case shift::core::parameter::Value::PARAMETER_LIST:
    case shift::core::parameter::Value::PARAMETER_ARRAY:
      break;
  }
  return Json::Value();
}

/* a rotation in the form of given, the orientation as the model file gives
it: a quaternion [x, y, z, w], Euler angles {"angles": [...], "axes": [...]}
about the same axes, or a 3x3 matrix row by row (also if the file gives
none, unless the parameter is a quaternion) */
static Json::Value OrientationToJson(
  ksl_mat3x3_t rotation, shift::core::parameter::Value::ValueType type,
  const Json::Value* given) {
  const bool quaternion =
    given ? given->isArray() && given->size() == 4 && (*given)[0].isNumeric()
          : type == shift::core::parameter::Value::KSL_QUATERNION;
  if(quaternion) {
    ksl_quaternion_t q;
    ksl_mat3x3_toQuaternion(&rotation, &q);
    return JsonNumbers(q.at, 4);
  }
  if(given && given->isObject() && given->isMember("angles")) {
    Json::Value euler = *given;
    ksl_vec3i_t axes = {{0, 1, 2}};
    if(given->isMember("axes") && (*given)["axes"].isArray() &&
       (*given)["axes"].size() == 3) {
      for(int i = 0; i < 3; i++) {
        const Json::Value& axis = (*given)["axes"][i];
        axes.at[i] = axis.isString() ? axis.asString()[0] - 'x'
                                     : axis.asInt();
      }
    }
    ksl_vec3_t angles;
    ksl_mat3x3_getEulerAnglesWithReference(
      &rotation, ksl_axis_getEulerAngles(axes), &angles, NULL);
    euler["angles"] = JsonNumbers(angles.at, 3);
    return euler;
  }
  return JsonMatrix(rotation.as_array, 3);
}

/* show a value in its row, with the rows below it of a long value */
static void SetRowValue(QTreeWidgetItem* row,
                        const shift::core::parameter::Value& value) {
  std::vector<std::pair<QString, QString>> rows;
  row->setText(1, FormatValue(value, rows));
  while(row->childCount() > static_cast<int>(rows.size())) {
    delete row->takeChild(row->childCount() - 1);
  }
  for(size_t i = 0; i < rows.size(); i++) {
    QTreeWidgetItem* child = static_cast<int>(i) < row->childCount()
                               ? row->child(static_cast<int>(i))
                               : new QTreeWidgetItem(row);
    child->setText(0, rows[i].first);
    child->setText(1, rows[i].second);
    child->setForeground(0, row->foreground(0));
    child->setForeground(1, row->foreground(1));
  }
}

void shift::gui::Workspace::ShowParameters(
  shift::core::parameter::List* object, const QString& title) {
  auto& panel = parameter_panel_;
  panel.tree->clear();
  panel.rows.clear();
  panel.editing_row = -1;
  panel.editor->setVisible(false);
  panel.selected = object;
  if(!object) {
    panel.group_box->setVisible(false);
    return;
  }
  panel.title->setText(title);
  AddParameterRows(object, nullptr, 0, FindObjectPath(object));
  panel.group_box->setVisible(true);
}

void shift::gui::Workspace::AddParameterRows(
  shift::core::parameter::List* object, QTreeWidgetItem* parent, int depth,
  const std::vector<JsonStep>* path) {
  auto& panel = parameter_panel_;
  if(depth > 32) {
    return;
  }
  const QBrush default_brush =
    panel.tree->palette().brush(QPalette::Disabled, QPalette::Text);
  // the object as the file gives it
  const Json::Value* json = path ? JsonAt(model_json_, *path) : nullptr;

  // keys that share their data are aliases of one value (as "R",
  // "rotation" and "quaternion" of a frame): one row shows them, under the
  // key the file gives, else the plainest of them
  std::map<const void*, std::vector<std::string>> aliases;
  for(const auto& parameter : object->get_parameters()) {
    if(parameter.second.has_data()) {
      aliases[parameter.second.get_address()].push_back(parameter.first);
    }
  }
  std::set<const void*> shown;
  for(const auto& parameter : object->get_parameters()) {
    shift::core::parameter::Value* value = object->Find(parameter.first);
    if(!value || !value->has_data() ||
       !shown.insert(value->get_address()).second) {
      continue;
    }
    std::vector<std::string> keys = aliases[value->get_address()];
    auto rank = [&](const std::string& key) {
      const bool in_file = json && json->isObject() && json->isMember(key);
      return std::make_tuple(!in_file,
                             object->Find(key)->maps_to_other_type(),
                             key.size(), key);
    };
    std::sort(keys.begin(), keys.end(),
              [&](const std::string& a, const std::string& b) {
                return rank(a) < rank(b);
              });
    const std::string& key = keys.front();
    value = object->Find(key);

    QTreeWidgetItem* row = parent ? new QTreeWidgetItem(parent)
                                  : new QTreeWidgetItem(panel.tree);
    row->setText(0, QString::fromStdString(key));
    QString tooltip = QString::fromStdString(value->get_description());
    if(keys.size() > 1) {
      QStringList others;
      for(size_t i = 1; i < keys.size(); i++) {
        others << QString::fromStdString(keys[i]);
      }
      tooltip += "\n(also given as " + others.join(", ") + ")";
    }
    if(value->is_required()) {
      tooltip += "\n(required)";
    }
    if(value->is_runtime()) {
      tooltip += "\n(may change while the model runs)";
    }
    std::vector<JsonStep> key_path;
    if(path) {
      key_path = *path;
      key_path.push_back({key, -1});
    }
    if(value->is_array()) {
      auto* array = value->get_data<shift::core::parameter::ArrayBase>();
      const size_t size = array ? array->get_size() : 0;
      row->setText(1, size == 0 ? QString("none")
                                : QString("%1 in the model tree").arg(size));
    } else if(value->is_list()) {
      auto* nested = value->get_data<shift::core::parameter::List>();
      if(HasObjects(nested, depth + 1)) {
        row->setText(1, "in the model tree");
      } else if(nested) {
        AddParameterRows(nested, row, depth + 1, path ? &key_path : nullptr);
      }
    } else {
      bool from_file = false;
      for(const auto& alias : keys) {
        from_file = from_file || object->Find(alias)->is_loaded_from_file();
      }
      if(!from_file) {
        // a default, not given in the model file
        row->setForeground(0, default_brush);
        row->setForeground(1, default_brush);
        tooltip += "\n(default: not in the model file)";
      }
      SetRowValue(row, *value);
      row->setData(0, Qt::UserRole, static_cast<int>(panel.rows.size()));
      ParameterRow parameter_row;
      parameter_row.item = row;
      parameter_row.value = value;
      parameter_row.keys = keys;
      parameter_row.in_file = path != nullptr;
      if(path) {
        parameter_row.path = *path;
      }
      panel.rows.push_back(parameter_row);
    }
    tooltip = tooltip.trimmed();
    row->setToolTip(0, tooltip);
    row->setToolTip(1, tooltip);
  }
}

void shift::gui::Workspace::UpdateParameterPanel() {
  auto& panel = parameter_panel_;
  if(!panel.selected || !simulation_.is_loaded()) {
    return;
  }
  const bool computing = simulation_.is_computing();
  for(const ParameterRow& row : panel.rows) {
    if(!computing || row.value->is_runtime()) {
      SetRowValue(row.item, *row.value);
    }
  }
}

// ---------------------------------------------------------- parameter editor

void shift::gui::Workspace::SelectParameterRow() {
  auto& panel = parameter_panel_;
  panel.editing_row = -1;
  panel.editor->setVisible(false);
  QTreeWidgetItem* item = panel.tree->currentItem();
  // the rows below a long value edit it
  while(item && !item->data(0, Qt::UserRole).isValid()) {
    item = item->parent();
  }
  if(!item || !simulation_.is_loaded()) {
    return;
  }
  const int index = item->data(0, Qt::UserRole).toInt();
  if(index < 0 || index >= static_cast<int>(panel.rows.size())) {
    return;
  }
  panel.editing_row = index;
  const ParameterRow& row = panel.rows[index];
  const shift::core::parameter::Value& value = *row.value;

  // new fields for the value
  for(QWidget* widget : panel.editor_fields->findChildren<QWidget*>(
        QString(), Qt::FindDirectChildrenOnly)) {
    delete widget;
  }
  delete panel.editor_fields->layout();
  auto* layout = new QHBoxLayout(panel.editor_fields);
  layout->setContentsMargins(0, 0, 0, 0);
  panel.line_edit = nullptr;
  panel.combo_box = nullptr;
  for(auto& angle : panel.angles) {
    angle = nullptr;
  }
  const auto type = value.get_value_type();
  if(type == shift::core::parameter::Value::BOOLEAN) {
    panel.combo_box = new QComboBox;
    panel.combo_box->addItems({"false", "true"});
    panel.combo_box->setCurrentIndex(value.get<bool>() ? 1 : 0);
    layout->addWidget(panel.combo_box);
  } else if(type == shift::core::parameter::Value::STRING &&
            !value.get_valid_string_values().empty()) {
    panel.combo_box = new QComboBox;
    for(const auto& option : value.get_valid_string_values()) {
      panel.combo_box->addItem(QString::fromStdString(option));
    }
    panel.combo_box->setCurrentText(
      QString::fromStdString(value.get<std::string>()));
    layout->addWidget(panel.combo_box);
  } else if(IsOrientation(value)) {
    double rpy[3];
    RollPitchYaw(OrientationOf(value), rpy);
    const char* names[3] = {"roll", "pitch", "yaw"};
    for(int i = 0; i < 3; i++) {
      layout->addWidget(new QLabel(names[i]));
      panel.angles[i] = new QDoubleSpinBox;
      panel.angles[i]->setRange(i == 1 ? -90.0 : -180.0, i == 1 ? 90.0 : 180.0);
      panel.angles[i]->setDecimals(3);
      panel.angles[i]->setSuffix("°");
      panel.angles[i]->setWrapping(i != 1);
      panel.angles[i]->setValue(rpy[i] * 180.0 / M_PI);
      panel.angles[i]->setToolTip(
        "R = Rz(yaw) Ry(pitch) Rx(roll), in degrees");
      layout->addWidget(panel.angles[i], 1);
    }
  } else {
    panel.line_edit = new QLineEdit;
    QString text;
    if(type == shift::core::parameter::Value::STRING) {
      text = QString::fromStdString(value.get<std::string>());
    } else if(type == shift::core::parameter::Value::DOUBLE) {
      text = QString::number(value.get<double>(), 'g', 12);
    } else if(type == shift::core::parameter::Value::INT) {
      text = QString::number(value.get<int>());
    } else {
      // JSON, as the file gives it if it does
      std::vector<JsonStep> key_path = row.path;
      key_path.push_back({row.keys.front(), -1});
      const Json::Value* json =
        row.in_file ? JsonAt(model_json_, key_path) : nullptr;
      Json::StreamWriterBuilder builder;
      builder.settings_["indentation"] = "";
      text = QString::fromStdString(
        Json::writeString(builder, json ? *json : ValueToJson(value)));
      panel.line_edit->setToolTip("the value as JSON, as in the model file");
    }
    panel.line_edit->setText(text);
    connect(panel.line_edit, &QLineEdit::returnPressed, this,
            &Workspace::ApplyParameterEdit);
    layout->addWidget(panel.line_edit);
  }

  panel.editor_key->setText(QString::fromStdString(row.keys.front()));
  QString note;
  bool editable = true;
  if(value.is_runtime()) {
    note = "applies at once, also while the model runs";
  } else if(!row.in_file) {
    note = "made when the model loads, not read from the model file: it "
           "cannot be edited";
    editable = false;
  } else {
    note = "read when the model loads: applying loads the model again";
    if(simulation_.has_results() || simulation_.is_computing()) {
      note += ", dropping the computed results";
    }
  }
  panel.editor_note->setText(note);
  panel.editor_note->setStyleSheet(QString());
  panel.editor_fields->setEnabled(editable);
  panel.apply_button->setEnabled(editable);
  panel.editor->setVisible(true);
}

void shift::gui::Workspace::ApplyParameterEdit() {
  auto& panel = parameter_panel_;
  if(panel.editing_row < 0 ||
     panel.editing_row >= static_cast<int>(panel.rows.size()) ||
     !simulation_.is_loaded()) {
    return;
  }
  const ParameterRow row = panel.rows[panel.editing_row];
  shift::core::parameter::Value& value = *row.value;
  const auto type = value.get_value_type();
  auto fail = [&panel](const QString& message) {
    panel.editor_note->setText(message);
    panel.editor_note->setStyleSheet("color: #c0392b");
  };

  // the new value, in the form of the model file
  Json::Value json;
  if(panel.combo_box) {
    json = type == shift::core::parameter::Value::BOOLEAN
             ? Json::Value(panel.combo_box->currentIndex() == 1)
             : Json::Value(panel.combo_box->currentText().toStdString());
  } else if(panel.angles[0]) {
    double rpy[3];
    for(int i = 0; i < 3; i++) {
      rpy[i] = panel.angles[i]->value() * M_PI / 180.0;
    }
    // in the form the file gives the orientation in, under its key
    std::vector<JsonStep> key_path = row.path;
    key_path.push_back({row.keys.front(), -1});
    const Json::Value* given =
      row.in_file ? JsonAt(model_json_, key_path) : nullptr;
    json = OrientationToJson(RotationOfRollPitchYaw(rpy), type, given);
  } else if(panel.line_edit) {
    const QString text = panel.line_edit->text().trimmed();
    bool ok = true;
    if(type == shift::core::parameter::Value::STRING) {
      json = Json::Value(text.toStdString());
    } else if(type == shift::core::parameter::Value::DOUBLE) {
      json = Json::Value(text.toDouble(&ok));
    } else if(type == shift::core::parameter::Value::INT) {
      json = Json::Value(text.toInt(&ok));
    } else {
      Json::CharReaderBuilder builder;
      std::string errors;
      std::istringstream in(text.toStdString());
      ok = Json::parseFromStream(builder, in, &json, &errors);
    }
    if(!ok) {
      fail("not a valid value");
      return;
    }
  }

  if(value.is_runtime()) {
    // set at once, through the parameter's own reader
    try {
      value.ReadJSON(json);
    } catch(std::exception& e) {
      fail(QString::fromStdString(e.what()));
      return;
    }
    if(row.in_file) {
      SetJsonParameter(row, json);
      model_edited_ = true;
      UpdateWindowTitle();
    }
    UpdateParameterPanel();
    return;
  }
  if(!row.in_file) {
    return;
  }
  if((simulation_.has_results() || simulation_.is_computing()) &&
     QMessageBox::question(
       this, "Apply edit",
       QString("%1 is read when the model loads. Applying it loads the model "
               "again and drops the computed results. Apply?")
         .arg(QString::fromStdString(row.keys.front()))) != QMessageBox::Yes) {
    return;
  }
  // the run settings of the Simulation tab are kept, unless the edit is of
  // the model parameters
  const bool keep_run = row.path.empty() || row.path.front().key != "model_parameters";
  const double end_time = simulation_.get_end_time();
  const double time_step = simulation_.get_time_step();
  const Json::Value previous = model_json_;
  SetJsonParameter(row, json);
  QString error;
  if(!ReloadModel(previous, error)) {
    // the model is as it was, with the parameter selected again
    fail("not applied: " + error.section('\n', 1).trimmed());
    return;
  }
  if(keep_run) {
    simulation_.set_end_time(end_time);
    simulation_.set_time_step(time_step);
    LoadModelParameters();
  }
}

void shift::gui::Workspace::SetJsonParameter(const ParameterRow& row,
                                             const Json::Value& json) {
  Json::Value* object = &model_json_;
  for(const JsonStep& step : row.path) {
    if(step.index >= 0) {
      object = &(*object)[step.index];
    } else {
      object = &(*object)[step.key]; // made if the file has none
    }
  }
  // the value replaces whichever of its keys the file gave
  for(const auto& key : row.keys) {
    object->removeMember(key);
  }
  (*object)[row.keys.front()] = json;
}

void shift::gui::Workspace::UpdateTree() {
  UpdateParameterPanel();
  if(joint_items_.empty() || !simulation_.is_loaded()) {
    return;
  }
  const auto& joints = simulation_.get_context()->get_active_joints();
  const auto& mobility = simulation_.get_context()->get_mobility_numbers();
  for(size_t i = 0; i < joint_items_.size() && i < joints.size(); i++) {
    QTreeWidgetItem* item = joint_items_[i];
    if(!item) {
      continue;
    }
    shift::core::joint::Base* joint = joints[i];
    if(joint->get_dq_count() == 1) {
      item->setText(1, Number(simulation_.get_shown_q(joint->get_q_index())));
      const int dq_index = joint->get_dq_index();
      item->setText(2, Number(joint->get_substructure() &&
                                  dq_index < static_cast<int>(mobility.size())
                                ? mobility[dq_index]
                                : 1.0,
                              3));
    } else {
      item->setText(1, QString());
      item->setText(2, QString());
    }
  }
  tree_dirty_ = false;
}

void shift::gui::Workspace::SelectTreeItem() {
  joint_panel_.group_box->setVisible(false);
  body_panel_.group_box->setVisible(false);
  geometry_panel_.group_box->setVisible(false);
  joint_panel_.selected_joint = nullptr;
  body_panel_.selected_body = -1;
  geometry_panel_.selected = nullptr;
  ClearSliders();
  ShowParameters(nullptr, QString());

  const auto items = tree_widget_->selectedItems();
  if(items.isEmpty() || !simulation_.is_loaded()) {
    return;
  }
  QTreeWidgetItem* item = items.front();
  const int index = item->data(0, Qt::UserRole).toInt();
  ShowParameters(ItemObject(item), item->text(0));
  if(index < 0) {
    return; // an object without a panel of its own
  }

  if(item->type() == kJointItem) {
    shift::core::joint::Base* joint =
      simulation_.get_context()->get_active_joints()[index];
    joint_panel_.selected_joint = joint;
    joint_panel_.title->setText(QString::fromStdString(joint->get_name()));
    joint_panel_.group_box->setVisible(true);
    SetSliders(joint);
    UpdateJointPanel();
  } else if(item->type() == kBodyItem) {
    body_panel_.selected_body = index;
    body_panel_.title->setText(QString::fromStdString(
      simulation_.get_bodies()[index]->get_name()));
    body_panel_.group_box->setVisible(true);
    UpdateBodyPanel();
    if(!camera_panel_.follow->isChecked()) {
      camera_panel_.body->setCurrentIndex(index);
    }
  } else if(item->type() == kGeometryItem) {
    auto& geometry = simulation_.get_context()->get_fixed_collision_geometry();
    if(index >= 0 && index < static_cast<int>(geometry.size())) {
      geometry_panel_.selected = geometry[index].get();
      geometry_panel_.group_box->setVisible(true);
      UpdateGeometryPanel();
    }
  }
}

void shift::gui::Workspace::UpdateGeometryPanel() {
  auto& panel = geometry_panel_;
  shift::core::collision_geometry::Base* geometry = panel.selected;
  if(!geometry) {
    return;
  }
  panel.title->setText(QString::fromStdString(geometry->get_name()));
  QString details =
    QString("type: %1").arg(QString::fromStdString(geometry->get_type()));
  if(!geometry->get_material().empty()) {
    details +=
      QString(", material: %1")
        .arg(QString::fromStdString(geometry->get_material()));
  }
  const bool plane =
    dynamic_cast<shift::core::collision_geometry::Plane*>(geometry) != nullptr;
  if(plane) {
    details += "\npose of the plane in the fixed frame; its normal (in the "
               "pose) points out of the solid";
  }
  panel.details->setText(details);

  // show the values without applying them back
  panel.contact->blockSignals(true);
  panel.contact->setChecked(geometry->is_contact_enabled());
  panel.contact->blockSignals(false);
  const ksl_SE3_t& pose = geometry->get_pose();
  ksl_vec3_t angles; // yaw, pitch, roll
  ksl_mat3x3_getEulerAnglesWithReference(&pose.R, KSL_AXIS_ZYX, &angles, NULL);
  for(int i = 0; i < 3; i++) {
    panel.translation[i]->blockSignals(true);
    panel.translation[i]->setValue(pose.t.at[i]);
    panel.translation[i]->blockSignals(false);
    panel.translation[i]->setEnabled(plane);
    panel.rotation[i]->blockSignals(true);
    panel.rotation[i]->setValue(angles.at[2 - i] * 180.0 / M_PI);
    panel.rotation[i]->blockSignals(false);
    panel.rotation[i]->setEnabled(plane);
  }
}

void shift::gui::Workspace::ApplyGeometryPose() {
  auto& panel = geometry_panel_;
  auto plane =
    dynamic_cast<shift::core::collision_geometry::Plane*>(panel.selected);
  if(!plane || simulation_.is_computing()) {
    return;
  }
  ksl_SE3_t pose;
  for(int i = 0; i < 3; i++) {
    pose.t.at[i] = panel.translation[i]->value();
  }
  ksl_vec3_t angles = ksl_vec3(panel.rotation[2]->value() * M_PI / 180.0,
                               panel.rotation[1]->value() * M_PI / 180.0,
                               panel.rotation[0]->value() * M_PI / 180.0);
  ksl_mat3x3_setFromEulerAngles(&pose.R, KSL_AXIS_ZYX, &angles);
  plane->set_pose(pose);
  UpdateFloor();
}

void shift::gui::Workspace::UpdateFloor() {
  if(!simulation_.is_loaded()) {
    return;
  }
  for(auto& geometry :
      simulation_.get_context()->get_fixed_collision_geometry()) {
    auto plane =
      dynamic_cast<shift::core::collision_geometry::Plane*>(geometry.get());
    if(plane) {
      // the fixed body's frame is the inertial frame
      const ksl_vec3_t n = plane->get_normal_wrt_body();
      const ksl_vec3_t& p = plane->get_point_wrt_body();
      // a level plane without contact (as the default floor of a model
      // without a ground plane) only marks the ground: keep it below the
      // model; a plane with contact is where the model stands
      if(!plane->is_contact_enabled() && std::fabs(n.z) > 0.999) {
        viewport_->set_floor_below_model(static_cast<float>(p.z));
      } else {
        viewport_->set_floor(QVector3D(p.x, p.y, p.z),
                             QVector3D(n.x, n.y, n.z));
      }
      return;
    }
  }
}

void shift::gui::Workspace::UpdateJointPanel() {
  shift::core::joint::Base* joint = joint_panel_.selected_joint;
  if(!joint) {
    return;
  }
  const QString details =
    QString("type: %1").arg(QString::fromStdString(joint->get_type()));
  joint_panel_.details->setText(details);

  // influence coefficients of the joint's first coordinate; they describe
  // the context configuration, which is the one shown unless results are
  // being played back
  double h[6] = {0, 0, 0, 0, 0, 0};
  const auto& dhva = simulation_.get_context()->get_dhva();
  const int dq_index = joint->get_dq_index();
  if(joint->get_dq_count() > 0 && dq_index >= 0 &&
     dq_index < static_cast<int>(dhva.get_dq_count()) &&
     !simulation_.has_results() && !simulation_.is_computing()) {
    for(int i = 0; i < 6; i++) {
      h[i] = dhva.h[dq_index].at[i];
    }
  }
  for(int i = 0; i < 6; i++) {
    joint_panel_.h[i]->setText(Number(h[i], 3));
  }
}

void shift::gui::Workspace::UpdateBodyPanel() {
  if(body_panel_.selected_body < 0) {
    return;
  }
  Poses poses;
  viewport_->get_poses(poses);
  if(body_panel_.selected_body >= static_cast<int>(poses.bodies.size())) {
    return;
  }
  const QMatrix4x4& m = poses.bodies[body_panel_.selected_body];
  for(int row = 0; row < 3; row++) {
    for(int column = 0; column < 4; column++) {
      body_panel_.m[4 * row + column]->setText(Number(m(row, column), 3));
    }
  }
}

// ---------------------------------------------------------------- sliders

void shift::gui::Workspace::ClearSliders() {
  for(auto handle : {&sliders_.tx, &sliders_.ty, &sliders_.tz, &sliders_.rx,
                     &sliders_.ry, &sliders_.rz}) {
    handle->box->setVisible(false);
    handle->q_index = handle->dq_index = -1;
    handle->euler_index = -1;
    handle->joint = nullptr;
    handle->enabled = false;
    handle->substructure = nullptr;
  }
  sliders_.group_box->setVisible(false);
  jogging_ = nullptr;
  target_move_ = TargetMove();
}

void shift::gui::Workspace::SetSliders(shift::core::joint::Base* joint) {
  const int q = joint->get_q_index();
  const int dq = joint->get_dq_index();
  shift::core::Substructure* substructure = joint->get_substructure();
  const std::string& type = joint->get_type();
  if(type == "revolute" || type == "screw" || type == "gear") {
    SetSlider(sliders_.rz, "R", q, dq, substructure);
  } else if(type == "prismatic") {
    SetSlider(sliders_.tz, "T", q, dq, substructure);
  } else if(type == "spherical" || type == "float") {
    // q = [quaternion] or [t, quaternion]; the rotation is jogged as roll,
    // pitch and yaw angles, starting from the angles of the current quaternion
    const int offset = type == "float" ? 3 : 0;
    if(type == "float") {
      SetSlider(sliders_.tx, "Tx", q, dq, substructure);
      SetSlider(sliders_.ty, "Ty", q + 1, dq + 1, substructure);
      SetSlider(sliders_.tz, "Tz", q + 2, dq + 2, substructure);
      if(substructure) {
        // dq = [v, omega] has v in the child frame, so a translation
        // coordinate has no single q entry to specify
        for(auto handle : {&sliders_.tx, &sliders_.ty, &sliders_.tz}) {
          handle->enabled = false;
          handle->box->setEnabled(false);
        }
      }
    }
    for(double& angle : sliders_.rpy_reference) {
      angle = 0.0;
    }
    SetSlider(sliders_.rx, "Roll", q + offset, dq + offset, substructure, 0);
    SetSlider(sliders_.ry, "Pitch", q + offset, dq + offset + 1, substructure,
              1);
    SetSlider(sliders_.rz, "Yaw", q + offset, dq + offset + 2, substructure,
              2);
    for(auto handle : {&sliders_.rx, &sliders_.ry, &sliders_.rz}) {
      handle->joint = joint;
    }
  } else {
    return;
  }
  // as in the original workspace, show only the slider groups and gain dials
  // of the joint's coordinates
  const bool translation = type == "prismatic" || type == "float";
  const bool rotation = type != "prismatic";
  sliders_.translation_group->setVisible(translation);
  sliders_.translation_gain_dial->setVisible(translation);
  sliders_.rotation_group->setVisible(rotation);
  sliders_.rotation_gain_dial->setVisible(rotation);
  sliders_.group_box->setVisible(true);
}

void shift::gui::Workspace::SetSlider(SliderHandle& handle, const char* label,
                                      int q_index, int dq_index,
                                      shift::core::Substructure* substructure,
                                      int euler_index) {
  handle.q_index = q_index;
  handle.dq_index = dq_index;
  handle.euler_index = euler_index;
  handle.substructure = substructure;
  // in a kinematic loop a joint coordinate is moved with
  // Substructure::IncrementSpecifiedCoordinate, and roll, pitch and yaw with
  // Substructure::SetSpecifiedRotation; SetSliders disables the translation
  // of a float joint in a loop, which has no single q entry
  handle.enabled = true;
  handle.label->setText(label);
  handle.box->setEnabled(handle.enabled);
  handle.box->setVisible(true);
  RecenterSlider(handle);
}

void shift::gui::Workspace::RecenterSlider(SliderHandle& handle) {
  if(jogging_ == &handle) {
    jogging_ = nullptr;
  }
  handle.slider->blockSignals(true);
  handle.slider->setValue(0);
  handle.slider->blockSignals(false);
  handle.spin_box->blockSignals(true);
  handle.spin_box->setValue(get_shown_value(handle));
  handle.spin_box->blockSignals(false);
}

void shift::gui::Workspace::PressSlider(SliderHandle& handle) {
  playing_ = false;
  target_move_ = TargetMove();
  jogging_ = nullptr;
  if(!handle.enabled || !simulation_.AdoptShownState()) {
    status_label_->setText("Stop the simulation to move joints");
    return;
  }
  if(handle.substructure) {
    // drive constraint error to zero before applying fast factorization, with
    // the coordinate (or all three rotation coordinates of a roll, pitch or
    // yaw slider) specified as independent
    try {
      if(handle.euler_index >= 0) {
        handle.substructure->SetupNewtonRaphsonSpecifiedRotation(
          simulation_.get_context()->get_dhva(), handle.joint);
      } else {
        handle.substructure->SetupNewtonRaphsonSpecifiedCoordinate(
          simulation_.get_context()->get_dhva(), handle.dq_index);
      }
    } catch(std::exception& e) {
      status_label_->setText(QString::fromStdString(e.what()));
      return;
    }
    simulation_.ShowContextState();
  }
  jogging_ = &handle;
  tree_dirty_ = true;
  viewport_->update();
}

void shift::gui::Workspace::SetTarget(SliderHandle& handle, double value) {
  playing_ = false;
  jogging_ = nullptr;
  PressSlider(handle);
  if(jogging_ != &handle) {
    RecenterSlider(handle);
    return;
  }
  jogging_ = nullptr;
  // allow up to 1000 steps for each radian or unit of translation
  const double distance =
    std::fabs(value - get_shown_value(handle));
  target_move_.handle = &handle;
  target_move_.target = value;
  target_move_.counter = 0;
  target_move_.max_count = static_cast<int>(distance * 1000) + 1;
}

double shift::gui::Workspace::get_mobility(const SliderHandle& handle) {
  // a rotation in a loop is checked for admissibility by
  // Substructure::SetSpecifiedRotation rather than scaled by the mobility of
  // a single rotation coordinate
  if(!handle.substructure || handle.euler_index >= 0) {
    return 1.0;
  }
  const auto& mobility = simulation_.get_context()->get_mobility_numbers();
  if(handle.dq_index < 0 ||
     handle.dq_index >= static_cast<int>(mobility.size())) {
    return 1.0;
  }
  return mobility[handle.dq_index];
}

bool shift::gui::Workspace::IncrementCoordinate(SliderHandle& handle,
                                                double delta) {
  if(handle.euler_index >= 0) {
    return SetRollPitchYaw(handle, get_shown_value(handle) + delta);
  }
  shift::core::Context* context = simulation_.get_context();
  try {
    if(handle.substructure) {
      // specify the coordinate as independent, increment it, and solve the
      // loop closure constraints by Newton-Raphson iteration
      handle.substructure->IncrementSpecifiedCoordinate(
        context->get_dhva(), handle.dq_index, delta,
        shift::core::Substructure::INCREMENT);
      if(context->get_newton_raphson_failure()) {
        return false;
      }
    } else {
      context->get_q()[handle.q_index] += delta;
      context->UpdateDH();
    }
  } catch(std::exception&) {
    return false;
  }
  simulation_.ShowContextState();
  return true;
}

bool shift::gui::Workspace::SetCoordinate(SliderHandle& handle,
                                          double value) {
  if(handle.euler_index >= 0) {
    return SetRollPitchYaw(handle, value);
  }
  shift::core::Context* context = simulation_.get_context();
  try {
    if(handle.substructure) {
      handle.substructure->IncrementSpecifiedCoordinate(
        context->get_dhva(), handle.dq_index, value,
        shift::core::Substructure::TARGET);
      if(context->get_newton_raphson_failure()) {
        return false;
      }
    } else {
      context->get_q()[handle.q_index] = value;
      context->UpdateDH();
    }
  } catch(std::exception&) {
    return false;
  }
  simulation_.ShowContextState();
  return true;
}

double shift::gui::Workspace::get_shown_value(const SliderHandle& handle) {
  if(handle.euler_index < 0) {
    return simulation_.get_shown_q(handle.q_index);
  }
  double rpy[3];
  get_shown_roll_pitch_yaw(handle.q_index, rpy);
  return rpy[handle.euler_index];
}

void shift::gui::Workspace::get_shown_roll_pitch_yaw(int q_index,
                                                     double rpy[3]) {
  // interpolated playback quaternions need not be unit length
  ksl_quaternion_t quaternion;
  for(int i = 0; i < 4; i++) {
    quaternion.at[i] = simulation_.get_shown_q(q_index + i);
  }
  ksl_quaternion_normalize(&quaternion);
  ksl_mat3x3_t r;
  ksl_quaternion_toMat3x3(&quaternion, &r);

  // R = Rz(yaw) Ry(pitch) Rx(roll) is the ZYX sequence with angles
  // [yaw, pitch, roll]; the reference keeps the angles continuous
  ksl_vec3_t reference =
    ksl_vec3(sliders_.rpy_reference[2], sliders_.rpy_reference[1],
             sliders_.rpy_reference[0]);
  ksl_vec3_t angles;
  ksl_mat3x3_getEulerAnglesWithReference(&r, KSL_AXIS_ZYX, &angles,
                                         &reference);
  rpy[0] = angles.at[2];
  rpy[1] = angles.at[1];
  rpy[2] = angles.at[0];
  for(int i = 0; i < 3; i++) {
    sliders_.rpy_reference[i] = rpy[i];
  }
}

bool shift::gui::Workspace::SetRollPitchYaw(SliderHandle& handle,
                                            double value) {
  shift::core::Context* context = simulation_.get_context();
  double rpy[3];
  get_shown_roll_pitch_yaw(handle.q_index, rpy);
  rpy[handle.euler_index] = value;

  ksl_vec3_t angles = ksl_vec3(rpy[2], rpy[1], rpy[0]);
  ksl_mat3x3_t r;
  ksl_mat3x3_setFromEulerAngles(&r, KSL_AXIS_ZYX, &angles);
  ksl_quaternion_t quaternion;
  ksl_mat3x3_toQuaternion(&r, &quaternion);

  if(handle.substructure) {
    // the dependent coordinates of the loop follow the rotation
    try {
      handle.substructure->SetSpecifiedRotation(context->get_dhva(),
                                                handle.joint, quaternion.at);
    } catch(std::exception& e) {
      jog_error_ = QString::fromStdString(e.what());
      return false;
    }
    for(int i = 0; i < 3; i++) {
      sliders_.rpy_reference[i] = rpy[i];
    }
    simulation_.ShowContextState();
    return true;
  }

  // q and -q are the same rotation; keep the sign of the current quaternion
  double* q = &context->get_q()[handle.q_index];
  double dot = 0.0;
  for(int i = 0; i < 4; i++) {
    dot += q[i] * quaternion.at[i];
  }
  const double sign = dot < 0.0 ? -1.0 : 1.0;
  for(int i = 0; i < 4; i++) {
    q[i] = sign * quaternion.at[i];
  }
  try {
    context->UpdateDH();
  } catch(std::exception&) {
    return false;
  }
  // continue from the angles as set, rather than as recovered from R
  for(int i = 0; i < 3; i++) {
    sliders_.rpy_reference[i] = rpy[i];
  }
  simulation_.ShowContextState();
  return true;
}

void shift::gui::Workspace::Jog() {
  bool moved = false;

  if(jogging_ && jogging_->slider->value() != 0) {
    // the slider deflection is a rate: increment the coordinate each frame;
    // constrained coordinates move in proportion to their mobility
    SliderHandle& h = *jogging_;
    double delta = h.slider->value() * h.single_step * DialGain(h.gain_dial);
    delta *= h.substructure ? std::pow(get_mobility(h), 1.5) : 1.5;
    if(std::fabs(delta) > 1e-16) {
      jog_error_.clear();
      if(!IncrementCoordinate(h, delta)) {
        status_label_->setText(jog_error_.isEmpty()
                                 ? "Newton-Raphson failed to converge"
                                 : jog_error_);
        RecenterSlider(h);
      }
      moved = true;
    }
    h.spin_box->blockSignals(true);
    h.spin_box->setValue(get_shown_value(h));
    h.spin_box->blockSignals(false);
  }

  if(target_move_.handle) {
    SliderHandle& h = *target_move_.handle;
    const double remaining =
      target_move_.target - get_shown_value(h);
    const double step =
      0.01 * (h.substructure ? std::pow(get_mobility(h), 1.5) : 1.0);
    bool done =
      ++target_move_.counter > target_move_.max_count || step < 1e-16;
    if(!done) {
      jog_error_.clear();
      bool ok;
      if(std::fabs(remaining) <= step) {
        ok = SetCoordinate(h, target_move_.target);
        done = true;
      } else {
        ok = IncrementCoordinate(h, remaining > 0 ? step : -step);
      }
      if(!ok) {
        status_label_->setText(jog_error_.isEmpty()
                                 ? "Newton-Raphson failed to converge"
                                 : jog_error_);
        done = true;
      }
      moved = true;
    }
    if(done) {
      target_move_ = TargetMove();
      RecenterSlider(h);
    }
  }

  if(moved) {
    tree_dirty_ = true;
    viewport_->update();
  }
}

// ------------------------------------------------------------- simulation

void shift::gui::Workspace::RunSimulation() {
  if(!simulation_.is_loaded() || simulation_.is_computing()) {
    return;
  }
  jogging_ = nullptr;
  target_move_ = TargetMove();
  simulation_.Run();
  ClearVideoRange();
  playback_time_ = 0.0;
  playing_ = true;
  was_computing_ = true;
  playback_panel_.slider->set_time_span(0.0, simulation_.get_end_time());
  playback_panel_.slider->set_progress_time(0.0);
  playback_panel_.slider->set_time(0.0);
  playback_panel_.play_pause_button->setIcon(playback_panel_.pause_icon);
  UpdateStatus();
}

void shift::gui::Workspace::StopSimulation() {
  simulation_.Stop();
  UpdateStatus();
}

void shift::gui::Workspace::ResetSimulation() {
  if(!simulation_.is_loaded()) {
    return;
  }
  QApplication::setOverrideCursor(Qt::WaitCursor);
  playing_ = false;
  jogging_ = nullptr;
  target_move_ = TargetMove();
  simulation_.Reset();
  QApplication::restoreOverrideCursor();
  ClearVideoRange();
  playback_time_ = 0.0;
  playback_panel_.slider->set_progress_time(0.0);
  playback_panel_.slider->set_time(0.0);
  playback_panel_.slider->Enable(false);
  playback_panel_.play_pause_button->setIcon(playback_panel_.play_icon);
  SelectTreeItem();
  tree_dirty_ = true;
  viewport_->update();
  UpdateStatus();
}

void shift::gui::Workspace::PlayPauseSimulation() {
  if(!simulation_.is_loaded()) {
    return;
  }
  if(!simulation_.has_results() && !simulation_.is_computing()) {
    RunSimulation();
    return;
  }
  if(playing_) {
    playing_ = false;
  } else {
    // restart from the beginning once the end has been reached
    if(!simulation_.is_computing() &&
       playback_time_ >= simulation_.get_computed_time()) {
      playback_time_ = 0.0;
    }
    playing_ = true;
  }
  playback_panel_.play_pause_button->setIcon(
    playing_ ? playback_panel_.pause_icon : playback_panel_.play_icon);
}

void shift::gui::Workspace::SetPlaybackTime(double t) {
  // dragging the playback slider pauses playback
  playing_ = false;
  playback_panel_.play_pause_button->setIcon(playback_panel_.play_icon);
  playback_time_ = t;
  simulation_.ShowTime(t);
  tree_dirty_ = true;
  viewport_->update();
}

void shift::gui::Workspace::Tick() {
  const double dt = tick_clock_.restart() / 1000.0;
  if(!simulation_.is_loaded()) {
    if(++tick_count_ % 15 == 0) {
      UpdateStatus();
    }
    return;
  }

  const bool computing = simulation_.is_computing();
  const bool has_results = simulation_.has_results();
  const double computed = simulation_.get_computed_time();
  if(computing || has_results) {
    playback_panel_.slider->set_progress_time(computed);
  }
  playback_panel_.slider->Enable(has_results);
  if(has_results) {
    video_panel_.trimmer->set_span(computed);
    if(!computing && computed != video_panel_.thumbnails_time &&
       !video_panel_.exporting && video_panel_.trimmer->isVisible()) {
      RenderThumbnails();
    }
  }

  if(playing_ && has_results) {
    playback_time_ += dt * playback_panel_.speed->value();
    if(playback_time_ >= computed) {
      // wait for the worker to catch up, or stop at the end of the results
      playback_time_ = computed;
      if(!computing) {
        playing_ = false;
        playback_panel_.play_pause_button->setIcon(playback_panel_.play_icon);
      }
    }
    simulation_.ShowTime(playback_time_);
    playback_panel_.slider->set_time(playback_time_);
    tree_dirty_ = true;
    viewport_->update();
  }

  if(was_computing_ && !computing) {
    // the run ended: report a failure once
    was_computing_ = false;
    const std::string error = simulation_.get_error();
    if(!error.empty()) {
      QMessageBox::warning(this, "Simulation stopped",
                           QString::fromStdString(error));
    }
  }

  Jog();

  if(++tick_count_ % 15 == 0) {
    if(tree_dirty_) {
      UpdateTree();
      UpdateJointPanel();
      UpdateBodyPanel();
    }
    UpdateStatus();
  }
}

void shift::gui::Workspace::UpdateStatus() {
  const bool loaded = simulation_.is_loaded();
  const bool computing = loaded && simulation_.is_computing();

  QString text;
  if(!loaded) {
    text = "Open a model (Ctrl+O)";
  } else if(computing) {
    text = "Computing…";
  } else if(!simulation_.get_error().empty()) {
    text = "Stopped: " + QString::fromStdString(simulation_.get_error());
  } else if(simulation_.has_results()) {
    text = playing_ ? "Playing" : "Paused";
  } else {
    text = "Ready: press Play or Run (Ctrl+R), or drag a joint slider";
  }
  if(!jogging_ && !target_move_.handle) {
    status_label_->setText(text);
  }

  auto& panel = model_parameters_panel_;
  panel.run_button->setEnabled(loaded && !computing);
  panel.stop_button->setEnabled(computing);
  panel.reset_button->setEnabled(loaded);
  panel.time_span->setEnabled(!computing);
  panel.time_step->setEnabled(!computing);
  video_panel_.export_button->setEnabled(loaded && !computing &&
                                         simulation_.has_results() &&
                                         !video_panel_.exporting);
  sliders_.group_box->setEnabled(!computing);
  geometry_panel_.group_box->setEnabled(!computing);

  if(loaded) {
    time_label_->setText(QString("t = %1 s, computed %2 / %3 s")
                           .arg(playback_time_, 0, 'f', 3)
                           .arg(simulation_.get_computed_time(), 0, 'f', 3)
                           .arg(simulation_.get_end_time(), 0, 'f', 3));
  } else {
    time_label_->clear();
  }
  const double rate = simulation_.get_real_time_factor();
  rate_label_->setText(computing && rate > 0.0
                         ? QString("  %1× real time").arg(rate, 0, 'f', 2)
                         : QString());

  const double seconds = fps_clock_.restart() / 1000.0;
  if(seconds > 0.0) {
    fps_label_->setText(QString("  %1 fps").arg(
      std::lround(viewport_->TakeFrameCount() / seconds)));
  }
}

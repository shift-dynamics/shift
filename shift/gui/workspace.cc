#include "shift/gui/workspace.h"

#include "shift/core/assembly.h"
#include "shift/core/collision_geometry/plane.h"
#include "shift/core/context.h"
#include "shift/core/substructure.h"
#include "shift/core/body/base.h"
#include "shift/core/body/fixed.h"
#include "shift/core/body/virtual.h"
#include "shift/core/joint/base.h"
#include "shift/core/light/base.h"
#include "shift/core/terrain/base.h"
#include "shift/gui/playback_slider.h"
#include "shift/gui/video_encoder.h"
#include "shift/gui/video_trimmer.h"
#include "shift/gui/viewport.h"

#include "ksl/mat3x3.h"
#include "ksl/quaternion.h"

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDial>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QImage>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QProgressDialog>
#include <QPushButton>
#include <QShortcut>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>


static constexpr int kBodyItem = QTreeWidgetItem::UserType + 1;
static constexpr int kJointItem = QTreeWidgetItem::UserType + 2;
static constexpr int kGeometryItem = QTreeWidgetItem::UserType + 3;
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
  tree_widget_->setColumnWidth(0, 220);
  scene_splitter->addWidget(tree_widget_);
  connect(tree_widget_, &QTreeWidget::itemSelectionChanged, this,
          &Workspace::SelectTreeItem);

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
  QApplication::setOverrideCursor(Qt::WaitCursor);
  viewport_->set_simulation(nullptr);
  playing_ = false;
  jogging_ = nullptr;
  target_move_ = TargetMove();
  ClearSliders();
  tree_widget_->clear();
  joint_items_.clear();
  geometry_panel_.selected = nullptr;

  std::string error;
  try {
    simulation_.Load(filename.toStdString());
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

  PopulateTree();
  LoadModelParameters();
  ClearVideoRange();
  playback_time_ = 0.0;
  playback_panel_.slider->set_time_span(0.0, simulation_.get_end_time());
  playback_panel_.slider->set_progress_time(0.0);
  playback_panel_.slider->set_time(0.0);
  playback_panel_.slider->Enable(false);
  playback_panel_.play_pause_button->setIcon(playback_panel_.play_icon);
  viewport_->set_simulation(&simulation_);
  camera_panel_.follow->setChecked(false);
  camera_panel_.body->clear();
  for(const auto* body : simulation_.get_bodies()) {
    camera_panel_.body->addItem(QString::fromStdString(body->get_name()));
  }
  camera_panel_.group_box->setVisible(true);
  UpdateFloor();
  // the floor plane would cut through a terrain
  floor_action_->setChecked(simulation_.get_scene().terrains.empty());
  setWindowTitle(QString("Shift — %1").arg(QFileInfo(filename).fileName()));
  UpdateStatus();
  return true;
}

void shift::gui::Workspace::SaveImage(const ImageRequest& request) {
  image_request_ = request;
  if(request.hide_floor) {
    floor_action_->setChecked(false);
  }
  if(request.samples > 0) {
    viewport_->set_render_mode(Viewport::RenderMode::kRayTraced);
  }
  if(request.time > 0.0 && simulation_.is_loaded()) {
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
    this, "Open model", QString(), "Shift models (*.json *.yaml *.yml);;All files (*)");
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

/* the tree follows the layout of the model file: the fixed body, with what
is attached to it, above the assemblies with their bodies, joints and nested
assemblies */
void shift::gui::Workspace::PopulateTree() {
  tree_widget_->clear();
  joint_items_.assign(simulation_.get_context()->get_active_joints().size(),
                      nullptr);
  AddFixedItems();
  for(auto& assembly : simulation_.get_context()->get_assemblies()) {
    AddAssemblyItem(assembly.get(), nullptr);
  }
  tree_widget_->expandAll();
  UpdateTree();
}

static QTreeWidgetItem* NewItem(QTreeWidgetItem* parent, QTreeWidget* tree,
                                const std::string& text, const char* icon,
                                int type = QTreeWidgetItem::Type) {
  QTreeWidgetItem* item =
    parent ? new QTreeWidgetItem(parent, {QString::fromStdString(text)}, type)
           : new QTreeWidgetItem(tree, {QString::fromStdString(text)}, type);
  item->setIcon(0, QIcon(icon));
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

void shift::gui::Workspace::AddFixedItems() {
  shift::core::Context* context = simulation_.get_context();
  // the fixed body is the first body of the simulation
  QTreeWidgetItem* fixed =
    NewItem(nullptr, tree_widget_, "fixed", ":/icons/anchor.png", kBodyItem);
  fixed->setData(0, Qt::UserRole, 0);

  auto& frames = context->get_frames();
  if(!frames.empty()) {
    QTreeWidgetItem* folder =
      NewItem(fixed, nullptr, "frames", ":/icons/frame_folder.png");
    for(auto& frame : frames) {
      NewItem(folder, nullptr, frame->get_name(), ":/icons/frame.png");
    }
  }
  auto& lights = context->get_lights();
  if(!lights.empty()) {
    QTreeWidgetItem* folder =
      NewItem(fixed, nullptr, "lights", ":/icons/light_folder.png");
    for(auto& light : lights) {
      NewItem(folder, nullptr, light->get_name(), ":/icons/light.png");
    }
  }
  auto& terrain = context->get_terrain();
  if(!terrain.empty()) {
    QTreeWidgetItem* folder =
      NewItem(fixed, nullptr, "terrain", ":/icons/terrain_folder.png");
    for(auto& patch : terrain) {
      NewItem(folder, nullptr, patch->get_name(), ":/icons/terrain.png");
    }
  }
  // collision geometry, such as the floor, directly under the fixed body
  auto& geometry = context->get_fixed_collision_geometry();
  for(size_t i = 0; i < geometry.size(); i++) {
    QTreeWidgetItem* item =
      NewItem(fixed, nullptr, geometry[i]->get_name(),
              GeometryIcon(geometry[i]->get_type()), kGeometryItem);
    item->setData(0, Qt::UserRole, int(i));
  }
}

void shift::gui::Workspace::AddAssemblyItem(shift::core::Assembly* assembly,
                                            QTreeWidgetItem* parent) {
  const std::string name =
    assembly->get_name().empty() ? "assembly" : assembly->get_name();
  QTreeWidgetItem* item =
    NewItem(parent, tree_widget_, name, ":/icons/assembly.png");

  if(!assembly->get_bodies().empty()) {
    QTreeWidgetItem* folder =
      NewItem(item, nullptr, "bodies", ":/icons/body_folder.png");
    for(auto& body : assembly->get_bodies()) {
      AddBodyItem(body.get(), folder);
    }
    if(folder->childCount() == 0) {
      delete folder;
    }
  }

  const auto& active = simulation_.get_context()->get_active_joints();
  if(!assembly->get_joints().empty()) {
    QTreeWidgetItem* folder =
      NewItem(item, nullptr, "joints", ":/icons/joint_folder.png");
    for(auto& joint : assembly->get_joints()) {
      auto found = std::find(active.begin(), active.end(), joint.get());
      if(found == active.end()) {
        continue; // inactive
      }
      const int index = static_cast<int>(found - active.begin());
      QTreeWidgetItem* joint_item =
        NewItem(folder, nullptr,
                joint->get_name() + " (" + joint->get_type() + ")",
                ":/icons/joint.png", kJointItem);
      joint_item->setData(0, Qt::UserRole, index);
      joint_items_[index] = joint_item;
    }
    if(folder->childCount() == 0) {
      delete folder;
    }
  }

  for(auto& nested : assembly->get_assemblies()) {
    AddAssemblyItem(nested.get(), item);
  }
}

void shift::gui::Workspace::AddBodyItem(shift::core::body::Base* body,
                                        QTreeWidgetItem* parent) {
  // virtual bodies are massless helpers generated for joints
  if(dynamic_cast<shift::core::body::Virtual*>(body)) {
    return;
  }
  const auto& bodies = simulation_.get_bodies();
  auto found = std::find(bodies.begin(), bodies.end(), body);
  if(found == bodies.end()) {
    return; // inactive
  }
  QTreeWidgetItem* item = NewItem(parent, nullptr, body->get_name(),
                                  ":/icons/body.png", kBodyItem);
  item->setData(0, Qt::UserRole, static_cast<int>(found - bodies.begin()));
}

void shift::gui::Workspace::UpdateTree() {
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

  const auto items = tree_widget_->selectedItems();
  if(items.isEmpty() || !simulation_.is_loaded()) {
    return;
  }
  QTreeWidgetItem* item = items.front();
  const int index = item->data(0, Qt::UserRole).toInt();

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

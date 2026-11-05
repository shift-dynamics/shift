#include "shift/gui/workspace.h"

#include "shift/core/context.h"
#include "shift/core/substructure.h"
#include "shift/core/body/base.h"
#include "shift/core/body/virtual.h"
#include "shift/core/joint/base.h"
#include "shift/gui/playback_slider.h"
#include "shift/gui/viewport.h"

#include <QAction>
#include <QApplication>
#include <QDial>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QSlider>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <cmath>
#include <exception>

namespace {

constexpr int kBodyItem = QTreeWidgetItem::UserType + 1;
constexpr int kJointItem = QTreeWidgetItem::UserType + 2;
constexpr int kTickMilliseconds = 16;

/*! gain dial 0..100 maps to a slider rate multiplier of 0.01..100 */
double DialGain(const QDial* dial) {
  return std::pow(10.0, (dial->value() - 50) / 25.0);
}

QString Number(double value, int decimals = 4) {
  return QString::number(value, 'f', decimals);
}

} // namespace

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

  dynamic_layout->addStretch();
  scene_splitter->addWidget(dynamic_box);

  // simulation tab: model parameters
  auto* simulation_box = new QWidget;
  auto* simulation_layout = new QVBoxLayout(simulation_box);
  simulation_layout->setContentsMargins(0, 0, 0, 0);
  simulation_layout->addWidget(AddModelParametersBox());
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

bool shift::gui::Workspace::Open(const QString& filename) {
  QApplication::setOverrideCursor(Qt::WaitCursor);
  viewport_->set_simulation(nullptr);
  playing_ = false;
  jogging_ = nullptr;
  target_move_ = TargetMove();
  ClearSliders();
  tree_widget_->clear();
  bodies_item_ = joints_item_ = nullptr;

  std::string error;
  try {
    simulation_.Load(filename.toStdString());
  } catch(std::exception& e) {
    error = e.what();
  }
  QApplication::restoreOverrideCursor();
  if(!error.empty()) {
    QMessageBox::critical(this, "Open failed",
                          QString("Could not load %1:\n%2")
                            .arg(filename, QString::fromStdString(error)));
    model_parameters_panel_.group_box->setVisible(false);
    UpdateStatus();
    return false;
  }

  PopulateTree();
  LoadModelParameters();
  playback_time_ = 0.0;
  playback_panel_.slider->set_time_span(0.0, simulation_.get_end_time());
  playback_panel_.slider->set_progress_time(0.0);
  playback_panel_.slider->set_time(0.0);
  playback_panel_.slider->Enable(false);
  playback_panel_.play_pause_button->setIcon(playback_panel_.play_icon);
  viewport_->set_simulation(&simulation_);
  setWindowTitle(QString("Shift — %1").arg(QFileInfo(filename).fileName()));
  UpdateStatus();
  return true;
}

void shift::gui::Workspace::OpenDialog() {
  const QString filename = QFileDialog::getOpenFileName(
    this, "Open model", QString(), "Shift models (*.json);;All files (*)");
  if(!filename.isEmpty()) {
    Open(filename);
  }
}

void shift::gui::Workspace::About() {
  QMessageBox::about(
    this, "About Shift",
    "<b>Shift</b> multibody dynamics<br>"
    "Kristopher Wehage<br>"
    "<a href=\"http://shift-dynamics.io/\">shift-dynamics.io</a>");
}

void shift::gui::Workspace::LoadModelParameters() {
  auto& panel = model_parameters_panel_;
  panel.group_box->setVisible(true);
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

void shift::gui::Workspace::PopulateTree() {
  tree_widget_->clear();
  bodies_item_ = new QTreeWidgetItem(tree_widget_, {"Bodies"});
  bodies_item_->setIcon(0, QIcon(":/icons/body_folder.png"));
  const auto& bodies = simulation_.get_bodies();
  for(size_t i = 0; i < bodies.size(); i++) {
    // virtual bodies are massless helpers generated for joints
    if(dynamic_cast<shift::core::body::Virtual*>(bodies[i])) {
      continue;
    }
    auto* item = new QTreeWidgetItem(
      bodies_item_, {QString::fromStdString(bodies[i]->get_name())},
      kBodyItem);
    item->setIcon(0, QIcon(":/icons/body.png"));
    item->setData(0, Qt::UserRole, int(i));
  }

  joints_item_ = new QTreeWidgetItem(tree_widget_, {"Joints"});
  joints_item_->setIcon(0, QIcon(":/icons/joint_folder.png"));
  const auto& joints = simulation_.get_context()->get_active_joints();
  for(size_t i = 0; i < joints.size(); i++) {
    auto* item = new QTreeWidgetItem(
      joints_item_,
      {QString::fromStdString(joints[i]->get_name() + " (" +
                              joints[i]->get_type() + ")")},
      kJointItem);
    item->setIcon(0, QIcon(":/icons/joint.png"));
    item->setData(0, Qt::UserRole, int(i));
  }
  tree_widget_->expandAll();
  UpdateTree();
}

void shift::gui::Workspace::UpdateTree() {
  if(!joints_item_ || !simulation_.is_loaded()) {
    return;
  }
  const auto& joints = simulation_.get_context()->get_active_joints();
  const auto& mobility = simulation_.get_context()->get_mobility_numbers();
  for(int i = 0; i < joints_item_->childCount(); i++) {
    QTreeWidgetItem* item = joints_item_->child(i);
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
  joint_panel_.selected_joint = nullptr;
  body_panel_.selected_body = -1;
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
  }
}

void shift::gui::Workspace::UpdateJointPanel() {
  shift::core::joint::Base* joint = joint_panel_.selected_joint;
  if(!joint) {
    return;
  }
  QString details =
    QString("type: %1").arg(QString::fromStdString(joint->get_type()));
  details += joint->get_substructure()
               ? ", in a kinematic substructure (Newton-Raphson)"
               : ", unconstrained";
  if(joint->get_type() == "spherical" || joint->get_type() == "float") {
    details += "<br>rotations are quaternions and cannot be jogged";
  }
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
  if(type == "revolute") {
    SetSlider(sliders_.rz, "R", q, dq, substructure, false);
  } else if(type == "prismatic") {
    SetSlider(sliders_.tz, "T", q, dq, substructure, false);
  } else if(type == "spherical") {
    SetSlider(sliders_.rx, "Rx", q, dq, substructure, true);
    SetSlider(sliders_.ry, "Ry", q + 1, dq + 1, substructure, true);
    SetSlider(sliders_.rz, "Rz", q + 2, dq + 2, substructure, true);
  } else if(type == "float") {
    SetSlider(sliders_.tx, "Tx", q, dq, substructure, false);
    SetSlider(sliders_.ty, "Ty", q + 1, dq + 1, substructure, false);
    SetSlider(sliders_.tz, "Tz", q + 2, dq + 2, substructure, false);
    SetSlider(sliders_.rx, "Rx", q + 3, dq + 3, substructure, true);
    SetSlider(sliders_.ry, "Ry", q + 4, dq + 4, substructure, true);
    SetSlider(sliders_.rz, "Rz", q + 5, dq + 5, substructure, true);
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
                                      bool is_quaternion) {
  handle.q_index = q_index;
  handle.dq_index = dq_index;
  handle.substructure = substructure;
  // Quaternion coordinates cannot be incremented one at a time (as in the
  // original workspace). Substructure::IncrementSpecifiedCoordinate indexes q
  // with the dq index, which requires the two to coincide.
  handle.enabled =
    !is_quaternion && (!substructure || q_index == dq_index);
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
  handle.spin_box->setValue(simulation_.get_shown_q(handle.q_index));
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
    // drive constraint error to zero before applying fast factorization
    try {
      handle.substructure->SetupNewtonRaphsonSpecifiedCoordinate(
        simulation_.get_context()->get_dhva(), handle.dq_index);
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
    std::fabs(value - simulation_.get_shown_q(handle.q_index));
  target_move_.handle = &handle;
  target_move_.target = value;
  target_move_.counter = 0;
  target_move_.max_count = static_cast<int>(distance * 1000) + 1;
}

double shift::gui::Workspace::get_mobility(const SliderHandle& handle) {
  if(!handle.substructure) {
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

void shift::gui::Workspace::Jog() {
  bool moved = false;

  if(jogging_ && jogging_->slider->value() != 0) {
    // the slider deflection is a rate: increment the coordinate each frame;
    // constrained coordinates move in proportion to their mobility
    SliderHandle& h = *jogging_;
    double delta = h.slider->value() * h.single_step * DialGain(h.gain_dial);
    delta *= h.substructure ? std::pow(get_mobility(h), 1.5) : 1.5;
    if(std::fabs(delta) > 1e-16) {
      if(!IncrementCoordinate(h, delta)) {
        status_label_->setText("Newton-Raphson failed to converge");
        RecenterSlider(h);
      }
      moved = true;
    }
    h.spin_box->blockSignals(true);
    h.spin_box->setValue(simulation_.get_shown_q(h.q_index));
    h.spin_box->blockSignals(false);
  }

  if(target_move_.handle) {
    SliderHandle& h = *target_move_.handle;
    const double remaining =
      target_move_.target - simulation_.get_shown_q(h.q_index);
    const double step =
      0.01 * (h.substructure ? std::pow(get_mobility(h), 1.5) : 1.0);
    bool done =
      ++target_move_.counter > target_move_.max_count || step < 1e-16;
    if(!done) {
      bool ok;
      if(std::fabs(remaining) <= step) {
        ok = SetCoordinate(h, target_move_.target);
        done = true;
      } else {
        ok = IncrementCoordinate(h, remaining > 0 ? step : -step);
      }
      if(!ok) {
        status_label_->setText("Newton-Raphson failed to converge");
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
  sliders_.group_box->setEnabled(!computing);

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

#include "shift/gui/mode_window.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QElapsedTimer>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

#include "shift/core/body/flex.h"
#include "shift/core/body/modal.h"
#include "shift/core/frame.h"
#include "shift/gui/simulation.h"

/* the role of a list row: kind 0 an elastic coordinate, 1 a mode computed */
static constexpr int kKindRole = Qt::UserRole;
static constexpr int kIndexRole = Qt::UserRole + 1;
/* a period of the animation (s) */
static constexpr double kPeriod = 1.5;

shift::gui::ModeWindow::ModeWindow(
  Simulation& simulation, int body,
  std::function<void(const std::vector<int>&)> apply,
  std::function<void()> redraw, QWidget* parent)
  : QDialog(parent)
  , simulation_(simulation)
  , body_(body)
  , apply_(std::move(apply))
  , redraw_(std::move(redraw)) {
  flex_ = dynamic_cast<const shift::core::body::Flex*>(
    simulation_.get_bodies()[body]);
  setAttribute(Qt::WA_DeleteOnClose);
  setWindowTitle(QString("Modes of %1")
                   .arg(QString::fromStdString(flex_->get_name())));

  /* the size of the body, for the amplitude */
  const auto& positions = flex_->get_positions();
  ksl_vec3_t low = ksl_vec3(1e300, 1e300, 1e300);
  ksl_vec3_t high = ksl_vec3(-1e300, -1e300, -1e300);
  for(unsigned int p = 0; p < flex_->get_mesh_node_count(); p++) {
    for(int c = 0; c < 3; c++) {
      low.at[c] = std::min(low.at[c], positions[p].at[c]);
      high.at[c] = std::max(high.at[c], positions[p].at[c]);
    }
  }
  ksl_vec3_t diagonal;
  ksl_subtract_vv(&high, &low, &diagonal);
  size_ = std::max(ksl_vec3_l2norm(&diagonal), 1e-9);

  auto* layout = new QVBoxLayout(this);
  list_ = new QListWidget;
  layout->addWidget(list_);

  /* the static modes: attachments after the reference (nodes 1 on), then
  the retained points */
  static const char* kMotions[6] = {"x", "y", "z", "rx", "ry", "rz"};
  const unsigned int statics = flex_->get_static_count();
  const unsigned int attachment_statics = 6 * (flex_->get_node_count() - 1);
  for(unsigned int j = 0; j < statics; j++) {
    QString text;
    if(j < attachment_statics) {
      const auto* frame = flex_->get_node_frame(1 + j / 6);
      text = QString("static: %1 %2")
               .arg(QString::fromStdString(frame->get_name()),
                    kMotions[j % 6]);
    } else {
      const unsigned int r = j - attachment_statics;
      text = QString("static: retained point %1 %2")
               .arg(r / 3)
               .arg(kMotions[r % 3]);
    }
    auto* item = new QListWidgetItem(text, list_);
    item->setData(kKindRole, 0);
    item->setData(kIndexRole, int(j));
  }
  const bool modal =
    dynamic_cast<const shift::core::body::Modal*>(flex_) != nullptr;
  const auto& kept = flex_->get_modes_kept();
  const auto& frequencies = flex_->get_frequencies();
  for(size_t k = 0; k < frequencies.size(); k++) {
    auto* item = new QListWidgetItem(
      QString("mode %1: %2 Hz").arg(k).arg(frequencies[k], 0, 'g', 5), list_);
    item->setData(kKindRole, 1);
    item->setData(kIndexRole, int(k));
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(std::find(kept.begin(), kept.end(), k) != kept.end()
                          ? Qt::Checked
                          : Qt::Unchecked);
  }

  auto* amplitude_row = new QHBoxLayout;
  amplitude_row->addWidget(new QLabel("amplitude"));
  amplitude_ = new QSlider(Qt::Horizontal);
  amplitude_->setRange(0, 100);
  amplitude_->setValue(20);
  amplitude_->setToolTip("the largest displacement drawn, as a fraction of "
                         "a tenth of the body's size");
  amplitude_row->addWidget(amplitude_);
  amplitude_label_ = new QLabel;
  amplitude_row->addWidget(amplitude_label_);
  animate_ = new QCheckBox("animate");
  animate_->setChecked(true);
  amplitude_row->addWidget(animate_);
  layout->addLayout(amplitude_row);

  auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close);
  if(modal && !frequencies.empty()) {
    QPushButton* apply_button =
      buttons->addButton("Apply modes kept", QDialogButtonBox::ApplyRole);
    apply_button->setToolTip("write the checked modes as modes_kept and load "
                             "the model again");
    connect(apply_button, &QPushButton::clicked, this, [this]() {
      std::vector<int> modes;
      for(int row = 0; row < list_->count(); row++) {
        QListWidgetItem* item = list_->item(row);
        if(item->data(kKindRole).toInt() == 1 &&
           item->checkState() == Qt::Checked) {
          modes.push_back(item->data(kIndexRole).toInt());
        }
      }
      /* the window goes with the model it shows */
      auto apply = apply_;
      close();
      apply(modes);
    });
  }
  connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  layout->addWidget(buttons);

  clock_ = new QElapsedTimer;
  clock_->start();
  timer_ = new QTimer(this);
  timer_->setInterval(16);
  connect(timer_, &QTimer::timeout, this, [this]() { Preview(); });
  connect(animate_, &QCheckBox::toggled, this, [this](bool on) {
    if(on) {
      timer_->start();
    } else {
      timer_->stop();
      Preview();
    }
  });
  connect(amplitude_, &QSlider::valueChanged, this, [this]() { Preview(); });
  connect(list_, &QListWidget::currentRowChanged, this,
          [this]() { Preview(); });
  if(list_->count() > 0) {
    list_->setCurrentRow(0);
  }
  timer_->start();
  resize(360, 420);
}

shift::gui::ModeWindow::~ModeWindow() {
  delete clock_;
}

void shift::gui::ModeWindow::Preview() {
  const double amplitude = amplitude_->value() / 100.0 * 0.1 * size_;
  amplitude_label_->setText(QString("%1").arg(amplitude, 0, 'g', 3));
  QListWidgetItem* item = list_->currentItem();
  if(!item) {
    simulation_.ClearShapePreview();
    redraw_();
    return;
  }
  const double phase =
    animate_->isChecked()
      ? std::sin(2.0 * M_PI * clock_->elapsed() / (1000.0 * kPeriod))
      : 1.0;
  simulation_.PreviewShape(body_, item->data(kKindRole).toInt(),
                           item->data(kIndexRole).toInt(), amplitude * phase);
  redraw_();
}

void shift::gui::ModeWindow::done(int result) {
  timer_->stop();
  simulation_.ClearShapePreview();
  redraw_();
  QDialog::done(result);
}

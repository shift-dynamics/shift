#include "shift/gui/playback_slider.h"

#include <QMouseEvent>
#include <QPainter>

#include <algorithm>
#include <cmath>

namespace {
constexpr int kTicks = 1000;
}

shift::gui::PlaybackSlider::PlaybackSlider(QWidget* parent)
  : QWidget(parent) {
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

QSize shift::gui::PlaybackSlider::sizeHint() const {
  return QSize(2 * (height_ + margin_), height_ + 2 * margin_);
}

void shift::gui::PlaybackSlider::set_time_span(double t_min, double t_max) {
  t_min_ = t_min;
  t_max_ = std::max(t_max, t_min + 1e-12);
  update();
}

void shift::gui::PlaybackSlider::set_time(double t) {
  const int ticks =
    static_cast<int>(std::lround((t - t_min_) / (t_max_ - t_min_) * kTicks));
  position_ = std::clamp(ticks, 0, kTicks);
  update();
}

void shift::gui::PlaybackSlider::set_progress_time(double t) {
  const int ticks =
    static_cast<int>(std::lround((t - t_min_) / (t_max_ - t_min_) * kTicks));
  progress_ = std::clamp(ticks, 0, kTicks);
  position_ = std::min(position_, progress_);
  update();
}

void shift::gui::PlaybackSlider::Enable(bool enabled) {
  enabled_ = enabled;
  if(!enabled) {
    clicked_ = false;
  }
  update();
}

int shift::gui::PlaybackSlider::OffsetOf(int ticks) const {
  const int start = height() / 2;
  const int range = width() - 2 * start;
  return start + static_cast<int>(float(ticks) / kTicks * range);
}

int shift::gui::PlaybackSlider::TicksAt(int x) const {
  const int start = height() / 2;
  const int range = std::max(width() - 2 * start, 1);
  return static_cast<int>(float(x - start) / float(range) * kTicks);
}

double shift::gui::PlaybackSlider::TimeAt(int ticks) const {
  return t_min_ + double(ticks) / kTicks * (t_max_ - t_min_);
}

void shift::gui::PlaybackSlider::SetPositionFromMouse(int x) {
  // the thumb cannot move past the computed portion of the time span
  position_ = std::clamp(TicksAt(x), 0, progress_);
  update();
  emit time_changed(TimeAt(position_));
}

void shift::gui::PlaybackSlider::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setPen(Qt::NoPen);
  p.setRenderHint(QPainter::Antialiasing, true);

  const int radius = height_ / 2;
  const int y = height() / 2;
  const int bar_height = height() - 2 * margin_;
  const int progress_offset = OffsetOf(progress_);
  const int position_offset = OffsetOf(position_);

  // draw order: track -> progress -> position -> thumb
  p.setBrush(track_brush_);
  p.drawRoundedRect(QRect(margin_, margin_, width() - 2 * margin_, bar_height),
                    8.0, 8.0);

  p.setBrush(progress_brush_);
  p.drawRoundedRect(
    QRect(margin_, margin_, progress_offset + radius - margin_, bar_height),
    8.0, 8.0);

  p.setBrush(position_brush_);
  p.drawRoundedRect(
    QRect(margin_, margin_, position_offset + radius - margin_, bar_height),
    8.0, 8.0);

  p.setBrush(enabled_ ? thumb_brush_ : track_brush_);
  p.drawEllipse(QRectF(position_offset - height() / 2.0, y - height() / 2.0,
                       height(), height()));
}

void shift::gui::PlaybackSlider::mousePressEvent(QMouseEvent* event) {
  if(event->button() == Qt::LeftButton && enabled_) {
    // clicking the track jumps there; dragging follows the mouse
    clicked_ = true;
    SetPositionFromMouse(int(event->position().x()));
  }
}

void shift::gui::PlaybackSlider::mouseMoveEvent(QMouseEvent* event) {
  if(clicked_ && enabled_) {
    SetPositionFromMouse(int(event->position().x()));
  }
}

void shift::gui::PlaybackSlider::mouseReleaseEvent(QMouseEvent* event) {
  if(event->button() == Qt::LeftButton) {
    clicked_ = false;
  }
}

void shift::gui::PlaybackSlider::enterEvent(QEnterEvent* event) {
  setCursor(Qt::PointingHandCursor);
  QWidget::enterEvent(event);
}

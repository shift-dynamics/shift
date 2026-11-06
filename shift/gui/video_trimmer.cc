#include "shift/gui/video_trimmer.h"

#include <QLabel>
#include <QMouseEvent>
#include <QPainter>

#include <algorithm>
#include <cmath>

static constexpr int kHandleWidth = 8;   // pixels, also the margin of the strip
static constexpr int kStripHeight = 48;  // pixels
static constexpr int kHandleReach = 10;  // pixels from a handle that grab it
static const QColor kHandleColor("#308CC6");
static const QColor kTrackColor("#494949");
static const QColor kDimColor(0, 0, 0, 130);

shift::gui::VideoTrimmer::VideoTrimmer(QWidget* parent)
  : QWidget(parent) {
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  setMouseTracking(true);
  preview_ = new QLabel(this, Qt::ToolTip);
  preview_->setStyleSheet("background: #1e1e20; color: #dcdcdc; padding: 2px");
  preview_->setAlignment(Qt::AlignCenter);
}

int shift::gui::VideoTrimmer::FrameCount(double start, double end, double speed,
                                         int frame_rate) {
  return static_cast<int>(
           std::floor((end - start) / speed * frame_rate + 1e-9)) +
         1;
}

double shift::gui::VideoTrimmer::FrameTime(int i, double start, double end,
                                           double speed, int frame_rate) {
  return std::min(start + i * speed / frame_rate, end);
}

QSize shift::gui::VideoTrimmer::sizeHint() const {
  return QSize(240, kStripHeight);
}

void shift::gui::VideoTrimmer::set_span(double span) {
  if(span == span_) {
    return;
  }
  const bool to_the_end = end_ >= span_;
  span_ = std::max(span, 0.0);
  set_range(start_, to_the_end ? span_ : end_);
  update();
}

void shift::gui::VideoTrimmer::set_range(double start, double end) {
  end = std::clamp(end, 0.0, span_);
  start = std::clamp(start, 0.0, end);
  if(start == start_ && end == end_) {
    return;
  }
  start_ = start;
  end_ = end;
  update();
  emit range_changed(start_, end_);
}

void shift::gui::VideoTrimmer::set_thumbnails(std::vector<QImage> thumbnails) {
  thumbnails_ = std::move(thumbnails);
  update();
}

void shift::gui::VideoTrimmer::Clear() {
  thumbnails_.clear();
  drag_ = Drag::kNone;
  preview_->hide();
  span_ = 0.0;
  set_range(0.0, 0.0);
  update();
}

double shift::gui::VideoTrimmer::TimeAt(int x) const {
  const int range = std::max(width() - 2 * kHandleWidth, 1);
  return std::clamp(double(x - kHandleWidth) / range, 0.0, 1.0) * span_;
}

int shift::gui::VideoTrimmer::OffsetOf(double t) const {
  const int range = width() - 2 * kHandleWidth;
  return kHandleWidth +
         (span_ > 0.0 ? static_cast<int>(std::lround(t / span_ * range)) : 0);
}

const QImage* shift::gui::VideoTrimmer::ThumbnailAt(double t) const {
  if(thumbnails_.empty() || !(span_ > 0.0)) {
    return nullptr;
  }
  const int last = static_cast<int>(thumbnails_.size()) - 1;
  const int i = static_cast<int>(std::lround(t / span_ * last));
  return &thumbnails_[std::clamp(i, 0, last)];
}

void shift::gui::VideoTrimmer::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHint(QPainter::SmoothPixmapTransform, true);
  const QRect strip(kHandleWidth, 0, width() - 2 * kHandleWidth, height());
  p.fillRect(strip, kTrackColor);

  // as many previews as fit at their own shape
  if(!thumbnails_.empty() && span_ > 0.0) {
    const QImage& first = thumbnails_.front();
    const double shape = double(first.width()) / std::max(first.height(), 1);
    const int cells = std::max(
      1, static_cast<int>(std::lround(strip.width() / (height() * shape))));
    for(int c = 0; c < cells; c++) {
      const int x0 = strip.left() + strip.width() * c / cells;
      const int x1 = strip.left() + strip.width() * (c + 1) / cells;
      const double t = span_ * (c + 0.5) / cells;
      p.drawImage(QRect(x0, 0, x1 - x0, height()), *ThumbnailAt(t));
    }
  }
  if(!(span_ > 0.0)) {
    return;
  }

  const int left = OffsetOf(start_), right = OffsetOf(end_);
  p.fillRect(QRect(strip.left(), 0, left - strip.left(), height()), kDimColor);
  p.fillRect(QRect(right, 0, strip.right() + 1 - right, height()), kDimColor);

  // the selection: a frame with a handle at each end
  p.fillRect(QRect(left, 0, right - left, 2), kHandleColor);
  p.fillRect(QRect(left, height() - 2, right - left, 2), kHandleColor);
  p.fillRect(QRect(left - kHandleWidth, 0, kHandleWidth, height()),
             kHandleColor);
  p.fillRect(QRect(right, 0, kHandleWidth, height()), kHandleColor);
}

void shift::gui::VideoTrimmer::DragTo(int x) {
  const double t = TimeAt(x);
  if(drag_ == Drag::kStart) {
    set_range(std::min(t, end_), end_);
    emit scrubbed(start_);
  } else if(drag_ == Drag::kEnd) {
    set_range(start_, std::max(t, start_));
    emit scrubbed(end_);
  } else if(drag_ == Drag::kStrip) {
    emit scrubbed(t);
  }
}

void shift::gui::VideoTrimmer::ShowPreview(int x) {
  const double t = TimeAt(x);
  const QImage* image = ThumbnailAt(t);
  if(!image) {
    preview_->hide();
    return;
  }
  preview_->setPixmap(QPixmap::fromImage(*image));
  preview_->adjustSize();
  preview_->move(mapToGlobal(QPoint(x - preview_->width() / 2,
                                    -preview_->height() - 4)));
  preview_->show();
}

void shift::gui::VideoTrimmer::mousePressEvent(QMouseEvent* event) {
  if(event->button() != Qt::LeftButton || !(span_ > 0.0)) {
    return;
  }
  const int x = int(event->position().x());
  const int to_start = std::abs(x - (OffsetOf(start_) - kHandleWidth / 2));
  const int to_end = std::abs(x - (OffsetOf(end_) + kHandleWidth / 2));
  if(std::min(to_start, to_end) > kHandleReach) {
    drag_ = Drag::kStrip;
  } else if(to_start == to_end) {
    // the handles meet: take the one with room to move that way
    drag_ = x < OffsetOf(start_) ? Drag::kStart : Drag::kEnd;
  } else {
    drag_ = to_start < to_end ? Drag::kStart : Drag::kEnd;
  }
  preview_->hide();
  DragTo(x);
}

void shift::gui::VideoTrimmer::mouseMoveEvent(QMouseEvent* event) {
  const int x = int(event->position().x());
  if(drag_ != Drag::kNone) {
    DragTo(x);
    return;
  }
  ShowPreview(x);
  const int to_start = std::abs(x - (OffsetOf(start_) - kHandleWidth / 2));
  const int to_end = std::abs(x - (OffsetOf(end_) + kHandleWidth / 2));
  setCursor(span_ > 0.0 && std::min(to_start, to_end) <= kHandleReach
              ? Qt::SizeHorCursor
              : Qt::ArrowCursor);
}

void shift::gui::VideoTrimmer::mouseReleaseEvent(QMouseEvent* event) {
  if(event->button() == Qt::LeftButton) {
    drag_ = Drag::kNone;
  }
}

void shift::gui::VideoTrimmer::leaveEvent(QEvent* event) {
  preview_->hide();
  QWidget::leaveEvent(event);
}

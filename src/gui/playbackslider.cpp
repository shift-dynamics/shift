#include "playbackslider.hpp"

Shift::PlayBackSlider::PlayBackSlider(QWidget* parent)
  : QAbstractSlider(parent)
  , m_switch(false)
  , m_clicked(false)
  , m_opacity(0.000)
  , m_height(16)
  , m_margin(4)
  , m_position(0)
  , m_progress(0)
  , m_positionOffset(m_margin)
  , m_progressOffset()
  , m_thumbBrush("#308CC6")
  , m_trackBrush("#494949")
  , m_positionBrush("#92C0DD")
  , m_progressBrush("#979899")
  , m_tMin(0)
  , m_tMax(100)
  , m_enabled(false)
  , m_anim(new QPropertyAnimation(this, "offset", this)) {
  setOffset(m_height / 2);
  m_y = m_height / 2;
  setBrush(m_thumbBrush);
}

Shift::PlayBackSlider::PlayBackSlider(const QBrush& positionBrush,
                                      const QBrush& thumbBrush, QWidget* parent)
  : QAbstractSlider(parent)
  , m_switch(false)
  , m_clicked(false)
  , m_opacity(0.000)
  , m_height(16)
  , m_margin(4)
  , m_position(0)
  , m_progress(0)
  , m_thumbBrush("#308CC6")
  , m_trackBrush("#494949")
  , m_positionBrush("#92C0DD")
  , m_progressBrush("#979899")
  , m_tMin(0)
  , m_tMax(100)
  , m_enabled(false)
  , m_anim(new QPropertyAnimation(this, "offset", this)) {
  setOffset(m_height / 2);
  m_y = m_height / 2;
  setBrush(thumbBrush);
  setPositionBrush(positionBrush);
  setThumbBrush(thumbBrush);
}

/*!
@brief updates offset in pixels for progress and current position
*/
void Shift::PlayBackSlider::updateOffset() {
  int border = height() / 2;
  int end = width() - border;
  int start = border;
  int range = end - start;
  m_progressOffset = start + (float)(m_progress) / 1000.0 * range;
  m_positionOffset = start + (float)(m_position) / 1000.0 * range;
}

/*!
@brief custom paint event that draws progress bar as a number of rounded
rectangles
*/
void Shift::PlayBackSlider::paintEvent(QPaintEvent*) {

  QPainter p(this);
  p.setPen(Qt::NoPen);
  p.setOpacity(1.0);
  p.setRenderHint(QPainter::Antialiasing, true);

  int radius = m_height / 2;

  /* draw order: track -> progress -> position -> thumb */

  /* track */
  p.setBrush(m_trackBrush);
  p.drawRoundedRect(
    QRect(m_margin, m_margin, width() - 2 * m_margin, height() - 2 * m_margin),
    8.0, 8.0);

  /* progress */
  p.setBrush(m_progressBrush);
  p.drawRoundedRect(QRect(m_margin, m_margin,
                          m_progressOffset + radius - m_margin,
                          height() - 2 * m_margin),
                    8.0, 8.0);

  /* position */
  p.setBrush(m_positionBrush);
  p.drawRoundedRect(QRect(m_margin, m_margin,
                          m_positionOffset + radius - m_margin,
                          height() - 2 * m_margin),
                    8.0, 8.0);

  /* thumb */
  p.setBrush(m_thumbBrush);
  p.drawEllipse(
    QRectF(m_positionOffset - height() / 2, m_y - radius, height(), height()));
}

/*!
@brief detect if mouse pressed in thumb
*/
void Shift::PlayBackSlider::mousePressEvent(QMouseEvent* e) {
  if(e->button() & Qt::LeftButton && m_enabled) {
    int x_min = m_positionOffset - (m_height / 2);
    int x_max = x_min + height();
    if(e->x() > x_min && e->x() <= x_max) {
      m_clicked = true;
    }
  }
  QAbstractSlider::mousePressEvent(e);
}

void Shift::PlayBackSlider::mouseReleaseEvent(QMouseEvent* e) {
  if(e->button() & Qt::LeftButton && m_enabled) {
    m_clicked = false;
    updateOffset();
    update();
  }
  QAbstractSlider::mouseReleaseEvent(e);
}

/*!
@brief track mouse movements if user clicked on the thumb and update
current position of thumb to follow mouse
*/
void Shift::PlayBackSlider::mouseMoveEvent(QMouseEvent* e) {
  if(m_clicked && m_enabled) {
    int start = height() / 2;
    int end = width() - start;
    int range = end - start;
    m_position = ((float)e->x() - start) / (float)range * 1000.0;
    setPosition(m_position);
  }
  QAbstractSlider::mouseMoveEvent(e);
}

/*!
@brief change to pointing hand cursor on enter
*/
void Shift::PlayBackSlider::enterEvent(QEvent* e) {
  setCursor(Qt::PointingHandCursor);
  QAbstractSlider::enterEvent(e);
}

/*!
@brief provides default size to qt application
*/
QSize Shift::PlayBackSlider::sizeHint() const {
  return QSize(2 * (m_height + m_margin), m_height + 2 * m_margin);
}

/*!
@brief set a position in ticks from 0-1000
*/
void Shift::PlayBackSlider::setPosition(int position) {
  position = std::max(0, position);
  m_position = std::min(position, m_progress);
  double t = (double)m_position / 1000.0 * m_tMax;
  emit timeChanged(t);
  updateOffset();
  update();
}

/*!
@brief set a progress in ticks from 0-1000
*/
void Shift::PlayBackSlider::setProgress(int progress) {
  progress = std::max(0, progress);
  m_progress = std::min(progress, 1000);
  updateOffset();
  update();
}

/*!
@brief redraw sliders and update positions on resize
*/
void Shift::PlayBackSlider::resizeEvent(QResizeEvent* e) {
  updateOffset();
  update();
  QAbstractSlider::resizeEvent(e);
}

/*!
@brief set time span of simulation
*/
void Shift::PlayBackSlider::setTimeSpan(double min, double max) {
  m_tMin = min;
  m_tMax = max;
}

/*!
@brief set current time in simulation, uses stored time span to convert
to ticks internally
*/
void Shift::PlayBackSlider::setTime(double t) {
  int position = int(t / m_tMax * 1000.0);
  setPosition(position);
}

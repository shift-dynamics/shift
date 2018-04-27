#ifndef _SHIFT_PLAYBACKSLIDER_HPP_
#define _SHIFT_PLAYBACKSLIDER_HPP_

#include <QtWidgets>

namespace Shift {
class PlayBackSlider : public QAbstractSlider {
  Q_OBJECT
  Q_PROPERTY(int offset READ offset WRITE setOffset)
  Q_PROPERTY(QBrush brush READ brush WRITE setBrush)

public:
  PlayBackSlider(QWidget* parent = nullptr);
  PlayBackSlider(const QBrush&, const QBrush&, QWidget* parent = nullptr);

  QSize sizeHint() const override;

  QBrush brush() const {
    return m_brush;
  }

  void setBrush(const QBrush& brush) {
    m_brush = brush;
  }

  QBrush getThumbBrush() const {
    return m_thumbBrush;
  }

  void setThumbBrush(const QBrush& brush) {
    m_thumbBrush = brush;
  }

  QBrush getProgressBrush() const {
    return m_progressBrush;
  }

  void setProgressBrush(const QBrush& brush) {
    m_progressBrush = brush;
  }

  QBrush getPositionBrush() const {
    return m_positionBrush;
  }

  void setPositionBrush(const QBrush& brush) {
    m_positionBrush = brush;
  }

  QBrush getTrackBrush() const {
    return m_trackBrush;
  }

  void setTrackBrush(const QBrush& brush) {
    m_trackBrush = brush;
  }

  int offset() const {
    return m_x;
  }

  void setOffset(int offset) {
    m_x = offset;
    update();
  }

  void updateOffset();

public slots:

  void setPosition(int offset);
  void setProgress(int progress);

  void setTime(double);
  void setTimeSpan(double, double);

  void enable(bool enabled) {
    m_enabled = enabled;
  }

signals:

  void timeChanged(double);

protected:
  void paintEvent(QPaintEvent*) override;
  void mouseReleaseEvent(QMouseEvent*) override;
  void enterEvent(QEvent*) override;
  void resizeEvent(QResizeEvent*) override;
  void mousePressEvent(QMouseEvent*) override;
  void mouseMoveEvent(QMouseEvent*) override;

private:
  bool m_switch;
  bool m_clicked;
  qreal m_opacity;
  int m_x;
  int m_y;
  int m_height;
  int m_margin;
  int m_position;       /*!< slider position in range from 0 to 1000 */
  int m_progress;       /*!< progress from 0 to 1000 */
  int m_positionOffset; /*!< slider position offset in pixels */
  int m_progressOffset; /*!< slider progress in pixels */
  QBrush m_thumbBrush;
  QBrush m_trackBrush;
  QBrush m_positionBrush;
  QBrush m_progressBrush;
  QBrush m_brush;
  double m_tMin;
  double m_tMax;
  bool m_enabled;
  QPropertyAnimation* m_anim = nullptr;
};
} // namespace Shift
#endif

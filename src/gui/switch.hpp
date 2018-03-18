#ifndef _SHIFT_SWITCH_HPP_
#define _SHIFT_SWITCH_HPP_

#include <QtWidgets>

namespace Shift {
class Switch : public QAbstractButton {
  Q_OBJECT
  Q_PROPERTY(int offset READ offset WRITE setOffset)
  Q_PROPERTY(QBrush brush READ brush WRITE setBrush)

public:
  Switch(QWidget* parent = nullptr);
  Switch(const QBrush& brush, QWidget* parent = nullptr);

  QSize sizeHint() const override;

  QBrush brush() const {
    return m_brush;
  }

  void setBrush(const QBrush& brush) {
    m_brush = brush;
  }

  int offset() const {
    return m_x;
  }

  void setOffset(int offset) {
    m_x = offset;
    update();
  }

public slots:

  void setChecked(bool);

protected:
  void paintEvent(QPaintEvent*) override;
  void mouseReleaseEvent(QMouseEvent*) override;
  void enterEvent(QEvent*) override;
  void resizeEvent(QResizeEvent*) override;

private:
  bool m_switch;
  qreal m_opacity;
  int m_x;
  int m_y;
  int m_height;
  int m_margin;
  QBrush m_thumb;
  QBrush m_track;
  QBrush m_brush;
  QPropertyAnimation* m_anim = nullptr;
};
} // namespace Shift
#endif

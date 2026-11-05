#ifndef SHIFT_GUI_PLAYBACK_SLIDER_H_
#define SHIFT_GUI_PLAYBACK_SLIDER_H_

#include <QBrush>
#include <QWidget>

namespace shift {
namespace gui {

/*!
@brief Playback slider showing computed progress and the playback position

The track is drawn in three layers: the full track, the portion of the time
span that has been computed (progress), and the playback position with a
draggable thumb. The thumb can only be dragged within the computed range.
Positions and progress are stored in ticks from 0 to 1000.
*/
class PlaybackSlider : public QWidget {
  Q_OBJECT

public:
  explicit PlaybackSlider(QWidget* parent = nullptr);

  QSize sizeHint() const override;

  void set_time_span(double t_min, double t_max);

  /*! \brief set the playback position in time; does not emit time_changed */
  void set_time(double t);

  /*! \brief set the computed portion of the time span in time */
  void set_progress_time(double t);

  /*! \brief allow dragging the thumb (only once simulation data exists) */
  void Enable(bool enabled);

signals:
  /*! \brief emitted when the user drags the thumb */
  void time_changed(double t);

protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void enterEvent(QEnterEvent* event) override;

private:
  int TicksAt(int x) const;
  double TimeAt(int ticks) const;
  int OffsetOf(int ticks) const;
  void SetPositionFromMouse(int x);

  bool clicked_ = false;
  bool enabled_ = false;
  int height_ = 16;
  int margin_ = 4;
  int position_ = 0; /*!< playback position in ticks */
  int progress_ = 0; /*!< computed progress in ticks */
  double t_min_ = 0.0;
  double t_max_ = 1.0;
  QBrush thumb_brush_{QColor("#308CC6")};
  QBrush track_brush_{QColor("#494949")};
  QBrush position_brush_{QColor("#92C0DD")};
  QBrush progress_brush_{QColor("#979899")};
};

} // namespace gui
} // namespace shift

#endif

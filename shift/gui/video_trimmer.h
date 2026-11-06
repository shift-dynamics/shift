#ifndef SHIFT_GUI_VIDEO_TRIMMER_H_
#define SHIFT_GUI_VIDEO_TRIMMER_H_

#include <QImage>
#include <QWidget>

#include <vector>

class QLabel;

namespace shift {
namespace gui {

/*!
@brief Film strip of the computed results with handles that crop the video

The strip spans the computed time, from 0 to the span. Two handles select the
part of it to export; the rest is dimmed. Dragging a handle, or dragging on the
strip, reports the time under the mouse (scrubbed), so that the viewport can
show that frame, and hovering shows a larger preview above the strip.

A range that reaches the end of the span keeps reaching it as the span grows.
*/
class VideoTrimmer : public QWidget {
  Q_OBJECT

public:
  explicit VideoTrimmer(QWidget* parent = nullptr);

  /*! \brief the frames of a video of the range: one every speed / frame_rate
   * of simulated time, the first at start */
  static int FrameCount(double start, double end, double speed, int frame_rate);
  /*! \brief the simulated time of frame i */
  static double FrameTime(int i, double start, double end, double speed,
                          int frame_rate);

  QSize sizeHint() const override;

  /*! \brief the end of the strip: the time computed so far */
  void set_span(double span);
  double get_span() const {
    return span_;
  }

  /*! \brief select from start to end, kept inside the span with start before
   * end; emits range_changed if the range changes */
  void set_range(double start, double end);
  double get_start() const {
    return start_;
  }
  double get_end() const {
    return end_;
  }

  /*! \brief previews evenly spaced over the span, the first at 0 and the last
   * at its end */
  void set_thumbnails(std::vector<QImage> thumbnails);

  /*! \brief no results: an empty span and no previews */
  void Clear();

signals:
  void range_changed(double start, double end);
  /*! \brief the user dragged to this time */
  void scrubbed(double t);

protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void leaveEvent(QEvent* event) override;

private:
  enum class Drag { kNone, kStart, kEnd, kStrip };

  double TimeAt(int x) const;
  int OffsetOf(double t) const;
  const QImage* ThumbnailAt(double t) const;
  void DragTo(int x);
  void ShowPreview(int x);

  double span_ = 0.0;
  double start_ = 0.0;
  double end_ = 0.0;
  Drag drag_ = Drag::kNone;
  std::vector<QImage> thumbnails_;
  QLabel* preview_ = nullptr; /*!< the larger preview, a window of its own */
};

} // namespace gui
} // namespace shift

#endif

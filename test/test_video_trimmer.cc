#include "shift/gui/video_trimmer.h"

#include <gtest/gtest.h>

#include <QApplication>
#include <QMouseEvent>

/* the widget needs an application; the offscreen platform needs no display */
static void EnsureApplication() {
  static int argc = 1;
  static char name[] = "shift-test";
  static char* argv[] = {name, nullptr};
  if(!QApplication::instance()) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    new QApplication(argc, argv);
  }
}

static void Mouse(QWidget& widget, QEvent::Type type, int x) {
  const QPointF point(x, 10.0);
  const Qt::MouseButton button =
    type == QEvent::MouseMove ? Qt::NoButton : Qt::LeftButton;
  QMouseEvent event(type, point, widget.mapToGlobal(point), button,
                    type == QEvent::MouseButtonRelease ? Qt::NoButton
                                                       : Qt::LeftButton,
                    Qt::NoModifier);
  QCoreApplication::sendEvent(&widget, &event);
}

static void Drag(QWidget& widget, int from, int to) {
  Mouse(widget, QEvent::MouseButtonPress, from);
  Mouse(widget, QEvent::MouseMove, to);
  Mouse(widget, QEvent::MouseButtonRelease, to);
}

using shift::gui::VideoTrimmer;

TEST(VideoTrimmer, FramesOfARange) {
  // 2 s at 30 frames per second, with the frame at each end
  EXPECT_EQ(VideoTrimmer::FrameCount(5.0, 7.0, 1.0, 30), 61);
  EXPECT_DOUBLE_EQ(VideoTrimmer::FrameTime(0, 5.0, 7.0, 1.0, 30), 5.0);
  EXPECT_DOUBLE_EQ(VideoTrimmer::FrameTime(60, 5.0, 7.0, 1.0, 30), 7.0);
  // slow motion: 0.1 s of simulation per second of video
  EXPECT_EQ(VideoTrimmer::FrameCount(5.0, 7.0, 0.1, 30), 601);
  // a single instant is a single frame, and no frame passes the end
  EXPECT_EQ(VideoTrimmer::FrameCount(3.0, 3.0, 1.0, 30), 1);
  EXPECT_DOUBLE_EQ(VideoTrimmer::FrameTime(5, 1.0, 1.05, 1.0, 30), 1.05);
}

TEST(VideoTrimmer, RangeStaysInsideTheSpan) {
  EnsureApplication();
  VideoTrimmer trimmer;
  trimmer.set_span(10.0);
  EXPECT_DOUBLE_EQ(trimmer.get_start(), 0.0);
  EXPECT_DOUBLE_EQ(trimmer.get_end(), 10.0);
  trimmer.set_range(-1.0, 20.0);
  EXPECT_DOUBLE_EQ(trimmer.get_start(), 0.0);
  EXPECT_DOUBLE_EQ(trimmer.get_end(), 10.0);
  trimmer.set_range(8.0, 4.0); // start is held at the end
  EXPECT_DOUBLE_EQ(trimmer.get_start(), 4.0);
  EXPECT_DOUBLE_EQ(trimmer.get_end(), 4.0);
}

TEST(VideoTrimmer, EndFollowsTheSpanOnlyIfItReachesIt) {
  EnsureApplication();
  VideoTrimmer trimmer;
  trimmer.set_span(10.0);
  trimmer.set_span(20.0);
  EXPECT_DOUBLE_EQ(trimmer.get_end(), 20.0);
  trimmer.set_range(2.0, 12.0);
  trimmer.set_span(30.0);
  EXPECT_DOUBLE_EQ(trimmer.get_end(), 12.0);
  trimmer.set_span(5.0); // a shorter run
  EXPECT_DOUBLE_EQ(trimmer.get_start(), 2.0);
  EXPECT_DOUBLE_EQ(trimmer.get_end(), 5.0);
}

TEST(VideoTrimmer, DraggingAHandleCropsAndScrubs) {
  EnsureApplication();
  VideoTrimmer trimmer;
  trimmer.resize(216, 48); // a strip of 200 pixels, 8 to 208
  trimmer.set_span(20.0);
  std::vector<double> scrubbed;
  QObject::connect(&trimmer, &VideoTrimmer::scrubbed,
                   [&](double t) { scrubbed.push_back(t); });

  Drag(trimmer, 4, 58); // the start handle to 50 of 200 pixels
  EXPECT_NEAR(trimmer.get_start(), 5.0, 1e-9);
  EXPECT_DOUBLE_EQ(trimmer.get_end(), 20.0);
  ASSERT_FALSE(scrubbed.empty());
  EXPECT_NEAR(scrubbed.back(), 5.0, 1e-9);

  Drag(trimmer, 212, 158); // the end handle to 150 of 200 pixels
  EXPECT_NEAR(trimmer.get_end(), 15.0, 1e-9);
  EXPECT_NEAR(scrubbed.back(), 15.0, 1e-9);
}

TEST(VideoTrimmer, HandlesCannotCross) {
  EnsureApplication();
  VideoTrimmer trimmer;
  trimmer.resize(216, 48);
  trimmer.set_span(20.0);
  trimmer.set_range(5.0, 15.0);
  Drag(trimmer, 54, 200); // the start handle, past the end
  EXPECT_DOUBLE_EQ(trimmer.get_start(), 15.0);
  EXPECT_DOUBLE_EQ(trimmer.get_end(), 15.0);
}

TEST(VideoTrimmer, DraggingOnTheStripScrubsWithoutCropping) {
  EnsureApplication();
  VideoTrimmer trimmer;
  trimmer.resize(216, 48);
  trimmer.set_span(20.0);
  double scrubbed = -1.0;
  QObject::connect(&trimmer, &VideoTrimmer::scrubbed,
                   [&](double t) { scrubbed = t; });
  Drag(trimmer, 108, 128);
  EXPECT_NEAR(scrubbed, 12.0, 1e-9);
  EXPECT_DOUBLE_EQ(trimmer.get_start(), 0.0);
  EXPECT_DOUBLE_EQ(trimmer.get_end(), 20.0);
}

TEST(VideoTrimmer, ClearEmptiesTheSpan) {
  EnsureApplication();
  VideoTrimmer trimmer;
  trimmer.set_span(10.0);
  trimmer.set_range(2.0, 8.0);
  trimmer.Clear();
  EXPECT_DOUBLE_EQ(trimmer.get_span(), 0.0);
  EXPECT_DOUBLE_EQ(trimmer.get_start(), 0.0);
  EXPECT_DOUBLE_EQ(trimmer.get_end(), 0.0);
  trimmer.set_span(4.0); // a new run selects all of it
  EXPECT_DOUBLE_EQ(trimmer.get_end(), 4.0);
}

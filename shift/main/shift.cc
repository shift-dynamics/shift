#include "shift/gui/workspace.h"

#include <QApplication>
#include <QCommandLineParser>
#include <QPalette>
#include <QStyleFactory>

#include <cmath>
#include <cstdio>

/*! \brief parse the command line options of the viewer into request;
 * returns false, with a message in error, if one is invalid */
static bool ParseImageRequest(const QCommandLineParser& parser,
                              shift::gui::Workspace::ImageRequest& request,
                              QString& error) {
  request.filename = parser.value("image");
  bool ok = true;
  if(parser.isSet("time")) {
    request.time = parser.value("time").toDouble(&ok);
    if(!ok || request.time < 0.0) {
      error = "--time takes a time of at least 0 s";
      return false;
    }
  }
  if(parser.isSet("size")) {
    const QStringList size = parser.value("size").split('x');
    int width = 0, height = 0;
    bool ok_height = false;
    if(size.size() == 2) {
      width = size[0].toInt(&ok);
      height = size[1].toInt(&ok_height);
    }
    if(size.size() != 2 || !ok || !ok_height || width < 16 || height < 16) {
      error = "--size takes WIDTHxHEIGHT in pixels, e.g. 1200x800";
      return false;
    }
    request.size = QSize(width, height);
  }
  if(parser.isSet("frame")) {
    const QStringList frame = parser.value("frame").split(',');
    double values[4] = {0.0, 0.0, 0.0, 0.0};
    bool all = frame.size() == 4;
    for(int i = 0; all && i < 4; i++) {
      values[i] = frame[i].toDouble(&all);
    }
    if(!all || !(values[3] > 0.0)) {
      error = "--frame takes X,Y,Z,RADIUS, with a radius above 0";
      return false;
    }
    request.set_frame = true;
    request.frame_center = QVector3D(values[0], values[1], values[2]);
    request.frame_radius = float(values[3]);
  }
  if(parser.isSet("view")) {
    const QStringList view = parser.value("view").split(',');
    bool ok_elevation = false;
    if(view.size() == 2) {
      request.azimuth = float(view[0].toDouble(&ok) * M_PI / 180.0);
      request.elevation = float(view[1].toDouble(&ok_elevation) * M_PI / 180.0);
    }
    if(view.size() != 2 || !ok || !ok_elevation) {
      error = "--view takes AZIMUTH,ELEVATION in degrees, e.g. 35,20";
      return false;
    }
    request.set_view = true;
  }
  if(parser.isSet("samples")) {
    request.samples = parser.value("samples").toInt(&ok);
    if(!ok || request.samples < 1) {
      error = "--samples takes a number of ray traced frames of at least 1";
      return false;
    }
  }
  request.hide_floor = parser.isSet("no-floor");
  request.follow_body = parser.value("follow");
  request.follow_ground = parser.isSet("follow-ground");
  if(request.follow_ground && request.follow_body.isEmpty()) {
    error = "--follow-ground needs --follow";
    return false;
  }
  return true;
}

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  QApplication::setApplicationName("shift");
  QApplication::setStyle(QStyleFactory::create("Fusion"));

  QCommandLineParser parser;
  parser.setApplicationDescription("Shift multibody dynamics viewer");
  parser.addHelpOption();
  parser.addPositionalArgument("model", "model JSON file to open");
  parser.addOptions({
    {"image", "render the model to an image file (PNG, JPEG, ...) and quit",
     "file"},
    {"time", "with --image: simulate to this time first (s)", "seconds"},
    {"size", "with --image: image size (default 1200x800)", "WIDTHxHEIGHT"},
    {"view",
     "with --image: camera azimuth about +z from +x towards +y, and "
     "elevation above the x-y plane, in degrees",
     "AZIMUTH,ELEVATION"},
    {"frame",
     "with --image: frame this sphere of the scene, not all of it, in the "
     "inertial frame",
     "X,Y,Z,RADIUS"},
    {"samples", "with --image: ray trace, averaging this many frames", "n"},
    {"no-floor", "with --image: hide the floor"},
    {"follow",
     "with --image: set the view at the start, and follow this body to --time",
     "body"},
    {"follow-ground",
     "with --follow: follow the ground under the body, level, so that the "
     "body is seen to bounce and tilt"},
  });
  parser.process(app);

  shift::gui::Workspace::ImageRequest request;
  const bool image = parser.isSet("image");
  if(image) {
    QString error;
    if(parser.positionalArguments().isEmpty()) {
      error = "--image needs a model file";
    }
    if(!error.isEmpty() || !ParseImageRequest(parser, request, error)) {
      std::fprintf(stderr, "shift: %s\n", error.toLocal8Bit().constData());
      return 1;
    }
  }

  QPalette palette;
  palette.setColor(QPalette::Window, QColor(45, 45, 48));
  palette.setColor(QPalette::WindowText, QColor(220, 220, 220));
  palette.setColor(QPalette::Base, QColor(30, 30, 32));
  palette.setColor(QPalette::AlternateBase, QColor(45, 45, 48));
  palette.setColor(QPalette::ToolTipBase, QColor(45, 45, 48));
  palette.setColor(QPalette::ToolTipText, QColor(220, 220, 220));
  palette.setColor(QPalette::Text, QColor(220, 220, 220));
  palette.setColor(QPalette::Button, QColor(55, 55, 58));
  palette.setColor(QPalette::ButtonText, QColor(220, 220, 220));
  palette.setColor(QPalette::Highlight, QColor(42, 130, 218));
  palette.setColor(QPalette::HighlightedText, Qt::white);
  palette.setColor(QPalette::Disabled, QPalette::Text, QColor(120, 120, 120));
  palette.setColor(QPalette::Disabled, QPalette::ButtonText,
                   QColor(120, 120, 120));
  QApplication::setPalette(palette);

  shift::gui::Workspace window;
  window.show();
  if(!parser.positionalArguments().isEmpty()) {
    const QString model = parser.positionalArguments().first();
    if(image) {
      // no dialogs: nobody may be there to close them
      QString error;
      if(!window.Open(model, &error)) {
        std::fprintf(stderr, "shift: %s\n", error.toLocal8Bit().constData());
        return 1;
      }
      window.SaveImage(request);
    } else {
      window.Open(model);
    }
  }
  return app.exec();
}

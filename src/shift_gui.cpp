#include "workspace.h"
#include <QApplication>

int main(int argc, char* argv[]) {

  // qputenv(QT_STYLE_OVERRIDE, "gtk2");
  // qputenv("QT_AUTO_SCREEN_SCALE_FACTOR", "1");
  // QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

  QApplication app(argc, argv);

  QFont font;
  font.setPointSize(12);
  font.setFamily(font.defaultFamily());
  app.setFont(font);

  Shift::Workspace workspace;
  workspace.setMouseTracking(true);
  // window.show();
  workspace.showMaximized();
  return app.exec();
}

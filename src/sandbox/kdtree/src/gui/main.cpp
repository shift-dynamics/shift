#include <QApplication>
#include "qtviewer.h"

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  qtviewer window;
 	window.setMouseTracking(true);
  window.setContentsMargins(QMargins(0, 0, 0, 0));
  window.show();
  return app.exec();
}

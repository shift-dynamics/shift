#include <QApplication>
#include "basicviewer.hpp"

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  BasicViewer window;
 	window.setMouseTracking(true);
  window.show();
  return app.exec();
}

#include "shift/gui/workspace.h"

#include <QApplication>
#include <QPalette>
#include <QStyleFactory>

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  QApplication::setApplicationName("shift");
  QApplication::setStyle(QStyleFactory::create("Fusion"));

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
  if(argc > 1) {
    window.Open(QString::fromLocal8Bit(argv[1]));
  }
  return app.exec();
}

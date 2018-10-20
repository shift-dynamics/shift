#include <QtCore/QDebug>
#include <QtWidgets/QTreeWidget>

#include "shift-gui/workspace.h"
#include "ui_mainwindow.h"

namespace Shift {

Workspace::Workspace(QWidget* parent)
  : QMainWindow(parent)
  , ui(new Ui::MainWindow) {
  ui->setupUi(this);
}

Workspace::~Workspace() {
  delete ui;
}

} // namespace Shift

#include <QtCore/QDebug>
#include <QtWidgets/QTreeWidget>

#include "shift-gui/workspace.h"
#include "ui_workspace.h"

namespace Shift {

Workspace::Workspace(QWidget* parent)
  : QMainWindow(parent)
  , ui_(new Ui::MainWindow) {
  ui_->setupUi(this);
}

Workspace::~Workspace() {
  delete ui_;
}

} // namespace Shift

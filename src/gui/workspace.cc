

#include "shift-gui/workspace.h"
#include "shift-gui/vulkanwindow.h"
#include "ui_workspace.h"

#include <QApplication>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QLoggingCategory>
#include <QVulkanInstance>
#include <QtCore/QDebug>
#include <QtWidgets/QTreeWidget>

namespace Shift {

Workspace::Workspace(VulkanWindow* vulkanWindow, QWidget* parent)
  : QMainWindow(parent)
  , ui_(new Ui::MainWindow) {
  ui_->setupUi(this);

  QHBoxLayout* layout = findChild<QHBoxLayout*>("horizontalLayout_2");

  QWidget* container = QWidget::createWindowContainer(vulkanWindow);

  container->setFocusPolicy(Qt::StrongFocus);
  container->setFocus();
  container->setMinimumWidth(100);

  layout->addWidget(container);

  // return app.exec();
}

Workspace::~Workspace() {
  delete ui_;
}

} // namespace Shift

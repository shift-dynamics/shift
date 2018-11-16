#include "shift-gui/vulkanwidget.h"
#include "shift-gui/workspace.h"

#include <QApplication>
#include <QLibraryInfo>
#include <QLoggingCategory>
#include <QPlainTextEdit>
#include <QPointer>
#include <QVulkanInstance>

Q_LOGGING_CATEGORY(lcVk, "qt.vulkan")

static QPointer<QPlainTextEdit> messageLogWidget;
static QtMessageHandler oldMessageHandler = nullptr;

static void messageHandler(QtMsgType msgType,
                           const QMessageLogContext& logContext,
                           const QString& text) {
  if(!messageLogWidget.isNull())
    messageLogWidget->appendPlainText(text);
  if(oldMessageHandler)
    oldMessageHandler(msgType, logContext, text);
}

int main(int argc, char* argv[]) {

  QApplication app(argc, argv);

  // Q_INIT_RESOURCE(geometry);
  // Q_INIT_RESOURCE(html);
  // Q_INIT_RESOURCE(icons);
  Q_INIT_RESOURCE(shaders);
  // Q_INIT_RESOURCE(textures);

  messageLogWidget = new QPlainTextEdit(QLatin1String(QLibraryInfo::build()) +
                                        QLatin1Char('\n'));
  messageLogWidget->setReadOnly(true);

  oldMessageHandler = qInstallMessageHandler(messageHandler);

  QLoggingCategory::setFilterRules(QStringLiteral("qt.vulkan=true"));

  QVulkanInstance inst;

#ifndef Q_OS_ANDROID
  inst.setLayers(QByteArrayList() << "VK_LAYER_LUNARG_standard_validation");
#else
  inst.setLayers(QByteArrayList() << "VK_LAYER_GOOGLE_threading"
                                  << "VK_LAYER_LUNARG_parameter_validation"
                                  << "VK_LAYER_LUNARG_object_tracker"
                                  << "VK_LAYER_LUNARG_core_validation"
                                  << "VK_LAYER_LUNARG_image"
                                  << "VK_LAYER_LUNARG_swapchain"
                                  << "VK_LAYER_GOOGLE_unique_objects");
#endif

  if(!inst.create()) {
    qFatal("Failed to create Vulkan instance: %d", inst.errorCode());
  }

  VulkanWindow* vulkanWindow = new VulkanWindow;
  vulkanWindow->setVulkanInstance(&inst);

  MainWindow mainWindow(vulkanWindow, messageLogWidget.data());
  QObject::connect(vulkanWindow, &VulkanWindow::vulkanInfoReceived, &mainWindow,
                   &MainWindow::onVulkanInfoReceived);
  QObject::connect(vulkanWindow, &VulkanWindow::frameQueued, &mainWindow,
                   &MainWindow::onFrameQueued);

  // Shift::Workspace workspace(vulkanWindow);
  //
  // workspace.setMouseTracking(true);
  //
  // // window.show();
  mainWindow.showMaximized();
  return app.exec();
}
